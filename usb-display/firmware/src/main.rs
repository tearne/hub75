//! USB display firmware — autonomous DMA-driven scanning.
//!
//! Two PIO state machines + four chained DMAs handle the entire panel scan
//! in hardware. The CPU only runs the Embassy async executor for USB.
//!
//! Architecture:
//!   SM0 (Data):    clocks pixel data + latch, fed by DMA chain CH0→CH1
//!   SM1 (Address): row addressing + OE/BCM timing, fed by DMA chain CH2→CH3
//!   IRQ handshake: SM1 signals "safe to latch", SM0 latches and signals back
//!
//! Reference: github.com/dgrantpete/Pi-Pico-Hub75-Driver
//!
//! Run:  cd usb-display/firmware && cargo run --release

#![no_std]
#![no_main]


use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio;
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::pio::{self as epio, Pio};
use embassy_rp::usb;
use embassy_rp::Peri;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use static_cell::StaticCell;

mod display;
use display::{
    DmaBuffer, FrameReceiver, ReceiveBuffer,
    WIDTH, DMA_BUF_WORDS, TIMING_BUF_WORDS,
};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => epio::InterruptHandler<PIO0>;
});

// ── Double-buffered pixel data for DMA ──────────────────────────────

static mut DMA_BUF_A: DmaBuffer = DmaBuffer::new();
static mut DMA_BUF_B: DmaBuffer = DmaBuffer::new();

/// Pointer to the active buffer's pixel array. DMA control channel reads this
/// to reload the data channel's read address. Updated atomically by USB task.
static mut ACTIVE_BUF_PTR: u32 = 0;

/// Timing buffer for BCM (looped by DMA).
static mut TIMING_BUF: [u32; TIMING_BUF_WORDS] = [0u32; TIMING_BUF_WORDS];

/// Base cycles for BCM timing (bit 0 display time).
const BCM_BASE_CYCLES: u32 = 25;

/// Update global display brightness (0–255). Takes effect on the next
/// Address SM timing cycle automatically — no DMA reconfiguration needed.
pub fn set_brightness(brightness: u8) {
    let buf = display::generate_timing_buffer(BCM_BASE_CYCLES, brightness);
    unsafe {
        TIMING_BUF = buf;
    }
}

/// Stop the autonomous scan gracefully.
/// Breaks the DMA chains, waits for completion, then disables state machines.
pub fn stop_scan() {
    let dma = embassy_rp::pac::DMA;
    let pio = embassy_rp::pac::PIO0;

    // Break DMA chains by chaining each channel to itself
    dma.ch(0).ctrl_trig().modify(|w| w.set_chain_to(0));
    dma.ch(2).ctrl_trig().modify(|w| w.set_chain_to(2));

    // Wait for data DMA (CH0) to finish current transfer
    while dma.ch(0).ctrl_trig().read().busy() {}
    // Wait for timing DMA (CH2) to finish current transfer
    while dma.ch(2).ctrl_trig().read().busy() {}

    // Force PIO IRQs to unblock any stalled state machines
    pio.irq_force().write(|w| {
        w.0 = 0x03; // force IRQ 0 and IRQ 1
    });

    // Wait for both SMs to stall (TX FIFOs empty)
    while pio.fstat().read().txempty() & 0x03 != 0x03 {
        cortex_m::asm::nop();
    }

    // Disable both state machines
    pio.ctrl().modify(|w| {
        w.set_sm_enable(0); // disable SM0
    });
    pio.ctrl().modify(|w| {
        w.set_sm_enable(0); // already 0, but explicit
    });

    // Set OE high (display off)
    // Set OE high (display off) via raw SIO register
    const SIO_GPIO_OUT_SET: *mut u32 = (0xd000_0000u32 + 0x018) as *mut u32;
    unsafe { SIO_GPIO_OUT_SET.write_volatile(display::OE) };

    defmt::info!("Scan stopped");
}

// ── Entry point ─────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Consume data pin peripherals so they don't get dropped and deconfigured.
    // PIO will drive them via steal(), but the originals must not be dropped.
    core::mem::forget(p.PIN_0);
    core::mem::forget(p.PIN_1);
    core::mem::forget(p.PIN_2);
    core::mem::forget(p.PIN_3);
    core::mem::forget(p.PIN_4);
    core::mem::forget(p.PIN_5);

    // Control pins for address — configured as GPIO outputs, leaked
    core::mem::forget(gpio::Output::new(p.PIN_6, gpio::Level::Low));
    core::mem::forget(gpio::Output::new(p.PIN_7, gpio::Level::Low));
    core::mem::forget(gpio::Output::new(p.PIN_8, gpio::Level::Low));
    core::mem::forget(gpio::Output::new(p.PIN_9, gpio::Level::Low));
    core::mem::forget(gpio::Output::new(p.PIN_10, gpio::Level::Low));
    // CLK, LAT — will be driven by PIO sideset, but init as output
    core::mem::forget(gpio::Output::new(p.PIN_11, gpio::Level::Low));
    core::mem::forget(gpio::Output::new(p.PIN_12, gpio::Level::Low));
    // OE — init HIGH (display off), Address SM will control via sideset
    core::mem::forget(gpio::Output::new(p.PIN_13, gpio::Level::High));

    // ── Initialize buffers ──────────────────────────────────────────
    unsafe {
        ACTIVE_BUF_PTR = DMA_BUF_A.as_ptr() as u32;
        TIMING_BUF = display::generate_timing_buffer(25, 4);
    }

    // Log estimated refresh rate
    let sys_clock_hz: u32 = 150_000_000;
    let pio_clock_hz = sys_clock_hz / 4; // clock_divider = 4
    let refresh = display::estimate_refresh_rate(pio_clock_hz, WIDTH as u32, 3, 25, 255);
    defmt::info!("Estimated refresh rate: {} Hz", refresh);

    // Debug: log first few words of the packed buffer
    defmt::info!("buf[0]={:08x} buf[1]={:08x} buf[2]={:08x} buf[3]={:08x}",
        unsafe { DMA_BUF_A.pixels[0] }, unsafe { DMA_BUF_A.pixels[1] },
        unsafe { DMA_BUF_A.pixels[2] }, unsafe { DMA_BUF_A.pixels[3] });

    // ── PIO setup ───────────────────────────────────────────────────
    let Pio { mut common, sm0, sm1, .. } = Pio::new(p.PIO0, Irqs);

    // Data pins (GPIO 0-5) for Data SM
    let r0 = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_0::steal() });
    let g0 = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_1::steal() });
    let b0 = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_2::steal() });
    let r1 = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_3::steal() });
    let g1 = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_4::steal() });
    let b1 = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_5::steal() });
    // CLK (GPIO 11) + LAT (GPIO 12) as sideset for Data SM
    let clk = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_11::steal() });
    let lat = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_12::steal() });
    // Add address pins back one at a time to find which one breaks GPIO 0-2
    let addr_a = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_6::steal() });
    let addr_b = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_7::steal() });
    let addr_c = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_8::steal() });
    let addr_d = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_9::steal() });
    let addr_e = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_10::steal() });
    let oe = common.make_pio_pin(unsafe { embassy_rp::peripherals::PIN_13::steal() });

    // ── Data SM (SM0): pixel clocking + latch ───────────────────────
    // Sideset: 2 bits — bit 0 = CLK (GPIO 11), bit 1 = LAT (GPIO 12)
    // OUT: 6 bits — GPIO 0-5 (pixel data)
    let data_side_set = pio::SideSet::new(false, 2, false);
    let mut da = pio::Assembler::<32>::new_with_side_set(data_side_set);
    let mut d_wrap_target = da.label();
    let mut d_wrap_source = da.label();
    let mut d_write_data = da.label();

    // Load pixel count - 1 into Y (consumed from FIFO once at startup)
    da.out_with_side_set(pio::OutDestination::Y, 32, 0b00);
    da.bind(&mut d_wrap_target);
    // Reload pixel counter from Y each row
    da.mov_with_side_set(pio::MovDestination::X, pio::MovOperation::None, pio::MovSource::Y, 0b00);
    // Clock pixels: 3 instructions per pixel (6 data + 2 discard + jmp)
    da.bind(&mut d_write_data);
    da.out_with_side_set(pio::OutDestination::PINS, 6, 0b00);      // 6 data bits, CLK low
    da.out_with_side_set(pio::OutDestination::NULL, 2, 0b01);      // discard 2, CLK high
    da.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut d_write_data, 0b00); // loop
    // Wait for address SM to signal safe-to-latch
    da.wait_with_side_set(1, pio::WaitSource::IRQ, 0, false, 0b00);
    // Latch + signal latch complete
    da.irq_with_side_set(false, false, 1, pio::IrqIndexMode::DIRECT, 0b10); // LAT high
    da.bind(&mut d_wrap_source);

    let data_prog = da.assemble_with_wrap(d_wrap_source, d_wrap_target);


    let mut data_cfg = epio::Config::default();
    data_cfg.use_program(&common.load_program(&data_prog), &[&clk, &lat]);
    data_cfg.set_out_pins(&[&r0, &g0, &b0, &r1, &g1, &b1]);
    data_cfg.shift_out = epio::ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: epio::ShiftDirection::Right,
    };
    data_cfg.fifo_join = epio::FifoJoin::TxOnly;
    data_cfg.clock_divider = 4u16.into();

    let mut data_sm = sm0;
    data_sm.set_config(&data_cfg);
    data_sm.set_pin_dirs(epio::Direction::Out, &[&r0, &g0, &b0, &r1, &g1, &b1, &clk, &lat]);

    // ── Address SM (SM1): row address + OE/BCM timing ───────────────
    let addr_side_set = pio::SideSet::new(false, 1, false);
    let mut aa = pio::Assembler::<32>::new_with_side_set(addr_side_set);
    let mut a_wrap_target = aa.label();
    let mut a_wrap_source = aa.label();
    let mut a_next_row = aa.label();
    let mut a_initialize = aa.label();
    let mut a_off_before = aa.label();
    let mut a_on_delay = aa.label();
    let mut a_off_after = aa.label();

    aa.jmp_with_side_set(pio::JmpCondition::Always, &mut a_initialize, 1);
    aa.bind(&mut a_wrap_target);
    aa.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut a_next_row, 1);
    aa.out_with_side_set(pio::OutDestination::NULL, 32, 1);
    aa.bind(&mut a_initialize);
    aa.out_with_side_set(pio::OutDestination::ISR, 32, 1);
    aa.set_with_side_set(pio::SetDestination::X, 31, 1);
    aa.bind(&mut a_next_row);
    aa.irq_with_side_set(false, false, 0, pio::IrqIndexMode::DIRECT, 1);
    aa.mov_with_side_set(
        pio::MovDestination::PINS, pio::MovOperation::Invert, pio::MovSource::X, 1
    );
    aa.wait_with_side_set(1, pio::WaitSource::IRQ, 1, false, 1);
    aa.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::ISR, 1);
    aa.bind(&mut a_off_before);
    aa.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut a_off_before, 1);
    aa.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::OSR, 1);
    aa.bind(&mut a_on_delay);
    aa.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut a_on_delay, 0);  // OE ON
    aa.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::ISR, 1);
    aa.bind(&mut a_off_after);
    aa.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut a_off_after, 1);
    aa.bind(&mut a_wrap_source);

    let addr_prog = aa.assemble_with_wrap(a_wrap_source, a_wrap_target);

    let mut addr_cfg = epio::Config::default();
    addr_cfg.use_program(&common.load_program(&addr_prog), &[&oe]);
    addr_cfg.set_out_pins(&[&addr_a, &addr_b, &addr_c, &addr_d, &addr_e]);
    addr_cfg.set_set_pins(&[&addr_a, &addr_b, &addr_c, &addr_d, &addr_e]);
    addr_cfg.shift_out = epio::ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: epio::ShiftDirection::Right,
    };
    addr_cfg.fifo_join = epio::FifoJoin::TxOnly;
    addr_cfg.clock_divider = 2u16.into();

    let mut addr_sm = sm1;
    addr_sm.set_config(&addr_cfg);
    addr_sm.set_pin_dirs(epio::Direction::Out, &[&addr_a, &addr_b, &addr_c, &addr_d, &addr_e, &oe]);

    // Push pixel count - 1 to Data SM FIFO before DMA takes over.
    // The PIO program pulls this into Y once at startup (before wrap_target).
    data_sm.tx().push((WIDTH - 1) as u32);

    // Get FIFO pointers before forgetting SMs
    let txf0_addr = data_sm.tx_fifo_ptr() as u32;
    let txf1_addr = addr_sm.tx_fifo_ptr() as u32;

    // ── Configure DMA chains via raw PAC registers ──────────────────
    let dma = embassy_rp::pac::DMA;

    // Data chain: CH0 (data buffer → SM0 TX) chains to CH1 (reload CH0 read addr)
    let ch0 = dma.ch(0);
    ch0.read_addr().write_value(unsafe { DMA_BUF_A.as_ptr() as u32 });
    ch0.write_addr().write_value(txf0_addr);
    ch0.trans_count().write(|w| {
        w.set_count(DMA_BUF_WORDS as u32);
        w.set_mode(embassy_rp::pac::dma::vals::TransCountMode::NORMAL);
    });
    ch0.ctrl_trig().write(|w| {
        w.set_en(false); // don't start yet — wait for SMs
        w.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
        w.set_incr_read(true);
        w.set_incr_write(false);
        w.set_chain_to(1);
        w.set_treq_sel(embassy_rp::pac::dma::vals::TreqSel::from_bits(0)); // PIO0 SM0 TX
        w.set_irq_quiet(true);
    });

    // CH1: reload CH0 read address from pointer indirection
    let ch1 = dma.ch(1);
    ch1.read_addr().write_value(unsafe { core::ptr::addr_of!(ACTIVE_BUF_PTR) as u32 });
    ch1.write_addr().write_value(ch0.al3_read_addr_trig().as_ptr() as u32);
    ch1.trans_count().write(|w| {
        w.set_count(1);
        w.set_mode(embassy_rp::pac::dma::vals::TransCountMode::NORMAL);
    });
    ch1.ctrl_trig().write(|w| {
        w.set_en(true);
        w.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
        w.set_incr_read(false);
        w.set_incr_write(false);
        w.set_chain_to(1); // chain to self (stop until CH0 triggers us)
        w.set_treq_sel(embassy_rp::pac::dma::vals::TreqSel::from_bits(0x3F)); // unpaced
        w.set_irq_quiet(true);
    });

    // Timing chain: CH2 (timing buffer → SM1 TX) chains to CH3
    let ch2 = dma.ch(2);
    ch2.read_addr().write_value(unsafe { core::ptr::addr_of!(TIMING_BUF) as u32 });
    ch2.write_addr().write_value(txf1_addr);
    ch2.trans_count().write(|w| {
        w.set_count(TIMING_BUF_WORDS as u32);
        w.set_mode(embassy_rp::pac::dma::vals::TransCountMode::NORMAL);
    });
    ch2.ctrl_trig().write(|w| {
        w.set_en(false); // don't start yet — wait for SMs
        w.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
        w.set_incr_read(true);
        w.set_incr_write(false);
        w.set_chain_to(3);
        w.set_treq_sel(embassy_rp::pac::dma::vals::TreqSel::from_bits(1)); // PIO0 SM1 TX
        w.set_irq_quiet(true);
    });

    // CH3: reload CH2 read address
    static mut TIMING_BUF_PTR: u32 = 0;
    unsafe { TIMING_BUF_PTR = core::ptr::addr_of!(TIMING_BUF) as u32 };

    let ch3 = dma.ch(3);
    ch3.read_addr().write_value(unsafe { core::ptr::addr_of!(TIMING_BUF_PTR) as u32 });
    ch3.write_addr().write_value(ch2.al3_read_addr_trig().as_ptr() as u32);
    ch3.trans_count().write(|w| {
        w.set_count(1);
        w.set_mode(embassy_rp::pac::dma::vals::TransCountMode::NORMAL);
    });
    ch3.ctrl_trig().write(|w| {
        w.set_en(true);
        w.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
        w.set_incr_read(false);
        w.set_incr_write(false);
        w.set_chain_to(3); // chain to self
        w.set_treq_sel(embassy_rp::pac::dma::vals::TreqSel::from_bits(0x3F)); // unpaced
        w.set_irq_quiet(true);
    });

    // ── Start: SMs first, then DMA ────────────────────────────────
    data_sm.set_enable(true);
    addr_sm.set_enable(true);

    ch0.ctrl_trig().modify(|w| w.set_en(true));
    ch2.ctrl_trig().modify(|w| w.set_en(true));

    // Forget SMs so Drop doesn't disable them
    core::mem::forget(data_sm);
    core::mem::forget(addr_sm);

    defmt::info!("Autonomous scan started — DMA + dual PIO");

    // ── USB task ────────────────────────────────────────────────────
    spawner.spawn(usb_task(p.USB).unwrap());

    // Main task just keeps the executor alive. Display scans autonomously.
    loop {
        embassy_futures::yield_now().await;
    }
}

// ── USB task ────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn usb_task(usb_periph: Peri<'static, USB>) {
    let driver = usb::Driver::new(usb_periph, Irqs);

    let mut config = embassy_usb::Config::new(0x16c0, 0x27dd);
    config.manufacturer = Some("tearne");
    config.product = Some("hub75");
    config.serial_number = Some("001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64);
    let mut usb = builder.build();
    let (_, mut receiver) = class.split();

    defmt::info!("USB task started — waiting for frames");

    let mut rx = FrameReceiver::new();
    let mut rx_buf = ReceiveBuffer::new();
    let mut write_idx: u8 = 1;
    let mut frames_this_sec: u32 = 0;
    let mut last_report = embassy_time::Instant::now();

    let usb_fut = usb.run();
    let rx_fut = async {
        let mut buf = [0u8; 64];
        loop {
            match receiver.read_packet(&mut buf).await {
                Ok(n) if n > 0 => {
                    if rx.feed(&buf[..n], &mut rx_buf) {
                        let dst = unsafe {
                            if write_idx == 0 { &mut *core::ptr::addr_of_mut!(DMA_BUF_A) }
                            else { &mut *core::ptr::addr_of_mut!(DMA_BUF_B) }
                        };
                        dst.load_from_rgb(&rx_buf.pixels);

                        unsafe {
                            ACTIVE_BUF_PTR = dst.as_ptr() as u32;
                        }
                        write_idx ^= 1;
                        frames_this_sec += 1;

                        let now = embassy_time::Instant::now();
                        if now.duration_since(last_report) >= embassy_time::Duration::from_secs(1) {
                            defmt::info!("{} fps received", frames_this_sec);
                            frames_this_sec = 0;
                            last_report = now;
                        }
                    }
                }
                _ => {}
            }
        }
    };

    embassy_futures::join::join(usb_fut, rx_fut).await;
}
