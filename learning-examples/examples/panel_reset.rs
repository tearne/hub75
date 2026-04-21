//! # Panel reset utility
//!
//! Flash between diagnostic runs to overwrite the panel's SRAM with
//! a known-good pattern. Shares spwm_12's init + startup flush + sync
//! sequence (so the DP3364S driver chips get synced the same way), then
//! displays solid **dim blue** on the whole panel, continuously.
//!
//! Why dim blue: visually distinct from diagnostic test patterns
//! (red/green/yellow), and "solid same colour everywhere" means any
//! residual slot-31→slot-0 leak is invisible (source and target are
//! the same colour), so the panel settles to a clean uniform colour
//! regardless of whether the active architecture exhibits the echo.
//!
//! Flash → see dim blue → flash next example → confirms the panel
//! did reboot and start fresh rather than retaining state.
//!
//! ```sh
//! cargo run --release --example panel_reset
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;

use hal::dma::{single_buffer, DMAExt};
use hal::pio::PIOExt;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// ── HUB75 pin mapping (Interstate 75 W) ──────────────────────────────
const R0: u32 = 1 << 0;
const G0: u32 = 1 << 1;
const B0: u32 = 1 << 2;
const R1: u32 = 1 << 3;
const G1: u32 = 1 << 4;
const B1: u32 = 1 << 5;
const ALL_RGB: u32 = R0 | G0 | B0 | R1 | G1 | B1;

const SCAN_LINES: usize = 32;

const CONFIG_REGS: [u16; 13] = [
    0x037F, 0x1100, 0x021F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
];

const VSYNC_PRE: u32 = 1;    const VSYNC_LAT: u32 = 3;
const VSYNC_WORDS: usize = 1;
const PRE_ACT_PRE: u32 = 2;  const PRE_ACT_LAT: u32 = 14;
const PRE_ACT_WORDS: usize = 4;
const WR_CFG_PRE: u32 = 123; const WR_CFG_LAT: u32 = 5;
const WR_CFG_WORDS: usize = 32;
const DATA_LATCH_PRE: u32 = 127; const DATA_LATCH_LAT: u32 = 1;
const DATA_LATCH_WORDS: usize = 32;

const VSYNC_OFFSET:   usize = 0;
const PRE_ACT_OFFSET: usize = VSYNC_OFFSET + 1 + VSYNC_WORDS;
const WR_CFG_OFFSET:  usize = PRE_ACT_OFFSET + 1 + PRE_ACT_WORDS;
const HEADER_WORDS:   usize = WR_CFG_OFFSET + 1 + WR_CFG_WORDS;

const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS;
const LATCHES_PER_LINE:  usize = 16;
const LATCHES_PER_FRAME: usize = SCAN_LINES * LATCHES_PER_LINE;

const DATA_OFFSET: usize = HEADER_WORDS;
const DATA_END:    usize = DATA_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE;

const POST_SCAN_CYCLES: usize = 4;
const SCAN_OFFSET: usize = DATA_END;
const POST_SCAN_WORDS: usize = POST_SCAN_CYCLES * SCAN_LINES;
const FRAME_WORDS: usize = SCAN_OFFSET + POST_SCAN_WORDS;

static mut FRAME_BUF: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

#[repr(C, align(128))]
struct ScanBufAligned([u32; SCAN_LINES]);
static mut SCAN_BUF: ScanBufAligned = ScanBufAligned([0u32; SCAN_LINES]);

const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;
const OE_CLK_W4: u32 = 4;
const OE_CLK_W12: u32 = 12;

#[derive(Copy, Clone)]
struct Rgb { r: u8, g: u8, b: u8 }
impl Rgb {
    const fn new(r: u8, g: u8, b: u8) -> Self { Self { r, g, b } }
    const BLACK: Rgb = Rgb::new(0, 0, 0);
}

static mut PIXELS: [[Rgb; 128]; 64] = [[Rgb::BLACK; 128]; 64];

static GAMMA14: [u16; 256] = {
    let mut lut = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let v = (i as u32 * i as u32 * 0x3FFF) / (255 * 255);
        lut[i] = v as u16;
        i += 1;
    }
    lut
};

const fn data_header(n_pre: u32, n_lat: u32) -> u32 {
    ((n_lat - 1) << 16) | ((n_pre - 1) << 1)
}

fn scan_word(display: u32, addr: u32, setup: u32, oe: u32) -> u32 {
    1 | ((display - 1) << 1) | (addr << 14)
      | ((setup - 1) << 19) | ((oe - 1) << 24)
}

fn scan_word_for_sync(scan_line: usize) -> u32 {
    let oe = if scan_line == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
    scan_word(DISPLAY_CLK, scan_line as u32, SETUP_CLK, oe)
}

fn pack_shift128(out: &mut [u32], word: u16, color_mask: u32) {
    let mask6 = color_mask & ALL_RGB;
    for w in 0..32 {
        let mut acc: u32 = 0;
        for slot in 0..4 {
            let pix = w * 4 + slot;
            let bit_pos = 15 - (pix % 16);
            let data = if (word >> bit_pos) & 1 == 1 { mask6 } else { 0 };
            acc |= data << (slot * 8);
        }
        out[w] = acc;
    }
}

fn init_frame_headers(buf: &mut [u32; FRAME_WORDS]) {
    buf[VSYNC_OFFSET] = data_header(VSYNC_PRE, VSYNC_LAT);
    buf[VSYNC_OFFSET + 1] = 0;

    buf[PRE_ACT_OFFSET] = data_header(PRE_ACT_PRE, PRE_ACT_LAT);
    for i in 0..PRE_ACT_WORDS {
        buf[PRE_ACT_OFFSET + 1 + i] = 0;
    }

    buf[WR_CFG_OFFSET] = data_header(WR_CFG_PRE, WR_CFG_LAT);

    for latch_idx in 0..LATCHES_PER_FRAME {
        buf[DATA_OFFSET + latch_idx * DATA_LATCH_STRIDE] =
            data_header(DATA_LATCH_PRE, DATA_LATCH_LAT);
    }

    for cycle in 0..POST_SCAN_CYCLES {
        for slot in 0..SCAN_LINES {
            let oe = if slot == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
            buf[SCAN_OFFSET + cycle * SCAN_LINES + slot] =
                scan_word(DISPLAY_CLK, slot as u32, SETUP_CLK, oe);
        }
    }
}

fn fill_pixels_solid_blue() {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    let blue = Rgb::new(0, 0, 64);
    for row in 0..64usize {
        for col in 0..128usize {
            pixels[row][col] = blue;
        }
    }
}

/// Verbatim copy of spwm_12's `pack_pixels` (minus PACK_SHIFT rotation
/// which isn't needed here). Safe because it's known-working.
fn pack_pixels(buf: &mut [u32; FRAME_WORDS]) {
    let pixels = unsafe { &*core::ptr::addr_of!(PIXELS) };
    for scan_line in 0..SCAN_LINES {
        let upper_row = scan_line;
        let lower_row = scan_line + 32;

        for channel in 0..LATCHES_PER_LINE {
            let latch_idx = scan_line * LATCHES_PER_LINE + channel;
            let base = DATA_OFFSET + latch_idx * DATA_LATCH_STRIDE + 1;
            let output = 15 - channel;

            let mut ur = [0u16; 8];
            let mut ug = [0u16; 8];
            let mut ub = [0u16; 8];
            let mut lr = [0u16; 8];
            let mut lg = [0u16; 8];
            let mut lb = [0u16; 8];
            for chip in 0..8usize {
                let col = chip * 16 + output;
                let u = pixels[upper_row][col];
                let l = pixels[lower_row][col];
                ur[chip] = GAMMA14[u.r as usize];
                ug[chip] = GAMMA14[u.g as usize];
                ub[chip] = GAMMA14[u.b as usize];
                lr[chip] = GAMMA14[l.r as usize];
                lg[chip] = GAMMA14[l.g as usize];
                lb[chip] = GAMMA14[l.b as usize];
            }

            for w in 0..32usize {
                let mut acc: u32 = 0;
                for slot in 0..4usize {
                    let pix = w * 4 + slot;
                    let chip = 7 - (pix >> 4);
                    let bit_pos = 15 - (pix & 15);
                    let shift = slot * 8;
                    acc |= (((ur[chip] >> bit_pos) & 1) as u32) << shift;
                    acc |= (((ug[chip] >> bit_pos) & 1) as u32) << (shift + 1);
                    acc |= (((ub[chip] >> bit_pos) & 1) as u32) << (shift + 2);
                    acc |= (((lr[chip] >> bit_pos) & 1) as u32) << (shift + 3);
                    acc |= (((lg[chip] >> bit_pos) & 1) as u32) << (shift + 4);
                    acc |= (((lb[chip] >> bit_pos) & 1) as u32) << (shift + 5);
                }
                buf[base + w] = acc;
            }
        }
    }
}

fn update_wr_cfg(buf: &mut [u32; FRAME_WORDS], reg_idx: usize) {
    let reg = CONFIG_REGS[reg_idx];
    pack_shift128(
        &mut buf[WR_CFG_OFFSET + 1 .. WR_CFG_OFFSET + 1 + WR_CFG_WORDS],
        reg, ALL_RGB,
    );
}

const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_CTRL_SET: u32 = PIO0_BASE + 0x2000;
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_CLKDIV: u32 = PIO0_BASE + 0x0C8;
const SM0_EN: u32 = 1 << 0;
const SM0_TXSTALL: u32 = 1 << 24;

const DMA_BASE: u32 = 0x5000_0000;
const CH1_READ_ADDR:  *mut u32 = (DMA_BASE + 0x040) as *mut u32;
const CH1_WRITE_ADDR: *mut u32 = (DMA_BASE + 0x044) as *mut u32;
const CH1_AL1_CTRL:   *mut u32 = (DMA_BASE + 0x050) as *mut u32;
const CH1_AL1_TRANS_COUNT_TRIG: *mut u32 = (DMA_BASE + 0x05C) as *mut u32;
const PIO0_TXF0: u32 = PIO0_BASE + 0x010;

const CH1_CTRL_VAL: u32 =
    (1 << 0) | (2 << 2) | (1 << 4) | (7 << 8) | (1 << 13) | (0 << 17);

fn enable_sm(mask: u32) { unsafe { (PIO0_CTRL_SET as *mut u32).write_volatile(mask) } }

fn set_clkdiv(int_div: u32) {
    unsafe { (SM0_CLKDIV as *mut u32).write_volatile(int_div << 16) };
}

fn wait_txstall(sm_stall_bit: u32) {
    unsafe {
        (PIO0_FDEBUG as *mut u32).write_volatile(sm_stall_bit);
        while (PIO0_FDEBUG as *const u32).read_volatile() & sm_stall_bit == 0 {
            cortex_m::asm::nop();
        }
    }
}

fn start_ring_scan() {
    let scan_addr = core::ptr::addr_of!(SCAN_BUF) as u32;
    unsafe {
        CH1_AL1_CTRL.write_volatile(CH1_CTRL_VAL);
        CH1_READ_ADDR.write_volatile(scan_addr);
        CH1_WRITE_ADDR.write_volatile(PIO0_TXF0);
        CH1_AL1_TRANS_COUNT_TRIG.write_volatile((15 << 28) | 32);
    }
}

fn stop_ring_scan() {
    unsafe {
        const CHAN_ABORT: *mut u32 = (DMA_BASE + 0x464) as *mut u32;
        CHAN_ABORT.write_volatile(1 << 1);
        while CHAN_ABORT.read_volatile() & (1 << 1) != 0 {
            cortex_m::asm::nop();
        }
    }
}

fn build_scan_buf() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SCAN_BUF) };
    for row in 0..SCAN_LINES {
        buf.0[row] = scan_word_for_sync(row);
    }
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).unwrap();

    let sio_hal = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0, pac.PADS_BANK0, sio_hal.gpio_bank0, &mut pac.RESETS,
    );
    let _pins = (
        pins.gpio0.into_push_pull_output(),  pins.gpio1.into_push_pull_output(),
        pins.gpio2.into_push_pull_output(),  pins.gpio3.into_push_pull_output(),
        pins.gpio4.into_push_pull_output(),  pins.gpio5.into_push_pull_output(),
        pins.gpio6.into_push_pull_output(),  pins.gpio7.into_push_pull_output(),
        pins.gpio8.into_push_pull_output(),  pins.gpio9.into_push_pull_output(),
        pins.gpio10.into_push_pull_output(), pins.gpio11.into_push_pull_output(),
        pins.gpio12.into_push_pull_output(), pins.gpio13.into_push_pull_output(),
    );
    let sio_regs = unsafe { &(*hal::pac::SIO::ptr()) };
    sio_regs.gpio_out_clr().write(|w| unsafe {
        w.bits(ALL_RGB | (0x1F << 6) | (1 << 11) | (1 << 12) | (1 << 13))
    });
    const IO_BANK0_BASE: u32 = 0x4002_8000;
    for p in 0..14u32 {
        let addr = IO_BANK0_BASE + 0x04 + p * 8;
        unsafe { (addr as *mut u32).write_volatile(6) };
    }
    const PADS_BANK0: u32 = 0x4003_8000;
    const GPIO12_PAD: u32 = PADS_BANK0 + 0x04 + 12 * 4;
    unsafe {
        let val = (GPIO12_PAD as *const u32).read_volatile();
        (GPIO12_PAD as *mut u32).write_volatile(val | (1 << 2));
    }

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let ss = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(ss);
    let mut dispatch    = a.label();
    let mut data_cmd    = a.label();
    let mut display_lp  = a.label();
    let mut setup_lp    = a.label();
    let mut oe_lp       = a.label();
    let mut pre_loop    = a.label();
    let mut lat_loop    = a.label();
    let mut wrap_source = a.label();

    a.bind(&mut dispatch);
    a.out_with_side_set(pio::OutDestination::X, 1, 0b00);
    a.jmp_with_side_set(pio::JmpCondition::XIsZero, &mut data_cmd, 0b00);
    a.out_with_side_set(pio::OutDestination::Y, 7, 0b00);
    a.bind(&mut display_lp);
    a.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::Y, 0b00);
    a.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::Y, 0b00);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut display_lp, 0b01);
    a.out_with_side_set(pio::OutDestination::PINS, 11, 0b00);
    a.out_with_side_set(pio::OutDestination::Y, 5, 0b00);
    a.bind(&mut setup_lp);
    a.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::Y, 0b00);
    a.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::Y, 0b00);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut setup_lp, 0b01);
    a.out_with_side_set(pio::OutDestination::Y, 4, 0b00);
    a.set_with_side_set(pio::SetDestination::PINS, 1, 0b00);
    a.bind(&mut oe_lp);
    a.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::Y, 0b00);
    a.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::Y, 0b00);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut oe_lp, 0b01);
    a.set_with_side_set(pio::SetDestination::PINS, 0, 0b00);
    a.out_with_side_set(pio::OutDestination::NULL, 4, 0b00);
    a.jmp_with_side_set(pio::JmpCondition::Always, &mut dispatch, 0b00);

    a.bind(&mut data_cmd);
    a.out_with_side_set(pio::OutDestination::X, 15, 0b00);
    a.out_with_side_set(pio::OutDestination::Y, 16, 0b00);
    a.bind(&mut pre_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b00);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b01);
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut pre_loop, 0b00);
    a.bind(&mut lat_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b10);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b11);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut lat_loop, 0b10);
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, dispatch);
    let installed = pio0.install(&prog).unwrap();

    let (mut sm, _, tx) =
        hal::pio::PIOBuilder::from_installed_program(installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 11)
            .set_pins(13, 1)
            .side_set_pin_base(11)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(3, 0)
            .build(sm0);

    sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (6,  hal::pio::PinDir::Output), (7,  hal::pio::PinDir::Output),
        (8,  hal::pio::PinDir::Output), (9,  hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output), (11, hal::pio::PinDir::Output),
        (12, hal::pio::PinDir::Output), (13, hal::pio::PinDir::Output),
    ]);
    let _sm = sm.start();
    let mut tx = tx;

    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut ch = dma.ch0;

    defmt::info!("panel_reset starting");

    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) });

    // Startup flush — same pattern as spwm_12: 26 iterations of
    // data-phase (all-zero DATA_LATCHes → writes black to every
    // scan_line's SRAM) + one scan pass. Syncs driver chips.
    const FLUSH_FRAMES: u32 = 26;
    for flush in 0..FLUSH_FRAMES {
        if (flush as usize) < CONFIG_REGS.len() {
            update_wr_cfg(
                unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) },
                flush as usize,
            );
        }
        set_clkdiv(2);
        let data_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(FRAME_BUF) as *const u32,
                DATA_END,
            )
        };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);

        set_clkdiv(3);
        let scan_slice = unsafe {
            let base = (core::ptr::addr_of!(FRAME_BUF) as *const u32).add(SCAN_OFFSET);
            core::slice::from_raw_parts(base, SCAN_LINES)
        };
        let scan_cfg = single_buffer::Config::new(ch, scan_slice, tx);
        let scan_xfer = scan_cfg.start();
        let (new_ch, _, new_tx) = scan_xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
    }
    defmt::info!("flush complete");

    // Sync phase — ring-mode scan interleaved with occasional data
    // pushes, matches spwm_12's sync bootstrap for ~1 s.
    build_scan_buf();
    let _ch1 = dma.ch1;
    set_clkdiv(3);
    start_ring_scan();

    const SYNC_FRAMES: u32 = 60;
    for _ in 0..SYNC_FRAMES {
        cortex_m::asm::delay(150_000 * 10);
        stop_ring_scan();
        wait_txstall(SM0_TXSTALL);

        set_clkdiv(2);
        let data_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(FRAME_BUF) as *const u32,
                DATA_END,
            )
        };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
        set_clkdiv(3);

        start_ring_scan();
    }
    stop_ring_scan();
    wait_txstall(SM0_TXSTALL);
    defmt::info!("sync complete");

    // Pack solid dim blue into the frame buffer.
    fill_pixels_solid_blue();
    pack_pixels(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) });

    // Main loop — continuously DMA the whole frame (data + post-scan).
    // Simple single-DMA continuous delivery; no split, no diagnostics.
    set_clkdiv(3);
    enable_sm(SM0_EN);
    loop {
        let buf_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(FRAME_BUF) as *const u32,
                FRAME_WORDS,
            )
        };
        let cfg = single_buffer::Config::new(ch, buf_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
    }
}
