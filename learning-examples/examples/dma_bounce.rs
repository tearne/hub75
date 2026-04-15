//! Autonomous DMA-driven HUB75 scanning with a bouncing square.
//!
//! This example demonstrates fully hardware-driven display scanning:
//! two PIO state machines and four chained DMA channels refresh the
//! panel continuously without any CPU involvement. The CPU just
//! updates a pixel buffer and swaps a pointer — the display keeps
//! scanning the latest frame automatically.
//!
//! Architecture overview:
//!
//!   ┌─────────────┐        ┌─────────────┐
//!   │ DMA CH0→CH1 │──data──▶ PIO SM0     │──RGB+CLK+LAT──▶ Panel
//!   │ (pixel data)│        │ (Data SM)   │                shift regs
//!   └─────────────┘        └──────┬──────┘
//!                            IRQ handshake
//!   ┌─────────────┐        ┌──────┴──────┐
//!   │ DMA CH2→CH3 │─timing─▶ PIO SM1     │──ADDR+OE──────▶ Panel
//!   │ (BCM timing)│        │ (Address SM)│                row select
//!   └─────────────┘        └─────────────┘
//!
//! Reference: github.com/dgrantpete/Pi-Pico-Hub75-Driver
//!
//! Run: cargo run --release --example dma_bounce

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;
use hal::pio::PIOExt;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// ── Display geometry ────────────────────────────────────────────────

const WIDTH: usize = 64;
const HEIGHT: usize = 64;
const ROW_PAIRS: usize = HEIGHT / 2;
const COLOR_DEPTH: usize = 8;
const WORDS_PER_ROW: usize = WIDTH / 4;
const WORDS_PER_BITPLANE: usize = ROW_PAIRS * WORDS_PER_ROW;
const PIXEL_DATA_WORDS: usize = COLOR_DEPTH * WORDS_PER_BITPLANE;
const TIMING_BUF_WORDS: usize = COLOR_DEPTH * 2;

// ── Pixel helpers ───────────────────────────────────────────────────

#[derive(Copy, Clone)]
struct Rgb { r: u8, g: u8, b: u8 }
impl Rgb {
    const fn new(r: u8, g: u8, b: u8) -> Self { Self { r, g, b } }
    const BLACK: Rgb = Rgb::new(0, 0, 0);
}

fn hsv(hue: u8) -> Rgb {
    let h = hue as u16;
    let region = h / 43;
    let remainder = (h - region * 43) * 6;
    let q = (255 - remainder) as u8;
    let t = remainder as u8;
    match region {
        0 => Rgb::new(255, t, 0),  1 => Rgb::new(q, 255, 0),
        2 => Rgb::new(0, 255, t),  3 => Rgb::new(0, q, 255),
        4 => Rgb::new(t, 0, 255),  _ => Rgb::new(255, 0, q),
    }
}

static GAMMA_LUT: [u8; 256] = {
    let mut lut = [0u8; 256]; let mut i = 0;
    while i < 256 {
        let isqrt = { let mut x = i; let mut y = (x+1)/2; while y < x { x = y; y = (x+i/x)/2; } x };
        let v = (i * isqrt + 8) / 16;
        lut[i] = if v > 255 { 255 } else { v as u8 }; i += 1;
    }
    lut
};

fn gamma(c: Rgb) -> Rgb {
    Rgb::new(GAMMA_LUT[c.r as usize], GAMMA_LUT[c.g as usize], GAMMA_LUT[c.b as usize])
}

// ── Bitplane buffer ─────────────────────────────────────────────────
//
// The display uses Binary Code Modulation (BCM): 8 passes per frame,
// one per bit of colour depth. Each pixel's RGB channels are split
// into individual bits and packed so DMA can stream them directly.
//
// Layout: [bitplane0: all rows] [bitplane1: all rows] ... [bitplane7]
// Each row = 16 × u32 words (4 pixels per word, 6 bits per pixel).

struct DmaBuffer { pixels: [u32; PIXEL_DATA_WORDS] }

impl DmaBuffer {
    const fn new() -> Self { Self { pixels: [0u32; PIXEL_DATA_WORDS] } }
    fn as_ptr(&self) -> *const u32 { self.pixels.as_ptr() }

    /// Pack an RGB pixel array into bitplane format with gamma correction.
    fn pack(&mut self, src: &[[Rgb; WIDTH]; HEIGHT]) {
        for bit in 0..COLOR_DEPTH {
            for row in 0..ROW_PAIRS {
                for word_idx in 0..WORDS_PER_ROW {
                    let mut word: u32 = 0;
                    for pix in 0..4 {
                        let col = word_idx * 4 + pix;
                        let upper = gamma(src[row][col]);
                        let lower = gamma(src[row + ROW_PAIRS][col]);
                        let bits = ((upper.r >> bit) & 1) as u32
                            | (((upper.g >> bit) & 1) as u32) << 1
                            | (((upper.b >> bit) & 1) as u32) << 2
                            | (((lower.r >> bit) & 1) as u32) << 3
                            | (((lower.g >> bit) & 1) as u32) << 4
                            | (((lower.b >> bit) & 1) as u32) << 5;
                        word |= bits << (pix * 8);
                    }
                    self.pixels[bit * WORDS_PER_BITPLANE + row * WORDS_PER_ROW + word_idx] = word;
                }
            }
        }
    }
}

// ── BCM timing buffer ───────────────────────────────────────────────
//
// The Address SM consumes pairs of [off_cycles, on_cycles] — one pair
// per bitplane. "on" is how long OE stays active (display visible);
// each bitplane doubles the previous to create binary weighting.

fn generate_timing(base: u32) -> [u32; TIMING_BUF_WORDS] {
    let mut buf = [0u32; TIMING_BUF_WORDS];
    for bit in 0..COLOR_DEPTH {
        buf[bit * 2] = 10;                // off (blanking)
        buf[bit * 2 + 1] = base << bit;   // on (BCM weight)
    }
    buf
}

// ── Static buffers (double-buffered + timing) ───────────────────────

static mut BUF_A: DmaBuffer = DmaBuffer::new();
static mut BUF_B: DmaBuffer = DmaBuffer::new();
static mut ACTIVE_BUF_PTR: u32 = 0;
static mut TIMING_BUF: [u32; TIMING_BUF_WORDS] = [0u32; TIMING_BUF_WORDS];
static mut TIMING_BUF_PTR: u32 = 0;

// ── Raw DMA register helpers ────────────────────────────────────────
//
// We write DMA registers directly rather than using the HAL, because
// the HAL doesn't expose DMA chaining. All addresses are for the
// RP2350 Secure bus (0x5000_xxxx).

const DMA_BASE: u32 = 0x5000_0000;
const PIO0_TXF0: u32 = 0x5020_0010; // SM0 TX FIFO
const PIO0_TXF1: u32 = 0x5020_0014; // SM1 TX FIFO

fn dma_ch_base(n: u32) -> u32 { DMA_BASE + n * 0x40 }

fn dma_write(ch: u32, offset: u32, val: u32) {
    unsafe { ((dma_ch_base(ch) + offset) as *mut u32).write_volatile(val) };
}
fn dma_read(ch: u32, offset: u32) -> u32 {
    unsafe { ((dma_ch_base(ch) + offset) as *const u32).read_volatile() }
}

const READ_ADDR: u32 = 0x00;
const WRITE_ADDR: u32 = 0x04;
const TRANS_COUNT: u32 = 0x08;
const CTRL_TRIG: u32 = 0x0C;
const AL3_READ_ADDR_TRIG: u32 = 0x3C;

fn ctrl(en: bool, inc_read: bool, chain_to: u32, treq: u32) -> u32 {
    (en as u32)             // bit 0: enable
        | (2 << 2)          // bits 2-3: data size = word (32-bit)
        | ((inc_read as u32) << 4)
        | ((chain_to & 0xF) << 13)
        | ((treq & 0x3F) << 17)
        | (1 << 23)         // bit 23: irq_quiet
}

// ── PIO programs ────────────────────────────────────────────────────

/// Install the Data SM program (SM0).
///
/// Clocks pixel data out to GPIO 0-5, with CLK (GPIO 11) and LAT (GPIO 12)
/// controlled via 2-bit sideset. Waits for the Address SM's IRQ before
/// latching, then signals back.
fn install_data_sm(
    pio: &mut hal::pio::PIO<hal::pac::PIO0>,
    sm: hal::pio::UninitStateMachine<(hal::pac::PIO0, hal::pio::SM0)>,
) -> hal::pio::Tx<(hal::pac::PIO0, hal::pio::SM0)> {
    let ss = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(ss);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut pixel_loop = a.label();

    a.out_with_side_set(pio::OutDestination::Y, 32, 0b00);    // load pixel count into Y
    a.bind(&mut wrap_target);
    a.mov_with_side_set(pio::MovDestination::X, pio::MovOperation::None, pio::MovSource::Y, 0b00);
    a.bind(&mut pixel_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b00);  // 6 data bits, CLK low
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b01);  // discard 2 padding, CLK high
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut pixel_loop, 0b00);
    a.wait_with_side_set(1, pio::WaitSource::IRQ, 0, false, 0b00); // wait for Address SM
    a.irq_with_side_set(false, false, 1, false, 0b10);        // signal back + LAT pulse
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, wrap_target);
    let installed = pio.install(&prog).unwrap();

    let (mut sm, _, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(hal::pio::Buffers::OnlyTx)
        .out_pins(0, 6)
        .side_set_pin_base(11)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(32)
        .clock_divisor_fixed_point(4, 0)
        .build(sm);

    sm.set_pindirs([
        (0, hal::pio::PinDir::Output), (1, hal::pio::PinDir::Output),
        (2, hal::pio::PinDir::Output), (3, hal::pio::PinDir::Output),
        (4, hal::pio::PinDir::Output), (5, hal::pio::PinDir::Output),
        (11, hal::pio::PinDir::Output), (12, hal::pio::PinDir::Output),
    ]);

    tx.write((WIDTH - 1) as u32); // pre-load pixel count
    let _sm = sm.start();
    tx
}

/// Install the Address SM program (SM1).
///
/// Cycles through 32 row addresses and controls OE (GPIO 13) via sideset
/// for BCM timing. Consumes timing pairs [off, on] from its TX FIFO
/// (fed by DMA). Coordinates with Data SM via IRQ handshake.
fn install_address_sm(
    pio: &mut hal::pio::PIO<hal::pac::PIO0>,
    sm: hal::pio::UninitStateMachine<(hal::pac::PIO0, hal::pio::SM1)>,
) {
    let ss = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::new_with_side_set(ss);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut next_row = a.label();
    let mut init = a.label();
    let mut off1 = a.label();
    let mut on = a.label();
    let mut off2 = a.label();

    a.jmp_with_side_set(pio::JmpCondition::Always, &mut init, 1);        // OE off
    a.bind(&mut wrap_target);
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut next_row, 1);
    a.out_with_side_set(pio::OutDestination::NULL, 32, 1);               // consume timing word
    a.bind(&mut init);
    a.out_with_side_set(pio::OutDestination::ISR, 32, 1);                // load off-cycles
    a.set_with_side_set(pio::SetDestination::X, 31, 1);                  // row counter
    a.bind(&mut next_row);
    a.irq_with_side_set(false, false, 0, false, 1);                      // signal Data SM
    a.mov_with_side_set(pio::MovDestination::PINS, pio::MovOperation::Invert, pio::MovSource::X, 1);
    a.wait_with_side_set(1, pio::WaitSource::IRQ, 1, false, 1);          // wait for latch
    a.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::ISR, 1);
    a.bind(&mut off1);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut off1, 1);   // OE off
    a.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::OSR, 1);
    a.bind(&mut on);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut on, 0);     // OE ON
    a.mov_with_side_set(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::ISR, 1);
    a.bind(&mut off2);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut off2, 1);   // OE off
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, wrap_target);
    let installed = pio.install(&prog).unwrap();

    let (mut sm, _, _) = hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(hal::pio::Buffers::OnlyTx)
        .out_pins(6, 5)
        .set_pins(6, 5)
        .side_set_pin_base(13)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(32)
        .clock_divisor_fixed_point(4, 0)
        .build(sm);

    sm.set_pindirs([
        (6, hal::pio::PinDir::Output), (7, hal::pio::PinDir::Output),
        (8, hal::pio::PinDir::Output), (9, hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output), (13, hal::pio::PinDir::Output),
    ]);
    let _sm = sm.start();
}

/// Set up four DMA channels in two self-restarting chains.
///
/// Chain 1 (CH0 → CH1): streams pixel bitplane data to Data SM.
/// Chain 2 (CH2 → CH3): streams BCM timing values to Address SM.
///
/// Each chain works the same way:
///   - The "data" channel (CH0/CH2) transfers the full buffer to PIO,
///     paced by the SM's DREQ (it only pushes when the FIFO has room).
///   - When it finishes, it chains to the "control" channel (CH1/CH3).
///   - The control channel writes the buffer's start address back into
///     the data channel's READ_ADDR register, which restarts it.
///   - For the pixel chain, the address comes from a pointer (ACTIVE_BUF_PTR)
///     so we can swap buffers by just updating that pointer.
fn setup_dma_chains() {
    // Pixel data: CH0 streams buffer → SM0, CH1 reloads CH0
    dma_write(0, READ_ADDR, unsafe { BUF_A.as_ptr() as u32 });
    dma_write(0, WRITE_ADDR, PIO0_TXF0);
    dma_write(0, TRANS_COUNT, PIXEL_DATA_WORDS as u32);
    dma_write(0, CTRL_TRIG, ctrl(false, true, 1, 0));   // disabled, inc_read, chain→CH1, treq=SM0

    dma_write(1, READ_ADDR, unsafe { core::ptr::addr_of!(ACTIVE_BUF_PTR) as u32 });
    dma_write(1, WRITE_ADDR, dma_ch_base(0) + AL3_READ_ADDR_TRIG);
    dma_write(1, TRANS_COUNT, 1);
    dma_write(1, CTRL_TRIG, ctrl(true, false, 1, 0x3F)); // enabled, chain→self, unpaced

    // Timing: CH2 streams timing → SM1, CH3 reloads CH2
    dma_write(2, READ_ADDR, unsafe { core::ptr::addr_of!(TIMING_BUF) as u32 });
    dma_write(2, WRITE_ADDR, PIO0_TXF1);
    dma_write(2, TRANS_COUNT, TIMING_BUF_WORDS as u32);
    dma_write(2, CTRL_TRIG, ctrl(false, true, 3, 1));    // disabled, chain→CH3, treq=SM1

    dma_write(3, READ_ADDR, unsafe { core::ptr::addr_of!(TIMING_BUF_PTR) as u32 });
    dma_write(3, WRITE_ADDR, dma_ch_base(2) + AL3_READ_ADDR_TRIG);
    dma_write(3, TRANS_COUNT, 1);
    dma_write(3, CTRL_TRIG, ctrl(true, false, 3, 0x3F)); // enabled, chain→self, unpaced
}

fn start_dma() {
    dma_write(0, CTRL_TRIG, dma_read(0, CTRL_TRIG) | 1); // enable CH0
    dma_write(2, CTRL_TRIG, dma_read(2, CTRL_TRIG) | 1); // enable CH2
}

/// Swap the display to show a new frame. DMA picks up the new pointer
/// automatically on its next cycle — no reconfiguration needed.
fn show_frame(buf: &DmaBuffer) {
    unsafe { ACTIVE_BUF_PTR = buf.as_ptr() as u32; }
}

// ── Entry point ─────────────────────────────────────────────────────

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).unwrap();

    // Enable DMA peripheral
    use hal::dma::DMAExt;
    let _dma = pac.DMA.split(&mut pac.RESETS);

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS,
    );

    // Assign all HUB75 pins to PIO0
    let _pins = (
        pins.gpio0.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio1.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio2.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio3.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio4.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio5.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio6.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio7.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio8.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio9.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio10.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio11.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio12.into_function::<hal::gpio::FunctionPio0>(),
        pins.gpio13.into_function::<hal::gpio::FunctionPio0>(),
    );

    // Initialize timing + buffer pointers
    unsafe {
        TIMING_BUF = generate_timing(25);
        TIMING_BUF_PTR = core::ptr::addr_of!(TIMING_BUF) as u32;
        ACTIVE_BUF_PTR = BUF_A.as_ptr() as u32;
    }

    // Set up PIO state machines and DMA chains
    let (mut pio0, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let _data_tx = install_data_sm(&mut pio0, sm0);
    install_address_sm(&mut pio0, sm1);
    setup_dma_chains();
    start_dma();

    defmt::info!("Autonomous DMA scan running — bouncing square");

    // ── Animation loop ──────────────────────────────────────────────
    //
    // Everything below this point is the ONLY work the CPU does.
    // The display is being refreshed at ~145 Hz entirely by hardware
    // (PIO + DMA). The CPU just draws into a pixel buffer and calls
    // show_frame() to swap the pointer. It could do anything else
    // here — handle USB, run a game, sleep — and the display would
    // keep scanning the last frame without missing a beat.
    let mut pixels = [[Rgb::BLACK; WIDTH]; HEIGHT];
    let mut write_idx: u8 = 1;
    let size: i32 = 12;
    let mut x: i32 = 10;
    let mut y: i32 = 10;
    let mut dx: i32 = 1;
    let mut dy: i32 = 1;
    let mut hue: u8 = 0;

    loop {
        // Draw
        for row in pixels.iter_mut() { for px in row.iter_mut() { *px = Rgb::BLACK; } }
        let color = hsv(hue);
        for py in y.max(0)..(y + size).min(HEIGHT as i32) {
            for px in x.max(0)..(x + size).min(WIDTH as i32) {
                pixels[py as usize][px as usize] = color;
            }
        }

        // Pack into inactive buffer and swap
        let buf = unsafe {
            if write_idx == 0 { &mut *core::ptr::addr_of_mut!(BUF_A) }
            else { &mut *core::ptr::addr_of_mut!(BUF_B) }
        };
        buf.pack(&pixels);
        show_frame(buf);
        write_idx ^= 1;

        // Bounce
        x += dx; y += dy;
        if x <= 0 || x + size >= WIDTH as i32 { dx = -dx; }
        if y <= 0 || y + size >= HEIGHT as i32 { dy = -dy; }
        hue = hue.wrapping_add(1);

        cortex_m::asm::delay(5_000_000);
    }
}
