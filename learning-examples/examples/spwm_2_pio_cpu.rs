//! # DP3364S S-PWM — Step 2: PIO + DMA with Per-Pixel Colour
//!
//! Second in the three-part progression. Builds on `spwm_1_cpu.rs`.
//!
//! **What this step adds:** PIO+DMA replaces CPU bitbang for all four
//! S-PWM commands (VSYNC, PRE_ACT, WR_CFG, DATA_LATCH). A per-pixel
//! RGB888 framebuffer with gamma correction drives a scrolling HSV
//! rainbow. The CPU still handles the scan loop (CLK + ROW/OE pulses)
//! via SIO bitbang, with pin-function switching between PIO and SIO
//! each frame.
//!
//! **What step 3 will add:** a second PIO SM for scan, eliminating the
//! CPU scan loop and enabling pack/scan overlap (display stays lit
//! during packing).
//!
//! ## PIO program (8 instructions)
//!
//! 2-bit sideset on GPIO 11–12 (CLK, LAT). Autopull = 32, LSB-first.
//!
//! ```text
//! .side_set 2
//! .wrap_target
//!     out   x, 16        side 0b00   ; X <- N_pre  - 1   (low 16 bits of header)
//!     out   y, 16        side 0b00   ; Y <- N_lat  - 1   (high 16 bits of header)
//! pre_loop:
//!     out   pins, 6      side 0b00   ; LAT low — shift a data pixel
//!     out   null, 2      side 0b01   ; CLK rising edge, LAT still low
//!     jmp   x--, pre_loop side 0b00
//! lat_loop:
//!     out   pins, 6      side 0b10   ; LAT high — shift a data pixel
//!     out   null, 2      side 0b11   ; CLK rising edge + LAT high → command
//!     jmp   y--, lat_loop side 0b10
//! .wrap                               ; next `out x, 16 side 0b00` drops LAT
//! ```
//!
//! ## Pin handover
//!
//! GPIO 12 (LAT) stays under PIO for the whole program; GPIO 0–5 (RGB)
//! and GPIO 11 (CLK) flip between SIO (for CPU-bitbanged display_loop)
//! and PIO0 (for the DMA-driven command stream).
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example spwm_2_pio_cpu
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
const ADDR_MASK: u32 = 0x1F << 6;
const CLK: u32 = 1 << 11;
const LAT: u32 = 1 << 12;
const OE: u32 = 1 << 13;
const ALL_HUB75: u32 = ALL_RGB | ADDR_MASK | CLK | LAT | OE;

// ── Panel geometry (64×128, 1/32 scan) ───────────────────────────────
const SCAN_LINES: usize = 32;
const ROW_PERIOD: usize = 128;

// ── DP3264S / DP3364S configuration registers ────────────────────────
const CONFIG_REGS: [u16; 13] = [
    0x1100, 0x021F, 0x037F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
];

// ── Per-command CLK counts ───────────────────────────────────────────
const VSYNC_PRE: u32 = 1;    const VSYNC_LAT: u32 = 3;
const VSYNC_WORDS: usize = 1;

const PRE_ACT_PRE: u32 = 2;  const PRE_ACT_LAT: u32 = 14;
const PRE_ACT_WORDS: usize = 4;

const WR_CFG_PRE: u32 = 123; const WR_CFG_LAT: u32 = 5;
const WR_CFG_WORDS: usize = 32;

const DATA_LATCH_PRE: u32 = 127; const DATA_LATCH_LAT: u32 = 1;
const DATA_LATCH_WORDS: usize = 32;

const VSYNC_OFFSET:      usize = 0;
const PRE_ACT_OFFSET:    usize = VSYNC_OFFSET + 1 + VSYNC_WORDS;      // 2
const WR_CFG_OFFSET:     usize = PRE_ACT_OFFSET + 1 + PRE_ACT_WORDS;  // 7
const DATA_LATCH_OFFSET: usize = WR_CFG_OFFSET + 1 + WR_CFG_WORDS;    // 40

const LATCHES_PER_FRAME: usize = SCAN_LINES * 16; // 512
const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS; // 33

const FRAME_WORDS: usize =
    DATA_LATCH_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE; // 16 936

static mut FRAME_BUF: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

// ── Pixel type and framebuffer ───────────────────────────────────────

#[derive(Copy, Clone)]
struct Rgb { r: u8, g: u8, b: u8 }
impl Rgb {
    const fn new(r: u8, g: u8, b: u8) -> Self { Self { r, g, b } }
    const BLACK: Rgb = Rgb::new(0, 0, 0);
}

static mut PIXELS: [[Rgb; 128]; 64] = [[Rgb::BLACK; 128]; 64];

// ── Gamma LUT: 8-bit -> 14-bit greyscale ─────────────────────────────

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

// ── HSV helper ───────────────────────────────────────────────────────

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

// ── Frame-buffer packing ─────────────────────────────────────────────

/// Pack a command header: low 16 bits = N_pre - 1, high 16 = N_lat - 1.
const fn header(n_pre: u32, n_lat: u32) -> u32 {
    ((n_lat - 1) << 16) | (n_pre - 1)
}

/// Fill 32 words (= 128 pixels = one 16-bit word shifted through 8
/// chips) with the bit pattern of `word` on the lines named by
/// `color_mask`. MSB of `word` goes out first, repeating per chip.
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

/// Write the static command headers (VSYNC, PRE_ACT, WR_CFG) into
/// FRAME_BUF. Called once at startup; DATA_LATCH headers are written
/// by `pack_pixels()`.
fn init_frame_headers() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };
    buf[VSYNC_OFFSET] = header(VSYNC_PRE, VSYNC_LAT);
    buf[VSYNC_OFFSET + 1] = 0;
    buf[PRE_ACT_OFFSET] = header(PRE_ACT_PRE, PRE_ACT_LAT);
    for i in 0..PRE_ACT_WORDS {
        buf[PRE_ACT_OFFSET + 1 + i] = 0;
    }
    buf[WR_CFG_OFFSET] = header(WR_CFG_PRE, WR_CFG_LAT);
    for l in 0..LATCHES_PER_FRAME {
        buf[DATA_LATCH_OFFSET + l * DATA_LATCH_STRIDE] =
            header(DATA_LATCH_PRE, DATA_LATCH_LAT);
    }
}

/// Pack the PIXELS framebuffer into FRAME_BUF's DATA_LATCH regions.
///
/// Gamma-corrected greyscale is precomputed per column (8 chips x
/// 6 channels = 48 lookups per latch) rather than per pixel-clock
/// (128 x 6 = 768), cutting gamma lookups from 393 K to 24 K.
fn pack_pixels() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };
    let pixels = unsafe { &*core::ptr::addr_of!(PIXELS) };

    for scan_line in 0..SCAN_LINES {
        let upper_row = scan_line;
        let lower_row = scan_line + 32;

        for channel in 0..16usize {
            let latch_idx = scan_line * 16 + channel;
            let base = DATA_LATCH_OFFSET + latch_idx * DATA_LATCH_STRIDE + 1;
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

                    let bits = ((ur[chip] >> bit_pos) & 1) as u32
                        | (((ug[chip] >> bit_pos) & 1) as u32) << 1
                        | (((ub[chip] >> bit_pos) & 1) as u32) << 2
                        | (((lr[chip] >> bit_pos) & 1) as u32) << 3
                        | (((lg[chip] >> bit_pos) & 1) as u32) << 4
                        | (((lb[chip] >> bit_pos) & 1) as u32) << 5;
                    acc |= bits << (slot * 8);
                }
                buf[base + w] = acc;
            }
        }
    }
}

/// Rewrite only the WR_CFG payload to carry the next register entry.
fn update_wr_cfg(reg_idx: usize) {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };
    let reg = CONFIG_REGS[reg_idx];
    pack_shift128(
        &mut buf[WR_CFG_OFFSET + 1 .. WR_CFG_OFFSET + 1 + WR_CFG_WORDS],
        reg, ALL_RGB,
    );
}

/// Fill the PIXELS framebuffer with a scrolling HSV rainbow.
fn fill_pixels(offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        for col in 0..128usize {
            let hue = (col as u32 * 255 / 127) as u8;
            pixels[row][col] = hsv(hue.wrapping_add(offset));
        }
    }
}

// ── Pin-function switching ───────────────────────────────────────────
//
// GPIO 12 (LAT) stays under PIO for the whole program; GPIO 0–5 (RGB)
// and GPIO 11 (CLK) flip between SIO (for CPU-bitbanged display_loop)
// and PIO0 (for the DMA-driven command stream).
const IO_BANK0_BASE: u32 = 0x4002_8000;
const FUNC_SIO: u32 = 5;
const FUNC_PIO0: u32 = 6;
const SHARED_PINS: [u32; 7] = [0, 1, 2, 3, 4, 5, 11];

#[inline(always)]
fn set_pin_func(gpio: u32, funcsel: u32) {
    let addr = IO_BANK0_BASE + 0x04 + gpio * 8;
    unsafe { (addr as *mut u32).write_volatile(funcsel) };
}

fn pins_to_sio() {
    for &p in &SHARED_PINS { set_pin_func(p, FUNC_SIO); }
}

fn pins_to_pio0() {
    for &p in &SHARED_PINS { set_pin_func(p, FUNC_PIO0); }
}

// ── SIO bitbang helpers (for display_loop only) ──────────────────────
type Sio = hal::pac::sio::RegisterBlock;

#[inline(always)]
fn clock_pulse(sio: &Sio) {
    sio.gpio_out_set().write(|w| unsafe { w.bits(CLK) });
    cortex_m::asm::nop();
    sio.gpio_out_clr().write(|w| unsafe { w.bits(CLK) });
}

fn send_clocks(sio: &Sio, n: usize) {
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_RGB) });
    for _ in 0..n { clock_pulse(sio); }
}

fn set_addr(sio: &Sio, row: usize) {
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ADDR_MASK) });
    sio.gpio_out_set().write(|w| unsafe { w.bits((row as u32) << 6) });
}

fn row_pulse(sio: &Sio, width: usize) {
    sio.gpio_out_set().write(|w| unsafe { w.bits(OE) });
    for _ in 0..width { clock_pulse(sio); }
    sio.gpio_out_clr().write(|w| unsafe { w.bits(OE) });
}

fn display_loop(sio: &Sio, cycles: usize) {
    let pre_addr = ROW_PERIOD - 28;
    let mut row: usize = 0;
    for _ in 0..cycles * SCAN_LINES {
        send_clocks(sio, pre_addr);
        set_addr(sio, row);
        send_clocks(sio, 28);
        let w = if row == 0 { 12 } else { 4 };
        row_pulse(sio, w);
        row += 1;
        if row >= SCAN_LINES { row = 0; }
    }
}

// ── Entry point ──────────────────────────────────────────────────────

#[hal::entry]
fn main() -> ! {
    // ── 1. Boot ──────────────────────────────────────────────────────
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).unwrap();

    // ── 2. Pins ──────────────────────────────────────────────────────
    //
    // All pins start as SIO outputs. GPIO 12 (LAT) is immediately
    // redirected to PIO and left there for the whole program; the
    // seven other shared pins flip back and forth.
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
    let sio = unsafe { &(*hal::pac::SIO::ptr()) };
    sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_HUB75) });

    // LAT: PIO owns it for the program lifetime.
    set_pin_func(12, FUNC_PIO0);

    // ── 3. PIO program (8 instructions) ──────────────────────────────
    let ss = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(ss);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut pre_loop = a.label();
    let mut lat_loop = a.label();

    a.bind(&mut wrap_target);
    a.out_with_side_set(pio::OutDestination::X, 16, 0b00);   // X = N_pre - 1
    a.out_with_side_set(pio::OutDestination::Y, 16, 0b00);   // Y = N_lat - 1

    a.bind(&mut pre_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b00); // CLK low, LAT low
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b01); // CLK high
    a.jmp_with_side_set(
        pio::JmpCondition::XDecNonZero, &mut pre_loop, 0b00,
    );

    a.bind(&mut lat_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b10); // CLK low, LAT high
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b11); // CLK high, LAT high
    a.jmp_with_side_set(
        pio::JmpCondition::YDecNonZero, &mut lat_loop, 0b10,
    );
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio0.install(&program).unwrap();

    let (mut sm, _, tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(hal::pio::Buffers::OnlyTx)
        .out_pins(0, 6)
        .side_set_pin_base(11)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(32)
        .clock_divisor_fixed_point(2, 0)
        .build(sm0);

    sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (11, hal::pio::PinDir::Output), (12, hal::pio::PinDir::Output),
    ]);

    let _sm = sm.start();

    // ── 4. DMA ───────────────────────────────────────────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut data_ch = dma.ch0;
    let mut data_tx = tx;

    defmt::info!("DP3364S S-PWM (PIO+DMA, per-pixel) starting");

    // ── 5. Main loop ─────────────────────────────────────────────────
    init_frame_headers();
    let mut reg_idx: usize = 0;
    let mut offset: u8 = 0;

    // Fill + pack first frame
    fill_pixels(offset);
    pack_pixels();

    loop {
        // WR_CFG for this frame
        update_wr_cfg(reg_idx);
        reg_idx = (reg_idx + 1) % CONFIG_REGS.len();

        // Data phase: DMA full buffer to SM0
        pins_to_pio0();
        let buf = unsafe { &*core::ptr::addr_of!(FRAME_BUF) };
        let cfg = single_buffer::Config::new(data_ch, buf, data_tx);
        let xfer = cfg.start();
        let (ch, _, new_tx) = xfer.wait();
        data_ch = ch;
        data_tx = new_tx;
        while !data_tx.is_empty() { cortex_m::asm::nop(); }
        cortex_m::asm::delay(50);

        // Scan phase: CPU bitbang
        pins_to_sio();
        display_loop(sio, 10);

        // Pack next frame (display dark during this — the cost of CPU scan)
        offset = offset.wrapping_add(2);
        fill_pixels(offset);
        pack_pixels();
    }
}
