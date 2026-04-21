//! # Thread A brightness probe — spwm_15 (single-SM program-swap) variant
//!
//! Purpose: produce a full-frame, uniform-brightness image on the
//! spwm_15 (single-SM, program-swap) architecture so its perceived
//! brightness can be compared directly against spwm_14's. spwm_15 has
//! already been confirmed echo-free; the open question from its header
//! is whether it is *as bright* as spwm_14, or whether the program-swap
//! architecture costs a few percent of on-time that a full-frame flood
//! would reveal.
//!
//! Pair with `brightness_14`. Same `MOVING` and `GREY_LEVEL` values on
//! both for the comparison to be meaningful.
//!
//! ## Fill mode (compile-time flag)
//!
//! Flip `MOVING` below:
//!   - `false` → solid mid-grey flood at `GREY_LEVEL` on every pixel.
//!     Brightness reference.
//!   - `true`  → diagonal hue-scrolling rainbow. Surfaces any
//!     periodic dim gaps. spwm_15 should have *none*; spwm_14 has
//!     the known ~2.6 ms SM-handover gap.
//!
//! ## Architecture — unchanged from spwm_15
//!
//! The only differences between this file and `spwm_15_single_sm_ported`
//! are the header, the two compile-time consts below, and the
//! `fill_pixels` body. PIO programs, program-swap helpers, DMA, scan
//! loop are verbatim — brightness is the only variable we want to
//! measure.
//!
//! ```sh
//! cargo run --release --example brightness_15
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

// ── Thread A controls ────────────────────────────────────────────────
/// `false` = solid grey flood (brightness probe).
/// `true`  = diagonal rainbow scroll (flicker probe).
const MOVING: bool = false;
/// Per-channel level for the solid-grey probe. Keep the same value as
/// `brightness_14`'s `GREY_LEVEL` for a valid comparison.
const GREY_LEVEL: u8 = 128;

// ── Timing instrumentation (DWT cycle counter) ──────────────────────
// Cortex-M DWT CYCCNT counts clk_sys cycles. Wraps every ~28.6 s at
// 150 MHz — plenty for 1 s report windows. Tried RP2350 TIMER0 first;
// its µs tick didn't start, probably because the TICKS peripheral is
// gated by something the HAL's `enable_tick_generation` doesn't touch.
// DWT is CPU-local and always works.
const SYS_CLK_MHZ: u32 = 150;
const DEMCR: u32     = 0xE000_EDFC;
const DWT_CTRL: u32  = 0xE000_1000;
const DWT_CYCCNT: u32 = 0xE000_1004;

fn enable_cycle_counter() {
    unsafe {
        let demcr = (DEMCR as *const u32).read_volatile();
        (DEMCR as *mut u32).write_volatile(demcr | (1 << 24)); // TRCENA
        (DWT_CYCCNT as *mut u32).write_volatile(0);
        let ctrl = (DWT_CTRL as *const u32).read_volatile();
        (DWT_CTRL as *mut u32).write_volatile(ctrl | 1); // CYCCNTENA
    }
}

fn now_cycles() -> u32 {
    unsafe { (DWT_CYCCNT as *const u32).read_volatile() }
}

// ── HUB75 pin mapping (Interstate 75 W) ──────────────────────────────
const R0: u32 = 1 << 0;
const G0: u32 = 1 << 1;
const B0: u32 = 1 << 2;
const R1: u32 = 1 << 3;
const G1: u32 = 1 << 4;
const B1: u32 = 1 << 5;
const ALL_RGB: u32 = R0 | G0 | B0 | R1 | G1 | B1;

// ── Panel geometry ───────────────────────────────────────────────────
const SCAN_LINES: usize = 32;

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

// ── Double-buffered frame data ───────────────────────────────────────
static mut FRAME_BUF_A: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];
static mut FRAME_BUF_B: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

// ── Core 1 stack ─────────────────────────────────────────────────────
static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

// ── Scan parameters ──────────────────────────────────────────────────
const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;
const OE_CLK_W4: u32 = 4;
const OE_CLK_W12: u32 = 12;

static mut SCAN_BUF: [u32; SCAN_LINES] = [0u32; SCAN_LINES];

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

// ── Scan buffer builder ─────────────────────────────────────────────

fn build_scan_buf() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SCAN_BUF) };
    for row in 0..SCAN_LINES {
        let oe = if row == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
        buf[row] = (DISPLAY_CLK - 1)
            | ((row as u32) << 7)
            | ((SETUP_CLK - 1) << 12)
            | ((oe - 1) << 17);
    }
}

// ── Frame-buffer packing ─────────────────────────────────────────────

const fn header(n_pre: u32, n_lat: u32) -> u32 {
    ((n_lat - 1) << 16) | (n_pre - 1)
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

fn pack_pixels(buf: &mut [u32; FRAME_WORDS]) {
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

// ── Fill pixels — Thread A probe ────────────────────────────────────
//
// `MOVING=false`: every pixel set to `Rgb(GREY_LEVEL, GREY_LEVEL,
// GREY_LEVEL)`. Offset is ignored. Gamma LUT still applies inside
// `pack_pixels`.
//
// `MOVING=true`: diagonal rainbow, `hue = (row + col + offset) mod 256`.

fn fill_pixels(offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        for col in 0..128usize {
            pixels[row][col] = if MOVING {
                let hue = (row as u16 + col as u16 + offset as u16) as u8;
                hsv(hue)
            } else {
                Rgb::new(GREY_LEVEL, GREY_LEVEL, GREY_LEVEL)
            };
        }
    }
}

// ── SIO FIFO raw register access ────────────────────────────────────

fn fifo_write(val: u32) {
    unsafe {
        while (0xD000_0050 as *const u32).read_volatile() & 2 == 0 {}
        (0xD000_0054 as *mut u32).write_volatile(val);
        cortex_m::asm::sev();
    }
}

fn fifo_try_read() -> Option<u32> {
    unsafe {
        if (0xD000_0050 as *const u32).read_volatile() & 1 != 0 {
            Some((0xD000_0058 as *const u32).read_volatile())
        } else {
            None
        }
    }
}

fn fifo_read() -> u32 {
    unsafe {
        loop {
            if (0xD000_0050 as *const u32).read_volatile() & 1 != 0 {
                return (0xD000_0058 as *const u32).read_volatile();
            }
            cortex_m::asm::wfe();
        }
    }
}

// ── Core 1 task ─────────────────────────────────────────────────────

fn core1_task() {
    loop {
        let msg = fifo_read();
        let offset = (msg & 0xFF) as u8;
        let buf_idx = ((msg >> 8) & 1) as u8;

        let buf = if buf_idx == 0 {
            unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) }
        } else {
            unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) }
        };

        fill_pixels(offset);

        pack_pixels(buf);

        update_wr_cfg(buf, (offset as usize / 2) % 13);

        fifo_write(1);
    }
}

// ── PIO register access + program-swap helpers ──────────────────────

const PIO0_BASE: u32    = 0x5020_0000;
const PIO0_CTRL_CLR: u32 = PIO0_BASE + 0x3000;
const PIO0_FDEBUG: u32  = PIO0_BASE + 0x008;
const SM0_CLKDIV: u32   = PIO0_BASE + 0x0C8;
const SM0_EXECCTRL: u32 = PIO0_BASE + 0x0CC;
const SM0_INSTR: u32    = PIO0_BASE + 0x0D8;
const SM0_PINCTRL: u32  = PIO0_BASE + 0x0DC;
const SM1_EXECCTRL: u32 = PIO0_BASE + 0x0E4;
const SM1_PINCTRL: u32  = PIO0_BASE + 0x0F4;
const SM1_EN: u32       = 1 << 1;
const SM0_TXSTALL: u32  = 1 << 24;

fn disable_sm(mask: u32) { unsafe { (PIO0_CTRL_CLR as *mut u32).write_volatile(mask) }; }

fn wait_txstall(sm_stall_bit: u32) {
    unsafe {
        (PIO0_FDEBUG as *mut u32).write_volatile(sm_stall_bit);
        while (PIO0_FDEBUG as *const u32).read_volatile() & sm_stall_bit == 0 {
            cortex_m::asm::nop();
        }
    }
}

fn set_clkdiv(int_div: u32) {
    unsafe { (SM0_CLKDIV as *mut u32).write_volatile(int_div << 16) };
}

fn swap_sm0_program(execctrl: u32, pinctrl: u32, program_start: u32) {
    unsafe {
        (SM0_EXECCTRL as *mut u32).write_volatile(execctrl);
        (SM0_PINCTRL as *mut u32).write_volatile(pinctrl);
        let jmp_always = program_start & 0x1F;
        (SM0_INSTR as *mut u32).write_volatile(jmp_always);
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

    enable_cycle_counter();

    // ── 2. Pins ──────────────────────────────────────────────────────
    let mut sio_hal = hal::Sio::new(pac.SIO);
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

    // ── 3. Pre-build the scan buffer ─────────────────────────────────
    build_scan_buf();

    // ── 4. Install both PIO programs ─────────────────────────────────
    let (mut pio0, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // ── SM0: data path ───────────────────────────────────────────────
    let data_ss = pio::SideSet::new(false, 2, false);
    let mut da = pio::Assembler::new_with_side_set(data_ss);
    let mut d_wrap_target = da.label();
    let mut d_wrap_source = da.label();
    let mut d_pre_loop = da.label();
    let mut d_lat_loop = da.label();

    da.bind(&mut d_wrap_target);
    da.out_with_side_set(pio::OutDestination::X, 16, 0b00);
    da.out_with_side_set(pio::OutDestination::Y, 16, 0b00);
    da.bind(&mut d_pre_loop);
    da.out_with_side_set(pio::OutDestination::PINS, 6, 0b00);
    da.out_with_side_set(pio::OutDestination::NULL, 2, 0b01);
    da.jmp_with_side_set(
        pio::JmpCondition::XDecNonZero, &mut d_pre_loop, 0b00,
    );
    da.bind(&mut d_lat_loop);
    da.out_with_side_set(pio::OutDestination::PINS, 6, 0b10);
    da.out_with_side_set(pio::OutDestination::NULL, 2, 0b11);
    da.jmp_with_side_set(
        pio::JmpCondition::YDecNonZero, &mut d_lat_loop, 0b10,
    );
    da.bind(&mut d_wrap_source);
    let data_prog = da.assemble_with_wrap(d_wrap_source, d_wrap_target);
    let data_installed = pio0.install(&data_prog).unwrap();
    let data_start = data_installed.offset() as u32;

    let (mut data_sm, _, data_tx) =
        hal::pio::PIOBuilder::from_installed_program(data_installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 6)
            .side_set_pin_base(11)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(2, 0)
            .build(sm0);

    data_sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (6,  hal::pio::PinDir::Output), (7,  hal::pio::PinDir::Output),
        (8,  hal::pio::PinDir::Output), (9,  hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output), (11, hal::pio::PinDir::Output),
        (12, hal::pio::PinDir::Output), (13, hal::pio::PinDir::Output),
    ]);
    let _data_sm = data_sm.start();

    let data_execctrl = unsafe { (SM0_EXECCTRL as *const u32).read_volatile() };
    let data_pinctrl  = unsafe { (SM0_PINCTRL  as *const u32).read_volatile() };

    // ── SM1: scan path ───────────────────────────────────────────────
    let scan_ss = pio::SideSet::new(false, 1, false);
    let mut sa = pio::Assembler::new_with_side_set(scan_ss);
    let mut s_wrap_target = sa.label();
    let mut s_wrap_source = sa.label();
    let mut s_display = sa.label();
    let mut s_setup = sa.label();
    let mut s_oe = sa.label();

    sa.bind(&mut s_wrap_target);
    sa.out_with_side_set(pio::OutDestination::Y, 7, 0);
    sa.bind(&mut s_display);
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut s_display, 1);
    sa.out_with_side_set(pio::OutDestination::PINS, 5, 0);
    sa.out_with_side_set(pio::OutDestination::Y, 5, 0);
    sa.bind(&mut s_setup);
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut s_setup, 1);
    sa.out_with_side_set(pio::OutDestination::Y, 4, 0);
    sa.set_with_side_set(pio::SetDestination::PINS, 1, 0);
    sa.bind(&mut s_oe);
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    sa.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut s_oe, 1);
    sa.set_with_side_set(pio::SetDestination::PINS, 0, 0);
    sa.out_with_side_set(pio::OutDestination::NULL, 11, 0);
    sa.bind(&mut s_wrap_source);
    let scan_prog = sa.assemble_with_wrap(s_wrap_source, s_wrap_target);
    let scan_installed = pio0.install(&scan_prog).unwrap();
    let scan_start = scan_installed.offset() as u32;

    let (_scan_sm_template, _, _scan_tx_unused) =
        hal::pio::PIOBuilder::from_installed_program(scan_installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(6, 5)
            .set_pins(13, 1)
            .side_set_pin_base(11)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .clock_divisor_fixed_point(3, 0)
            .build(sm1);

    let scan_execctrl = unsafe { (SM1_EXECCTRL as *const u32).read_volatile() };
    let scan_pinctrl  = unsafe { (SM1_PINCTRL  as *const u32).read_volatile() };
    disable_sm(SM1_EN);

    defmt::info!(
        "brightness_15 (single-SM) — MOVING={=bool} GREY_LEVEL={=u8}",
        MOVING, GREY_LEVEL,
    );

    // ── 5. DMA — single channel ch0 ──────────────────────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut ch = dma.ch0;
    let _ch1_reserved = dma.ch1;
    let mut tx = data_tx;

    // ── 6. Init frame headers + initial pixel fill ───────────────────
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) });

    fill_pixels(0);
    pack_pixels(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });

    // ── 7. Spawn core 1 ──────────────────────────────────────────────
    let mut mc = hal::multicore::Multicore::new(
        &mut pac.PSM, &mut pac.PPB, &mut sio_hal.fifo,
    );
    let cores = mc.cores();
    cores[1].spawn(unsafe { CORE1_STACK.take().unwrap() }, core1_task).unwrap();

    let swap_to_scan = || {
        set_clkdiv(3);
        swap_sm0_program(scan_execctrl, scan_pinctrl, scan_start);
    };
    let swap_to_data = || {
        set_clkdiv(2);
        swap_sm0_program(data_execctrl, data_pinctrl, data_start);
    };

    // ── 8. Startup flush using buffer A ──────────────────────────────
    for flush in 0..14u32 {
        if (flush as usize) < CONFIG_REGS.len() {
            update_wr_cfg(
                unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) },
                flush as usize,
            );
        }
        let buf = unsafe { &*core::ptr::addr_of!(FRAME_BUF_A) };
        let cfg = single_buffer::Config::new(ch, buf, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);

        swap_to_scan();
        let scan = unsafe { &*core::ptr::addr_of!(SCAN_BUF) };
        let scan_cfg = single_buffer::Config::new(ch, scan, tx);
        let scan_xfer = scan_cfg.start();
        let (new_ch, _, new_tx) = scan_xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);

        swap_to_data();
    }
    defmt::info!("flush complete");

    // ── 9. Main loop ─────────────────────────────────────────────────
    let mut active_buf: u8 = 0;
    let mut offset: u8 = 0;

    offset = offset.wrapping_add(2);
    let mut core1_buf: u8 = 1 - active_buf;
    fifo_write(offset as u32 | ((core1_buf as u32) << 8));
    let mut core1_done = false;

    {
        let buf = if active_buf == 0 {
            unsafe { &*core::ptr::addr_of!(FRAME_BUF_A) }
        } else {
            unsafe { &*core::ptr::addr_of!(FRAME_BUF_B) }
        };
        let cfg = single_buffer::Config::new(ch, buf, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
    }

    swap_to_scan();

    // Timing accumulators (cycles) — reset every ~1 s report window.
    let mut scan_cy_total: u64 = 0;
    let mut data_cy_total: u64 = 0;
    let mut scan_count: u32 = 0;
    let mut data_count: u32 = 0;
    let mut last_report = now_cycles();

    loop {
        let t_scan0 = now_cycles();
        let scan = unsafe { &*core::ptr::addr_of!(SCAN_BUF) };
        let scan_cfg = single_buffer::Config::new(ch, scan, tx);
        let scan_xfer = scan_cfg.start();
        let (new_ch, _, new_tx) = scan_xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
        scan_cy_total += now_cycles().wrapping_sub(t_scan0) as u64;
        scan_count += 1;

        if !core1_done {
            if fifo_try_read().is_some() {
                core1_done = true;
            }
        }

        if core1_done {
            let t_data0 = now_cycles();
            swap_to_data();

            active_buf = core1_buf;
            let buf = if active_buf == 0 {
                unsafe { &*core::ptr::addr_of!(FRAME_BUF_A) }
            } else {
                unsafe { &*core::ptr::addr_of!(FRAME_BUF_B) }
            };
            let cfg = single_buffer::Config::new(ch, buf, tx);
            let xfer = cfg.start();
            let (new_ch, _, new_tx) = xfer.wait();
            ch = new_ch;
            tx = new_tx;
            wait_txstall(SM0_TXSTALL);

            swap_to_scan();
            data_cy_total += now_cycles().wrapping_sub(t_data0) as u64;
            data_count += 1;

            offset = offset.wrapping_add(2);
            core1_buf = 1 - active_buf;
            fifo_write(offset as u32 | ((core1_buf as u32) << 8));
            core1_done = false;
        }

        // Periodic report — every ~1 s of wall time (150 M cycles).
        let now = now_cycles();
        if now.wrapping_sub(last_report) >= SYS_CLK_MHZ * 1_000_000 {
            let scan_avg_us = if scan_count > 0 {
                (scan_cy_total / (SYS_CLK_MHZ as u64 * scan_count as u64)) as u32
            } else { 0 };
            let data_avg_us = if data_count > 0 {
                (data_cy_total / (SYS_CLK_MHZ as u64 * data_count as u64)) as u32
            } else { 0 };
            let total = scan_cy_total + data_cy_total;
            let dark_permille = if total > 0 {
                (data_cy_total * 1000 / total) as u32
            } else { 0 };
            defmt::info!(
                "scan: {=u32}×{=u32}µs  data: {=u32}×{=u32}µs  dark: {=u32}‰",
                scan_count, scan_avg_us, data_count, data_avg_us, dark_permille,
            );
            scan_cy_total = 0;
            data_cy_total = 0;
            scan_count = 0;
            data_count = 0;
            last_report = now;
        }
    }
}
