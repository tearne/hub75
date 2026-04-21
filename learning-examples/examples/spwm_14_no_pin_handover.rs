//! # DP3364S S-PWM — Step 4: Dual-Core Packing
//!
//! Fourth in the SPWM progression. Builds on `spwm_3_dual_pio.rs`.
//!
//! **What this step adds:** core 1 handles `fill_pixels` + `pack_pixels`
//! while core 0 drives the display via PIO+DMA. Double-buffered frame
//! data ensures no tearing.
//!
//! **Architecture:** core 0 scans continuously from chip SRAM, only
//! pausing briefly (~2.6ms) to DMA new frame data when core 1 signals
//! a freshly packed buffer is ready. Brightness is constant regardless
//! of how long packing takes — scan duty cycle is decoupled from pack
//! performance.
//!
//! ```text
//!   Core 0: scan loop (SM1) ──→ core 1 done? ──→ pause, DMA new
//!           ↑                     no → keep scanning    data (SM0),
//!           └──────────────────────────────────────── resume scan
//!
//!   Core 1: wait for signal → fill pixels → pack → signal done
//! ```
//!
//! Communication via SIO FIFO (hardware mailbox between cores).
//!
//! **Remaining limitation:** each data load requires a ~2.6ms dark gap
//! for the SM0↔SM1 handover (shared CLK pin). At high pack rates this
//! is imperceptible (frequent short gaps = steady dim). At low pack
//! rates (<60 FPS) individual gaps become visible as flicker. Eliminating
//! this would require a single-SM architecture that interleaves data
//! loading with scan — no handover, no dark gap.
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example spwm_4_dual_core
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

/// Write the static command headers (VSYNC, PRE_ACT, WR_CFG) into
/// the given frame buffer. Called once at startup for each buffer;
/// DATA_LATCH headers are written by `pack_pixels()`.
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

/// Pack the PIXELS framebuffer into the given frame buffer's
/// DATA_LATCH regions.
///
/// Gamma-corrected greyscale is precomputed per column (8 chips x
/// 6 channels = 48 lookups per latch) rather than per pixel-clock
/// (128 x 6 = 768), cutting gamma lookups from 393 K to 24 K.
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

// ── Fill pixels helper ──────────────────────────────────────────────

/// Echo-diagnostic pattern matching spwm_12's test: row 31 red
/// (upper wrap source), row 63 green (lower wrap source). Any echo
/// should appear on row 0 (red) / row 32 (green).
fn fill_pixels(_offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        let c = if row == 31 {
            Rgb::new(255, 0, 0)
        } else if row == 63 {
            Rgb::new(0, 255, 0)
        } else {
            Rgb::BLACK
        };
        for col in 0..128usize {
            pixels[row][col] = c;
        }
    }
}

/// Diagonal rainbow: `hue = (row + col + offset) mod 256`.
#[allow(dead_code)]
fn fill_pixels_rainbow(offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        for col in 0..128usize {
            let hue = (row as u16 + col as u16 + offset as u16) as u8;
            pixels[row][col] = hsv(hue);
        }
    }
}

// ── SIO FIFO raw register access ────────────────────────────────────
// After Multicore::new() consumes the HAL SioFifo, both cores use
// raw register access for inter-core communication.

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
            cortex_m::asm::wfe(); // sleep until event
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

        // Fill pixels with scrolling rainbow
        fill_pixels(offset);

        // Pack into target buffer
        pack_pixels(buf);

        // Update WR_CFG in the target buffer
        update_wr_cfg(buf, (offset as usize / 2) % 13);

        // Signal done
        fifo_write(1);
    }
}

// ── SM enable/disable + pindir handover ─────────────────────────────

const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_CTRL_SET: u32 = PIO0_BASE + 0x2000;
const PIO0_CTRL_CLR: u32 = PIO0_BASE + 0x3000;
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_PINCTRL: u32 = PIO0_BASE + 0x0DC;
const SM1_PINCTRL: u32 = PIO0_BASE + 0x0F4;
const SM0_INSTR: u32 = PIO0_BASE + 0x0D8;
const SM1_INSTR: u32 = PIO0_BASE + 0x0F0;
const SM0_EN: u32 = 1 << 0;
const SM1_EN: u32 = 1 << 1;
const SM0_TXSTALL: u32 = 1 << 24;
const SM1_TXSTALL: u32 = 1 << 25;

const SET_PINDIRS_0: u32 = 0xE080;
const SET_PINDIRS_1: u32 = 0xE081;
const SET_PINDIRS_3: u32 = 0xE083;

const PINCTRL_SET_COUNT_MASK: u32 = 0x7 << 26;
const PINCTRL_SET_BASE_MASK: u32 = 0x1F << 5;

fn enable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_SET as *mut u32).write_volatile(mask) };
}
fn disable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_CLR as *mut u32).write_volatile(mask) };
}

fn wait_txstall(sm_stall_bit: u32) {
    unsafe {
        (PIO0_FDEBUG as *mut u32).write_volatile(sm_stall_bit);
        while (PIO0_FDEBUG as *const u32).read_volatile() & sm_stall_bit == 0 {
            cortex_m::asm::nop();
        }
    }
}

fn force_set_pindirs(pinctrl_addr: u32, instr_addr: u32, base: u32, count: u32, instr: u32) {
    unsafe {
        let saved = (pinctrl_addr as *const u32).read_volatile();
        let modified = (saved & !(PINCTRL_SET_COUNT_MASK | PINCTRL_SET_BASE_MASK))
            | (count << 26) | (base << 5);
        (pinctrl_addr as *mut u32).write_volatile(modified);
        (instr_addr as *mut u32).write_volatile(instr);
        (pinctrl_addr as *mut u32).write_volatile(saved);
    }
}

// E20: handover functions stubbed out to test whether the pindir
// transitions at phase boundaries are what suppresses the echo.
// Both SMs keep pindirs=1 on their assigned pins throughout; only
// disable_sm/enable_sm alternate execution.
fn release_sm0_clk_lat() {}
fn claim_sm0_clk_lat() {}
fn release_sm1_clk() {}
fn claim_sm1_clk() {}

#[allow(dead_code)]
fn _keep_force_set_pindirs_in_scope(a: u32, b: u32, c: u32, d: u32, e: u32) {
    force_set_pindirs(a, b, c, d, e);
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
        (11, hal::pio::PinDir::Output), (12, hal::pio::PinDir::Output),
    ]);
    let _data_sm = data_sm.start();

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

    let (mut scan_sm, _, scan_tx) =
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

    scan_sm.set_pindirs([
        (6,  hal::pio::PinDir::Output), (7,  hal::pio::PinDir::Output),
        (8,  hal::pio::PinDir::Output), (9,  hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output), (11, hal::pio::PinDir::Output),
        (13, hal::pio::PinDir::Output),
    ]);
    let _scan_sm = scan_sm.start();
    disable_sm(SM1_EN);

    // ── 5. DMA — ch0 feeds SM0, ch1 feeds SM1 ────────────────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut data_ch = dma.ch0;
    let mut scan_ch = dma.ch1;
    let mut data_tx = data_tx;
    let mut scan_tx = scan_tx;

    defmt::info!("DP3364S S-PWM (dual-core, double-buffered) starting");

    // ── 6. Init frame headers + initial pixel fill ───────────────────
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) });

    // Initial fill into buffer A (PIXELS is all-black from static init,
    // but we fill with offset 0 for consistency with the main loop).
    fill_pixels(0);
    pack_pixels(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });

    // ── 7. Spawn core 1 ──────────────────────────────────────────────
    let mut mc = hal::multicore::Multicore::new(
        &mut pac.PSM, &mut pac.PPB, &mut sio_hal.fifo,
    );
    let cores = mc.cores();
    cores[1].spawn(unsafe { CORE1_STACK.take().unwrap() }, core1_task).unwrap();

    // ── 8. Startup flush using buffer A ──────────────────────────────
    for flush in 0..14u32 {
        if (flush as usize) < CONFIG_REGS.len() {
            update_wr_cfg(
                unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) },
                flush as usize,
            );
        }
        let buf = unsafe { &*core::ptr::addr_of!(FRAME_BUF_A) };
        let cfg = single_buffer::Config::new(data_ch, buf, data_tx);
        let xfer = cfg.start();
        let (ch, _, new_tx) = xfer.wait();
        data_ch = ch;
        data_tx = new_tx;
        wait_txstall(SM0_TXSTALL);

        release_sm0_clk_lat();
        disable_sm(SM0_EN);
        claim_sm1_clk();
        let scan = unsafe { &*core::ptr::addr_of!(SCAN_BUF) };
        let scan_cfg = single_buffer::Config::new(scan_ch, scan, scan_tx);
        let scan_xfer = scan_cfg.start();
        enable_sm(SM1_EN);
        let (ch, _, new_tx) = scan_xfer.wait();
        scan_ch = ch;
        scan_tx = new_tx;
        wait_txstall(SM1_TXSTALL);
        release_sm1_clk();
        disable_sm(SM1_EN);
        claim_sm0_clk_lat();
        enable_sm(SM0_EN);
    }
    defmt::info!("flush complete");

    // ── 9. Main loop — dual-core double-buffered ─────────────────────
    //
    // Core 0 DMAs the active buffer while core 1 packs the inactive
    // buffer. After both finish, swap. No tearing, no dark gaps from
    // packing.
    let mut active_buf: u8 = 0; // 0 = A is display buffer, 1 = B
    let mut offset: u8 = 0;

    // Kick off core 1 with the first pack job
    offset = offset.wrapping_add(2);
    let mut core1_buf: u8 = 1 - active_buf;
    fifo_write(offset as u32 | ((core1_buf as u32) << 8));
    let mut core1_done = false;

    // Do the initial data load (SM0 already enabled)
    {
        let buf = if active_buf == 0 {
            unsafe { &*core::ptr::addr_of!(FRAME_BUF_A) }
        } else {
            unsafe { &*core::ptr::addr_of!(FRAME_BUF_B) }
        };
        let data_cfg = single_buffer::Config::new(data_ch, buf, data_tx);
        let data_xfer = data_cfg.start();
        let (ch, _, new_tx) = data_xfer.wait();
        data_ch = ch;
        data_tx = new_tx;
        wait_txstall(SM0_TXSTALL);
    }

    // Handover to scan — SM1 stays active from here
    release_sm0_clk_lat();
    disable_sm(SM0_EN);
    claim_sm1_clk();
    enable_sm(SM1_EN);

    loop {
        // ── Scan one full pass (32 rows) ─────────────────────────
        let scan = unsafe { &*core::ptr::addr_of!(SCAN_BUF) };
        let scan_cfg = single_buffer::Config::new(scan_ch, scan, scan_tx);
        let scan_xfer = scan_cfg.start();
        let (ch, _, new_tx) = scan_xfer.wait();
        scan_ch = ch;
        scan_tx = new_tx;
        wait_txstall(SM1_TXSTALL);

        // ── Check if core 1 finished packing ─────────────────────
        if !core1_done {
            if fifo_try_read().is_some() {
                core1_done = true;
            }
        }

        // ── If new frame ready, briefly load it then resume scan ─
        if core1_done {
            // Handover to SM0 for data load
            release_sm1_clk();
            disable_sm(SM1_EN);
            claim_sm0_clk_lat();
            enable_sm(SM0_EN);

            // Swap and DMA new data
            active_buf = core1_buf;
            let buf = if active_buf == 0 {
                unsafe { &*core::ptr::addr_of!(FRAME_BUF_A) }
            } else {
                unsafe { &*core::ptr::addr_of!(FRAME_BUF_B) }
            };
            let data_cfg = single_buffer::Config::new(data_ch, buf, data_tx);
            let data_xfer = data_cfg.start();
            let (ch, _, new_tx) = data_xfer.wait();
            data_ch = ch;
            data_tx = new_tx;
            wait_txstall(SM0_TXSTALL);

            // Handover back to scan
            release_sm0_clk_lat();
            disable_sm(SM0_EN);
            claim_sm1_clk();
            enable_sm(SM1_EN);

            // Kick off next pack job on core 1
            offset = offset.wrapping_add(2);
            core1_buf = 1 - active_buf;
            fifo_write(offset as u32 | ((core1_buf as u32) << 8));
            core1_done = false;
        }
    }
}
