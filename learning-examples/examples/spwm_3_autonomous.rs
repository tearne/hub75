//! # HUB75 driver for the Pimoroni Interstate 75 W / DP3364S panel
//!
//! Drives a 64×128 HUB75 LED matrix whose column drivers are DP3364S
//! S-PWM chips (8 chips × 16 columns = 128 columns per scan-line,
//! with 32 upper + 32 lower scan-lines addressed via a 5-bit ADDR bus).
//!
//! ## Architecture overview
//!
//! - **One PIO state machine** (SM0 on PIO0) runs a unified program
//!   with two entry points (`data_cmd` at PC 0, `scan_entry` at PC 11).
//!   The CPU switches between data and scan phases by writing
//!   `SM0_INSTR` (force JMP), `SM0_PINCTRL` (OUT_COUNT swap — see
//!   *Echo suppression* below), and `SM0_EXECCTRL` (WRAP_TOP swap to
//!   route PIO wrap to the correct phase's loop body).
//! - **Two DMA channels** feed the PIO TX FIFO: channel 0 streams the
//!   frame buffer's data section (one-shot per frame); channel 1 runs
//!   ring-mode, replaying a 32-word `SCAN_BUF` `POST_SCAN_CYCLES`
//!   times per frame. Both channels raise `DMA_IRQ_0` on completion.
//! - **Core 0** orchestrates the main loop. `wait_dma()` sleeps on
//!   `wfi` between phase transitions, so core 0 is in `wfi` for
//!   ~99 % of each frame — free for application work.
//! - **Core 1** packs the next frame's pixel data into DATA_LATCH
//!   words while core 0 displays the current frame. Pack takes
//!   ~4.9 ms; the display frame is ~7.8 ms, so content-update rate
//!   matches the display refresh rate (~128 Hz). Double-buffered
//!   frames (`FRAME_BUF_A` / `FRAME_BUF_B`) let pack and display run
//!   concurrently without locking.
//!
//! ## Echo suppression (hard-won)
//!
//! The DP3364S emits a faint "echo" of scan-line 31 on scan-line 0 if
//! the ADDR pins (GPIO 6..10) are held at 0 during the data phase.
//! The RP2350 PIO's `out PINS, 6` writes 6 bits to pins 0..5, and
//! when `PINCTRL.OUT_COUNT` is 11 (the scan-phase value), it
//! *zero-fills* pins 6..10 on every execution. Holding OUT_COUNT at 6
//! during the data phase prevents the zero-fill; pins 6..10 retain
//! their last-driven value from the previous scan (= scan-line 31),
//! and the echo vanishes.
//!
//! The phase swap therefore writes OUT_COUNT along with the JMP and
//! the wrap-range change. Everything else (SIDESET, SET_BASE,
//! SET_COUNT, OUT_BASE) stays constant.
//!
//! ## Hardware assumptions
//!
//! - **Board:** Pimoroni Interstate 75 W (RP2350A).
//! - **Panel pinout (GPIO):**
//!   - 0..5  : R0 G0 B0 R1 G1 B1 (upper and lower RGB)
//!   - 6..10 : ADDR[0..4] (scan-line address)
//!   - 11    : CLK
//!   - 12    : LAT
//!   - 13    : OE (active low)
//! - **Chip:** 8× DP3364S column drivers in series, 128-wide SRAM
//!   (GROUP_SET = 0x7F, first thing written in the boot flush).
//! - **Clock:** PIO CLK is 25 MHz during the data phase (at the
//!   DP3364S datasheet maximum). Data clkdiv = 2, scan clkdiv = 3.
//!
//! ## Tunables the user can change at runtime
//!
//! - `current_scan_cycles` (near end of `main`): 50 = brightest,
//!   shorter refresh; 20 = dimmer, higher refresh. Panel metrics
//!   logged every 256 frames.
//! - `fill_pixels()` selects the content source. Default animates a
//!   diagonal rainbow via `fill_pixels_rainbow(offset)`.
//!
//! ```sh
//! cargo run --release --example spwm_3_autonomous
//! ```
//!
//! ## See also
//!
//! [`reference/HUB75_DP3364S_RP2350_NOTES.md`](../reference/HUB75_DP3364S_RP2350_NOTES.md)
//! in this crate — distilled reference covering hardware facts, the
//! wire protocol, RP2350 PIO gotchas, and the echo artifact. That
//! document references this file by name in several places; if this
//! file is ever renamed or moved, update the notes too.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;

use core::sync::atomic::{AtomicU32, Ordering};
use hal::dma::{single_buffer, DMAExt, SingleChannel};
use hal::pac::interrupt;
use hal::pio::PIOExt;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// ── DMA IRQ completion flags ───────────────────────────────────────
//
// Bit 0 = channel 0 (data DMA) finished.
// Bit 1 = channel 1 (ring scan DMA) finished.
//
// Set by the DMA_IRQ_0 handler, consumed by the main loop's `wait_dma`.
// Use `Ordering::SeqCst` throughout to avoid having to reason about
// barriers; the overhead is irrelevant at 128 IRQs/sec.
static DMA_DONE: AtomicU32 = AtomicU32::new(0);

// ── Core-1 pack-timing counters ─────────────────────────────────────
//
// Core 1 accumulates `pack_pixels` cycle totals and a per-pack max.
// Core 0 reads and resets these at each report window.
static PACK_CY_TOTAL: AtomicU32 = AtomicU32::new(0);
static PACK_CY_MAX:   AtomicU32 = AtomicU32::new(0);
static PACK_COUNT:    AtomicU32 = AtomicU32::new(0);

// DMA register offsets we touch from the IRQ handler and the main
// loop's ring-scan setup. (The HAL's DMA abstraction doesn't expose
// IRQ-status clearing for the channel-independent INTS0 register.)
const DMA_BASE_RAW:  u32      = 0x5000_0000;
const DMA_INTE0_RAW: *mut u32 = (DMA_BASE_RAW + 0x404) as *mut u32;
const DMA_INTS0_RAW: *mut u32 = (DMA_BASE_RAW + 0x40C) as *mut u32;

#[interrupt]
fn DMA_IRQ_0() {
    unsafe {
        let pending = DMA_INTS0_RAW.read_volatile();
        DMA_INTS0_RAW.write_volatile(pending);     // W1C — clear handled bits
        DMA_DONE.fetch_or(pending, Ordering::SeqCst);
    }
}

/// Sleep with `wfi` until the given channel-bits are observed set in
/// [`DMA_DONE`], then clear those bits. Returns the number of cycles
/// spent inside the wait so the caller can report core-0 idle time.
fn wait_dma(bits: u32) -> u32 {
    let start = now_cycles();
    loop {
        let done = DMA_DONE.load(Ordering::SeqCst);
        if done & bits == bits {
            DMA_DONE.fetch_and(!bits, Ordering::SeqCst);
            return now_cycles().wrapping_sub(start);
        }
        cortex_m::asm::wfi();
    }
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
// GROUP_SET (0x03) is FIRST: it controls SRAM address wrap, and at boot
// the chip's default is 0x3F (64-wide). If we write DATA_LATCHes with
// that stale GROUP_SET, writes miss half the SRAM. Setting it first
// guarantees every subsequent DATA_LATCH addresses the full 128-wide
// SRAM correctly.
const CONFIG_REGS: [u16; 13] = [
    0x037F, 0x1100, 0x021F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
];

// ── Per-command CLK counts ───────────────────────────────────────────
// PIO consumes 8 bits per pre/lat loop iteration; header is 32 bits;
// autopull fires every 32 bits consumed. For each command, (PRE+LAT)
// must be a multiple of 4 so commands end on a 32-bit boundary and
// the next dispatch reads a fresh word.
const VSYNC_PRE: u32 = 1;    const VSYNC_LAT: u32 = 3;
const VSYNC_WORDS: usize = 1;

const PRE_ACT_PRE: u32 = 2;  const PRE_ACT_LAT: u32 = 14;
const PRE_ACT_WORDS: usize = 4;

const WR_CFG_PRE: u32 = 123; const WR_CFG_LAT: u32 = 5;
const WR_CFG_WORDS: usize = 32;

const DATA_LATCH_PRE: u32 = 127; const DATA_LATCH_LAT: u32 = 1;
const DATA_LATCH_WORDS: usize = 32;

// ── Frame layout — fixed order VSYNC → PRE_ACT → WR_CFG → DATA ──────
// This ordering is what reliably bootstraps the DP3364S SRAM write
// pointer from cold boot; variants with VSYNC elsewhere or an extra
// "Enable All Output" command were tried and failed to sync.
const VSYNC_OFFSET:   usize = 0;
const PRE_ACT_OFFSET: usize = VSYNC_OFFSET + 1 + VSYNC_WORDS;     // 2
const WR_CFG_OFFSET:  usize = PRE_ACT_OFFSET + 1 + PRE_ACT_WORDS; // 7
const HEADER_WORDS:   usize = WR_CFG_OFFSET + 1 + WR_CFG_WORDS;   // 40

const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS;                // 33
// Fixed by chip geometry: each DP3364S chip has 16 column outputs,
// and each DATA_LATCH command sends one column-per-chip. Over 16
// latches we cover all 128 panel columns (8 chips × 16 cols). Not
// tunable — `pack_pixels` indexes pixel columns via `15 - channel`
// and assumes the result is in 0..=15.
const LATCHES_PER_LINE:  usize = 16;
const LATCHES_PER_FRAME: usize = SCAN_LINES * LATCHES_PER_LINE;       // 512

const DATA_OFFSET: usize = HEADER_WORDS;                              // 40
const DATA_END:    usize = DATA_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE; // 16 936

// Frame buffer = header section + all DATA_LATCHes. Scan words live
// in `SCAN_BUF` (below), read by the ring-mode DMA; no per-frame scan
// words are needed in the frame buffer.
const FRAME_WORDS: usize = DATA_END; // 16 936

// ── Double-buffered frame data ───────────────────────────────────────
// Core 0 DMAs out of one buffer; core 1 packs into the other.
static mut FRAME_BUF_A: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];
static mut FRAME_BUF_B: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

// ── Scan buffer for ring-mode DMA ───────────────────────────────────
// 32 scan words (one per scan-line). Populated once at boot by
// `build_scan_buf`. The ring-mode DMA wraps this region every 32 words
// for the whole scan phase. 128-byte aligned so the DMA RING_SIZE=7
// boundary is natural.
#[repr(C, align(128))]
struct ScanBufAligned([u32; SCAN_LINES]);
static mut SCAN_BUF: ScanBufAligned = ScanBufAligned([0u32; SCAN_LINES]);

// ── Core 1 stack ─────────────────────────────────────────────────────
static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

// ── Scan parameters ──────────────────────────────────────────────────
const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;
// OE pulse width in PIO cycles. scan_line 0 needs the wider W12 pulse
// for SRAM-write-pointer alignment at boot; all other scan_lines use
// W4. Attempting uniform W12 for every scan_line breaks the
// upper/lower-half sync on the DP3364S.
const OE_CLK_W4: u32 = 4;
const OE_CLK_W12: u32 = 12;

// ── Pixel type and framebuffer ───────────────────────────────────────

#[derive(Copy, Clone)]
struct Rgb { r: u8, g: u8, b: u8 }
impl Rgb {
    const fn new(r: u8, g: u8, b: u8) -> Self { Self { r, g, b } }
    const BLACK: Rgb = Rgb::new(0, 0, 0);
}

static mut PIXELS: [[Rgb; 128]; 64] = [[Rgb::BLACK; 128]; 64];

// ── Gamma LUT: 8-bit -> 14-bit greyscale ─────────────────────────────
//
// Quadratic weighting. Linear and cubic alternatives were measured
// against a gradient test pattern; neither improved dim-end step
// visibility because gamma choice only redistributes the chip's 14-
// bit PWM range, it doesn't add resolution. Smoother dim gradients
// would need dithering in `pack_pixels` (spatial Bayer or temporal
// frame-over-frame), which is a pack-algorithm change, not a gamma
// one.
static GAMMA14: [u16; 256] = {
    let mut lut = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        lut[i] = ((i as u32 * i as u32 * 0x3FFF) / (255 * 255)) as u16;
        i += 1;
    }
    lut
};

// ── HSV helper ───────────────────────────────────────────────────────

#[allow(dead_code)]
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

// ── Data header ─────────────────────────────────────────────────────
// The first 32-bit word of every command tells the PIO program how
// many pre-latch + latch clocks to produce. Loaded by `data_cmd` via
// `OUT X, 15; OUT Y, 16; OUT NULL, 1`.
//   bits  0..14 : n_pre - 1  (up to 32767)
//   bits 15..30 : n_lat - 1  (up to 65535)
//   bit      31 : pad

const fn data_header(n_pre: u32, n_lat: u32) -> u32 {
    (n_pre - 1) | ((n_lat - 1) << 15)
}

// ── Scan word ───────────────────────────────────────────────────────
// One 32-bit word per scan-line, read by `scan_entry`:
//   bits  0..6  : display  count - 1  (brightness-modulated on-time)
//   bits  7..12 : 0x3F                 (RGB pins driven HIGH in scan)
//   bits 13..17 : scan address (0..31)
//   bits 18..22 : setup    count - 1   (idle between display and OE)
//   bits 23..26 : oe       count - 1   (OE pulse width)
//   bits 27..31 : pad

fn scan_word(display: u32, addr: u32, setup: u32, oe: u32) -> u32 {
    (display - 1)              // bits 0-6
    | (0x3F << 7)              // bits 7-12  RGB high
    | (addr << 13)             // bits 13-17 addr pins 6-10
    | ((setup - 1) << 18)      // bits 18-22
    | ((oe - 1) << 23)         // bits 23-26
}

/// Scan word populating [`SCAN_BUF`]. Gives scan-line 0 a wider OE
/// pulse (W12) and every other scan-line the shorter W4 pulse.
/// The W12 on line 0 is what aligns the 8 driver chips' internal
/// SRAM write pointers during the ~1 s boot sync phase. Applying W12
/// to every scan_line breaks the upper/lower-half sync, so the two
/// widths must stay split.
fn scan_word_for_sync(scan_line: usize) -> u32 {
    let oe = if scan_line == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
    scan_word(DISPLAY_CLK, scan_line as u32, SETUP_CLK, oe)
}

// ── Offset helpers ──────────────────────────────────────────────────

const fn latch_header_offset(scan_line: usize, channel: usize) -> usize {
    let latch_idx = scan_line * LATCHES_PER_LINE + channel;
    DATA_OFFSET + latch_idx * DATA_LATCH_STRIDE
}

// ── Frame-buffer packing ─────────────────────────────────────────────

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

/// Populate the fixed parts of a frame buffer: per-command headers
/// (VSYNC, PRE_ACT, WR_CFG) and the 512 DATA_LATCH headers. The
/// DATA_LATCH payloads are written by [`pack_pixels`] at runtime,
/// and the WR_CFG payload is updated per-iteration by
/// [`update_wr_cfg`] during the boot flush.
fn init_frame_headers(buf: &mut [u32; FRAME_WORDS]) {
    buf[VSYNC_OFFSET] = data_header(VSYNC_PRE, VSYNC_LAT);
    buf[VSYNC_OFFSET + 1] = 0;

    buf[PRE_ACT_OFFSET] = data_header(PRE_ACT_PRE, PRE_ACT_LAT);
    for i in 0..PRE_ACT_WORDS {
        buf[PRE_ACT_OFFSET + 1 + i] = 0;
    }

    buf[WR_CFG_OFFSET] = data_header(WR_CFG_PRE, WR_CFG_LAT);

    for scan_line in 0..SCAN_LINES {
        for channel in 0..LATCHES_PER_LINE {
            buf[latch_header_offset(scan_line, channel)] =
                data_header(DATA_LATCH_PRE, DATA_LATCH_LAT);
        }
    }
}

/// Pack-time compensation for the 4-scan-line hardware shift under
/// SYNC_MODE bit 7 = 1 (e.g. reg 0x0C88). Leave at 0 when bit 7 is
/// clear (0x0C08 / 0x0C48) — the hardware then displays scan-line N
/// at row N with no offset.
const PACK_SHIFT: isize = 0;

/// Nibble-scatter lookup used by [`pack_pixels`].
///
/// For a 4-bit input `x = abcd` (MSB `a` = bit3 of nibble), returns
/// `(a << 0) | (b << 8) | (c << 16) | (d << 24)` — i.e. scatters the
/// nibble's four bits to the bit-0 position of each of the u32's
/// four bytes, in the byte order the DATA_LATCH word expects (the
/// MSB of the nibble = earliest-clocked pixel = byte 0).
#[rustfmt::skip]
const SCATTER: [u32; 16] = [
    0x00000000, 0x01000000, 0x00010000, 0x01010000,
    0x00000100, 0x01000100, 0x00010100, 0x01010100,
    0x00000001, 0x01000001, 0x00010001, 0x01010001,
    0x00000101, 0x01000101, 0x00010101, 0x01010101,
];

/// Pack the PIXELS framebuffer into the given frame buffer's
/// DATA_LATCH regions.
///
/// Structure:
/// 1. Outer loop over (scan_line, channel).
/// 2. Inner loop is chip-major (not word-major). For each of the 8
///    chips, load its 6 gamma-expanded u16 colour values once, then
///    produce that chip's 4 DATA_LATCH output words directly using
///    the `SCATTER` LUT.
/// 3. Each output word is composed from six nibble-scatters ORed
///    together — no per-pixel bit-by-bit extraction loop.
///
/// Measured ~4.9 ms per frame on a 64×128 panel at 150 MHz.
fn pack_pixels(buf: &mut [u32; FRAME_WORDS]) {
    let pixels = unsafe { &*core::ptr::addr_of!(PIXELS) };

    for scan_line in 0..SCAN_LINES {
        let source_line =
            (scan_line as isize + PACK_SHIFT).rem_euclid(SCAN_LINES as isize) as usize;
        let upper_row = source_line;
        let lower_row = source_line + 32;

        for channel in 0..LATCHES_PER_LINE {
            let base = latch_header_offset(scan_line, channel) + 1;
            let output = 15 - channel;

            // chip 7 → words 0..3, chip 6 → 4..7, … chip 0 → 28..31.
            for chip in 0..8usize {
                let col = chip * 16 + output;
                let u = pixels[upper_row][col];
                let l = pixels[lower_row][col];
                let ur = GAMMA14[u.r as usize] as u32;
                let ug = GAMMA14[u.g as usize] as u32;
                let ub = GAMMA14[u.b as usize] as u32;
                let lr = GAMMA14[l.r as usize] as u32;
                let lg = GAMMA14[l.g as usize] as u32;
                let lb = GAMMA14[l.b as usize] as u32;

                let base_word = base + (7 - chip) * 4;

                // Four words per chip, one nibble of the 16-bit gamma
                // value each. Word 0 = bits 15..12 (earliest-clocked
                // bit-planes); word 3 = bits 3..0.
                for w_offset in 0..4usize {
                    let shift = (3 - w_offset) * 4;
                    let word =
                          (SCATTER[((ur >> shift) & 0xF) as usize]     )
                        | (SCATTER[((ug >> shift) & 0xF) as usize] << 1)
                        | (SCATTER[((ub >> shift) & 0xF) as usize] << 2)
                        | (SCATTER[((lr >> shift) & 0xF) as usize] << 3)
                        | (SCATTER[((lg >> shift) & 0xF) as usize] << 4)
                        | (SCATTER[((lb >> shift) & 0xF) as usize] << 5);
                    buf[base_word + w_offset] = word;
                }
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

/// Write `PIXELS` from whatever content source this build targets.
/// Called by core 1 before every pack. Swap in a different content
/// source here (still image, image buffer, text rendering, etc.) —
/// core 1 has plenty of budget for non-trivial work.
fn fill_pixels(offset: u8) {
    fill_pixels_rainbow(offset);
}

/// Diagnostic gradient pattern used during gamma / bit-depth
/// investigations. Four 16-row horizontal bands — R, G, B, W — each
/// stepping 0 → 255 across the 128 columns. Swap into `fill_pixels`
/// to use.
#[allow(dead_code)]
fn fill_pixels_gradient() {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        let band = row / 16;
        for col in 0..128usize {
            let v = (col * 255 / 127) as u8;
            pixels[row][col] = match band {
                0 => Rgb::new(v, 0, 0),
                1 => Rgb::new(0, v, 0),
                2 => Rgb::new(0, 0, v),
                _ => Rgb::new(v, v, v),
            };
        }
    }
}

/// Animated diagonal rainbow: `hue = (row + col + offset) mod 256`.
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
    // DWT cycle counter is per-core; enable core-1's independently so
    // we can time the pack locally.
    enable_cycle_counter();

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

        // Time just the pack itself (fill_pixels is fast; DMA-side
        // content-rate bottleneck is pack, not fill).
        let t0 = now_cycles();
        pack_pixels(buf);
        let elapsed = now_cycles().wrapping_sub(t0);

        PACK_CY_TOTAL.fetch_add(elapsed, Ordering::Relaxed);
        PACK_COUNT.fetch_add(1, Ordering::Relaxed);
        // Non-atomic max-update: OK because there's only one writer.
        let cur_max = PACK_CY_MAX.load(Ordering::Relaxed);
        if elapsed > cur_max {
            PACK_CY_MAX.store(elapsed, Ordering::Relaxed);
        }

        fifo_write(1);
    }
}

// ── PIO helpers ─────────────────────────────────────────────────────

const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_CTRL_SET: u32 = PIO0_BASE + 0x2000;
const PIO0_CTRL_CLR: u32 = PIO0_BASE + 0x3000;
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_CLKDIV: u32 = PIO0_BASE + 0x0C8;
const SM0_EXECCTRL: u32 = PIO0_BASE + 0x0CC;
const SM0_INSTR: u32 = PIO0_BASE + 0x0D8;
const SM0_PINCTRL: u32 = PIO0_BASE + 0x0DC;
const SM0_EN: u32 = 1 << 0;
const SM0_TXSTALL: u32 = 1 << 24;

fn enable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_SET as *mut u32).write_volatile(mask) };
}
fn disable_sm(mask: u32) {
    unsafe { (PIO0_CTRL_CLR as *mut u32).write_volatile(mask) };
}

/// Write the per-SM clock divisor. Integer-only to avoid PIO's
/// fractional-clkdiv jitter. Data phase uses 2 (PIO at 75 MHz →
/// DP3364S CLK at 25 MHz, at datasheet maximum). Scan phase uses 3
/// (PIO at 50 MHz → effective scan rate ~16.7 MHz).
fn set_clkdiv(int_div: u32) {
    unsafe { (SM0_CLKDIV as *mut u32).write_volatile(int_div << 16) };
}

/// Busy-wait until the SM's TX FIFO has drained and the SM has
/// stalled on empty-FIFO. Very short (microseconds) so we don't
/// bother with a `wfi` path. Call between DMAs to ensure the pipeline
/// has fully emptied before reconfiguring the SM.
fn wait_txstall(sm_stall_bit: u32) {
    unsafe {
        (PIO0_FDEBUG as *mut u32).write_volatile(sm_stall_bit);
        while (PIO0_FDEBUG as *const u32).read_volatile() & sm_stall_bit == 0 {
            cortex_m::asm::nop();
        }
    }
}

// ── DMA ring-mode scan (ch1, raw register access) ───────────────────
//
// Accessed via raw registers rather than the HAL because the HAL's
// single_buffer abstraction doesn't expose ring-mode trivially, and
// we want to control TRANS_COUNT_TRIG's top 4 bits for endless vs
// bounded operation.
const DMA_BASE: u32 = 0x5000_0000;
const CH1_READ_ADDR:            *mut u32 = (DMA_BASE + 0x040) as *mut u32;
const CH1_WRITE_ADDR:           *mut u32 = (DMA_BASE + 0x044) as *mut u32;
const CH1_AL1_CTRL:             *mut u32 = (DMA_BASE + 0x050) as *mut u32;
const CH1_AL1_TRANS_COUNT_TRIG: *mut u32 = (DMA_BASE + 0x05C) as *mut u32;
const CHAN_ABORT:               *mut u32 = (DMA_BASE + 0x464) as *mut u32;
const PIO0_TXF0: u32 = PIO0_BASE + 0x010;

/// CH1 control register value, used by both `start_ring_scan` and
/// `start_ring_scan_bounded`. Bit fields:
///   bit 0   : EN
///   bits 3:2: DATA_SIZE = 2 (32-bit word)
///   bit 4   : INCR_READ = 1 (advance read address)
///   bits 10:8: TREQ_SEL = 7 (selects DREQ for PIO0 TX0, for paced DMA)
///   bit 13  : RING = 1 (wrap read address every 2^5 = 32 words)
const CH1_CTRL_VAL: u32 =
      (1 << 0)
    | (2 << 2)
    | (1 << 4)
    | (7 << 8)
    | (1 << 13);

/// Start an endless ring-mode DMA on ch1 reading SCAN_BUF. Used in the
/// boot sync phase — the caller aborts via `stop_ring_scan` when done.
fn start_ring_scan() {
    let scan_addr = core::ptr::addr_of!(SCAN_BUF) as u32;
    unsafe {
        CH1_AL1_CTRL.write_volatile(CH1_CTRL_VAL);
        CH1_READ_ADDR.write_volatile(scan_addr);
        CH1_WRITE_ADDR.write_volatile(PIO0_TXF0);
        // Bits 31:28 = 0xF → endless mode (ignore count, run forever).
        CH1_AL1_TRANS_COUNT_TRIG.write_volatile((15 << 28) | 32);
    }
}

/// Start a bounded ring-mode DMA on ch1: `n_cycles × 32` transfers
/// reading SCAN_BUF with the read address wrapping every 32 words,
/// then the channel stops and raises DMA_IRQ_0 (requires INTE0 bit 1
/// set). Used in the main loop.
fn start_ring_scan_bounded(n_cycles: u32) {
    let scan_addr = core::ptr::addr_of!(SCAN_BUF) as u32;
    let count = n_cycles * SCAN_LINES as u32;
    unsafe {
        CH1_AL1_CTRL.write_volatile(CH1_CTRL_VAL);
        CH1_READ_ADDR.write_volatile(scan_addr);
        CH1_WRITE_ADDR.write_volatile(PIO0_TXF0);
        // Bits 31:28 = 0 → NORMAL mode (stop after `count` transfers).
        // Bits 27:0  = count.
        CH1_AL1_TRANS_COUNT_TRIG.write_volatile(count);
    }
}

/// Abort the currently-running ring-mode DMA on ch1 and wait for the
/// abort to complete.
fn stop_ring_scan() {
    unsafe {
        CHAN_ABORT.write_volatile(1 << 1);
        while CHAN_ABORT.read_volatile() & (1 << 1) != 0 {
            cortex_m::asm::nop();
        }
    }
}

/// Populate SCAN_BUF once at boot with the per-scan-line scan words.
fn build_scan_buf() {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SCAN_BUF) };
    for row in 0..SCAN_LINES {
        buf.0[row] = scan_word_for_sync(row);
    }
}

// ── Timing instrumentation (DWT cycle counter) ──────────────────────
const SYS_CLK_MHZ: u32 = 150;
const DEMCR: u32      = 0xE000_EDFC;
const DWT_CTRL: u32   = 0xE000_1000;
const DWT_CYCCNT: u32 = 0xE000_1004;

fn enable_cycle_counter() {
    unsafe {
        let demcr = (DEMCR as *const u32).read_volatile();
        (DEMCR as *mut u32).write_volatile(demcr | (1 << 24));
        (DWT_CYCCNT as *mut u32).write_volatile(0);
        let ctrl = (DWT_CTRL as *const u32).read_volatile();
        (DWT_CTRL as *mut u32).write_volatile(ctrl | 1);
    }
}

fn now_cycles() -> u32 {
    unsafe { (DWT_CYCCNT as *const u32).read_volatile() }
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

    // Pull-down on LAT (GPIO 12) for safety during startup
    const PADS_BANK0: u32 = 0x4003_8000;
    const GPIO12_PAD: u32 = PADS_BANK0 + 0x04 + 12 * 4;
    unsafe {
        let val = (GPIO12_PAD as *const u32).read_volatile();
        (GPIO12_PAD as *mut u32).write_volatile(val | (1 << 2));
    }

    // ── 3. Install the PIO program ───────────────────────────────────
    //
    // One PIO program with two entry points:
    //   `data_cmd`   (PC  0) — handles VSYNC/PRE_ACT/WR_CFG/DATA_LATCH
    //                           commands; CLK toggles via side-set.
    //   `scan_entry` (PC 11) — runs one scan line (display / setup /
    //                           OE pulse with timed delays).
    //
    // The CPU switches phase by writing SM0_INSTR (force JMP to the
    // other entry point), SM0_EXECCTRL (point WRAP_TOP/BOTTOM at the
    // phase's own loop), and SM0_PINCTRL (OUT_COUNT swap — see the
    // echo note in the file header).
    //
    // Side-set (1 pin, pin 11 = CLK): bit 0 = CLK high.
    // SET pin group (base 12, count 2 = LAT + OE):
    //   SET PINS 0b00 : LAT=0 OE=0
    //   SET PINS 0b01 : LAT=1 OE=0   (used in data phase)
    //   SET PINS 0b10 : LAT=0 OE=1   (used in scan phase)
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let ss = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::new_with_side_set(ss);

    let mut data_cmd         = a.label();
    let mut scan_entry       = a.label();
    let mut display_lp       = a.label();
    let mut setup_lp         = a.label();
    let mut oe_lp            = a.label();
    let mut pre_loop         = a.label();
    let mut lat_loop         = a.label();
    let mut data_wrap_source = a.label();
    let mut scan_wrap_source = a.label();

    // ─── DATA path (PC 0, wrap_target) ───────────────────────────────
    a.bind(&mut data_cmd);
    a.out_with_side_set(pio::OutDestination::X, 15, 0);    // n_pre-1
    a.out_with_side_set(pio::OutDestination::Y, 16, 0);    // n_lat-1
    a.out_with_side_set(pio::OutDestination::NULL, 1, 0);  // pad bit 31

    a.bind(&mut pre_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 1);  // CLK rise
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut pre_loop, 0);

    // Raise LAT for lat_loop
    a.set_with_side_set(pio::SetDestination::PINS, 0b01, 0);

    a.bind(&mut lat_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 1);  // CLK rise
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut lat_loop, 0);

    // Drop LAT; data wrap fires AFTER this instruction (back to data_cmd)
    a.set_with_side_set(pio::SetDestination::PINS, 0b00, 0);
    a.bind(&mut data_wrap_source);

    // ─── SCAN path (reached only via forced JMP scan_entry) ──────────
    a.bind(&mut scan_entry);
    a.out_with_side_set(pio::OutDestination::Y, 7, 0);
    a.bind(&mut display_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut display_lp, 1);

    a.out_with_side_set(pio::OutDestination::PINS, 11, 0);
    a.out_with_side_set(pio::OutDestination::Y, 5, 0);

    a.bind(&mut setup_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut setup_lp, 1);

    a.out_with_side_set(pio::OutDestination::Y, 4, 0);
    a.set_with_side_set(pio::SetDestination::PINS, 0b10, 0);  // OE on

    a.bind(&mut oe_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut oe_lp, 1);

    a.set_with_side_set(pio::SetDestination::PINS, 0b00, 0);  // OE off
    a.out_with_side_set(pio::OutDestination::NULL, 5, 0);     // drain
    // scan_wrap_source: scan-phase wrap fires AFTER this instruction
    // back to scan_entry. No explicit JMP — relies on PIO wrap.
    a.bind(&mut scan_wrap_source);

    // Assemble with data's wrap. Scan's wrap is applied at runtime
    // by overwriting EXECCTRL WRAP_TOP/BOTTOM before scan phase.
    let mut prog = a.assemble_with_wrap(data_wrap_source, data_cmd);
    prog = prog.set_origin(Some(0));
    let installed = pio0.install(&prog).unwrap();

    // Label positions in the program (must stay in sync with assembly).
    // NOTE: pio-0.2.1's assemble_with_wrap does `source -= 1` internally
    // (pio-0.2.1/src/lib.rs:574) because `bind` labels the NEXT
    // instruction slot. So the hardware WRAP_TOP is the label value - 1.
    // We store WRAP_TOP directly here (what EXECCTRL bits 16:12 want).
    //   data_cmd          = 0   (entry; WRAP_BOTTOM for data)
    //   data_wrap_source  = 11  (label), WRAP_TOP = 10 (last data instr)
    //   scan_entry        = 11  (entry; WRAP_BOTTOM for scan)
    //   scan_wrap_source  = 27  (label), WRAP_TOP = 26 (last scan instr)
    const DATA_CMD_LOCAL: u32 = 0;
    const DATA_WRAP_TOP_LOCAL: u32 = 10;
    const SCAN_ENTRY_LOCAL: u32 = 11;
    const SCAN_WRAP_TOP_LOCAL: u32 = 26;

    let prog_offset = installed.offset() as u32;
    let data_cmd_pc = prog_offset + DATA_CMD_LOCAL;
    let scan_entry_pc = prog_offset + SCAN_ENTRY_LOCAL;
    defmt::info!(
        "PIO layout: data_cmd={=u32} scan_entry={=u32}  data_wrap_top={=u32}  scan_wrap_top={=u32}",
        data_cmd_pc, scan_entry_pc,
        prog_offset + DATA_WRAP_TOP_LOCAL,
        prog_offset + SCAN_WRAP_TOP_LOCAL,
    );

    let (mut sm, _, tx) =
        hal::pio::PIOBuilder::from_installed_program(installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 11)
            .set_pins(12, 2)  // SET group covers LAT (12) + OE (13)
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

    // ── 3b. Compute EXECCTRL swap values ─────────────────────────────
    // Read the EXECCTRL configured by PIOBuilder (data's wrap from
    // assemble_with_wrap). Scan's EXECCTRL is the same except for
    // WRAP_TOP/WRAP_BOTTOM.
    let base_execctrl = unsafe { (SM0_EXECCTRL as *const u32).read_volatile() };
    const EXECCTRL_WRAP_MASK: u32 = (0x1F << 12) | (0x1F << 7);
    let execctrl_no_wrap = base_execctrl & !EXECCTRL_WRAP_MASK;
    let data_execctrl = execctrl_no_wrap
        | (((prog_offset + DATA_WRAP_TOP_LOCAL) & 0x1F) << 12)
        | (((prog_offset + DATA_CMD_LOCAL) & 0x1F) << 7);
    let scan_execctrl = execctrl_no_wrap
        | (((prog_offset + SCAN_WRAP_TOP_LOCAL) & 0x1F) << 12)
        | (((prog_offset + SCAN_ENTRY_LOCAL) & 0x1F) << 7);

    // Sanity check: data_execctrl we just computed should equal
    // base_execctrl (what PIOBuilder wrote from assemble_with_wrap).
    // If these diverge, the WRAP_TOP constant is off.
    defmt::info!(
        "EXECCTRL: base=0x{=u32:08x} data=0x{=u32:08x} scan=0x{=u32:08x}",
        base_execctrl, data_execctrl, scan_execctrl,
    );

    // ── 3c. Compute PINCTRL swap values ──────────────────────────────
    // PINCTRL bit layout (same RP2040 & RP2350):
    //   [31:29] SIDESET_COUNT   [28:26] SET_COUNT
    //   [25:20] OUT_COUNT       [19:15] IN_BASE
    //   [14:10] SIDESET_BASE    [9:5]   SET_BASE
    //   [4:0]   OUT_BASE
    //
    // Only OUT_COUNT differs between phases; everything else matches
    // what PIOBuilder set (SET 12/2, SIDESET 11/1, OUT_BASE=0).
    //
    //   Data phase: OUT_COUNT=6  — `out PINS, 6` writes only pins 0..5
    //                              (RGB). Pins 6..10 (ADDR) retain
    //                              their last-driven value from the
    //                              previous scan — KEY to echo-free
    //                              display, see header.
    //   Scan phase: OUT_COUNT=11 — `out PINS, 11` writes RGB + ADDR.
    let data_pinctrl: u32 =
        (1 << 29) | (2 << 26) | (6 << 20) | (0 << 15)
        | (11 << 10) | (12 << 5) | 0;
    let scan_pinctrl: u32 =
        (1 << 29) | (2 << 26) | (11 << 20) | (0 << 15)
        | (11 << 10) | (12 << 5) | 0;

    let base_pinctrl = unsafe { (SM0_PINCTRL as *const u32).read_volatile() };
    defmt::info!(
        "PINCTRL: base=0x{=u32:08x} data=0x{=u32:08x} scan=0x{=u32:08x}",
        base_pinctrl, data_pinctrl, scan_pinctrl,
    );

    // Phase-swap closures — write PINCTRL + EXECCTRL + force JMP.
    // PINCTRL first so the subsequent forced JMP executes under the
    // new pin config; INSTR last because the JMP consumes it.
    let swap_to_data = || unsafe {
        (SM0_PINCTRL  as *mut u32).write_volatile(data_pinctrl);
        (SM0_EXECCTRL as *mut u32).write_volatile(data_execctrl);
        (SM0_INSTR    as *mut u32).write_volatile(data_cmd_pc & 0x1F);
    };
    let swap_to_scan = || unsafe {
        (SM0_PINCTRL  as *mut u32).write_volatile(scan_pinctrl);
        (SM0_EXECCTRL as *mut u32).write_volatile(scan_execctrl);
        (SM0_INSTR    as *mut u32).write_volatile(scan_entry_pc & 0x1F);
    };

    // ── 4. DMA — ch0 (data) + ch1 (ring scan), both IRQ-driven ──────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut ch = dma.ch0;

    // Enable IRQ0 output from ch0 (HAL helper; writes INTE0 bit 0).
    ch.enable_irq0();
    // Enable IRQ0 output from ch1 (raw write — we drive ch1 via raw
    // registers for ring-mode, so INTE0 bit 1 needs explicit set).
    unsafe {
        let cur = DMA_INTE0_RAW.read_volatile();
        DMA_INTE0_RAW.write_volatile(cur | (1 << 1));
    }
    // Unmask DMA_IRQ_0 in the NVIC so the handler runs and `wfi` wakes.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(hal::pac::Interrupt::DMA_IRQ_0);
    }

    defmt::info!("HUB75 driver starting");

    // ── 5. Initialise static frame-buffer structure + scan buffer ───
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) });
    build_scan_buf();

    // ── 6. Startup flush — write each configuration register from
    //       CONFIG_REGS through the chip's WR_CFG command. First frame
    //       writes GROUP_SET (sets 128-wide SRAM addressing); later
    //       frames cycle through the rest. Extra iterations past the
    //       13 registers ensure the chip SRAM is fully overwritten
    //       with zeros before real pixel data arrives.
    const FLUSH_FRAMES: u32 = 26;
    for flush in 0..FLUSH_FRAMES {
        if (flush as usize) < CONFIG_REGS.len() {
            update_wr_cfg(
                unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) },
                flush as usize,
            );
        }

        // Data-only DMA (header + 512 DATA_LATCHes) at clkdiv=2.
        set_clkdiv(2);
        swap_to_data();
        let data_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(FRAME_BUF_A) as *const u32,
                DATA_END,
            )
        };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);

        // One scan pass (32 scan words from SCAN_BUF) at clkdiv=3.
        set_clkdiv(3);
        swap_to_scan();
        let scan_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(SCAN_BUF) as *const u32,
                SCAN_LINES,
            )
        };
        let scan_cfg = single_buffer::Config::new(ch, scan_slice, tx);
        let scan_xfer = scan_cfg.start();
        let (new_ch2, _, new_tx2) = scan_xfer.wait();
        ch = new_ch2;
        tx = new_tx2;
        wait_txstall(SM0_TXSTALL);
    }

    defmt::info!("flush complete");

    // ── 6b. Sync phase — ~1 s of ring-scan with interleaved one-shot
    //        data DMAs. Empirically this is what aligns the 8 driver
    //        chips' internal SRAM write pointers to a common offset;
    //        once synced, state persists for the rest of boot.
    // Keep ch1's HAL handle bound so the HAL doesn't tear it down —
    // we drive ch1 directly via raw registers for ring-mode.
    let _ch1 = dma.ch1;

    set_clkdiv(3);
    swap_to_scan();
    start_ring_scan();

    const SYNC_FRAMES: u32 = 60; // ~1 s at 60 Hz
    for _ in 0..SYNC_FRAMES {
        // Crude delay to let ring-mode run for ~one core-1 pack period
        cortex_m::asm::delay(150_000 * 10); // ~10 ms at 150 MHz

        stop_ring_scan();
        wait_txstall(SM0_TXSTALL);

        set_clkdiv(2);
        swap_to_data();
        let data_slice = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(FRAME_BUF_A) as *const u32,
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
        swap_to_scan();

        start_ring_scan();
    }

    stop_ring_scan();
    wait_txstall(SM0_TXSTALL);
    defmt::info!("sync phase complete");

    // Seed FRAME_BUF_B's WR_CFG payload so both buffers emit identical
    // WR_CFG content in the main loop (the flush only populated
    // FRAME_BUF_A). Last register written is the final CONFIG_REGS
    // entry (0x0D12).
    update_wr_cfg(
        unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) },
        CONFIG_REGS.len() - 1,
    );

    // ── 7. Pack first real frame and spawn core 1 ───────────────────
    fill_pixels(0);
    pack_pixels(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });

    let mut mc = hal::multicore::Multicore::new(
        &mut pac.PSM, &mut pac.PPB, &mut sio_hal.fifo,
    );
    let cores = mc.cores();
    cores[1].spawn(unsafe { CORE1_STACK.take().unwrap() }, core1_task).unwrap();

    let mut active_buf: u8 = 0;
    let mut core1_buf: u8 = 1;
    let mut offset: u8 = 0;

    // Kick off core 1 with the first pack job
    offset = offset.wrapping_add(2);
    fifo_write(offset as u32 | ((core1_buf as u32) << 8));

    // Clear any IRQ-done bits left over from the flush/sync phases.
    // (Both phases trigger data and scan DMAs; those completions have
    // been flagged in DMA_DONE and would make the first `wait_dma`
    // return spuriously.)
    DMA_DONE.store(0, Ordering::SeqCst);
    unsafe { DMA_INTS0_RAW.write_volatile(0xFFFF_FFFF); }

    // ── Baseline instrumentation ────────────────────────────────────
    // Per-phase cycle accumulators, reset every REPORT_WINDOW frames.
    // Cycle units: 150 MHz core clock → 1 cy = 6.67 ns.
    let mut data_cy_total:  u64 = 0;
    let mut scan_cy_total:  u64 = 0;   // time spent inside scan DMAs
    let mut wfi_cy_total:   u64 = 0;   // time core 0 spent in `wfi`
    let mut frame_cy_total: u64 = 0;   // whole main-loop iter
    let mut display_frames: u32 = 0;   // every main-loop iter = 1
    let mut content_frames: u32 = 0;   // only when core 1 delivered new pack
    let mut core1_waits:    u32 = 0;   // iters where core 1 wasn't ready
    let mut last_report_cy: u32 = now_cycles();
    const REPORT_WINDOW_FRAMES: u32 = 256;

    // Scan-cycle count per frame — the only runtime tunable that
    // trades off brightness against refresh rate. Kept as a binding
    // (not `const`) so the final API can expose it as a parameter.
    //
    //   20 → ~128 Hz / ~34 % dark (refresh-rate sweet spot; default)
    //   30 →  ~96 Hz / ~27 % dark
    //   50 →  ~64 Hz / ~17 % dark (brightness sweet spot)
    //
    // Above ~57 cycles the refresh drops into perceptible flicker.
    // Below ~20 cycles brightness loss is noticeable.
    let current_scan_cycles: u32 = 20;

    // Main loop — one iteration per display frame.
    //
    //   1. Data phase: swap_to_data + stream FRAME_BUF[0..DATA_END]
    //      through the PIO via DMA channel 0. ~2.66 ms.
    //   2. Scan phase: swap_to_scan + ring-mode DMA channel 1 replays
    //      SCAN_BUF `current_scan_cycles` times. 5-13 ms depending on
    //      the cycle count.
    //   3. Buffer-swap check: poll SIO FIFO for core-1 pack-done
    //      signal; swap active buffer if new pack arrived.
    //   4. Every REPORT_WINDOW_FRAMES frames, log metrics.
    //
    // Core 0 is in `wfi` inside wait_dma() for ~99 % of each frame.
    loop {
        let t_frame0 = now_cycles();

        let buf_ptr = if active_buf == 0 {
            core::ptr::addr_of!(FRAME_BUF_A) as *const u32
        } else {
            core::ptr::addr_of!(FRAME_BUF_B) as *const u32
        };

        // Data phase — one-shot DMA of [0 .. DATA_END] from the
        // active frame buffer through the PIO at CLK = 25 MHz.
        let t_data0 = now_cycles();
        set_clkdiv(2);
        enable_sm(SM0_EN);
        swap_to_data();
        let data_slice = unsafe { core::slice::from_raw_parts(buf_ptr, DATA_END) };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        // `wait_dma` sleeps on `wfi`; the DMA_IRQ_0 handler wakes us.
        wfi_cy_total += wait_dma(1 << 0) as u64;
        // Transfer has completed already; `wait()` is now instant.
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
        disable_sm(SM0_EN);

        data_cy_total += now_cycles().wrapping_sub(t_data0) as u64;

        // Scan phase — one ring-mode DMA transfers
        // `current_scan_cycles × 32` words, wrapping SCAN_BUF every
        // 32 words. Core 0 `wfi`-sleeps for the whole ~5-13 ms phase.
        let t_scan0 = now_cycles();
        set_clkdiv(3);
        enable_sm(SM0_EN);
        swap_to_scan();
        start_ring_scan_bounded(current_scan_cycles);
        wfi_cy_total += wait_dma(1 << 1) as u64;
        wait_txstall(SM0_TXSTALL);
        disable_sm(SM0_EN);
        scan_cy_total += now_cycles().wrapping_sub(t_scan0) as u64;

        display_frames += 1;
        frame_cy_total += now_cycles().wrapping_sub(t_frame0) as u64;

        if fifo_try_read().is_some() {
            content_frames += 1;
            active_buf = core1_buf;
            core1_buf = 1 - active_buf;
            offset = offset.wrapping_add(2);
            fifo_write(offset as u32 | ((core1_buf as u32) << 8));
        } else {
            core1_waits += 1;
        }

        if display_frames >= REPORT_WINDOW_FRAMES {
            let now = now_cycles();
            let window_cy = now.wrapping_sub(last_report_cy) as u64;

            // Averages per display-frame (µs).
            let data_avg_us  = (data_cy_total  / (SYS_CLK_MHZ as u64 * display_frames as u64)) as u32;
            let scan_avg_us  = (scan_cy_total  / (SYS_CLK_MHZ as u64 * display_frames as u64)) as u32;
            let frame_avg_us = (frame_cy_total / (SYS_CLK_MHZ as u64 * display_frames as u64)) as u32;

            // Dark ratio = data_cy / (data_cy + scan_cy), in permille.
            let lit_dark = data_cy_total + scan_cy_total;
            let dark_permille = if lit_dark > 0 {
                (data_cy_total * 1000 / lit_dark) as u32
            } else { 0 };

            // Refresh rates.
            // Display-frame Hz × 100 (keep one decimal).
            let display_hz_x100 = if window_cy > 0 {
                ((display_frames as u64 * SYS_CLK_MHZ as u64 * 100_000_000u64) / window_cy) as u32
            } else { 0 };
            let content_hz_x100 = if window_cy > 0 {
                ((content_frames as u64 * SYS_CLK_MHZ as u64 * 100_000_000u64) / window_cy) as u32
            } else { 0 };
            // Per-line refresh = display_hz × current_scan_cycles.
            let line_hz = (display_hz_x100 / 100) * current_scan_cycles;

            // Core-0 idle fraction: how much of the main-loop time was
            // spent inside `wait_dma` (i.e. `wfi`). 1000 ‰ = core 0
            // asleep the entire frame; 0 ‰ = busy-waiting everywhere.
            let wfi_permille = if frame_cy_total > 0 {
                (wfi_cy_total * 1000 / frame_cy_total) as u32
            } else { 0 };

            // Pack stats from core 1 (reset after read).
            let pack_cy_total = PACK_CY_TOTAL.swap(0, Ordering::Relaxed);
            let pack_cy_max   = PACK_CY_MAX.swap(0, Ordering::Relaxed);
            let pack_count    = PACK_COUNT.swap(0, Ordering::Relaxed);
            let pack_avg_us = if pack_count > 0 {
                pack_cy_total / SYS_CLK_MHZ / pack_count
            } else { 0 };
            let pack_max_us = pack_cy_max / SYS_CLK_MHZ;
            // Core 1 busy fraction = avg_pack_time × content_rate
            // = (pack_cy_total / pack_count) × (pack_count / window_cy)
            // = pack_cy_total / window_cy. Permille.
            let core1_busy_permille = if window_cy > 0 {
                ((pack_cy_total as u64) * 1000 / window_cy) as u32
            } else { 0 };

            defmt::info!(
                "cycles={=u32}  frame {=u32}µs (data {=u32}µs scan {=u32}µs dark {=u32}‰)  display {=u32}.{=u32} Hz  content {=u32}.{=u32} Hz  line {=u32} Hz  pack avg={=u32}µs max={=u32}µs core1_busy={=u32}‰  core1_waits {=u32}/{=u32}  core0_wfi {=u32}‰",
                current_scan_cycles,
                frame_avg_us, data_avg_us, scan_avg_us, dark_permille,
                display_hz_x100 / 100, display_hz_x100 % 100,
                content_hz_x100 / 100, content_hz_x100 % 100,
                line_hz,
                pack_avg_us, pack_max_us, core1_busy_permille,
                core1_waits, display_frames,
                wfi_permille,
            );

            data_cy_total  = 0;
            scan_cy_total  = 0;
            wfi_cy_total   = 0;
            frame_cy_total = 0;
            display_frames = 0;
            content_frames = 0;
            core1_waits    = 0;
            last_report_cy = now;
        }
    }
}
