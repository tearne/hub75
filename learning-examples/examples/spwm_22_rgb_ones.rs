//! # DP3364S S-PWM — Step 22: Thread B experiment 4 (RGB driven to ONE)
//!
//! ## Purpose
//!
//! Discriminate between "driving RGB pins during scan causes echo"
//! (any value) vs "driving RGB pins to ZERO during scan causes echo"
//! (value-specific). Identical to spwm_21 in every way except the
//! scan-word bit pattern: RGB bits (7-12) are set to ONE instead of
//! ZERO. PIO program, pin base, OUT widths, word position layout —
//! all byte-identical to spwm_21.
//!
//! ## Outcomes
//!
//! - **Echo persists** → the *act of driving RGB pins during scan*
//!   is the mechanism, regardless of value. That localizes the issue
//!   to "any PIO-driven RGB activity during scan phase".
//! - **Echo vanishes** → specifically *driving RGB to zero* is the
//!   mechanism. Strongly suggests the chip treats a zero-data CLK
//!   pulse differently from a one-data CLK pulse — possibly some
//!   chip-internal pattern recognition or address-decode feature.
//! - **Display garbles** → RGB-one bits combined with CLK pulses
//!   get latched somewhere by the chip. Itself informative but
//!   makes echo reading harder.
//!
//! ## Change from spwm_21
//!
//! One line: in `build_scan_buf`, OR the RGB field (bits 7-12) with
//! `0x3F` so all six RGB bits are set.
//!
//! ```sh
//! cargo run --release --example spwm_22_rgb_ones
//! ```
//!
//! ---
//!
//! # Inherited spwm_21 header follows
//!
//! # DP3364S S-PWM — Step 21: Thread B experiment 3 (RGB drive during scan)
//!
//! ## Purpose
//!
//! Reverse-differential from spwm_17 (echo-free). One targeted change:
//! the scan PIO program now executes `OUT PINS, 11` instead of
//! `OUT PINS, 5`, so RGB pins 0-5 are driven to zero on every scan
//! word — matching spwm_12's echoing scan path. Everything else
//! (program swap, fixed-rate refresh, all other instructions)
//! unchanged.
//!
//! ## Hypothesis
//!
//! The chip interprets some aspect of RGB-pin behaviour during scan
//! as a state-carrying signal. In spwm_17 the scan program leaves
//! RGB alone (pins retain whatever state the data phase left them
//! in); in spwm_12 every scan word actively drives them to zero.
//! That's the single most pin-visible difference between the two
//! scan paths.
//!
//! ## Outcomes
//!
//! - **Echo returns** → mechanism is RGB-pin-drive-during-scan.
//!   Concrete, actionable: next step is to figure out whether the
//!   issue is the pin edge, the specific zero value, or the fact
//!   that the RGB drive happens at all during scan.
//! - **Echo stays away** → RGB drive isn't the mechanism. Move on
//!   to the next structural difference (type-bit dispatch prelude,
//!   or wrap-point addresses).
//!
//! ## Changes from spwm_17
//!
//! 1. Scan PIO program: `OUT PINS, 5` → `OUT PINS, 11`, and tail
//!    drain adjusted from `OUT NULL, 11` → `OUT NULL, 5` to keep
//!    the 32-bit autopull budget aligned.
//! 2. Scan PIOBuilder: `out_pins(6, 5)` → `out_pins(0, 11)`.
//! 3. Scan-word bit layout in `build_scan_buf` shifts the address
//!    field from bits 7-11 to bits 13-17, inserting 6 zero bits
//!    at 7-12 for the RGB pins.
//!
//! ```sh
//! cargo run --release --example spwm_21_rgb_drive
//! ```
//!
//! ---
//!
//! # Inherited spwm_17 header follows
//!
//! # DP3364S S-PWM — Step 17: combined echo-free + flicker-free
//!
//! Seventeenth in the SPWM progression. **Confirmed echo-free and
//! flicker-free on 2026-04-21** — the first architecture in this
//! project's progression that clears both failure modes. Combines:
//!
//!   - **spwm_15's program swap on a single SM** (confirmed echo-free
//!     — pipeline reset at each data↔scan phase boundary kills the
//!     scan_line-31 → scan_line-0 SRAM leak).
//!
//!   - **spwm_12's fixed-rate `POST_SCAN_CYCLES` loop** (confirmed
//!     flicker-free by `brightness_12` — data phase fires at ~64 Hz
//!     regardless of core-1 pack rate, above human flicker fusion).
//!
//! ## Why both are needed
//!
//! The Thread-A brightness probes (2026-04-21) measured:
//!
//! | Probe            | Data freq | Dark fraction | Flicker             |
//! |------------------|----------:|--------------:|:--------------------|
//! | brightness_14    |     39 Hz |         10.2% | visible             |
//! | brightness_15    |     37 Hz |          9.7% | visible             |
//! | brightness_12    |     64 Hz |         16.9% | **none perceived**  |
//!
//! Frequency dominates over duty cycle for perceived flicker. spwm_12's
//! fixed-rate refresh clears the ~60 Hz fusion threshold; spwm_14/15's
//! pack-rate-triggered refresh does not. **But spwm_12 echoes**, so it
//! was never shippable. spwm_17 takes the refresh loop shape from 12
//! and the pipeline-reset mechanism from 15.
//!
//! ## Architecture
//!
//! One SM (SM0), two installed PIO programs (data + scan). At each
//! phase boundary SM0's `EXECCTRL`/`PINCTRL` are rewritten and its PC
//! is forced to the new program's entry point via `SMx_INSTR`.
//!
//! Main loop per iteration:
//!
//! ```text
//!   swap_to_data()                        ← echo fix
//!   DMA data buffer  (~2.6 ms, dark)
//!   swap_to_scan()                        ← echo fix
//!   for _ in 0..POST_SCAN_CYCLES {        ← flicker fix
//!     DMA SCAN_BUF (32 words, ~253 µs, bright)
//!   }
//!   if core1 done: swap buffers, trigger next pack
//! ```
//!
//! Unlike spwm_15, the data phase fires *every* iteration, not only
//! when core 1 has a fresh frame. If core 1 is still packing, the
//! current frame is re-displayed — the SRAM in the chip already holds
//! valid content, so re-loading identical data still gives a fully
//! bright scan. This decouples display refresh from pack rate, which
//! is exactly what spwm_12 did.
//!
//! `POST_SCAN_CYCLES = 50` matches brightness_12's measured 64 Hz /
//! 17% dark.
//!
//! ## Measured timing (on panel, 2026-04-21)
//!
//! | Metric          | Predicted | Measured      |
//! |-----------------|----------:|--------------:|
//! | scan pass       |  ~258 µs  |      258 µs   |
//! | data phase      |  ~2.6 ms  |     2641 µs   |
//! | data freq       |   ~64 Hz  |       65 Hz   |
//! | dark            |    ~17 %  |        16.9 % |
//!
//! Rainbow pattern (`ECHO_TEST=false`): "nice solid smooth image",
//! no flicker.
//! E25 pattern (`ECHO_TEST=true`): no echo on rows 0/32 — the
//! program-swap pipeline reset inherited from spwm_15 holds under
//! the fixed-rate loop shape inherited from spwm_12.
//!
//! ```sh
//! cargo run --release --example spwm_17_combined
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

// ── Test-pattern toggle ─────────────────────────────────────────────
/// `false` = diagonal scrolling rainbow (default — shows motion,
///           surfaces flicker).
/// `true`  = E25 static four-block pattern (row 31 RED/BLUE, row 63
///           GREEN/YELLOW, rest black). Makes any echo on rows 0/32
///           unambiguous, as it did in spwm_12/15 evaluation.
const ECHO_TEST: bool = true;

/// Freeze the rainbow pattern (offset always 0) for brightness
/// comparison. Motion masks small brightness differences — use
/// `MOVING = false` when probing dark-fraction tradeoffs. Has no
/// effect when `ECHO_TEST = true`.
const MOVING: bool = true; // demo default; set false to freeze for brightness probes

/// Data-phase PIO clock divisor. Settled at 3 (DCLK = 25 MHz,
/// in-spec per datasheet §6.1 max FCLK) after A/B on 2026-04-22:
/// clkdiv=2 (37.5 MHz, overclocked) gave no visible brightness
/// benefit over clkdiv=3 on our panel — the ~7 pp extra dark
/// fraction is imperceptible at these refresh rates. Dropping to
/// spec buys reliability margin and lifetime for essentially free.
///
/// History of other tested values:
///   - 1.0  (75 MHz):  gross corruption, power-cycle required.
///   - 1.5  (avg 50):  intermittent — fractional clkdiv's period
///                     jitter contains clkdiv=1 instantaneous peaks.
///   - 1.75 (avg 43):  worse than 1.5, more jitter bursts.
///   - 2.0  (37.5):    worked, but overclocked 50 % over spec.
///   - 3.0  (25.0):    in-spec, visually identical to 2.0 — chosen.
///
/// Fractional clkdivs are a dead end for HUB75 shift chains
/// regardless of target average rate (period jitter always hits
/// the N=floor instantaneous rate). Only integer divisors are
/// usable. See `reference/DP3364S/README.md` for details.
const DATA_CLKDIV: u16 = 3;


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
// Values chosen 2026-04-22 after an A/B sweep on-panel:
//   - reg 0x08 = 0xFF (max output-current scale, §10 Iout formula).
//     Sweep across 0x80 / 0xBF / 0xDF / 0xFF showed 0xBF → 0xFF is
//     visibly brighter with no instability on our panel.
//   - reg 0x0F = 0x10 (datasheet-nominal base trim, 70 µA bracket
//     factor). A/B'd against HUB320's 0x20: no visible difference,
//     consistent with LED saturation at reg0x08=0xFF.
// reg 0x0E is deliberately left out — undocumented in our copy of the
// datasheet. Writing an undefined value is pure risk.
const CONFIG_REGS: [u16; 14] = [
    0x1100, 0x021F, 0x037F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08FF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
    0x0F10,
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

const LATCHES_PER_FRAME: usize = SCAN_LINES * 16;                     // 512
const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS;                // 33

const FRAME_WORDS: usize =
    DATA_LATCH_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE;        // 16 936

/// Scan cycles per data phase. Chosen by the 2026-04-22 sweep:
///
/// | cycles | data freq | dark % | verdict                       |
/// |-------:|----------:|-------:|-------------------------------|
/// |     20 |   ~129 Hz |  ~37 % | flicker-free, clearly dim     |
/// |     30 |    ~98 Hz |  ~27 % | flicker-free, subtly dim      |
/// |     50 |     65 Hz |  17 %  | **sweet spot — comfortable**  |
/// |     57 |     58 Hz |  15 %  | borderline flicker            |
/// |     65 |    ~51 Hz |   —    | visible flicker               |
/// |     80 |    ~42 Hz |   —    | clear flicker                 |
///
/// Flicker edge is at ~58 Hz data rate (just above the textbook
/// ~60 Hz fusion threshold once you account for subjective comfort
/// margin). Brightness rolls off only when dark fraction roughly
/// doubles — 17 % → 27 % is barely visible on still content, but
/// 17 % → 37 % is obvious. Going above 50 costs flicker with
/// essentially no brightness gain. Going below 50 costs brightness
/// (especially on still content) with flicker margin you don't need.
const POST_SCAN_CYCLES: usize = 50;

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

// ── Gamma LUT ────────────────────────────────────────────────────────
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
    // spwm_21 scan-word layout (right-shift, autopull 32):
    //   bits  0-6  : DISPLAY_CLK - 1     (OUT Y, 7)
    //   bits  7-12 : 0 (RGB pins 0-5, driven to zero)   \
    //   bits 13-17 : row (addr pins 6-10)                } OUT PINS, 11
    //   bits 18-22 : SETUP_CLK - 1        (OUT Y, 5)
    //   bits 23-26 : oe - 1               (OUT Y, 4)
    //   bits 27-31 : 0 (OUT NULL, 5 drain)
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SCAN_BUF) };
    for row in 0..SCAN_LINES {
        let oe = if row == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
        buf[row] = (DISPLAY_CLK - 1)
            | (0x3F << 7)                   // spwm_22: RGB bits all ONE
            | ((row as u32) << 13)
            | ((SETUP_CLK - 1) << 18)
            | ((oe - 1) << 23);
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

// ── Fill pixels ─────────────────────────────────────────────────────
//
// Rainbow (ECHO_TEST=false): diagonal hue scroll, surfaces flicker.
// E25 (ECHO_TEST=true):      static four-block pattern. Any echo on
//                             row 0 or row 32 mirrors the source row
//                             with preserved colour per half. spwm_17
//                             is predicted clean (pipeline-reset from
//                             program swap at each data↔scan boundary).
fn fill_pixels(offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        for col in 0..128usize {
            pixels[row][col] = if ECHO_TEST {
                if row == 31 {
                    if col < 64 { Rgb::new(255, 0, 0) } else { Rgb::new(0, 0, 255) }
                } else if row == 63 {
                    if col < 64 { Rgb::new(0, 255, 0) } else { Rgb::new(255, 255, 0) }
                } else {
                    Rgb::BLACK
                }
            } else {
                let eff_offset = if MOVING { offset } else { 0 };
                let hue = (row as u16 + col as u16 + eff_offset as u16) as u8;
                hsv(hue)
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
        update_wr_cfg(buf, (offset as usize / 2) % CONFIG_REGS.len());
        fifo_write(1);
    }
}

// ── PIO register access + program-swap helpers ──────────────────────
const PIO0_BASE: u32     = 0x5020_0000;
const PIO0_CTRL_CLR: u32 = PIO0_BASE + 0x3000;
const PIO0_FDEBUG: u32   = PIO0_BASE + 0x008;
const SM0_CLKDIV: u32    = PIO0_BASE + 0x0C8;
const SM0_EXECCTRL: u32  = PIO0_BASE + 0x0CC;
const SM0_INSTR: u32     = PIO0_BASE + 0x0D8;
const SM0_PINCTRL: u32   = PIO0_BASE + 0x0DC;
const SM1_EXECCTRL: u32  = PIO0_BASE + 0x0E4;
const SM1_PINCTRL: u32   = PIO0_BASE + 0x0F4;
const SM1_EN: u32        = 1 << 1;
const SM0_TXSTALL: u32   = 1 << 24;

fn disable_sm(mask: u32) { unsafe { (PIO0_CTRL_CLR as *mut u32).write_volatile(mask) }; }

fn wait_txstall(sm_stall_bit: u32) {
    unsafe {
        (PIO0_FDEBUG as *mut u32).write_volatile(sm_stall_bit);
        while (PIO0_FDEBUG as *const u32).read_volatile() & sm_stall_bit == 0 {
            cortex_m::asm::nop();
        }
    }
}

fn set_clkdiv(int_div: u16) {
    unsafe { (SM0_CLKDIV as *mut u32).write_volatile((int_div as u32) << 16) };
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

    // ── 4. Install both PIO programs (data + scan) ───────────────────
    let (mut pio0, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // SM0: data path.
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
            .clock_divisor_fixed_point(DATA_CLKDIV, 0)
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

    // SM1: scan path — configuration template only.
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
    // spwm_21: OUT PINS, 11 (was 5) — drives RGB pins 0-5 to 0 plus
    // address pins 6-10. Matches spwm_12's scan-path OUT PINS width.
    sa.out_with_side_set(pio::OutDestination::PINS, 11, 0);
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
    // spwm_21: OUT NULL 5 (was 11) — 6 bits moved from drain into the
    // expanded OUT PINS. Total word consumption still 32 bits.
    sa.out_with_side_set(pio::OutDestination::NULL, 5, 0);
    sa.bind(&mut s_wrap_source);
    let scan_prog = sa.assemble_with_wrap(s_wrap_source, s_wrap_target);
    let scan_installed = pio0.install(&scan_prog).unwrap();
    let scan_start = scan_installed.offset() as u32;

    let (_scan_sm_template, _, _scan_tx_unused) =
        hal::pio::PIOBuilder::from_installed_program(scan_installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 11)  // spwm_21: was out_pins(6, 5)
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

    defmt::info!("spwm_22 Thread B exp4: scan-path OUT PINS 11 (drives RGB to 1)");
    defmt::info!(
        "POST_SCAN_CYCLES={=u32}  DATA_CLKDIV={=u16} (DCLK ≈ {=u32} MHz, in-spec)",
        POST_SCAN_CYCLES as u32,
        DATA_CLKDIV,
        SYS_CLK_MHZ / (DATA_CLKDIV as u32 * 2),
    );

    // ── 5. DMA — single channel ──────────────────────────────────────
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
        set_clkdiv(DATA_CLKDIV);
        swap_sm0_program(data_execctrl, data_pinctrl, data_start);
    };

    // ── 8. Startup flush (inherited from spwm_15) ────────────────────
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

    // ── 9. Main loop — fixed-rate data + N scan cycles ───────────────
    let mut active_buf: u8 = 0;
    let mut offset: u8 = 0;

    offset = offset.wrapping_add(2);
    let mut core1_buf: u8 = 1 - active_buf;
    fifo_write(offset as u32 | ((core1_buf as u32) << 8));
    let mut core1_done = false;

    // Timing accumulators (cycles).
    let mut scan_cy_total: u64 = 0;
    let mut data_cy_total: u64 = 0;
    let mut scan_count: u32 = 0;
    let mut data_count: u32 = 0;
    let mut last_report = now_cycles();


    loop {
        // Data phase — always fires, regardless of core-1 pack state.
        // If core 1 hasn't produced a new frame yet, we re-load the
        // currently active buffer. The chip's SRAM ends up with the
        // same content; no visual effect other than the dark gap.
        let t_data0 = now_cycles();
        swap_to_data();

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

        // Scan phase — POST_SCAN_CYCLES back-to-back full scans. Each
        // is one 32-word DMA reading SCAN_BUF, producing one full
        // top-to-bottom refresh of the 32 scan_lines.
        for _ in 0..POST_SCAN_CYCLES {
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
        }

        // Pick up a newly-packed buffer (if any) and kick off the next
        // pack. If core 1 is still packing, keep displaying the current
        // buffer — that's the flicker fix.
        if !core1_done {
            if fifo_try_read().is_some() {
                core1_done = true;
            }
        }
        if core1_done {
            active_buf = core1_buf;
            offset = offset.wrapping_add(2);
            core1_buf = 1 - active_buf;
            fifo_write(offset as u32 | ((core1_buf as u32) << 8));
            core1_done = false;
        }

        // Periodic report — every ~1 s of wall time.
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
