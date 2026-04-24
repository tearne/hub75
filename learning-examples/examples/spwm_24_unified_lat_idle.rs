//! # DP3364S S-PWM — Step 24: unified-program, LAT undriven during scan
//!
//! ## Purpose
//!
//! Discriminate between the remaining candidates for why spwm_23's
//! unified-program architecture still echoes even with RGB=1 scan
//! words, when spwm_22's scan-only program with the same RGB=1 does
//! not. This file tests whether spwm_17/22's "LAT not driven by the
//! scan program" behaviour is the differentiator.
//!
//! ## Prior state
//!
//! | file    | architecture                       | RGB during scan | echo |
//! | ------- | ---------------------------------- | --------------- | ---- |
//! | spwm_17 | swap, 1-bit side-set scan program  | undriven        | no   |
//! | spwm_22 | swap, 1-bit side-set, OUT PINS 11  | ONE             | no   |
//! | spwm_23 | unified, 2-bit side-set, OUT PINS 11 | ONE             | echo |
//!
//! spwm_22 → spwm_23 is the generalization that failed. Structural
//! differences include: swap vs unified, type-bit dispatch prelude
//! vs none, 1-bit vs 2-bit side-set (→ LAT driven-to-zero every
//! instruction vs LAT left alone during scan).
//!
//! ## This experiment
//!
//! Refactor spwm_23's PIO program to use 1-bit side-set (CLK only),
//! moving LAT control to SET instructions. During data phase two SET
//! writes raise/lower LAT around lat_loop. During scan phase the two
//! existing SETs (OE enable/disable) will incidentally also write
//! LAT=0 alongside OE — matching the chip-visible LAT behaviour of
//! spwm_22 (LAT always 0 during scan), but with two SET-based
//! writes per scan word rather than spwm_22's zero writes. That's
//! a much closer approximation than spwm_23's ~15 side-set-based
//! LAT=0 writes per scan word.
//!
//! ## Changes from spwm_23
//!
//! 1. `SideSet::new(false, 2, false)` → `SideSet::new(false, 1, false)`
//! 2. `set_pins(13, 1)` → `set_pins(12, 2)` — SET now covers LAT + OE
//! 3. All PIO instructions' side-set values drop the LAT bit:
//!    0b00→0, 0b01→1, 0b10→0 (needs preceding SET), 0b11→1
//! 4. Data path: insert `SET PINS 0b01` (LAT=1, OE=0) before lat_loop,
//!    `SET PINS 0b00` (LAT=0, OE=0) after
//! 5. Scan path: existing OE enable changes from `SET PINS 1` →
//!    `SET PINS 0b10` (LAT=0, OE=1); OE disable stays `SET PINS 0`
//!    (now interpreted as both pins = 0).
//!
//! ## Outcomes
//!
//! - **Echo-free, display correct**: side-set-based LAT=0 rewriting
//!   was the missing piece. Unified-program architecture with 1-bit
//!   side-set works. Simplification achieved (one SM, one program,
//!   no swap, though with extra SET instructions).
//! - **Echo still present**: LAT redriving isn't the mechanism
//!   either. Remaining candidates (type-bit dispatch prelude, or the
//!   act of having two installed programs at all) become primary.
//! - **Display garbled**: bit budget or SET timing broke something.
//!   Need to debug before drawing conclusions.
//!
//! ```sh
//! cargo run --release --example spwm_24_unified_lat_idle
//! ```
//!
//! ---
//!
//! # Inherited spwm_23 header follows
//!
//! # DP3364S S-PWM — Step 23: unified-program, RGB=1 during scan
//!
//! ## Purpose
//!
//! The simplification test. spwm_12's unified-PIO-program architecture
//! (one SM, one program, type-bit dispatch, echo-affected) but with
//! one targeted change: scan-word RGB bits are set to ONE instead of
//! ZERO. If Thread B's finding holds (echo triggered by RGB=0 during
//! scan), this should produce:
//!
//!   - one SM
//!   - one PIO program
//!   - no program-swap machinery
//!   - no boundary ceremony
//!   - **and no echo**
//!
//! That is the full simplification goal we set out for Thread B.
//!
//! ## Change from spwm_12
//!
//! One line: `scan_word()` sets bits 8-13 (RGB field) to `0x3F`
//! instead of 0. All callers — inline post-scan words, sync-phase
//! scan buffer — inherit the change automatically.
//!
//! ## Risks / gotchas to watch for
//!
//! - **Sync phase:** the startup sync phase uses W12 on scan_line 0
//!   to align driver chip SRAM pointers. Driving RGB=1 during sync
//!   might interact with that process and fail to bootstrap. If the
//!   panel is blank or scrambled at startup, split the change so
//!   sync uses RGB=0 and main loop uses RGB=1.
//! - **Chip state carryover:** spwm_17's echo-free behaviour is
//!   subtly different from "RGB=1 during scan" — spwm_17 leaves RGB
//!   pins in whatever state the data phase left them. RGB=1 is a
//!   different state than "last-data-bit state". If there's a
//!   value-specific sensitivity other than the one tested in
//!   spwm_22, this could expose it.
//!
//! ## Outcomes
//!
//! - **Display correct, no echo:** simplification achieved. spwm_17
//!   can be retired. This is the target result.
//! - **Echo present:** Thread B's RGB=1 finding doesn't generalize
//!   from the spwm_17-derived spwm_22 architecture back to spwm_12's
//!   unified-program architecture. Something else in the spwm_12 /
//!   spwm_22 difference matters.
//! - **Garbled display:** RGB=1 interacts with something in the
//!   unified program (the type-bit dispatch prelude, or the wider
//!   OUT PINS 11 instruction) that breaks. Fall back to more
//!   targeted tests.
//!
//! ```sh
//! cargo run --release --example spwm_23_unified_rgb_one
//! ```
//!
//! ---
//!
//! # Inherited spwm_12 header follows
//!
//! # DP3364S S-PWM — Step 12: split DMA (echo investigation, HISTORICAL)
//!
//! Twelfth in the SPWM progression. Copied from spwm_11 (unrotated
//! post-scan, `POST_SCAN_CYCLES = 50`) and modified to emit each frame
//! as **two DMAs** with a `wait_txstall` between them:
//!
//! ```text
//!   DMA 1: [VSYNC | PRE_ACT | WR_CFG | DATA_LATCH × 512]   (DATA_END words)
//!   wait_txstall                                           ← clean pipeline gap
//!   DMA 2: [SCAN_WORD × 32 × POST_SCAN_CYCLES]             (post-scan region)
//!   wait_txstall
//! ```
//!
//! ## Role in the investigation (now closed)
//!
//! This file was the diagnostic vehicle for the scan_line-31 → scan_line-0
//! echo. It is the simplest program that reliably reproduces the echo,
//! so E21–E25 were implemented here to probe individual hypotheses. The
//! investigation is now **complete** — see spwm_15 for the resolution.
//! This file is preserved as the historical echo reproducer and for the
//! experiment log in the E21–E25 sections below.
//!
//! **Bottom line:** the echo came from our **unified PIO program with
//! type-bit dispatch**. Replacing that with two separate PIO programs
//! swapped on the same SM at each phase boundary (spwm_15) eliminated
//! the echo entirely. Details in spwm_15's header.
//!
//! ## E19 result (the original purpose of this file)
//!
//! **Echo still visible.** Inserting `wait_txstall` between the data
//! and post-scan DMAs did not eliminate the scan_line 31 → scan_line 0
//! leak. Pipeline-drain at the MCU side is not sufficient. That ruled
//! out the "DATA → SCAN boundary adjacency" hypothesis at the
//! coarse-grained level. Further null results accumulated in E17,
//! E18, E20 and the other toggles already in this file (see the
//! consts `INTER_DMA_DELAY_CYCLES`, `INTER_DMA_INJECT`,
//! `INTER_DMA_HIGHZ_CYCLES`).
//!
//! ## E21–E25 diagnostic sweep
//!
//! Motivation: between spwm_12 (echo) and spwm_14 (clean), the
//! not-yet-individually-isolated variables are PIO state carryover,
//! TX FIFO residue, single vs dual DMA channel, and unified vs
//! separate PIO programs. The spwm_15 refactor targets the last one
//! but is expensive to execute. These five cheaper experiments narrow
//! the hypothesis space first. Each is a small localised change to
//! spwm_12.
//!
//! Record for each: **Status** (not run / run / ongoing) and
//! **Observation** (echo unchanged / reduced / eliminated / other).
//!
//! ### E21 — SM_RESTART between phases
//! Write `PIO0_CTRL.SM_RESTART` bit 4 (SM0) after `wait_txstall` and
//! before the post-scan DMA. Per RP2350 datasheet §12.5, this clears:
//! the input and output shift counters, the ISR contents, the delay
//! counter, the waiting-on-IRQ state, any stalled instruction, any
//! stalled OUT/PULL/IN/PUSH due to full-empty FIFOs, and the status
//! register. It does **not** clear X, Y, OSR contents, the FIFOs
//! themselves, or the PC.
//!
//! - **Prediction:** if the echo is caused by carryover of any of the
//!   things this clears (most likely the output shift counter, since
//!   autopull state could plausibly misalign the scan phase's first
//!   OUT), echo disappears.
//! - **Implementation:** const `E21_SM_RESTART` (0/1); when 1,
//!   `restart_sm(SM0_RESTART)` fires between data-phase `disable_sm`
//!   and the inter-DMA toggles. Compared against `echo_baseline`
//!   (same architecture, no E21) by flashing both in turn.
//! - **Status:** run 2026-04-21 (both modes flashed against each
//!   other, panel reset via `panel_reset` between runs).
//! - **Observation:** **echo still present, no perceptible change
//!   from baseline.** Prediction falsified. The echo is not caused
//!   by carryover of any of the state `SM_RESTART` clears: input/
//!   output shift counters, ISR, delay counter, waiting-on-IRQ
//!   state, stalled instructions, stalled FIFO ops, or the status
//!   register. Remaining PIO-internal suspects are the X/Y scratch
//!   registers and OSR content — exactly what E22 targets.
//!
//! ### E22 — explicit OSR / X / Y scrub via SMx_INSTR
//! Force-execute `MOV OSR, NULL`, `MOV X, NULL`, `MOV Y, NULL` via
//! `SM0_INSTR` at the phase boundary, while the SM is running-but-
//! stalled on autopull (before `disable_sm`). Reaches X, Y, OSR
//! content — the scratch state `SM_RESTART` specifically leaves
//! alone. Writes to `SMx_INSTR` override the stalled instruction
//! in the next PIO cycle; a short `delay(10)` between writes
//! (~67 ns at 150 MHz, ~5 PIO cycles at 75 MHz) ensures each scrub
//! executes before the next overwrites.
//!
//! - **Prediction (initial):** if residue in OSR / X / Y carries the
//!   echo, scrubbing all three eliminates it.
//! - **Status:** abandoned 2026-04-21 — experiment design was flawed.
//! - **Observation:** panel went blank, scan phase visibly broken.
//!   Diagnosis: `MOV OSR, NULL` resets the output shift counter to
//!   0 as a side effect, which un-stalls the SM from its autopull
//!   wait. The SM then executes pending instructions using zero
//!   data as command bytes (consuming 32 zero bits through dispatch
//!   / data_cmd), stuffing X=0 Y=0 and stalling at a different
//!   point in data_cmd's `out Y, 16`. When the scan DMA starts,
//!   scan words get interpreted as data-command payload bits,
//!   producing nonsense on the pins.
//! - **Deeper conclusion:** the experiment was testing state that
//!   can't actually carry the echo. OSR is strictly consumed by
//!   OUT instructions — every bit is output exactly once, and at
//!   stall time the counter is 32 (OSR fully drained). There is
//!   no residue. X and Y are overwritten by `out X, ...` /
//!   `out Y, ...` at the top of the data path before use, so
//!   residue there is also inert. Combined with the E21 null, all
//!   plausible per-SM internal state is now exonerated. Focus
//!   should shift to what happens **outside** the PIO — the FIFO
//!   (E23 sanity check), the chip-side shift registers (E24), or
//!   the spatial nature of the leak (E25).
//!
//! ### E23 — FIFO-empty assertion post-stall (SKIPPED)
//! Would have confirmed our assumption that wait_txstall implies the
//! TX FIFO is genuinely empty. Skipped because E21's null already
//! indirectly confirms the autopull-stall state; E25 then made this
//! irrelevant by showing the leak is chip-side.
//!
//! ### E24 — dummy drain words between phases (SKIPPED)
//! Would have tested chip-side shift-register residue by clocking
//! extra zero DATA_LATCHes between data and scan. Skipped because
//! E25's finding (echo is per-column SRAM-level, preserving colour
//! exactly, no smear) argues strongly against a serial-chain mechanism
//! — bits stuck in a shift register would produce smearing, not
//! 1:1 column-preserved mirroring. The pipeline-continuity hypothesis
//! was then validated architecturally by spwm_15's success.
//!
//! ### E25 — colour-asymmetric / spatial test patterns
//! Replace the uniform-per-row pattern with spatially distinct
//! blocks so the echo's structure is readable. Current E25 pattern:
//!
//! |              | cols 0-63 | cols 64-127 |
//! | ------------ | --------- | ----------- |
//! | row 31       | RED       | BLUE        |
//! | row 63       | GREEN     | YELLOW      |
//!
//! Shape of the echo on rows 0 / 32 narrows the mechanism —
//! per-column SRAM leak, serial shift-register leak, cross-half
//! bleed, column-selective, etc. See `fill_pixels` docstring for
//! the full decision table.
//!
//! - **Implementation:** modified `fill_pixels` in both spwm_12
//!   and echo_baseline to emit the four-block pattern. No other
//!   code changes.
//! - **Status:** run 2026-04-21.
//! - **Observation:** four colours on the source rows; echo on the
//!   target rows **exactly mirrors** the source rows' colour and
//!   column layout. Row 0: left-half faint red, right-half faint
//!   blue, sharp boundary at col 64. Row 32: left-half faint
//!   green, right-half faint yellow, same sharp boundary. No
//!   colour mixing, no smearing, no horizontal shift, no cross-
//!   half bleed, intensity uniform across each half.
//! - **Conclusion:** the echo is a **per-column, SRAM-address-
//!   level leak**. Column N of scan_line 31's SRAM bleeds into
//!   column N of scan_line 0's SRAM (upper half); similarly for
//!   63 → 32 (lower half). Each colour channel is preserved
//!   exactly. The mechanism operates on individual SRAM cells in
//!   1:1 column correspondence.
//!
//!   **Rules out:**
//!     - serial shift-register leak (would smear / shift the
//!       boundary at col 64);
//!     - cross-half bleed (upper source → lower target, etc.);
//!     - colour-channel-specific mechanism (all channels echo
//!       equally);
//!     - column-selective mechanism (all 128 columns affected
//!       uniformly).
//!
//!   **Remaining hypothesis space** is chip-internal, specific to
//!   the scan_line 31 ↔ 0 (and 63 ↔ 32) SRAM address pair. Most
//!   plausible: the DP3364S's address wrap/decode during the
//!   last DATA_LATCH of scan_line 31 does something that causes
//!   scan_line 0's SRAM cells to be written as well. This cannot
//!   be the MCU pipeline: our command stream is data-correct and
//!   per-column, so the chip is receiving correct bits for
//!   scan_line 31 and somehow also writing scan_line 0 from those
//!   same bits.
//!
//! ## Final outcome
//!
//! The cheap diagnostics (E21, E25) ruled MCU-side state in/out and
//! localised the mechanism to a chip-internal per-column SRAM leak at
//! command-stream wrap points. The spwm_15 refactor (single SM with
//! two separate PIO programs, swapped at phase boundaries) was then
//! confirmed on-panel to eliminate the echo entirely, while retaining
//! the flicker-reduction goal (no ~2.6 ms dual-SM dark gap). The
//! distinguishing variable was **unified-program-with-type-bit-dispatch
//! vs separate-programs-with-distinct-wrap-regions** — not single-SM
//! vs dual-SM as originally framed.
//!
//! ## Test pattern (current — E25)
//!
//! Row 31: cols 0-63 red, cols 64-127 blue.
//! Row 63: cols 0-63 green, cols 64-127 yellow.
//! Echo (if any) lands on row 0 (upper target) and row 32 (lower
//! target). `POST_SCAN_CYCLES` stays at 50 for per-row refresh
//! around 3 kHz. See `fill_pixels` docstring for how to read the
//! echo shape.
//!
//! ```sh
//! cargo run --release --example spwm_12_split_dma
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
// Matches spwm_6's working command order. Earlier experiments with
// VSYNC at other positions + an extra "Enable All Output" (EAO) LAT-12
// command all failed to bootstrap sync from cold on DP3364S; see the
// Experiment log in the header docs for the null-result evidence.
const VSYNC_OFFSET:   usize = 0;
const PRE_ACT_OFFSET: usize = VSYNC_OFFSET + 1 + VSYNC_WORDS;     // 2
const WR_CFG_OFFSET:  usize = PRE_ACT_OFFSET + 1 + PRE_ACT_WORDS; // 7
const HEADER_WORDS:   usize = WR_CFG_OFFSET + 1 + WR_CFG_WORDS;   // 40

const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS;                // 33
const LATCHES_PER_LINE:  usize = 16;
const LATCHES_PER_FRAME: usize = SCAN_LINES * LATCHES_PER_LINE;       // 512

const DATA_OFFSET: usize = HEADER_WORDS;                              // 40
const DATA_END:    usize = DATA_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE; // 16 936

// E17 sweep variable. Each cycle = 32 scan_words × 132 DCLKs per
// scan_word = 4224 DCLKs. At 16.7 MHz → ~253 µs per cycle.
// Wraps-per-frame = POST_SCAN_CYCLES (every cycle boundary is a
// 31 → 0 wrap in the unrotated architecture).
//
// Sweep order: 50 (baseline) → 12 → 6 → 3 → 1, plus intermediates.
const POST_SCAN_CYCLES: usize = 50;

const SCAN_OFFSET: usize = DATA_END;
const POST_SCAN_WORDS: usize = POST_SCAN_CYCLES * SCAN_LINES;

const FRAME_WORDS: usize = SCAN_OFFSET + POST_SCAN_WORDS; // 18 536

// ── Double-buffered frame data ───────────────────────────────────────
static mut FRAME_BUF_A: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];
static mut FRAME_BUF_B: [u32; FRAME_WORDS] = [0u32; FRAME_WORDS];

// ── Scan buffer for ring-mode DMA (sync-phase only) ──────────────────
// 128-byte aligned so DMA RING_SIZE=7 wraps cleanly every 32 words.
#[repr(C, align(128))]
struct ScanBufAligned([u32; SCAN_LINES]);
static mut SCAN_BUF: ScanBufAligned = ScanBufAligned([0u32; SCAN_LINES]);

// ── Core 1 stack ─────────────────────────────────────────────────────
static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

// Extra idle gap between DMA 1 and DMA 2. Tested up to 100_000 cycles
// (~670 µs) → no effect on the echo, so pipeline-drain isn't the cure.
// Left here at 0 as documentation.
const INTER_DMA_DELAY_CYCLES: u32 = 0;

// VSYNC-injection experiment: after DMA 1 (data) finishes, manually
// push a VSYNC header + payload word into the PIO TX FIFO before
// starting DMA 2 (post-scan). If VSYNC is the pipeline-reset signal
// that spwm_4 implicitly benefits from, a mid-frame extra VSYNC
// should kill the echo on scan_line 0.
//   0 = no injection
//   1 = inject VSYNC
//   2 = inject VSYNC + PRE_ACT (like spwm_4's per-frame header)
const INTER_DMA_INJECT: u32 = 0;

// Pin high-Z experiment: mimic what spwm_4 actually does (state-
// machine handover releases CLK/LAT to input between phases). Here
// we force SM0 to set pindirs=0 on CLK (pin 11) and LAT (pin 12)
// between DMA 1 and DMA 2, optionally delay, then reclaim them.
//   0 = disabled
//   N = release, delay N cortex-m cycles (~6.67 ns each), reclaim
const INTER_DMA_HIGHZ_CYCLES: u32 = 0;

// ── Scan parameters ──────────────────────────────────────────────────
const DISPLAY_CLK: u32 = 100;
const SETUP_CLK: u32 = 28;
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

// ── Data header with type bit ───────────────────────────────────────

const fn data_header(n_pre: u32, n_lat: u32) -> u32 {
    // bit 0 = 0 (data type)
    ((n_lat - 1) << 16) | ((n_pre - 1) << 1)
}

// ── Scan word with type bit ─────────────────────────────────────────

fn scan_word(display: u32, addr: u32, setup: u32, oe: u32) -> u32 {
    1                           // bit 0 = 1 (scan type)
    | ((display - 1) << 1)     // bits 7:1
    | (0x3F << 8)              // bits 13:8 → RGB pins 0-5 driven HIGH (spwm_23)
    | (addr << 14)             // bits 18:14 → GPIO 6-10
    | ((setup - 1) << 19)      // bits 23:19
    | ((oe - 1) << 24)         // bits 27:24
}

/// Scan word for the main-loop padded-DMA region: uniform W4 OE on
/// every row, so rows 0 and 32 (both driven by scan_line 0) have the
/// same brightness as the rest of the panel in steady state.
fn scan_word_for(scan_line: usize) -> u32 {
    scan_word(DISPLAY_CLK, scan_line as u32, SETUP_CLK, OE_CLK_W4)
}

/// Scan word for the boot sync phase's ring-mode DMA: W12 OE on
/// scan_line 0, W4 elsewhere. The W12 pulse is what actually resets
/// and aligns the 8 driver chips' SRAM write pointers — the only
/// mechanism we've found that bootstraps sync on DP3364S. This is
/// used only during the ~1 s startup sync phase; after that, main
/// loop scans come from the padded region and use W4 uniformly.
fn scan_word_for_sync(scan_line: usize) -> u32 {
    let oe = if scan_line == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
    scan_word(DISPLAY_CLK, scan_line as u32, SETUP_CLK, oe)
}

// ── Offset helpers ──────────────────────────────────────────────────

const fn latch_header_offset(scan_line: usize, channel: usize) -> usize {
    let latch_idx = scan_line * LATCHES_PER_LINE + channel;
    DATA_OFFSET + latch_idx * DATA_LATCH_STRIDE
}

const fn post_scan_offset(cycle: usize, scan_line: usize) -> usize {
    SCAN_OFFSET + cycle * SCAN_LINES + scan_line
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

/// Write the static portions of a frame buffer: VSYNC / PRE_ACT / WR_CFG
/// headers, DATA_LATCH headers, inline SCAN_WORDs, and the post-scan
/// padding region.
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

    // Emit W12 on the first scan_line of EVERY cycle (mimicking
    // spwm_4's ring-mode scan buffer which has W12 at scan_line 0 and
    // wraps every 32 scans, giving the chip a W12 per group rather
    // than once per frame). Datasheet §11.5 only prescribes Group 0
    // Line 0, but spwm_4 (echo-free) emits it every group, so test
    // the same.
    for cycle in 0..POST_SCAN_CYCLES {
        for slot in 0..SCAN_LINES {
            let oe = if slot == 0 { OE_CLK_W12 } else { OE_CLK_W4 };
            buf[post_scan_offset(cycle, slot)] =
                scan_word(DISPLAY_CLK, slot as u32, SETUP_CLK, oe);
        }
    }
}

/// Pack-time compensation for the 4-scan_line hardware shift that
/// appears under SYNC_MODE bit 7 = 1 (e.g. reg 0x0C88). Set to 0
/// when bit 7 is clear (0x0C08 or 0x0C48) since the hardware then
/// displays scan_line N at row N with no offset.
const PACK_SHIFT: isize = 0;

/// Pack the PIXELS framebuffer into the given frame buffer's
/// DATA_LATCH regions.
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

/// E25 diagnostic pattern. Four distinct colour-blocks on the two
/// source rows so the echo's spatial and colour structure is
/// readable:
///
/// |            | cols 0-63 | cols 64-127 |
/// | ---------- | --------- | ----------- |
/// | row 31 (upper source) | RED    | BLUE    |
/// | row 63 (lower source) | GREEN  | YELLOW  |
///
/// Read the echo on row 0 (upper target) and row 32 (lower target):
///   - Same colour per half, same column boundary at col 64 →
///     per-column SRAM-address leak.
///   - Colours preserved but column boundary shifted or smeared →
///     serial shift-register leak inside the driver chips.
///   - Wrong colours (e.g. left-half green on row 0) → cross-half
///     bleed or colour-channel-specific mechanism.
///   - Echo strong on some columns, absent on others → column-
///     selective mechanism (investigate which).
fn fill_pixels(_offset: u8) {
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        for col in 0..128usize {
            pixels[row][col] = if row == 31 {
                if col < 64 { Rgb::new(255, 0, 0) }       // red
                else        { Rgb::new(0, 0, 255) }       // blue
            } else if row == 63 {
                if col < 64 { Rgb::new(0, 255, 0) }       // green
                else        { Rgb::new(255, 255, 0) }     // yellow
            } else {
                Rgb::BLACK
            };
        }
    }
}

/// Diagonal rainbow: `hue = (row + col + offset) mod 256`. Swap in
/// by renaming to `fill_pixels`.
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
        // Stage D1 (revised): don't rotate WR_CFG content per frame.
        // Flush leaves both buffers with a constant known reg value
        // (whatever the last flush iteration wrote). If the 4-line
        // offset changes, rotation content is implicated. If it
        // persists, the WR_CFG command itself is responsible.

        fifo_write(1);
    }
}

// ── PIO helpers ─────────────────────────────────────────────────────

const PIO0_BASE: u32 = 0x5020_0000;
const PIO0_CTRL_SET: u32 = PIO0_BASE + 0x2000;
const PIO0_CTRL_CLR: u32 = PIO0_BASE + 0x3000;
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_CLKDIV: u32 = PIO0_BASE + 0x0C8;
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

// PIO SET instruction encodings (for force_set_pindirs).
//   E080 = SET pindirs, 0   (all released SET pins → input)
//   E083 = SET pindirs, 3   (2 lowest SET pins → output)
const SET_PINDIRS_0: u32 = 0xE080;
const SET_PINDIRS_3: u32 = 0xE083;

// PINCTRL bit-fields (SET base/count) we temporarily hijack.
const PINCTRL_SET_COUNT_MASK: u32 = 0x7 << 26;
const PINCTRL_SET_BASE_MASK: u32 = 0x1F << 5;

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

/// Briefly reassign SM0's SET base/count to the given pins, force-
/// execute one PIO instruction, then restore PINCTRL. Ported from
/// spwm_4 — safe to call while SM0 is stalled on a TX wait.
fn force_set_pindirs(base: u32, count: u32, instr: u32) {
    unsafe {
        let saved = (SM0_PINCTRL as *const u32).read_volatile();
        let modified = (saved & !(PINCTRL_SET_COUNT_MASK | PINCTRL_SET_BASE_MASK))
            | (count << 26) | (base << 5);
        (SM0_PINCTRL as *mut u32).write_volatile(modified);
        (SM0_INSTR as *mut u32).write_volatile(instr);
        (SM0_PINCTRL as *mut u32).write_volatile(saved);
    }
}

// ── DMA ring-mode scan (ch1, raw register access, sync-phase only) ──
const DMA_BASE: u32 = 0x5000_0000;
const CH1_READ_ADDR:  *mut u32 = (DMA_BASE + 0x040) as *mut u32;
const CH1_WRITE_ADDR: *mut u32 = (DMA_BASE + 0x044) as *mut u32;
const CH1_AL1_CTRL:   *mut u32 = (DMA_BASE + 0x050) as *mut u32;
const CH1_AL1_TRANS_COUNT_TRIG: *mut u32 = (DMA_BASE + 0x05C) as *mut u32;
const PIO0_TXF0: u32 = PIO0_BASE + 0x010;

const CH1_CTRL_VAL: u32 =
    (1 << 0)
    | (2 << 2)
    | (1 << 4)
    | (7 << 8)
    | (1 << 13)
    | (0 << 17);

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

    // Pull-down on LAT (GPIO 12) for safety during startup
    const PADS_BANK0: u32 = 0x4003_8000;
    const GPIO12_PAD: u32 = PADS_BANK0 + 0x04 + 12 * 4;
    unsafe {
        let val = (GPIO12_PAD as *const u32).read_volatile();
        (GPIO12_PAD as *mut u32).write_volatile(val | (1 << 2));
    }

    // ── 3. Install unified PIO program (unchanged from spwm_6) ──────
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // spwm_24: 1-bit side-set (CLK only). LAT moved to SET-based control.
    let ss = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::new_with_side_set(ss);

    let mut dispatch    = a.label();
    let mut data_cmd    = a.label();
    let mut display_lp  = a.label();
    let mut setup_lp    = a.label();
    let mut oe_lp       = a.label();
    let mut pre_loop    = a.label();
    let mut lat_loop    = a.label();
    let mut wrap_source = a.label();

    // spwm_24: all side-set values are 1-bit (CLK only).
    //   0 = CLK low, 1 = CLK high.
    // LAT (pin 12) and OE (pin 13) controlled via SET (base 12, count 2):
    //   SET PINS 0b00 : LAT=0, OE=0
    //   SET PINS 0b01 : LAT=1, OE=0
    //   SET PINS 0b10 : LAT=0, OE=1

    // DISPATCH (wrap_target)
    a.bind(&mut dispatch);
    a.out_with_side_set(pio::OutDestination::X, 1, 0);
    a.jmp_with_side_set(pio::JmpCondition::XIsZero, &mut data_cmd, 0);

    // SCAN path (type=1)
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
    // OE enable: LAT=0, OE=1
    a.set_with_side_set(pio::SetDestination::PINS, 0b10, 0);

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

    // OE disable: LAT=0, OE=0
    a.set_with_side_set(pio::SetDestination::PINS, 0b00, 0);
    a.out_with_side_set(pio::OutDestination::NULL, 4, 0);
    a.jmp_with_side_set(pio::JmpCondition::Always, &mut dispatch, 0);

    // DATA path (type=0)
    a.bind(&mut data_cmd);
    a.out_with_side_set(pio::OutDestination::X, 15, 0);
    a.out_with_side_set(pio::OutDestination::Y, 16, 0);

    a.bind(&mut pre_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 1);
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut pre_loop, 0);

    // Raise LAT before lat_loop (was done via side-set 0b10 in spwm_23)
    a.set_with_side_set(pio::SetDestination::PINS, 0b01, 0);

    a.bind(&mut lat_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 1);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut lat_loop, 0);

    // Drop LAT after lat_loop — executes once before wrap to dispatch
    a.set_with_side_set(pio::SetDestination::PINS, 0b00, 0);
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, dispatch);
    let installed = pio0.install(&prog).unwrap();

    let (mut sm, _, tx) =
        hal::pio::PIOBuilder::from_installed_program(installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 11)
            .set_pins(12, 2)  // spwm_24: SET covers LAT (12) + OE (13)
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

    // ── 4. DMA — single channel, continuous frame loop ──────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut ch = dma.ch0;

    defmt::info!("DP3364S S-PWM (padded-frame, continuous DMA) starting");
    defmt::info!("spwm_24: unified program, 1-bit side-set, SET-based LAT; scan RGB=1");

    // ── 5. Init frame headers (DATA_LATCH payloads stay at zero) ────
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) });
    init_frame_headers(unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_B) });

    // ── 6. Startup flush — matches spwm_6's proven two-phase pattern:
    //       data-only DMA at clkdiv 2, then one scan pass at clkdiv 3,
    //       with wait_txstall between. This sequence is what actually
    //       syncs the 8 driver chips' SRAM write pointers to zero (we
    //       established empirically that spwm_6 does this and the sync
    //       persists across a program switch as long as panel power
    //       stays on).
    //
    //       First frame writes GROUP_SET; subsequent ones cycle through
    //       the remaining config regs. Extra iterations beyond the 13
    //       regs ensure SRAM is fully overwritten with zeros.
    const FLUSH_FRAMES: u32 = 26;
    for flush in 0..FLUSH_FRAMES {
        if (flush as usize) < CONFIG_REGS.len() {
            update_wr_cfg(
                unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF_A) },
                flush as usize,
            );
        }

        // --- Data-only DMA (HEADER + 512 DATA_LATCHes), at clkdiv 2 ---
        set_clkdiv(2);
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

        // --- One scan pass (32 scan words), at clkdiv 3 ---
        set_clkdiv(3);
        let scan_slice = unsafe {
            let base = (core::ptr::addr_of!(FRAME_BUF_A) as *const u32).add(SCAN_OFFSET);
            core::slice::from_raw_parts(base, SCAN_LINES)
        };
        let scan_cfg = single_buffer::Config::new(ch, scan_slice, tx);
        let scan_xfer = scan_cfg.start();
        let (new_ch2, _, new_tx2) = scan_xfer.wait();
        ch = new_ch2;
        tx = new_tx2;
        wait_txstall(SM0_TXSTALL);
    }

    defmt::info!("flush complete");

    // ── 6b. Sync phase — match spwm_6's main loop pattern for ~1 s.
    //        Empirically this is what actually locks the 8 driver chips'
    //        SRAM write pointers to a common offset. Once synced, state
    //        persists into the padded-continuous main loop.
    //
    //        Pattern: ring-mode scan (ch1) running continuously, with
    //        brief one-shot data DMAs (ch0) interrupting it. Each cycle
    //        matches spwm_6 exactly: stop-ring → data → restart-ring.
    build_scan_buf();
    // Keep ch1 bound (don't drop immediately) so HAL doesn't reset it.
    // We access ch1 via raw registers for ring-mode; the HAL type just
    // needs to not interfere.
    let _ch1 = dma.ch1;

    set_clkdiv(3);
    start_ring_scan();

    const SYNC_FRAMES: u32 = 60; // ~1 s at 60 Hz
    for _ in 0..SYNC_FRAMES {
        // Crude delay to let ring-mode run for ~one core-1 pack period
        cortex_m::asm::delay(150_000 * 10); // ~10 ms at 150 MHz

        stop_ring_scan();
        wait_txstall(SM0_TXSTALL);

        set_clkdiv(2);
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

        start_ring_scan();
    }

    stop_ring_scan();
    wait_txstall(SM0_TXSTALL);
    defmt::info!("sync phase complete");

    // Stage D1 (revised): seed FRAME_BUF_B's WR_CFG payload so both
    // buffers emit identical constant WR_CFG content in the main loop.
    // Last flushed reg is CONFIG_REGS[12] = 0x0D12.
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

    const TIMER0_BASE: u32 = 0x400B_0000;
    const TIMELR: *const u32 = (TIMER0_BASE + 0x0C) as *const u32;
    fn timer_us() -> u32 {
        unsafe { TIMELR.read_volatile() }
    }
    let mut frame_count: u32 = 0;
    let mut last_report = timer_us();

    // Kick off core 1 with the first pack job
    offset = offset.wrapping_add(2);
    fifo_write(offset as u32 | ((core1_buf as u32) << 8));

    // Split-DMA loop (E19). Each frame goes out as two DMAs with a
    // wait_txstall between:
    //   DMA 1: header + DATA_LATCHes   [0 .. DATA_END]
    //   gap   (pipeline reset)
    //   DMA 2: post-scan scan words    [SCAN_OFFSET .. FRAME_WORDS]
    loop {
        let buf_ptr = if active_buf == 0 {
            core::ptr::addr_of!(FRAME_BUF_A) as *const u32
        } else {
            core::ptr::addr_of!(FRAME_BUF_B) as *const u32
        };

        // DMA 1: header + DATA_LATCHes. Match spwm_4's data-phase
        // clkdiv of 2 (75 MHz PIO, ~37.5 MHz DCLK).
        set_clkdiv(2);
        enable_sm(SM0_EN);
        let data_slice = unsafe { core::slice::from_raw_parts(buf_ptr, DATA_END) };
        let cfg = single_buffer::Config::new(ch, data_slice, tx);
        let xfer = cfg.start();
        let (new_ch, _, new_tx) = xfer.wait();
        ch = new_ch;
        tx = new_tx;
        wait_txstall(SM0_TXSTALL);
        disable_sm(SM0_EN);

        if INTER_DMA_DELAY_CYCLES > 0 {
            cortex_m::asm::delay(INTER_DMA_DELAY_CYCLES);
        }

        if INTER_DMA_HIGHZ_CYCLES > 0 {
            // Release CLK/LAT (pins 11-12) → high-Z, delay, reclaim.
            force_set_pindirs(11, 2, SET_PINDIRS_0);
            cortex_m::asm::delay(INTER_DMA_HIGHZ_CYCLES);
            force_set_pindirs(11, 2, SET_PINDIRS_3);
        }

        if INTER_DMA_INJECT >= 1 {
            // Push VSYNC (2 words) directly to PIO TX FIFO. After the
            // wait_txstall above, FIFO is empty; `OnlyTx` mode gives
            // 8-word depth, so up to 7 words fit without overrun.
            unsafe {
                let txf = PIO0_TXF0 as *mut u32;
                txf.write_volatile(data_header(VSYNC_PRE, VSYNC_LAT));
                txf.write_volatile(0);
                if INTER_DMA_INJECT >= 2 {
                    // + PRE_ACT (1 header + 4 payload = 5 words).
                    txf.write_volatile(data_header(PRE_ACT_PRE, PRE_ACT_LAT));
                    for _ in 0..PRE_ACT_WORDS {
                        txf.write_volatile(0);
                    }
                }
            }
            wait_txstall(SM0_TXSTALL);
        }

        // DMA 2: post-scan region, emitted as POST_SCAN_CYCLES back-
        // to-back 32-word DMAs (mimicking spwm_14's per-group DMA
        // pattern which has a small CPU gap between every group).
        set_clkdiv(3);
        enable_sm(SM0_EN);
        for cycle in 0..POST_SCAN_CYCLES {
            let scan_slice = unsafe {
                core::slice::from_raw_parts(
                    buf_ptr.add(SCAN_OFFSET + cycle * SCAN_LINES),
                    SCAN_LINES,
                )
            };
            let cfg = single_buffer::Config::new(ch, scan_slice, tx);
            let xfer = cfg.start();
            let (new_ch, _, new_tx) = xfer.wait();
            ch = new_ch;
            tx = new_tx;
            wait_txstall(SM0_TXSTALL);
        }
        disable_sm(SM0_EN);

        if fifo_try_read().is_some() {
            frame_count += 1;
            active_buf = core1_buf;
            core1_buf = 1 - active_buf;
            offset = offset.wrapping_add(2);
            fifo_write(offset as u32 | ((core1_buf as u32) << 8));

            if frame_count & 0xFF == 0 {
                let now = timer_us();
                let elapsed_us = now.wrapping_sub(last_report);
                if elapsed_us > 0 {
                    let fps_x10 = 2_560_000_000u32 / elapsed_us;
                    defmt::info!("FPS: {}.{}", fps_x10 / 10, fps_x10 % 10);
                }
                last_report = now;
            }
        }
    }
}
