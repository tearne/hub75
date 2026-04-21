//! # DP3364S S-PWM — Step 9: rotated post-scan (echo-location test)
//!
//! Ninth in the SPWM progression. Copied from
//! `spwm_8_padded_continuous.rs` to test **one hypothesis** about the
//! scan_line-31 → scan_line-0 data echo that spwm_8 exhibits:
//!
//! > The echo is a pipeline-adjacency effect at the post-scan cycle
//! > wrap. Whichever scan_line sits just before the wrap sees its
//! > data leak onto whichever scan_line sits just after. Time/rate
//! > is irrelevant (proven by E4, E14a/b/c null results in spwm_8);
//! > what matters is which two scan_lines are adjacent in the
//! > command stream.
//!
//! Test: rotate the post-scan cycle start by `ROTATE_OFFSET` so
//! cycles run `[ROTATE_OFFSET, …, 31, 0, …, ROTATE_OFFSET-1]` and the
//! intra-frame wrap becomes `(ROTATE_OFFSET-1) → ROTATE_OFFSET`
//! instead of `31 → 0`. Prediction: the echo moves to scan_line
//! `ROTATE_OFFSET`, carrying scan_line `ROTATE_OFFSET-1`'s data.
//!
//! **Frame-boundary caveat.** The VSYNC-driven write-pointer reset
//! depends on the last scan_line before VSYNC being 31 (E1 null
//! result in spwm_8). So the LAST post-scan cycle of each frame
//! stays unrotated (`[0..31]`) to preserve clean sync. The rest use
//! the rotated order. Per frame we get:
//!   - 48 intra-frame wraps at `(ROTATE_OFFSET-1) → ROTATE_OFFSET`
//!   -  1 transition `(ROTATE_OFFSET-1) → 0` (last rotated → final normal)
//!   -  1 frame boundary `31 → ROTATE_OFFSET` (last of frame N → first of N+1)
//!
//! To see the echo's move clearly, the echo-probe `fill_pixels` lights
//! `ROTATE_OFFSET-1` (the new wrap source) cyan/yellow.
//!
//! ```sh
//! cargo run --release --example spwm_9_rotated_post_scan
//! ```
//!
//! ## Result (2026-04-20) — adjacency theory confirmed
//!
//! With `ROTATE_OFFSET = 15` and cyan packed at scan_line 14
//! (= `ROTATE_OFFSET - 1`):
//!
//! | Observation              | Location  | Interpretation                     |
//! |--------------------------|-----------|------------------------------------|
//! | Main bright cyan         | row 30    | Write-pointer shifted +16 relative |
//! |                          |           | to intended slot 14. Rotation of   |
//! |                          |           | intra-frame cycles perturbs the    |
//! |                          |           | chip's VSYNC reset (same class of  |
//! |                          |           | side effect as E1 reverse order).  |
//! |                          |           | The shift is a confound, not part  |
//! |                          |           | of the finding.                    |
//! | Echo cyan                | row 15    | **Exact prediction.** Rotation put |
//! |                          |           | the wrap at 14 → 15; echo followed |
//! |                          |           | to scan_line 15.                   |
//! | No echo on row 0         | —         | The 1-per-frame transitions        |
//! |                          |           | (14 → 0 and 31 → 15) are too rare  |
//! |                          |           | to accumulate a visible echo.      |
//! | No echo on row 31        | —         | Same — no wrap lands there any     |
//! |                          |           | more in this rotation.             |
//!
//! ### What this tells us about the echo mechanism
//!
//! 1. **Strictly command-stream adjacency at wrap points.** Whichever
//!    scan_line fires *just before* the wrap has its data leak onto
//!    whichever scan_line fires *just after*. Not tied to addr=0.
//!    Not tied to specific physical rows of the panel.
//!
//! 2. **Intensity integrates over wrap events per frame.** 48 intra-
//!    frame wraps per frame produce a clearly visible echo; 1 wrap
//!    per frame is invisible. So the echo isn't a single sudden
//!    flash — it's the sum of many dim pipeline stumbles.
//!
//! 3. **Not rate-sensitive** (E4/E14 null). Not fixed by any chip
//!    register (E6/E7/E9/E10 null). Not a settling-time issue.
//!    It's a structural pipeline continuity artefact — the chip's
//!    SPWM driver-load stage can't cleanly hand over between two
//!    adjacent scan_words if it never gets a "pipeline reset" event
//!    (CLK-stop / phase handover, as in spwm_4/5).
//!
//! ### Implications for future mitigation work
//!
//! - **Pack-time echo compensation.** Since we can now predict
//!   exactly which scan_line shows which other scan_line's data, we
//!   can blend-in the source row's content at the destination at
//!   pack time. Costs a bit of contrast on the destination row.
//! - **Position the wrap on a "border" row.** If the panel's
//!   physical row 0 is part of a non-pixel border, place the wrap
//!   there. Not applicable to our 64-row panel — every row shows
//!   content.
//! - **Return to split-phase per frame.** spwm_4's architecture has
//!   a natural CLK-stop between scan pass and next frame's data
//!   phase — that's why it has no echo. Trades spwm_8's continuous-
//!   DMA simplicity for a single clean wrap per frame.
//! - **Hybrid: very-few-wraps per frame.** If we had one scan pass
//!   per frame at ~3 kHz frame rate, each row would refresh once per
//!   frame and the single wrap per frame is invisible (confirmed
//!   above). But data phase dominates at that rate — not feasible
//!   in our continuous-DMA layout without a phase split.

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
    0x037F, 0x1100, 0x021F, 0x043F, 0x0534, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C08, 0x0D12,
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

// Padding scans fill the rest of the ~16.7 ms frame period. 50 cycles
// × 32 rows × 132 CLKs at DCLK 16.7 MHz ≈ 12.65 ms, plus the ~4 ms
// data phase ≈ 16.65 ms total.
const POST_SCAN_CYCLES: usize = 50;

// E15 (this experiment): rotate the scan_line order within each
// post-scan cycle except the last. ROTATE_OFFSET = 15 means cycles
// 0..=48 emit `[15,16,…,31,0,1,…,14]` (wrap 14→15), and cycle 49
// stays `[0,1,…,31]` so the last scan before VSYNC is still 31.
const ROTATE_OFFSET: usize = 15;
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
    // bits 13:8: 0 (RGB zeros)
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

    for cycle in 0..POST_SCAN_CYCLES {
        // Last cycle stays unrotated so the frame ends on scan_line
        // 31 for a clean VSYNC write-pointer reset.
        let rotate = if cycle == POST_SCAN_CYCLES - 1 { 0 } else { ROTATE_OFFSET };
        for slot in 0..SCAN_LINES {
            let scan_line = (slot + rotate) % SCAN_LINES;
            buf[post_scan_offset(cycle, slot)] = scan_word_for(scan_line);
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

/// Echo-diagnostic pattern for the rotation test. All black except:
///   - row 31 bright green, row 63 bright red (control — old wrap
///     source in spwm_8)
///   - row `ROTATE_OFFSET-1` bright cyan, row `ROTATE_OFFSET-1 + 32`
///     bright yellow (test — new intra-frame wrap source)
///
/// Prediction: in spwm_9, echoes should appear predominantly at
/// scan_line `ROTATE_OFFSET` (rows `ROTATE_OFFSET` and `ROTATE_OFFSET+32`)
/// carrying the cyan/yellow, and only faintly at scan_line 0 (carrying
/// green/red from the single frame-boundary wrap).
fn fill_pixels(_offset: u8) {
    let test_src_upper = ROTATE_OFFSET - 1;            // e.g. 14
    let test_src_lower = ROTATE_OFFSET - 1 + 32;       // e.g. 46
    let pixels = unsafe { &mut *core::ptr::addr_of_mut!(PIXELS) };
    for row in 0..64usize {
        let c = if row == test_src_upper {
            Rgb::new(0, 255, 255)     // test: new wrap source (upper) — cyan
        } else if row == test_src_lower {
            Rgb::new(255, 255, 0)     // test: new wrap source (lower) — yellow
        } else {
            Rgb::BLACK
        };
        for col in 0..128usize {
            pixels[row][col] = c;
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
const PIO0_FDEBUG: u32 = PIO0_BASE + 0x008;
const SM0_CLKDIV: u32 = PIO0_BASE + 0x0C8;
const SM0_TXSTALL: u32 = 1 << 24;

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

    // DISPATCH (wrap_target)
    a.bind(&mut dispatch);
    a.out_with_side_set(pio::OutDestination::X, 1, 0b00);
    a.jmp_with_side_set(pio::JmpCondition::XIsZero, &mut data_cmd, 0b00);

    // SCAN path (type=1)
    a.out_with_side_set(pio::OutDestination::Y, 7, 0b00);
    a.bind(&mut display_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut display_lp, 0b01);

    a.out_with_side_set(pio::OutDestination::PINS, 11, 0b00);
    a.out_with_side_set(pio::OutDestination::Y, 5, 0b00);

    a.bind(&mut setup_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut setup_lp, 0b01);

    a.out_with_side_set(pio::OutDestination::Y, 4, 0b00);
    a.set_with_side_set(pio::SetDestination::PINS, 1, 0b00);

    a.bind(&mut oe_lp);
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.mov_with_side_set(
        pio::MovDestination::Y, pio::MovOperation::None,
        pio::MovSource::Y, 0b00,
    );
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut oe_lp, 0b01);

    a.set_with_side_set(pio::SetDestination::PINS, 0, 0b00);
    a.out_with_side_set(pio::OutDestination::NULL, 4, 0b00);
    a.jmp_with_side_set(pio::JmpCondition::Always, &mut dispatch, 0b00);

    // DATA path (type=0)
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

    // ── 4. DMA — single channel, continuous frame loop ──────────────
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut ch = dma.ch0;

    defmt::info!("DP3364S S-PWM (padded-frame, continuous DMA) starting");

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

    // Continuous DMA loop. Each iteration sends header + interleaved
    // body + post-scan padding (~16.7 ms). When core 1 signals a new
    // frame is ready, swap the active buffer on the next iteration.
    loop {
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
