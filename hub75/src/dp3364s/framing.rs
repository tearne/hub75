//! DP3364S frame-buffer layout and command headers.
//!
//! A frame buffer is a u32 array fed to the DP3364S column-driver chips
//! through PIO. It opens with a small header section (VSYNC, PRE_ACT,
//! WR_CFG) and continues with 32×16 = 512 DATA_LATCH commands carrying
//! the actual pixel data. Every command word is preceded by a 32-bit
//! "data_header" telling the PIO program how many pre-latch and latch
//! clocks the command uses.
//!
//! `init_frame_headers` populates the fixed parts of the buffer once at
//! boot; `pack_pixels` fills the DATA_LATCH payloads at runtime.

// ── Pin masks ───────────────────────────────────────────────────────

const R0: u32 = 1 << 0;
const G0: u32 = 1 << 1;
const B0: u32 = 1 << 2;
const R1: u32 = 1 << 3;
const G1: u32 = 1 << 4;
const B1: u32 = 1 << 5;
pub const ALL_RGB: u32 = R0 | G0 | B0 | R1 | G1 | B1;

// ── Panel geometry (DP3364S on Interstate 75 W) ─────────────────────

pub const SCAN_LINES: usize = 32;

/// Each DP3364S has 16 column outputs; each DATA_LATCH command sends one
/// column-per-chip. 16 latches per scan-line covers all 128 columns
/// (8 chips × 16). Not tunable — fixed by chip geometry.
pub const LATCHES_PER_LINE: usize = 16;
pub const LATCHES_PER_FRAME: usize = SCAN_LINES * LATCHES_PER_LINE;

// ── DP3364S configuration registers ─────────────────────────────────
//
// GROUP_SET (0x03) is FIRST: it controls SRAM address wrap, and at boot
// the chip's default is 0x3F (64-wide). If we write DATA_LATCHes with
// that stale GROUP_SET, writes miss half the SRAM. Setting it first
// guarantees every subsequent DATA_LATCH addresses the full 128-wide
// SRAM correctly.
pub const CONFIG_REGS: [u16; 13] = [
    0x037F, 0x1100, 0x021F, 0x043F, 0x0504, 0x0642, 0x0700,
    0x08BF, 0x0960, 0x0ABE, 0x0B8B, 0x0C88, 0x0D12,
];

// ── Per-command CLK counts ──────────────────────────────────────────
// PIO consumes 8 bits per pre/lat loop iteration; header is 32 bits;
// autopull fires every 32 bits consumed. For each command, (PRE+LAT)
// must be a multiple of 4 so commands end on a 32-bit boundary and the
// next dispatch reads a fresh word.

const VSYNC_PRE: u32 = 1;
const VSYNC_LAT: u32 = 3;
const VSYNC_WORDS: usize = 1;

const PRE_ACT_PRE: u32 = 2;
const PRE_ACT_LAT: u32 = 14;
const PRE_ACT_WORDS: usize = 4;

const WR_CFG_PRE: u32 = 123;
const WR_CFG_LAT: u32 = 5;
const WR_CFG_WORDS: usize = 32;

const DATA_LATCH_PRE: u32 = 127;
const DATA_LATCH_LAT: u32 = 1;
const DATA_LATCH_WORDS: usize = 32;

// ── Frame layout — fixed VSYNC → PRE_ACT → WR_CFG → DATA ────────────
// This order is what reliably bootstraps the DP3364S SRAM write
// pointer from cold boot; variants with VSYNC elsewhere or an extra
// "Enable All Output" command were tried and failed to sync.

const VSYNC_OFFSET: usize = 0;
const PRE_ACT_OFFSET: usize = VSYNC_OFFSET + 1 + VSYNC_WORDS;        // 2
const WR_CFG_OFFSET: usize = PRE_ACT_OFFSET + 1 + PRE_ACT_WORDS;     // 7
const HEADER_WORDS: usize = WR_CFG_OFFSET + 1 + WR_CFG_WORDS;        // 40

pub const DATA_LATCH_STRIDE: usize = 1 + DATA_LATCH_WORDS;           // 33
pub const DATA_OFFSET: usize = HEADER_WORDS;                         // 40
pub const DATA_END: usize =
    DATA_OFFSET + LATCHES_PER_FRAME * DATA_LATCH_STRIDE;             // 16 936
pub const FRAME_WORDS: usize = DATA_END;

// ── Header word format ──────────────────────────────────────────────
// The first 32-bit word of every command tells the PIO program how
// many pre-latch + latch clocks to produce.
//   bits  0..14 : n_pre - 1  (up to 32767)
//   bits 15..30 : n_lat - 1  (up to 65535)
//   bit      31 : pad

const fn data_header(n_pre: u32, n_lat: u32) -> u32 {
    (n_pre - 1) | ((n_lat - 1) << 15)
}

pub const fn latch_header_offset(scan_line: usize, channel: usize) -> usize {
    let latch_idx = scan_line * LATCHES_PER_LINE + channel;
    DATA_OFFSET + latch_idx * DATA_LATCH_STRIDE
}

// ── Initialisation ──────────────────────────────────────────────────

/// Populate the fixed parts of a frame buffer: per-command headers
/// (VSYNC, PRE_ACT, WR_CFG) and the 512 DATA_LATCH headers. Pixel-data
/// payloads are written by `pack_pixels`; the WR_CFG payload is updated
/// per-iteration by `update_wr_cfg` during the boot flush.
pub fn init_frame_headers(buf: &mut [u32; FRAME_WORDS]) {
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

/// Spread a 16-bit configuration word across the WR_CFG payload's 32
/// u32 words, one bit per packed slot, gated by the RGB pin mask. Used
/// during the boot flush to write each `CONFIG_REGS` value through the
/// chip's WR_CFG command.
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

/// Write `CONFIG_REGS[reg_idx]` into the buffer's WR_CFG payload slot.
pub fn update_wr_cfg(buf: &mut [u32; FRAME_WORDS], reg_idx: usize) {
    let reg = CONFIG_REGS[reg_idx];
    pack_shift128(
        &mut buf[WR_CFG_OFFSET + 1..WR_CFG_OFFSET + 1 + WR_CFG_WORDS],
        reg,
        ALL_RGB,
    );
}

