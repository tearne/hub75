//! RGB → DATA_LATCH bitplane pack for the DP3364S.
//!
//! Reads an `[[Rgb; WIDTH]; HEIGHT]` pixel grid and writes the 512
//! DATA_LATCH payloads into the frame buffer. The pack is chip-major:
//! for each of the 8 chips, load its 6 gamma-expanded u16 colour values
//! once, then emit the chip's 4 DATA_LATCH output words via the
//! `SCATTER` LUT — no per-pixel bit-extraction loop. ~4.9 ms per frame
//! on a 64×128 panel at 150 MHz.

use super::framing::{
    latch_header_offset, FRAME_WORDS, LATCHES_PER_LINE, SCAN_LINES,
};
use crate::dp3364s::{HEIGHT, WIDTH};
use crate::panel::Rgb;

// ── Gamma LUT: 8-bit -> 14-bit greyscale ────────────────────────────
//
// Quadratic weighting. Linear and cubic alternatives were measured
// against a gradient test pattern; neither improved dim-end step
// visibility because gamma choice only redistributes the chip's 14-bit
// PWM range, it doesn't add resolution.
static GAMMA14: [u16; 256] = {
    let mut lut = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        lut[i] = ((i as u32 * i as u32 * 0x3FFF) / (255 * 255)) as u16;
        i += 1;
    }
    lut
};

// ── Nibble-scatter LUT ──────────────────────────────────────────────
//
// For a 4-bit input `x = abcd` (MSB `a` = bit3 of nibble), returns
// `(a << 0) | (b << 8) | (c << 16) | (d << 24)` — scatters the
// nibble's four bits to bit-0 of each of the u32's four bytes, in the
// byte order the DATA_LATCH word expects (MSB of nibble = earliest-
// clocked pixel = byte 0).
#[rustfmt::skip]
const SCATTER: [u32; 16] = [
    0x00000000, 0x01000000, 0x00010000, 0x01010000,
    0x00000100, 0x01000100, 0x00010100, 0x01010100,
    0x00000001, 0x01000001, 0x00010001, 0x01010001,
    0x00000101, 0x01000101, 0x00010101, 0x01010101,
];

/// Pack the pixel grid into the frame buffer's DATA_LATCH regions.
pub fn pack_pixels(
    pixels: &[[Rgb; WIDTH]; HEIGHT],
    buf: &mut [u32; FRAME_WORDS],
) {
    for scan_line in 0..SCAN_LINES {
        let upper_row = scan_line;
        let lower_row = scan_line + 32;

        for channel in 0..LATCHES_PER_LINE {
            let base = latch_header_offset(scan_line, channel) + 1;
            let output = 15 - channel;

            // chip 7 → words 0..3, chip 6 → 4..7, … chip 0 → 28..31.
            for chip in 0..8usize {
                let col = chip * 16 + output;
                // The 128×64 panel we tested has its column order
                // reversed on the chip-to-pixel wiring (clock runs
                // backwards, text appears mirrored). Read the source
                // pixel from the mirrored x-coordinate to compensate.
                #[cfg(not(feature = "panel-spwm-128x64"))]
                let read_col = col;
                #[cfg(feature = "panel-spwm-128x64")]
                let read_col = WIDTH - 1 - col;
                let u = pixels[upper_row][read_col];
                let l = pixels[lower_row][read_col];
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
                    let word = (SCATTER[((ur >> shift) & 0xF) as usize])
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
