//! Pixel packing: RGB grid → bitplane DMA buffer with gamma applied.
//!
//! Layout: `[bitplane 0: row0..row(H/2-1)] [bitplane 1: ...] ... [bitplane 7]`.
//! Each row contains `W/4` u32 words; each word packs four pixels' worth of
//! 6 RGB-pin bits (R0 G0 B0 R1 G1 B1) for the current bitplane.

use crate::panel::Rgb;

use super::COLOR_DEPTH;

/// 8-bit isqrt-based gamma. Quadratic-ish curve compresses dim values
/// without losing range at the bright end. Carried over from the
/// `usb-display/firmware` implementation.
static GAMMA_LUT: [u8; 256] = {
    let mut lut = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        let isqrt = {
            let mut x = i;
            let mut y = (x + 1) / 2;
            while y < x {
                x = y;
                y = (x + i / x) / 2;
            }
            x
        };
        let v = (i * isqrt + 8) / 16;
        lut[i] = if v > 255 { 255 } else { v as u8 };
        i += 1;
    }
    lut
};

fn gamma(color: Rgb) -> Rgb {
    Rgb::new(
        GAMMA_LUT[color.r as usize],
        GAMMA_LUT[color.g as usize],
        GAMMA_LUT[color.b as usize],
    )
}

/// Pack `src` into the bitplane DMA buffer `dst`. The buffer is
/// `[[u32; W]; H]`-shaped so it can be const-generic; treat as flat
/// for indexing.
pub fn pack_pixels<const W: usize, const H: usize>(
    src: &[[Rgb; W]; H],
    dst: &mut [[u32; W]; H],
) {
    let row_pairs = H / 2;
    let words_per_row = W / 4;
    let words_per_bitplane = row_pairs * words_per_row;
    let flat = dst.as_flattened_mut();

    for bit in 0..COLOR_DEPTH {
        for row in 0..row_pairs {
            for word_idx in 0..words_per_row {
                let mut word: u32 = 0;
                for pix in 0..4 {
                    let col = word_idx * 4 + pix;
                    let upper = gamma(src[row][col]);
                    let lower = gamma(src[row + row_pairs][col]);

                    let r0 = ((upper.r >> bit) & 1) as u32;
                    let g0 = ((upper.g >> bit) & 1) as u32;
                    let b0 = ((upper.b >> bit) & 1) as u32;
                    let r1 = ((lower.r >> bit) & 1) as u32;
                    let g1 = ((lower.g >> bit) & 1) as u32;
                    let b1 = ((lower.b >> bit) & 1) as u32;

                    // GPIOs 0..5 wire to the panel's R0/G0/B0/R1/G1/B1
                    // shift inputs in that order. The 64×32 panel we
                    // tested has G and B swapped on its HUB75
                    // connector, so we drive what should be G onto
                    // pin 2 and what should be B onto pin 1 to
                    // compensate.
                    #[cfg(not(feature = "panel-shift-64x32"))]
                    let pixel_data =
                        r0 | (g0 << 1) | (b0 << 2) | (r1 << 3) | (g1 << 4) | (b1 << 5);
                    #[cfg(feature = "panel-shift-64x32")]
                    let pixel_data =
                        r0 | (b0 << 1) | (g0 << 2) | (r1 << 3) | (b1 << 4) | (g1 << 5);
                    word |= pixel_data << (pix * 8);
                }
                flat[bit * words_per_bitplane + row * words_per_row + word_idx] = word;
            }
        }
    }
}
