//! Projection: maps metric data into panel-coordinate space.
//!
//! Two stages on opposite sides of the slate:
//! - Row rendering (write path): metric value → pixel row → ring.
//! - Splat (read path): each ring entry painted at a continuous sub-pixel
//!   `y` with a vertical-Gaussian kernel, additively into the canvas.

use crate::presentation::{CORE_COUNT, LOGICAL_HEIGHT, LOGICAL_WIDTH, TOTAL_ROWS, cpu_core_left};
use crate::slate::{Pixel, Ring, Slate};

// ── Window-size curve ──────────────────────────────────────────────

/// Geometric base for the window-aggregation curve (1.07 scaled by 100).
const WINDOW_BASE_NUM: u64 = 107;
const WINDOW_BASE_DEN: u64 = 100;

const WINDOW_SIZES: [usize; TOTAL_ROWS] = compute_window_sizes();
const WINDOW_STARTS: [usize; TOTAL_ROWS] = compute_window_starts();
pub const RING_LENGTH: usize = WINDOW_STARTS[TOTAL_ROWS - 1] + WINDOW_SIZES[TOTAL_ROWS - 1];

const fn compute_window_sizes() -> [usize; TOTAL_ROWS] {
    let mut sizes = [1usize; TOTAL_ROWS];
    let scale: u64 = 1_000_000_000;
    let mut val: u64 = scale; // 1.07^0 = 1
    let mut k = 0;
    while k < TOTAL_ROWS {
        sizes[k] = ((val + scale - 1) / scale) as usize;
        val = val * WINDOW_BASE_NUM / WINDOW_BASE_DEN;
        k += 1;
    }
    sizes
}

const fn compute_window_starts() -> [usize; TOTAL_ROWS] {
    let sizes = compute_window_sizes();
    let mut starts = [0usize; TOTAL_ROWS];
    let mut k = 1;
    while k < TOTAL_ROWS {
        starts[k] = starts[k - 1] + sizes[k - 1];
        k += 1;
    }
    starts
}

// ── Row rendering ──────────────────────────────────────────────────

pub const CPU_COLOUR: Pixel = [50, 230, 230]; // cyan

/// Render one tick of a metric value into a pixel row of given width.
/// Pixels are binary on/off; lit pixels share `colour`. Lit columns are
/// chosen pseudo-randomly under `seed` so the same tick always produces
/// the same pattern.
pub fn render_row(value: f32, width: usize, colour: Pixel, seed: u32) -> Vec<Pixel> {
    let n = pixel_count(value, width);
    let mut row = vec![[0u8; 3]; width];
    for col in pick_columns(seed, width, n) {
        row[col] = colour;
    }
    row
}

/// Number of pixels to light for a fraction `v` in a width-`W` slice.
/// Any positive value lights at least one pixel; `v = 0` lights none.
fn pixel_count(value: f32, width: usize) -> usize {
    if value <= 0.0 { return 0; }
    ((value.clamp(0.0, 1.0) * width as f32).round() as usize).max(1)
}

/// Pick `n` distinct column indices in `[0, width)`, deterministic by seed.
fn pick_columns(seed: u32, width: usize, n: usize) -> Vec<usize> {
    let mut indices: Vec<(u32, usize)> =
        (0..width).map(|i| (mix32(seed, i as u32), i)).collect();
    indices.sort_unstable_by_key(|&(h, _)| h);
    indices.into_iter().take(n).map(|(_, i)| i).collect()
}

fn mix32(a: u32, b: u32) -> u32 {
    let mut x = a.wrapping_add(b.wrapping_mul(0x9e3779b9));
    x ^= x >> 16;
    x = x.wrapping_mul(0x85ebca6b);
    x ^= x >> 13;
    x.wrapping_mul(0xc2b2ae35) ^ (x >> 16)
}

// ── Splat ──────────────────────────────────────────────────────────

const SIGMA: f32 = 0.7;
const TWO_SIGMA_SQ: f32 = 2.0 * SIGMA * SIGMA;
const SPLAT_HALF_PIX: i32 = 1;

/// Brightness compensation per entry. Bottom rows have many entries
/// landing close together and would saturate without scaling; top rows
/// have one entry per row and need full strength. `0.5 / N(r)^0.65` sits
/// between the mean-uniform `1/N` and variance-uniform `1/√N` answers,
/// matching sysmon1's perceptual tuning. The `0.5` is global headroom.
const ALPHA_SCALE: f32 = 0.5;
const ALPHA_EXPONENT: f32 = 0.65;

/// Render the visible canvas (32 × 64) by splatting every ring entry
/// of every metric at its continuous sub-pixel `y`. `t` is the elapsed
/// fraction within the current master tick — between 0 and 1. As `t`
/// advances, every entry slides downward by `1/N(r)` of a row.
pub fn render_canvas(slate: &Slate, t: f32) -> Vec<Pixel> {
    let mut accumulator = vec![[0u32; 3]; LOGICAL_WIDTH * LOGICAL_HEIGHT];
    for core_idx in 0..CORE_COUNT {
        splat_ring(&slate.cpu[core_idx], cpu_core_left(core_idx), t, &mut accumulator);
    }
    pack_to_u8(&accumulator)
}

fn splat_ring(ring: &Ring, left: usize, t: f32, accumulator: &mut [[u32; 3]]) {
    for r in 0..TOTAL_ROWS {
        let n_r = WINDOW_SIZES[r] as f32;
        let alpha = ALPHA_SCALE / n_r.powf(ALPHA_EXPONENT);
        for w in 0..WINDOW_SIZES[r] {
            let age = WINDOW_STARTS[r] + w;
            let Some(row) = ring.get(age) else { return };
            let y = (r as f32 - 1.0) + (w as f32 + t) / n_r;
            splat_row_at_y(row, left, y, alpha, accumulator);
        }
    }
}

/// Paint a pre-laid pixel row at sub-pixel `y` with a vertical Gaussian
/// kernel of half-width `SPLAT_HALF_PIX`. Contributions add into the
/// accumulator; clipping to the visible canvas means rows splatted at
/// `y` outside `[0, LOGICAL_HEIGHT)` only contribute via the kernel's
/// tail to nearby visible rows.
fn splat_row_at_y(
    row: &[Pixel],
    left: usize,
    y: f32,
    alpha: f32,
    accumulator: &mut [[u32; 3]],
) {
    let cy = y.round() as i32;
    for dy in -SPLAT_HALF_PIX..=SPLAT_HALF_PIX {
        let py = cy + dy;
        if py < 0 || py >= LOGICAL_HEIGHT as i32 { continue; }
        let py = py as usize;
        let yd = py as f32 - y;
        let weight = (-yd * yd / TWO_SIGMA_SQ).exp() * alpha;
        for col in 0..row.len() {
            let pixel = row[col];
            if pixel == [0, 0, 0] { continue; }
            let idx = py * LOGICAL_WIDTH + (left + col);
            for c in 0..3 {
                let add = (pixel[c] as f32 * weight) as u32;
                accumulator[idx][c] = accumulator[idx][c].saturating_add(add);
            }
        }
    }
}

fn pack_to_u8(accumulator: &[[u32; 3]]) -> Vec<Pixel> {
    accumulator.iter().map(|p| [
        p[0].min(255) as u8,
        p[1].min(255) as u8,
        p[2].min(255) as u8,
    ]).collect()
}
