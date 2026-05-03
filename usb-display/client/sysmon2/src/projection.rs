//! Projection: per-row band rendering.
//!
//! Each metric's column slice is filled by rendering its `MetricBands`
//! top-to-bottom: band 0 occupies the top BAND_HEIGHT rows of the
//! slice, band 1 the next BAND_HEIGHT, and so on. Each row is a
//! pre-laid pixel row (one random dot pattern, intensity-scaled by
//! the row's aggregated value). Rendering is integer-aligned — no
//! splat, no sub-pixel slide, no time-compression curve.

use crate::bands::{BAND_COUNT, BAND_HEIGHT, MetricBands};
use crate::presentation::{
    CORE_COUNT, DISK_READ_LEFT, DISK_WRITE_LEFT, LOGICAL_HEIGHT, LOGICAL_WIDTH, NET_DOWN_LEFT,
    NET_UP_LEFT, RAM_LEFT, cpu_core_left,
};
use crate::slate::{Pixel, Slate};

pub const CPU_COLOUR:        Pixel = [ 50, 230, 230]; // cyan
pub const RAM_COLOUR:        Pixel = [230,  60, 170]; // magenta
pub const DISK_READ_COLOUR:  Pixel = [255, 175,  40]; // amber-gold
pub const DISK_WRITE_COLOUR: Pixel = [240,  50,  50]; // red
pub const NET_DOWN_COLOUR:   Pixel = [ 40, 220,  90]; // green
pub const NET_UP_COLOUR:     Pixel = [200, 240,  40]; // yellow-lime

const MIN_LED: f32 = 8.0;

/// Toggle: when true, fill each metric's column slice with a dim
/// background between dots. When false, unlit pixels stay black.
const BACKGROUND_ENABLED: bool = false;

/// Background dim level (only applied when `BACKGROUND_ENABLED`).
const BACKGROUND_DIM: f32 = 0.04;

/// Floor for any lit dot — ensures the dimmest dot is clearly brighter
/// than the dim background (when on).
const DOT_FLOOR: f32 = 0.20;

pub fn render_canvas(slate: &Slate, shift: usize) -> Vec<Pixel> {
    let mut accumulator = vec![[0.0f32; 3]; LOGICAL_WIDTH * LOGICAL_HEIGHT];

    for core_idx in 0..CORE_COUNT {
        render_metric(&slate.cpu[core_idx], cpu_core_left(core_idx), CPU_COLOUR, &mut accumulator);
    }
    render_metric(&slate.ram,        RAM_LEFT,        RAM_COLOUR,        &mut accumulator);
    render_metric(&slate.disk_read,  DISK_READ_LEFT,  DISK_READ_COLOUR,  &mut accumulator);
    render_metric(&slate.disk_write, DISK_WRITE_LEFT, DISK_WRITE_COLOUR, &mut accumulator);
    render_metric(&slate.net_down,   NET_DOWN_LEFT,   NET_DOWN_COLOUR,   &mut accumulator);
    render_metric(&slate.net_up,     NET_UP_LEFT,     NET_UP_COLOUR,     &mut accumulator);

    let canvas = pack_to_u8(&accumulator);
    if shift == 0 { canvas } else { shift_horizontal(canvas, shift) }
}

/// Slide the canvas horizontally by `shift` columns (with wrap) in
/// logical space. Used for screen-burn mitigation — once per hour
/// of wall-clock the shift advances by 1 column, so over time every
/// LED experiences every metric.
fn shift_horizontal(canvas: Vec<Pixel>, shift: usize) -> Vec<Pixel> {
    let s = shift % LOGICAL_WIDTH;
    if s == 0 { return canvas; }
    let mut out = vec![[0u8; 3]; canvas.len()];
    for y in 0..LOGICAL_HEIGHT {
        for x in 0..LOGICAL_WIDTH {
            let src_x = (x + LOGICAL_WIDTH - s) % LOGICAL_WIDTH;
            out[y * LOGICAL_WIDTH + x] = canvas[y * LOGICAL_WIDTH + src_x];
        }
    }
    out
}

fn render_metric(
    bands: &MetricBands,
    left: usize,
    colour: Pixel,
    accumulator: &mut [[f32; 3]],
) {
    for band_idx in 0..BAND_COUNT {
        let band = &bands.bands[band_idx];
        let band_top = band_idx * BAND_HEIGHT;
        for row_idx in 0..BAND_HEIGHT {
            let row = band.rows[row_idx];
            let panel_y = band_top + row_idx;
            if panel_y >= LOGICAL_HEIGHT { break; }
            paint_row(row, panel_y, left, bands.width, colour, accumulator);
        }
    }
}

/// Paint one band-row at `panel_y`. Lit pattern columns get the
/// metric colour at intensity `max(value, DOT_FLOOR)`. Unlit columns
/// either get the dim background (if `BACKGROUND_ENABLED`) or stay
/// black.
fn paint_row(
    row: crate::bands::BandRow,
    panel_y: usize,
    left: usize,
    width: usize,
    colour: Pixel,
    accumulator: &mut [[f32; 3]],
) {
    let has_dots = row.value > 0.0 && row.pattern != 0;
    let dot_intensity = row.value.max(DOT_FLOOR);
    for col in 0..width {
        let bit = 1u8 << col;
        let lit = has_dots && (row.pattern & bit) != 0;
        let intensity = if lit {
            dot_intensity
        } else if BACKGROUND_ENABLED {
            BACKGROUND_DIM
        } else {
            continue;
        };
        let idx = panel_y * LOGICAL_WIDTH + (left + col);
        for c in 0..3 {
            accumulator[idx][c] += colour[c] as f32 * intensity;
        }
    }
}

fn pack_to_u8(accumulator: &[[f32; 3]]) -> Vec<Pixel> {
    accumulator.iter().map(|p| lift_pixel(*p)).collect()
}

/// If a pixel is sub-threshold but non-zero, scale so its dominant
/// channel reaches `MIN_LED`, with other channels scaling
/// proportionally to preserve the colour ratio.
fn lift_pixel(p: [f32; 3]) -> Pixel {
    let max = p[0].max(p[1]).max(p[2]);
    let scale = if max > 0.0 && max < MIN_LED { MIN_LED / max } else { 1.0 };
    [
        (p[0] * scale).clamp(0.0, 255.0) as u8,
        (p[1] * scale).clamp(0.0, 255.0) as u8,
        (p[2] * scale).clamp(0.0, 255.0) as u8,
    ]
}
