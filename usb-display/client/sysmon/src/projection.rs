//! Projection: per-row band rendering.
//!
//! Each metric's column slice is filled by rendering its `MetricBands`
//! top-to-bottom: band 0 occupies the top of the slice, band 1 the
//! next, and so on. Each row carries an `above_mean` flag, fixed at
//! commit time. Painting picks one of two HSL colour variants per
//! metric — both at the metric's hue and a fixed lightness, but
//! above-mean rows render at full saturation while below-mean rows
//! drop saturation. Rendering is integer-aligned — no splat, no
//! sub-pixel slide, no time-compression curve.

use crate::bands::{BAND_COUNT, MetricBands};
use crate::presentation::{
    CORE_COUNT, DISK_READ_LEFT, DISK_WRITE_LEFT, LOGICAL_HEIGHT, LOGICAL_WIDTH, NET_DOWN_LEFT,
    NET_UP_LEFT, RAM_LEFT, cpu_core_left,
};
use crate::slate::{Pixel, Slate};

/// Per-metric base colours, carried over from the original (pre-sysmon2)
/// sysmon palette. Hand-tuned RGB triplets — the high-saturation variant
/// renders these as-is; the low-saturation variant desaturates each
/// toward its own grey mid-point.
const METRIC_COLOURS: [Pixel; 6] = [
    /* DiskWrite */ [240,  50,  50], // red
    /* DiskRead  */ [255, 175,  40], // amber-gold
    /* NetDown   */ [ 40, 220,  90], // green
    /* Cpu       */ [ 50, 230, 230], // cyan
    /* NetUp     */ [200, 240,  40], // yellow-lime
    /* Ram       */ [230,  60, 170], // magenta
];

const IDX_DISK_WRITE: usize = 0;
const IDX_DISK_READ:  usize = 1;
const IDX_NET_DOWN:   usize = 2;
const IDX_CPU:        usize = 3;
const IDX_NET_UP:     usize = 4;
const IDX_RAM:        usize = 5;

/// Saturation factor for below-mean rows. 1.0 = original colour;
/// 0.0 = pure grey at the same brightness; 0.4 = noticeably muted
/// while still recognisably the metric's colour.
const SATURATION_LOW: f32 = 0.4;

const MIN_LED: f32 = 8.0;

/// Toggle: when true, fill each metric's column slice with a dim
/// background between dots. When false, unlit pixels stay black.
const BACKGROUND_ENABLED: bool = false;

/// Background dim level (only applied when `BACKGROUND_ENABLED`).
const BACKGROUND_DIM: f32 = 0.04;

/// Pair of colour variants for a metric: `(below_mean = low_sat, above_mean = high_sat)`.
type ColourPair = (Pixel, Pixel);

fn variants_for(metric_idx: usize) -> ColourPair {
    let high = METRIC_COLOURS[metric_idx];
    (desaturate(high, SATURATION_LOW), high)
}

fn desaturate(rgb: Pixel, factor: f32) -> Pixel {
    let avg = (rgb[0] as f32 + rgb[1] as f32 + rgb[2] as f32) / 3.0;
    [
        (avg + factor * (rgb[0] as f32 - avg)).clamp(0.0, 255.0) as u8,
        (avg + factor * (rgb[1] as f32 - avg)).clamp(0.0, 255.0) as u8,
        (avg + factor * (rgb[2] as f32 - avg)).clamp(0.0, 255.0) as u8,
    ]
}

pub fn render_canvas(slate: &Slate, shift: usize) -> Vec<Pixel> {
    let mut accumulator = vec![[0.0f32; 3]; LOGICAL_WIDTH * LOGICAL_HEIGHT];

    let cpu_pair = variants_for(IDX_CPU);
    for core_idx in 0..CORE_COUNT {
        render_metric(&slate.cpu[core_idx], cpu_core_left(core_idx), cpu_pair, &mut accumulator);
    }
    render_metric(&slate.ram,        RAM_LEFT,        variants_for(IDX_RAM),        &mut accumulator);
    render_metric(&slate.disk_read,  DISK_READ_LEFT,  variants_for(IDX_DISK_READ),  &mut accumulator);
    render_metric(&slate.disk_write, DISK_WRITE_LEFT, variants_for(IDX_DISK_WRITE), &mut accumulator);
    render_metric(&slate.net_down,   NET_DOWN_LEFT,   variants_for(IDX_NET_DOWN),   &mut accumulator);
    render_metric(&slate.net_up,     NET_UP_LEFT,     variants_for(IDX_NET_UP),     &mut accumulator);

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
    colours: ColourPair,
    accumulator: &mut [[f32; 3]],
) {
    let mut band_top = 0;
    for band_idx in 0..BAND_COUNT {
        let band = &bands.bands[band_idx];
        for row_idx in 0..band.height {
            let row = band.rows[row_idx];
            let panel_y = band_top + row_idx;
            if panel_y >= LOGICAL_HEIGHT { break; }
            paint_row(row, panel_y, left, bands.width, colours, accumulator);
        }
        band_top += band.height;
    }
}

/// Paint one band-row at `panel_y`. Lit dots use the warm variant if
/// the row sat at-or-above its band's prior mean, the cool variant if
/// below. Unlit columns get the dim background (if `BACKGROUND_ENABLED`)
/// or stay black.
fn paint_row(
    row: crate::bands::BandRow,
    panel_y: usize,
    left: usize,
    width: usize,
    colours: ColourPair,
    accumulator: &mut [[f32; 3]],
) {
    let has_dots = row.value > 0.0 && row.pattern != 0;
    let (cool, warm) = colours;
    let dot_colour = if row.above_mean { warm } else { cool };
    for col in 0..width {
        let bit = 1u8 << col;
        let lit = has_dots && (row.pattern & bit) != 0;
        let (intensity, colour) = if lit {
            (1.0, dot_colour)
        } else if BACKGROUND_ENABLED {
            (BACKGROUND_DIM, dot_colour)
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
