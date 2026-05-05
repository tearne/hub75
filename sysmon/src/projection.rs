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

/// Two palettes for A/B comparison. `main.rs` flips between them every
/// wall-clock second. `PALETTE_A` is the baseline (currently the
/// original pre-sysmon2 hand-tuned RGB palette); `PALETTE_B` is the
/// experimental knob — edit it, rebuild, eyeball the alternation.
pub const PALETTE_A: [Pixel; 6] = [
    /* DiskWrite */ [240,  50,  50], // red
    /* DiskRead  */ [236, 156,  19], // amber (hue 38°, sat 0.85, light 0.50)
    /* NetDown   */ [ 16, 198,  68], // green (hue 137°, sat 0.85, light 0.42)
    /* Cpu       */ [ 11,  98, 218], // blue (hue 215°, sat 0.90, light 0.45)
    /* NetUp     */ [175, 215,  15], // lime (hue 72°, sat 0.87, light 0.45)
    /* Ram       */ [135,  25, 190], // bluer purple (hue 280°, sat 0.77, light 0.42)
];

#[allow(dead_code)]
pub const PALETTE_B: [Pixel; 6] = PALETTE_A;

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

/// Toggle: when true, fill each metric's column slice with a dim
/// background between dots. When false, unlit pixels stay black.
const BACKGROUND_ENABLED: bool = false;

/// Background dim level (only applied when `BACKGROUND_ENABLED`).
const BACKGROUND_DIM: f32 = 0.04;

/// Floor on per-row brightness scaling. Lit dots are dimmed by the
/// row's value so a low metric reads as quiet as well as sparse, but
/// they never drop below this fraction of full intensity — otherwise
/// the lone dot at value≈0 would be invisible.
const MIN_DOT_BRIGHTNESS: f32 = 0.1;

/// Pair of colour variants for a metric: `(below_mean = low_sat, above_mean = high_sat)`.
type ColourPair = (Pixel, Pixel);

fn variants_for(metric_idx: usize, palette: &[Pixel; 6]) -> ColourPair {
    let high = palette[metric_idx];
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

/// Owns the reusable buffers for the render pipeline. Allocate once
/// at startup; call `.render(...)` each frame to fill the panel-shaped
/// frame buffer in place. `&[Pixel]` returned borrows `self.frame`.
pub struct Renderer {
    canvas: Vec<Pixel>,
    frame: Vec<Pixel>,
}

impl Renderer {
    pub fn new() -> Self {
        let pixels = LOGICAL_WIDTH * LOGICAL_HEIGHT;
        Self {
            canvas: vec![[0u8; 3]; pixels],
            frame: vec![[0u8; 3]; pixels],
        }
    }

    pub fn render(
        &mut self,
        slate: &Slate,
        shift: usize,
        palette: &[Pixel; 6],
        label: char,
    ) -> &[Pixel] {
        self.canvas.fill([0u8; 3]);

        let cpu_pair = variants_for(IDX_CPU, palette);
        for core_idx in 0..CORE_COUNT {
            render_metric(&slate.cpu[core_idx], cpu_core_left(core_idx), cpu_pair, &mut self.canvas);
        }
        render_metric(&slate.ram,        RAM_LEFT,        variants_for(IDX_RAM,        palette), &mut self.canvas);
        render_metric(&slate.disk_read,  DISK_READ_LEFT,  variants_for(IDX_DISK_READ,  palette), &mut self.canvas);
        render_metric(&slate.disk_write, DISK_WRITE_LEFT, variants_for(IDX_DISK_WRITE, palette), &mut self.canvas);
        render_metric(&slate.net_down,   NET_DOWN_LEFT,   variants_for(IDX_NET_DOWN,   palette), &mut self.canvas);
        render_metric(&slate.net_up,     NET_UP_LEFT,     variants_for(IDX_NET_UP,     palette), &mut self.canvas);

        draw_label(&mut self.canvas, label);
        crate::display::shift_and_rotate(&self.canvas, &mut self.frame, shift);
        &self.frame
    }
}

/// Draw 'A' or 'B' as a 3×5 white pixel block at bottom-right of the
/// logical canvas. Pre-rotation, so coordinates are portrait
/// (LOGICAL_WIDTH cols × LOGICAL_HEIGHT rows).
fn draw_label(canvas: &mut [Pixel], label: char) {
    let glyph: [u8; 5] = match label {
        'A' => [0b010, 0b101, 0b111, 0b101, 0b101],
        'B' => [0b110, 0b101, 0b110, 0b101, 0b110],
        _   => return,
    };
    let left = LOGICAL_WIDTH - 3;
    let top  = LOGICAL_HEIGHT - 5;
    for (row_idx, bits) in glyph.iter().enumerate() {
        for col in 0..3 {
            let lit = bits & (1 << (2 - col)) != 0;
            if lit {
                let idx = (top + row_idx) * LOGICAL_WIDTH + (left + col);
                canvas[idx] = [255, 255, 255];
            }
        }
    }
}

fn render_metric(
    bands: &MetricBands,
    left: usize,
    colours: ColourPair,
    canvas: &mut [Pixel],
) {
    let mut band_top = 0;
    for band_idx in 0..BAND_COUNT {
        let band = &bands.bands[band_idx];
        for row_idx in 0..band.height {
            let row = band.rows[row_idx];
            let panel_y = band_top + row_idx;
            if panel_y >= LOGICAL_HEIGHT { break; }
            paint_row(row, panel_y, left, bands.width, colours, canvas);
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
    canvas: &mut [Pixel],
) {
    let has_dots = row.value > 0.0 && row.pattern != 0;
    let (cool, warm) = colours;
    let base_colour = if row.above_mean { warm } else { cool };
    let brightness = row.value.clamp(0.0, 1.0).max(MIN_DOT_BRIGHTNESS);
    let dot_colour = scale_pixel(base_colour, brightness);
    for col in 0..width {
        let bit = 1u8 << col;
        let lit = has_dots && (row.pattern & bit) != 0;
        let pixel = if lit {
            dot_colour
        } else if BACKGROUND_ENABLED {
            scale_pixel(base_colour, BACKGROUND_DIM)
        } else {
            continue;
        };
        let idx = panel_y * LOGICAL_WIDTH + (left + col);
        canvas[idx] = pixel;
    }
}

fn scale_pixel(rgb: Pixel, factor: f32) -> Pixel {
    [
        (rgb[0] as f32 * factor).clamp(0.0, 255.0) as u8,
        (rgb[1] as f32 * factor).clamp(0.0, 255.0) as u8,
        (rgb[2] as f32 * factor).clamp(0.0, 255.0) as u8,
    ]
}
