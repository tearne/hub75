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

use std::sync::OnceLock;

use crate::bands::{BAND_COUNT, MetricBands};
use crate::presentation::{
    CORE_COUNT, DISK_READ_LEFT, DISK_WRITE_LEFT, LOGICAL_HEIGHT, LOGICAL_WIDTH, NET_DOWN_LEFT,
    NET_UP_LEFT, RAM_LEFT, cpu_core_left,
};
use crate::slate::{Pixel, Slate};

/// Two palettes for A/B comparison. `main.rs` flips between them every
/// 5 seconds when `AB_PALETTE_MODE` is on. `PALETTE_A` is the
/// shipping baseline; `PALETTE_B` is the experimental knob — edit
/// it, rebuild, eyeball the alternation.
pub const PALETTE_A: [Pixel; 6] = [
    /* DiskWrite */ [220,  50,  50], // red (hue 0°, sat 0.63, light 0.53)
    /* DiskRead  */ [236, 156,  19], // amber (hue 38°, sat 0.85, light 0.50)
    /* NetDown   */ [  0, 153,   0], // rich green (hue 120°, sat 1.00, light 0.30)
    /* Cpu       */ [  0,  51, 153], // rich blue (hue 220°, sat 1.00, light 0.30)
    /* NetUp     */ [175, 215,  15], // lime (hue 72°, sat 0.87, light 0.45)
    /* Ram       */ [135,  25, 190], // bluer purple (hue 280°, sat 0.77, light 0.42)
];

/// Experiment: dim the lime / NetUp colour a touch — push to max
/// saturation (was 0.87) and drop lightness from 0.45 → 0.40.
#[allow(dead_code)]
pub const PALETTE_B: [Pixel; 6] = [
    /* DiskWrite */ [240,  50,  50], // unchanged
    /* DiskRead  */ [236, 156,  19], // unchanged
    /* NetDown   */ [  0, 153,   0], // unchanged from new A
    /* Cpu       */ [  0,  51, 153], // unchanged from new A
    /* NetUp     */ [163, 204,   0], // lime dimmer (hue 72°, sat 1.00, light 0.40)
    /* Ram       */ [135,  25, 190], // unchanged
];

const IDX_DISK_WRITE: usize = 0;
const IDX_DISK_READ:  usize = 1;
const IDX_NET_DOWN:   usize = 2;
const IDX_CPU:        usize = 3;
const IDX_NET_UP:     usize = 4;
const IDX_RAM:        usize = 5;

/// Chroma factor for below-mean rows (in OKLCh). 1.0 = original
/// colour; 0.0 = pure grey at the same lightness. At very low
/// chroma values the perceptual hue tilts (e.g. dark blue can read
/// as purple) so we keep this above 0.5 — high enough that the hue
/// is unambiguous, low enough to leave a clear above/below cue.
const SATURATION_LOW: f32 = 0.7;

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

/// Lower the OKLCh chroma by `factor` while keeping lightness and
/// hue exact. Perceptually uniform: a desaturated dark blue stays
/// blue rather than drifting toward purple as the average-blend
/// approach used to do at low lightnesses.
fn desaturate(rgb: Pixel, factor: f32) -> Pixel {
    let (l, c, h) = crate::oklch::srgb_to_oklch(rgb);
    crate::oklch::oklch_to_srgb(l, c * factor, h)
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

        crate::display::shift_and_rotate(&self.canvas, &mut self.frame, shift);
        draw_label_on_frame(&mut self.frame, label);
        &self.frame
    }
}

/// Draw 'A' (user-view bottom-left) or 'B' (user-view bottom-right)
/// as a 3×5 glyph. The side itself signals which palette is active,
/// so the flip is spottable at a glance.
///
/// Drawn *after* the screen-burn shift so the label stays put while
/// the data scrolls underneath (otherwise the 3-column glyph could
/// straddle the shift wrap boundary and split across edges).
///
/// Coordinate notes: the user views the panel rotated 90° CCW from
/// its native (PANEL_W × PANEL_H) orientation, so the rendered image
/// reads as portrait (LOGICAL_WIDTH × LOGICAL_HEIGHT). Both labels
/// live at panel column 0–4 (= user's bottom row); A sits at panel
/// rows 0–2 (= user's left), B at panel rows PANEL_H-3 .. PANEL_H-1
/// (= user's right).
fn draw_label_on_frame(frame: &mut [Pixel], label: char) {
    use hub75_client::{HEIGHT as PANEL_H, WIDTH as PANEL_W};
    let (glyph, base_y): ([u8; 5], usize) = match label {
        'A' => ([0b010, 0b101, 0b111, 0b101, 0b101], 0),
        'B' => ([0b110, 0b101, 0b110, 0b101, 0b110], PANEL_H - 3),
        _   => return,
    };
    for row_idx in 0..5 {
        for col in 0..3 {
            let lit = glyph[row_idx] & (1 << (2 - col)) != 0;
            if lit {
                let panel_x = 4 - row_idx;
                let panel_y = base_y + col;
                frame[panel_y * PANEL_W + panel_x] = [255, 255, 255];
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
        let idx = panel_y * LOGICAL_WIDTH + structured_column_map()[left + col];
        canvas[idx] = pixel;
    }
}

/// Layout: keep the original `<non-CPU> <CPU>` pair sequence, but
/// pull RAM out of its trailing strip and slot one of its four
/// columns after each `<non-CPU> <CPU>` pair as a single-column
/// divider. Pattern, panel-col → metric:
/// ```
///  0..2  DiskW          ← pair 1
///  3..6  CPU0
///    7   RAM[0]
///  8..10 DiskR          ← pair 2
/// 11..14 CPU1
///   15   RAM[1]
/// 16..18 NetD           ← pair 3
/// 19..22 CPU2
///   23   RAM[2]
/// 24..26 NetU           ← pair 4
/// 27..30 CPU3
///   31   RAM[3]
/// ```
/// Each non-CPU/CPU pair is visually framed by a single RAM dot,
/// turning RAM into a periodic accent rather than a strip.
fn structured_column_map() -> &'static [usize; LOGICAL_WIDTH] {
    static MAP: OnceLock<[usize; LOGICAL_WIDTH]> = OnceLock::new();
    MAP.get_or_init(|| {
        // panel_col → original (contiguous-layout) column.
        let panel_to_original: [usize; LOGICAL_WIDTH] = [
             0,  1,  2,             // DiskW
             3,  4,  5,  6,         // CPU0
            28,                     // RAM[0]
             7,  8,  9,             // DiskR
            10, 11, 12, 13,         // CPU1
            29,                     // RAM[1]
            14, 15, 16,             // NetD
            17, 18, 19, 20,         // CPU2
            30,                     // RAM[2]
            21, 22, 23,             // NetU
            24, 25, 26, 27,         // CPU3
            31,                     // RAM[3]
        ];
        let mut map = [0usize; LOGICAL_WIDTH];
        for (panel_col, &original_col) in panel_to_original.iter().enumerate() {
            map[original_col] = panel_col;
        }
        map
    })
}

fn scale_pixel(rgb: Pixel, factor: f32) -> Pixel {
    [
        (rgb[0] as f32 * factor).clamp(0.0, 255.0) as u8,
        (rgb[1] as f32 * factor).clamp(0.0, 255.0) as u8,
        (rgb[2] as f32 * factor).clamp(0.0, 255.0) as u8,
    ]
}
