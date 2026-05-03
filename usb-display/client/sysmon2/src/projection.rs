//! Projection: maps metric data into panel-coordinate space.
//!
//! Two stages on opposite sides of the slate:
//! - Row rendering (write path): metric value → ring entry. Trivial:
//!   the value is stored directly, no pixel row built.
//! - Vertical mapping (read path): each ring entry's value is converted
//!   to a horizontal bar (deterministic, sub-pixel-precise) at render
//!   time, then splatted at a continuous sub-pixel `y` with a
//!   vertical-Gaussian kernel.

use std::time::{Duration, Instant};

use crate::presentation::{
    CORE_COUNT, CORE_WIDTH, DISK_READ_LEFT, DISK_WRITE_LEFT, HALF_WIDTH, LOGICAL_HEIGHT,
    LOGICAL_WIDTH, NET_DOWN_LEFT, NET_UP_LEFT, RAM_LEFT, TOTAL_ROWS, cpu_core_left,
};
use crate::slate::{
    CPU_MULTIPLIER, DISK_MULTIPLIER, NET_MULTIPLIER, Pixel, RAM_MULTIPLIER, RAM_RING_LENGTH,
    RamRing, Ring, Slate,
};

// ── Window-size curve ──────────────────────────────────────────────

/// Geometric base for the window-aggregation curve (1.10 scaled by 100).
const WINDOW_BASE_NUM: u64 = 110;
const WINDOW_BASE_DEN: u64 = 100;

const WINDOW_SIZES: [usize; TOTAL_ROWS] = compute_window_sizes();
const WINDOW_STARTS: [usize; TOTAL_ROWS] = compute_window_starts();
pub const RING_LENGTH: usize = WINDOW_STARTS[TOTAL_ROWS - 1] + WINDOW_SIZES[TOTAL_ROWS - 1];

const fn compute_window_sizes() -> [usize; TOTAL_ROWS] {
    let mut sizes = [1usize; TOTAL_ROWS];
    let scale: u64 = 1_000_000_000;
    let mut val: u64 = scale;
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

// ── Vertical mapping ───────────────────────────────────────────────

pub const CPU_COLOUR:        Pixel = [ 50, 230, 230]; // cyan
pub const RAM_COLOUR:        Pixel = [230,  60, 170]; // magenta
pub const DISK_READ_COLOUR:  Pixel = [255, 175,  40]; // amber-gold
pub const DISK_WRITE_COLOUR: Pixel = [240,  50,  50]; // red
pub const NET_DOWN_COLOUR:   Pixel = [ 40, 220,  90]; // green
pub const NET_UP_COLOUR:     Pixel = [200, 240,  40]; // yellow-lime

const SIGMA: f32 = 0.7;
const TWO_SIGMA_SQ: f32 = 2.0 * SIGMA * SIGMA;
const SPLAT_HALF_PIX: i32 = 1;

/// Brightness compensation per entry. Bottom rows have many entries
/// landing close together and would saturate without scaling. The 0.65
/// exponent sits between the mean-uniform `1/N` and variance-uniform
/// `1/√N` answers, matching sysmon1's perceptual tuning. The 0.5 is
/// global headroom.
const ALPHA_SCALE: f32 = 0.5;
const ALPHA_EXPONENT: f32 = 0.65;

/// LED hardware floor: when a pixel is sub-threshold but non-zero, the
/// pixel is scaled so its dominant channel reaches `MIN_LED`, with the
/// other channels scaling along to preserve the colour ratio. Zero
/// stays zero. Without the scale, lifting each channel independently
/// would wash sub-threshold pixels to grey.
const MIN_LED: f32 = 8.0;

/// Render the visible canvas (32 × 64) by splatting every ring entry
/// of every metric at its continuous sub-pixel `y`. Each metric's
/// elapsed-fraction `t` is computed against its own sample period
/// (`master_rate × multiplier`), so slow metrics interpolate over a
/// longer wall-clock interval — the slide rate per panel pixel stays
/// the same in panel coordinates.
pub fn render_canvas(slate: &Slate, master_rate: Duration, frame_at: Instant) -> Vec<Pixel> {
    let mut accumulator = vec![[0.0f32; 3]; LOGICAL_WIDTH * LOGICAL_HEIGHT];

    let cpu_t  = elapsed_fraction(frame_at, slate.last_cpu_sample_at,  master_rate * CPU_MULTIPLIER);
    let ram_t  = elapsed_fraction(frame_at, slate.last_ram_sample_at,  master_rate * RAM_MULTIPLIER);
    let disk_t = elapsed_fraction(frame_at, slate.last_disk_sample_at, master_rate * DISK_MULTIPLIER);
    let net_t  = elapsed_fraction(frame_at, slate.last_net_sample_at,  master_rate * NET_MULTIPLIER);

    for core_idx in 0..CORE_COUNT {
        splat_ring(&slate.cpu[core_idx], cpu_core_left(core_idx), CORE_WIDTH, CPU_COLOUR, cpu_t, &mut accumulator);
    }
    splat_ram_ring(&slate.ram, RAM_LEFT, ram_t, &mut accumulator);
    splat_ring(&slate.disk_read,  DISK_READ_LEFT,  HALF_WIDTH, DISK_READ_COLOUR,  disk_t, &mut accumulator);
    splat_ring(&slate.disk_write, DISK_WRITE_LEFT, HALF_WIDTH, DISK_WRITE_COLOUR, disk_t, &mut accumulator);
    splat_ring(&slate.net_down,   NET_DOWN_LEFT,   HALF_WIDTH, NET_DOWN_COLOUR,   net_t,  &mut accumulator);
    splat_ring(&slate.net_up,     NET_UP_LEFT,     HALF_WIDTH, NET_UP_COLOUR,     net_t,  &mut accumulator);

    pack_to_u8(&accumulator)
}

fn elapsed_fraction(now: Instant, since: Instant, period: Duration) -> f32 {
    if now <= since { return 0.0; }
    let elapsed = now.duration_since(since).as_secs_f32();
    (elapsed / period.as_secs_f32()).clamp(0.0, 1.0)
}

fn splat_ring(
    ring: &Ring,
    left: usize,
    width: usize,
    colour: Pixel,
    t: f32,
    accumulator: &mut [[f32; 3]],
) {
    for r in 0..TOTAL_ROWS {
        let n_r = WINDOW_SIZES[r] as f32;
        let alpha = ALPHA_SCALE / n_r.powf(ALPHA_EXPONENT);
        for w in 0..WINDOW_SIZES[r] {
            let age = WINDOW_STARTS[r] + w;
            let Some(value) = ring.get(age) else { return };
            let y = (r as f32 - 1.0) + (w as f32 + t) / n_r;
            splat_bar_at_y(value, left, width, colour, y, alpha, accumulator);
        }
    }
}

/// Paint a horizontal bar of length `value × width` at sub-pixel `y`
/// with a vertical Gaussian envelope. The bar grows symmetrically
/// outward from the centre of the column slice. Per-column intensity
/// is `overlap × value`, so the bar is both spatially shorter *and*
/// dimmer at low values rather than maxing out one column at a time.
fn splat_bar_at_y(
    value: f32,
    left: usize,
    width: usize,
    colour: Pixel,
    y: f32,
    alpha: f32,
    accumulator: &mut [[f32; 3]],
) {
    if value <= 0.0 { return; }
    let v = value.clamp(0.0, 1.0);
    let bar_half = v * width as f32 / 2.0;
    let centre = width as f32 / 2.0;
    let bar_start = centre - bar_half;
    let bar_end   = centre + bar_half;

    let cy = y.round() as i32;
    for dy in -SPLAT_HALF_PIX..=SPLAT_HALF_PIX {
        let py = cy + dy;
        if py < 0 || py >= LOGICAL_HEIGHT as i32 { continue; }
        let py = py as usize;
        let yd = py as f32 - y;
        let row_weight = (-yd * yd / TWO_SIGMA_SQ).exp() * alpha;
        for col in 0..width {
            let col_start = col as f32;
            let col_end   = col_start + 1.0;
            let overlap = (col_end.min(bar_end) - col_start.max(bar_start)).max(0.0);
            if overlap == 0.0 { continue; }
            let pixel_weight = overlap * v * row_weight;
            let idx = py * LOGICAL_WIDTH + (left + col);
            for c in 0..3 {
                accumulator[idx][c] += colour[c] as f32 * pixel_weight;
            }
        }
    }
}

// ── RAM (random-dot, linear-time) ──────────────────────────────────

/// RAM doesn't share the bar-fill or window-curve scheme. Each ring
/// entry is already a pre-laid pixel row of random dots; the splat
/// just paints it at the entry's panel-row index plus the elapsed
/// fraction `t` for sub-pixel sliding.
fn splat_ram_ring(
    ring: &RamRing,
    left: usize,
    t: f32,
    accumulator: &mut [[f32; 3]],
) {
    for i in 0..RAM_RING_LENGTH {
        let Some(row) = ring.get(i) else { return };
        let y = (i as f32 - 1.0) + t;
        splat_prelaid_row_at_y(row, left, y, RAM_ALPHA, accumulator);
    }
}

const RAM_ALPHA: f32 = 1.0;

/// Generate one RAM ring entry: a width-`W` pixel row with `n =
/// max(1, round(value × W))` randomly-placed lit pixels. Each lit
/// pixel's intensity is also scaled by `value`, so both dot count
/// and per-dot brightness rise with the metric — same visual axis
/// as the centred bars use for the other metrics. The seed makes
/// column choice stable per sample.
pub fn render_ram_row(value: f32, width: usize, colour: Pixel, seed: u32) -> Vec<Pixel> {
    let n = pixel_count(value, width);
    let v = value.clamp(0.0, 1.0);
    let scaled = [
        (colour[0] as f32 * v).round() as u8,
        (colour[1] as f32 * v).round() as u8,
        (colour[2] as f32 * v).round() as u8,
    ];
    let mut row = vec![[0u8; 3]; width];
    for col in pick_columns(seed, width, n) {
        row[col] = scaled;
    }
    row
}

fn pixel_count(value: f32, width: usize) -> usize {
    if value <= 0.0 { return 0; }
    ((value.clamp(0.0, 1.0) * width as f32).round() as usize).max(1)
}

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

fn splat_prelaid_row_at_y(
    row: &[Pixel],
    left: usize,
    y: f32,
    alpha: f32,
    accumulator: &mut [[f32; 3]],
) {
    let cy = y.round() as i32;
    for dy in -SPLAT_HALF_PIX..=SPLAT_HALF_PIX {
        let py = cy + dy;
        if py < 0 || py >= LOGICAL_HEIGHT as i32 { continue; }
        let py = py as usize;
        let yd = py as f32 - y;
        let row_weight = (-yd * yd / TWO_SIGMA_SQ).exp() * alpha;
        for col in 0..row.len() {
            let pixel = row[col];
            if pixel == [0, 0, 0] { continue; }
            let idx = py * LOGICAL_WIDTH + (left + col);
            for c in 0..3 {
                accumulator[idx][c] += pixel[c] as f32 * row_weight;
            }
        }
    }
}

// ── Output packing ─────────────────────────────────────────────────

fn pack_to_u8(accumulator: &[[f32; 3]]) -> Vec<Pixel> {
    accumulator.iter().map(|p| lift_pixel(*p)).collect()
}

/// If the pixel is sub-threshold but non-zero, scale so its dominant
/// channel reaches `MIN_LED` while preserving the colour ratio. True
/// black stays black; above-threshold pixels clamp normally.
fn lift_pixel(p: [f32; 3]) -> Pixel {
    let max = p[0].max(p[1]).max(p[2]);
    let scale = if max > 0.0 && max < MIN_LED { MIN_LED / max } else { 1.0 };
    [
        (p[0] * scale).clamp(0.0, 255.0) as u8,
        (p[1] * scale).clamp(0.0, 255.0) as u8,
        (p[2] * scale).clamp(0.0, 255.0) as u8,
    ]
}
