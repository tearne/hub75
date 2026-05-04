//! Mondaine-style Swiss Railway Clock on the HUB75 panel via USB.
//!
//! Fetches time from an NTP server and displays an analogue clock face
//! with a sweeping second hand. The classic design is inverted for the
//! LED display: black background, white markers, red second hand with
//! the iconic lollipop tip.
//!
//! Run:  cargo run --example clock

use hub75_client::{Hub75Client, WIDTH, HEIGHT};
use chrono::{Local, Timelike};
use std::time::{Duration, Instant};

const W: usize = WIDTH;
const H: usize = HEIGHT;
const CX: f32 = (W as f32 - 1.0) / 2.0; // 31.5
const CY: f32 = (H as f32 - 1.0) / 2.0; // 31.5
const RADIUS: f32 = 32.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = std::env::args().nth(1);
    let mut client = match port {
        Some(p) => Hub75Client::open(&p)?,
        None => Hub75Client::open_auto()?,
    };

    println!("Connected. Showing clock. Ctrl+C to stop.");

    let mut frame_buf = [[0u8; 3]; W * H];
    let frame_interval = Duration::from_millis(33); // ~30 fps for smooth second hand

    loop {
        let frame_start = Instant::now();

        // Get local time
        let now = Local::now();
        let h = now.time().hour() as f32;
        let m = now.time().minute() as f32;
        let s = now.time().second() as f32 + now.timestamp_subsec_millis() as f32 / 1000.0;
        let hours = h + m / 60.0 + s / 3600.0;
        let minutes = m + s / 60.0;
        let seconds = s;

        // Clear to black
        for px in frame_buf.iter_mut() {
            *px = [0, 0, 0];
        }

        // Draw clock face
        draw_hour_markers(&mut frame_buf);
        draw_minute_markers(&mut frame_buf);

        // Draw hands (hour and minute are white, second is red)
        let hour_angle = (hours % 12.0) / 12.0 * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
        let min_angle = minutes / 60.0 * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
        let sec_angle = seconds / 60.0 * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;

        draw_hand(&mut frame_buf, hour_angle, RADIUS * 0.58, 2.0, 4.0, [255, 255, 255]);
        draw_hand(&mut frame_buf, min_angle, RADIUS * 0.85, 1.5, 7.0, [255, 255, 255]);
        // Second hand drawn last so it appears on top of everything
        draw_second_hand(&mut frame_buf, sec_angle);

        client.send_frame_rgb(&frame_buf)?;

        let elapsed = frame_start.elapsed();
        if elapsed < frame_interval {
            std::thread::sleep(frame_interval - elapsed);
        }
    }
}

// ── Drawing helpers ─────────────────────────────────────────────────

/// Plot a pixel with anti-aliasing (overwrite mode). Blends with black
/// rather than adding to existing colour, so it works on top of other elements.
fn set_pixel_over(buf: &mut [[u8; 3]; W * H], x: f32, y: f32, color: [u8; 3]) {
    let ix = x.floor() as i32;
    let iy = y.floor() as i32;
    let fx = x - ix as f32;
    let fy = y - iy as f32;

    let weights = [
        ((1.0 - fx) * (1.0 - fy), ix, iy),
        (fx * (1.0 - fy), ix + 1, iy),
        ((1.0 - fx) * fy, ix, iy + 1),
        (fx * fy, ix + 1, iy + 1),
    ];

    for (w, px, py) in weights {
        if w > 0.01 && px >= 0 && px < W as i32 && py >= 0 && py < H as i32 {
            let idx = py as usize * W + px as usize;
            let existing = buf[idx];
            // Blend: lerp from existing toward color by weight
            buf[idx] = [
                (existing[0] as f32 * (1.0 - w) + color[0] as f32 * w) as u8,
                (existing[1] as f32 * (1.0 - w) + color[1] as f32 * w) as u8,
                (existing[2] as f32 * (1.0 - w) + color[2] as f32 * w) as u8,
            ];
        }
    }
}

/// Plot a pixel with anti-aliasing (additive mode). Adds colour to existing
/// pixels — good for drawing on black backgrounds.
fn set_pixel(buf: &mut [[u8; 3]; W * H], x: f32, y: f32, color: [u8; 3]) {
    let ix = x.floor() as i32;
    let iy = y.floor() as i32;
    let fx = x - ix as f32;
    let fy = y - iy as f32;

    let weights = [
        ((1.0 - fx) * (1.0 - fy), ix, iy),
        (fx * (1.0 - fy), ix + 1, iy),
        ((1.0 - fx) * fy, ix, iy + 1),
        (fx * fy, ix + 1, iy + 1),
    ];

    for (w, px, py) in weights {
        if px >= 0 && px < W as i32 && py >= 0 && py < H as i32 {
            let idx = py as usize * W + px as usize;
            let existing = buf[idx];
            buf[idx] = [
                existing[0].saturating_add((color[0] as f32 * w) as u8),
                existing[1].saturating_add((color[1] as f32 * w) as u8),
                existing[2].saturating_add((color[2] as f32 * w) as u8),
            ];
        }
    }
}

fn set_disk(buf: &mut [[u8; 3]; W * H], cx: f32, cy: f32, r: f32, color: [u8; 3]) {
    let min_x = (cx - r - 1.0).floor() as i32;
    let max_x = (cx + r + 1.0).ceil() as i32;
    let min_y = (cy - r - 1.0).floor() as i32;
    let max_y = (cy + r + 1.0).ceil() as i32;
    for iy in min_y..=max_y {
        for ix in min_x..=max_x {
            if ix >= 0 && ix < W as i32 && iy >= 0 && iy < H as i32 {
                let dx = ix as f32 - cx;
                let dy = iy as f32 - cy;
                let dist = (dx * dx + dy * dy).sqrt();
                let alpha = (r - dist + 0.5).clamp(0.0, 1.0);
                if alpha > 0.0 {
                    let idx = iy as usize * W + ix as usize;
                    let existing = buf[idx];
                    buf[idx] = [
                        existing[0].saturating_add((color[0] as f32 * alpha) as u8),
                        existing[1].saturating_add((color[1] as f32 * alpha) as u8),
                        existing[2].saturating_add((color[2] as f32 * alpha) as u8),
                    ];
                }
            }
        }
    }
}

fn draw_thick_line(buf: &mut [[u8; 3]; W * H], x0: f32, y0: f32, x1: f32, y1: f32, thickness: f32, color: [u8; 3]) {
    let dx = x1 - x0;
    let dy = y1 - y0;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 0.001 { return; }
    let steps = (len * 3.0) as usize;
    let half = thickness / 2.0;

    // Normal perpendicular to the line
    let nx = -dy / len;
    let ny = dx / len;

    for i in 0..=steps {
        let t = i as f32 / steps as f32;
        let cx = x0 + dx * t;
        let cy = y0 + dy * t;
        // Fill across the thickness
        let substeps = (thickness * 2.0) as i32;
        for j in -substeps..=substeps {
            let s = j as f32 / (substeps as f32) * half;
            set_pixel(buf, cx + nx * s, cy + ny * s, color);
        }
    }
}

fn draw_face(buf: &mut [[u8; 3]; W * H]) {
    // Subtle grey circle outline
    let r = RADIUS + 1.5;
    for angle_step in 0..360 {
        let a = (angle_step as f32).to_radians();
        let x = CX + r * a.cos();
        let y = CY + r * a.sin();
        set_pixel(buf, x, y, [40, 40, 40]);
    }
}

fn draw_hour_markers(buf: &mut [[u8; 3]; W * H]) {
    for h in 0..12 {
        let angle = (h as f32) / 12.0 * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
        let inner = RADIUS * 0.75;
        let outer = RADIUS * 0.95;
        let x0 = CX + inner * angle.cos();
        let y0 = CY + inner * angle.sin();
        let x1 = CX + outer * angle.cos();
        let y1 = CY + outer * angle.sin();
        draw_thick_line(buf, x0, y0, x1, y1, 1.2, [255, 255, 255]);
    }
}

fn draw_minute_markers(buf: &mut [[u8; 3]; W * H]) {
    for m in 0..60 {
        if m % 5 == 0 { continue; } // skip hour positions
        let angle = (m as f32) / 60.0 * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
        let inner = RADIUS * 0.88;
        let outer = RADIUS * 0.95;
        let x0 = CX + inner * angle.cos();
        let y0 = CY + inner * angle.sin();
        let x1 = CX + outer * angle.cos();
        let y1 = CY + outer * angle.sin();
        set_pixel(buf, x0, y0, [200, 200, 200]);
        set_pixel(buf, x1, y1, [200, 200, 200]);
    }
}

fn draw_hand(buf: &mut [[u8; 3]; W * H], angle: f32, length: f32, thickness: f32, tail: f32, color: [u8; 3]) {
    let x0 = CX - tail * angle.cos();
    let y0 = CY - tail * angle.sin();
    let x1 = CX + length * angle.cos();
    let y1 = CY + length * angle.sin();
    draw_tapered_line(buf, x0, y0, x1, y1, thickness + 1.0, thickness, color);
}

fn draw_tapered_line(buf: &mut [[u8; 3]; W * H], x0: f32, y0: f32, x1: f32, y1: f32, thick_start: f32, thick_end: f32, color: [u8; 3]) {
    let dx = x1 - x0;
    let dy = y1 - y0;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 0.001 { return; }
    let steps = (len * 3.0) as usize;
    let nx = -dy / len;
    let ny = dx / len;
    for i in 0..=steps {
        let t = i as f32 / steps as f32;
        let cx = x0 + dx * t;
        let cy = y0 + dy * t;
        let half = (thick_start + (thick_end - thick_start) * t) / 2.0;
        let substeps = (half * 4.0) as i32;
        for j in -substeps..=substeps {
            let s = j as f32 / (substeps as f32) * half;
            set_pixel(buf, cx + nx * s, cy + ny * s, color);
        }
    }
}

fn draw_second_hand(buf: &mut [[u8; 3]; W * H], angle: f32) {
    let red = [255, 0, 0];

    // Thin line from behind centre to the lollipop (overwrite mode)
    let tail = 6.0;
    let lollipop_dist = RADIUS * 0.68;
    let x0 = CX - tail * angle.cos();
    let y0 = CY - tail * angle.sin();
    let x1 = CX + lollipop_dist * angle.cos();
    let y1 = CY + lollipop_dist * angle.sin();
    draw_thick_line_over(buf, x0, y0, x1, y1, 0.6, red);

    // Lollipop circle at the tip (overwrite mode)
    let lx = CX + lollipop_dist * angle.cos();
    let ly = CY + lollipop_dist * angle.sin();
    set_disk_over(buf, lx, ly, 3.0, red);
}

fn draw_thick_line_over(buf: &mut [[u8; 3]; W * H], x0: f32, y0: f32, x1: f32, y1: f32, thickness: f32, color: [u8; 3]) {
    let dx = x1 - x0;
    let dy = y1 - y0;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 0.001 { return; }
    let steps = (len * 3.0) as usize;
    let half = thickness / 2.0;
    let nx = -dy / len;
    let ny = dx / len;
    for i in 0..=steps {
        let t = i as f32 / steps as f32;
        let cx = x0 + dx * t;
        let cy = y0 + dy * t;
        let substeps = (thickness * 2.0) as i32;
        for j in -substeps..=substeps {
            let s = j as f32 / (substeps as f32) * half;
            set_pixel_over(buf, cx + nx * s, cy + ny * s, color);
        }
    }
}

fn set_disk_over(buf: &mut [[u8; 3]; W * H], cx: f32, cy: f32, r: f32, color: [u8; 3]) {
    let min_x = (cx - r - 1.0).floor() as i32;
    let max_x = (cx + r + 1.0).ceil() as i32;
    let min_y = (cy - r - 1.0).floor() as i32;
    let max_y = (cy + r + 1.0).ceil() as i32;
    for iy in min_y..=max_y {
        for ix in min_x..=max_x {
            if ix >= 0 && ix < W as i32 && iy >= 0 && iy < H as i32 {
                let dx = ix as f32 - cx;
                let dy = iy as f32 - cy;
                let dist = (dx * dx + dy * dy).sqrt();
                let alpha = (r - dist + 0.5).clamp(0.0, 1.0);
                if alpha > 0.0 {
                    let idx = iy as usize * W + ix as usize;
                    let existing = buf[idx];
                    buf[idx] = [
                        (existing[0] as f32 * (1.0 - alpha) + color[0] as f32 * alpha) as u8,
                        (existing[1] as f32 * (1.0 - alpha) + color[1] as f32 * alpha) as u8,
                        (existing[2] as f32 * (1.0 - alpha) + color[2] as f32 * alpha) as u8,
                    ];
                }
            }
        }
    }
}
