//! Conway's Game of Life on the HUB75 panel via USB.
//!
//! Each living cell has its own hue. When a new cell is born, it inherits
//! the average hue of its neighbours plus a small shift — so interacting
//! regions develop their own colour character over time.
//!
//! Run:  cargo run --example life

use hub75_client::{Hub75Client, WIDTH, HEIGHT};
use std::time::{Duration, Instant};

const W: usize = WIDTH;
const H: usize = HEIGHT;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = std::env::args().nth(1);
    let mut client = match port {
        Some(p) => Hub75Client::open(&p)?,
        None => Hub75Client::open_auto()?,
    };

    println!("Connected. Running Life. Ctrl+C to stop.");

    let (mut alive, mut hues) = random_grid(0.4);
    let mut gen: u64 = 0;
    let mut frame_buf = [[0u8; 3]; W * H];
    let mut fps_timer = Instant::now();
    let mut fps_count: u32 = 0;
    let frame_interval = Duration::from_millis(56); // ~18 fps (USB throughput limit)

    loop {
        let frame_start = Instant::now();
        // Render — each cell shows its own hue
        for i in 0..W * H {
            frame_buf[i] = if alive[i] { hsv(hues[i]) } else { [0, 0, 0] };
        }

        client.send_frame_rgb(&frame_buf)?;
        fps_count += 1;

        let elapsed = fps_timer.elapsed();
        if elapsed >= Duration::from_secs(1) {
            println!("  {:.1} fps, gen {}", fps_count as f64 / elapsed.as_secs_f64(), gen);
            fps_count = 0;
            fps_timer = Instant::now();
        }

        step(&mut alive, &mut hues);
        gen += 1;

        let elapsed = frame_start.elapsed();
        if elapsed < frame_interval {
            std::thread::sleep(frame_interval - elapsed);
        }
    }
}

fn random_grid(density: f64) -> (Vec<bool>, Vec<u8>) {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    use std::time::SystemTime;

    let mut h = DefaultHasher::new();
    SystemTime::now().hash(&mut h);
    let mut seed = h.finish();

    let mut alive = Vec::with_capacity(W * H);
    let mut hues = Vec::with_capacity(W * H);
    for _ in 0..W * H {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        alive.push((seed % 1000) < (density * 1000.0) as u64);
        hues.push((seed >> 8) as u8);
    }
    (alive, hues)
}

fn step(alive: &mut Vec<bool>, hues: &mut Vec<u8>) {
    let prev_alive = alive.clone();
    let prev_hues = hues.clone();

    for y in 0..H {
        for x in 0..W {
            let mut neighbours = 0u8;
            let mut hue_sum: u32 = 0;
            let mut hue_count: u32 = 0;

            for dy in [H - 1, 0, 1] {
                for dx in [W - 1, 0, 1] {
                    if dy == 0 && dx == 0 { continue; }
                    let ni = ((y + dy) % H) * W + ((x + dx) % W);
                    if prev_alive[ni] {
                        neighbours += 1;
                        hue_sum += prev_hues[ni] as u32;
                        hue_count += 1;
                    }
                }
            }

            let i = y * W + x;
            let was_alive = prev_alive[i];
            let now_alive = if was_alive {
                neighbours == 2 || neighbours == 3
            } else {
                neighbours == 3
            };

            alive[i] = now_alive;

            if now_alive {
                if !was_alive && hue_count > 0 {
                    // Newborn: average neighbour hue + a small shift
                    hues[i] = ((hue_sum / hue_count) as u8).wrapping_add(10);
                } else if was_alive && hue_count > 0 {
                    // Surviving: nudge slightly toward neighbours
                    let avg = (hue_sum / hue_count) as u8;
                    let current = prev_hues[i];
                    let diff = avg.wrapping_sub(current);
                    if diff != 0 {
                        hues[i] = current.wrapping_add(if diff < 128 { 3 } else { 253 });
                    }
                }
            }
        }
    }
}

fn hsv(hue: u8) -> [u8; 3] {
    let h = hue as u16;
    let region = h / 43;
    let remainder = ((h % 43) * 6).min(255);
    let q = (255u16.saturating_sub(remainder)) as u8;
    let t = remainder as u8;
    match region {
        0 => [255, t, 0],
        1 => [q, 255, 0],
        2 => [0, 255, t],
        3 => [0, q, 255],
        4 => [t, 0, 255],
        _ => [255, 0, q],
    }
}
