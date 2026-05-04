//! Conway's Game of Life on the HUB75 panel via USB.
//!
//! Each living cell has its own hue. When a new cell is born, it inherits
//! the average hue of its neighbours plus a small shift — so interacting
//! regions develop their own colour character over time.
//!
//! Stagnation handling: each generation we count pixels whose state
//! matches the state from `HISTORY_K` generations ago. With `K = 6`,
//! period-1, period-2 and period-3 cycles all yield ~100% pixel-level
//! match (LCM(1,2,3) = 6); a healthy simulation matches far less.
//! When the match exceeds the threshold for a few consecutive
//! generations we sprinkle in random "particles" to perturb the field
//! back into evolution.
//!
//! Run:  cargo run --example life --features panel-WxH

use hub75_client::{Hub75Client, WIDTH, HEIGHT};
use std::time::{Duration, Instant};

const W: usize = WIDTH;
const H: usize = HEIGHT;

const HISTORY_K: usize = 6;
const STAGNANT_PCT_THRESHOLD: usize = 95;
const STAGNANT_PERSIST: u32 = 3;
const PARTICLE_COUNT: usize = 12;

/// Hue shift applied to newborn cells, on top of their neighbour-average
/// hue. Higher = field cycles through the colour wheel faster (more
/// colours visible per minute, less time stuck in any one hue band).
const NEWBORN_HUE_SHIFT: u8 = 10;
/// Nudge magnitude for surviving cells toward their neighbours' average
/// hue per generation. Higher = faster spatial smoothing of hue regions.
const SURVIVOR_NUDGE: u8 = 1;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = std::env::args().nth(1);
    let mut client = match port {
        Some(p) => Hub75Client::open(&p)?,
        None => Hub75Client::open_auto()?,
    };

    println!("Connected. Running Life on {}×{}. Ctrl+C to stop.", W, H);

    // RGB sanity check: 1 s of solid yellow, then 1 s of solid green.
    // Yellow is [255, 255, 0]; if R/B are swapped on the panel it'll
    // render as cyan. Green is [0, 255, 0] — should be unmistakable.
    println!("  RGB sanity: 1 s yellow, 1 s green");
    let yellow = [[255u8, 255, 0]; W * H];
    let green = [[0u8, 255, 0]; W * H];
    client.send_frame_rgb(&yellow)?;
    std::thread::sleep(Duration::from_secs(1));
    client.send_frame_rgb(&green)?;
    std::thread::sleep(Duration::from_secs(1));

    let (mut alive, mut hues) = random_grid(0.4);
    let mut gen: u64 = 0;
    let mut frame_buf = [[0u8; 3]; W * H];
    let mut fps_timer = Instant::now();
    let mut fps_count: u32 = 0;
    let frame_interval = Duration::from_millis(56); // ~18 fps (USB throughput limit)

    let mut history: Vec<Vec<bool>> = (0..HISTORY_K).map(|_| vec![false; W * H]).collect();
    let mut history_idx: usize = 0;
    let mut warmup_remaining: usize = HISTORY_K;
    let mut stagnant_streak: u32 = 0;
    let mut rng_seed: u64 = seed_from_clock();

    loop {
        let frame_start = Instant::now();
        // Render — each cell shows its own hue
        for i in 0..W * H {
            frame_buf[i] = if alive[i] { hsv(hues[i]) } else { [0, 0, 0] };
        }

        client.send_frame_rgb(&frame_buf)?;
        fps_count += 1;

        // Compare current state to the K-gens-ago snapshot. The pixel
        // count of matches is the stagnation metric.
        let snapshot = &history[history_idx];
        let stagnant_pixels = (0..W * H).filter(|&i| alive[i] == snapshot[i]).count();
        let stagnant_pct = stagnant_pixels * 100 / (W * H);

        let elapsed = fps_timer.elapsed();
        if elapsed >= Duration::from_secs(1) {
            // Population hue stats over alive cells. The mean is taken on the unit
            // circle so the wrap-around at 0/255 doesn't cancel things out — without
            // this, a field full of red (hues clustered around 0 and 255) would
            // average to "cyan".
            let mut sx: f64 = 0.0;
            let mut sy: f64 = 0.0;
            let mut alive_n: u32 = 0;
            for i in 0..W * H {
                if alive[i] {
                    let theta = hues[i] as f64 * core::f64::consts::TAU / 256.0;
                    sx += theta.cos();
                    sy += theta.sin();
                    alive_n += 1;
                }
            }
            let mean_hue = if alive_n > 0 {
                let mut t = sy.atan2(sx);
                if t < 0.0 { t += core::f64::consts::TAU; }
                ((t * 256.0 / core::f64::consts::TAU) as u32) as u8
            } else { 0 };

            println!(
                "  {:.1} fps, gen {}, alive {}, mean_hue {}, stagnant {}/{} ({}%)",
                fps_count as f64 / elapsed.as_secs_f64(),
                gen,
                alive_n,
                mean_hue,
                stagnant_pixels,
                W * H,
                stagnant_pct,
            );
            fps_count = 0;
            fps_timer = Instant::now();
        }

        if warmup_remaining > 0 {
            warmup_remaining -= 1;
        } else if stagnant_pct >= STAGNANT_PCT_THRESHOLD {
            stagnant_streak += 1;
            if stagnant_streak >= STAGNANT_PERSIST {
                inject_particles(&mut alive, &mut hues, PARTICLE_COUNT, &mut rng_seed);
                stagnant_streak = 0;
                warmup_remaining = HISTORY_K;
                println!("  injected {} particles at gen {}", PARTICLE_COUNT, gen);
            }
        } else {
            stagnant_streak = 0;
        }

        // Overwrite the oldest history slot with the current state and
        // advance the ring.
        history[history_idx].copy_from_slice(&alive);
        history_idx = (history_idx + 1) % HISTORY_K;

        step(&mut alive, &mut hues);
        gen += 1;

        let elapsed = frame_start.elapsed();
        if elapsed < frame_interval {
            std::thread::sleep(frame_interval - elapsed);
        }
    }
}

fn seed_from_clock() -> u64 {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    use std::time::SystemTime;
    let mut h = DefaultHasher::new();
    SystemTime::now().hash(&mut h);
    h.finish()
}

fn xorshift(seed: &mut u64) -> u64 {
    *seed ^= *seed << 13;
    *seed ^= *seed >> 7;
    *seed ^= *seed << 17;
    *seed
}

fn inject_particles(alive: &mut [bool], hues: &mut [u8], count: usize, seed: &mut u64) {
    for _ in 0..count {
        let idx = (xorshift(seed) as usize) % (W * H);
        alive[idx] = true;
        hues[idx] = (xorshift(seed) >> 8) as u8;
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
                    // Newborn: inherit average neighbour hue plus the
                    // forward drift constant.
                    hues[i] = ((hue_sum / hue_count) as u8).wrapping_add(NEWBORN_HUE_SHIFT);
                } else if was_alive && hue_count > 0 {
                    // Surviving: nudge slightly toward neighbours.
                    let avg = (hue_sum / hue_count) as u8;
                    let current = prev_hues[i];
                    let diff = avg.wrapping_sub(current);
                    if diff != 0 {
                        let step = if diff < 128 {
                            SURVIVOR_NUDGE
                        } else {
                            (256u16 - SURVIVOR_NUDGE as u16) as u8
                        };
                        hues[i] = current.wrapping_add(step);
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
    // G and B output channels are swapped from the standard HSV
    // mapping — produces the swapped palette the user prefers as
    // life's deliberate look, regardless of the underlying panel's
    // wiring.
    match region {
        0 => [255, 0, t],
        1 => [q, 0, 255],
        2 => [0, t, 255],
        3 => [0, 255, q],
        4 => [t, 255, 0],
        _ => [255, q, 0],
    }
}
