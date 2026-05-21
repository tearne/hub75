//! Minimal example: stream random dots to a 64x32 HUB75 panel.
//!
//! Every second, add a new random-coloured dot at a random position.
//! Keep at most 20 dots — when the queue is full, the oldest dot is
//! dropped to make room. Re-render and send every tick.
//!
//! Data flow, in one sentence:
//!   the panel is 64 columns × 32 rows; each frame is a 64*32 array of
//!   [r, g, b] triples (row-major, top-left origin); call
//!   `client.send_frame_rgb(&frame)` to push it.

use std::collections::VecDeque;
use std::thread;
use std::time::Duration;

use hub75_client::{Hub75Client, HEIGHT, WIDTH};
use rand::Rng;

const MAX_DOTS: usize = 20;
const TICK: Duration = Duration::from_secs(1);

struct Dot {
    x: usize,
    y: usize,
    rgb: [u8; 3],
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = Hub75Client::open(None)?;
    let mut dots: VecDeque<Dot> = VecDeque::with_capacity(MAX_DOTS);
    let mut frame = vec![[0u8; 3]; WIDTH * HEIGHT];

    println!("Streaming random dots to the panel. Ctrl+C to stop.");
    loop {
        if dots.len() == MAX_DOTS {
            dots.pop_front();
        }
        dots.push_back(make_random_dot());

        // Clear, then paint each dot.
        for pixel in frame.iter_mut() {
            *pixel = [0, 0, 0];
        }
        for dot in &dots {
            frame[dot.y * WIDTH + dot.x] = dot.rgb;
        }

        client.send_frame_rgb(&frame)?;
        thread::sleep(TICK);
    }
}

fn make_random_dot() -> Dot {
    let mut rng = rand::rng();
    Dot {
        x: rng.random_range(0..WIDTH),
        y: rng.random_range(0..HEIGHT),
        rgb: [rng.random(), rng.random(), rng.random()],
    }
}
