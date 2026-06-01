//! Minimal reference for sending frames to a HUB75 panel.
//!
//! The panel is 64x32. A frame is exactly 64*32 = 2048 `[r, g, b]` pixels, laid out
//! row by row starting top-left. You fill that array however you like, then hand it to
//! `client.send_frame_rgb(&frame)`. That is the whole protocol — everything below is
//! just an example of *what* to put in the array.
//!
//! This one keeps the last 20 random dots on screen, adding a fresh one twice a second.
//! If the panel is unplugged it waits and reconnects rather than exiting, so it can be
//! left running on a desk.

use hub75_client::{Error, Hub75Client};
use rand::Rng;
use std::collections::VecDeque;
use std::time::Duration;

const WIDTH: usize = 64;
const HEIGHT: usize = 32;
const DOT_COUNT: usize = 20;

/// Pick a dot at a random position with a random colour: `(x, y, [r, g, b])`.
fn make_random_dot() -> (usize, usize, [u8; 3]) {
    let mut rng = rand::rng();
    let x = rng.random_range(0..WIDTH);
    let y = rng.random_range(0..HEIGHT);
    let rgb = [rng.random(), rng.random(), rng.random()];
    (x, y, rgb)
}

/// Open the first panel, retrying once a second while none is present. A non-disconnect
/// error (e.g. a permissions problem) is fatal — there's nothing to wait for.
fn connect_panel() -> Hub75Client {
    loop {
        match Hub75Client::open(None) {
            Ok(client) => return client,
            Err(Error::Disconnected) => std::thread::sleep(Duration::from_secs(1)),
            Err(e) => panic!("cannot open panel: {e}"),
        }
    }
}

fn main() {
    let mut client = connect_panel();

    // The dots currently on screen, oldest at the front so it falls off first.
    let mut dots: VecDeque<(usize, usize, [u8; 3])> = VecDeque::with_capacity(DOT_COUNT);

    loop {
        dots.push_back(make_random_dot());
        if dots.len() > DOT_COUNT {
            dots.pop_front();
        }

        // A frame is WIDTH*HEIGHT pixels. Start all-black, then paint each dot into it.
        let mut frame = [[0u8; 3]; WIDTH * HEIGHT];
        for &(x, y, rgb) in &dots {
            frame[y * WIDTH + x] = rgb;
        }

        // Only `Disconnected` is worth recovering from — reconnect and carry on. Any
        // other error (e.g. a wrong-sized frame, which is a bug) should surface loudly
        // rather than be silently retried.
        match client.send_frame_rgb(&frame) {
            Ok(()) => {}
            Err(Error::Disconnected) => {
                eprintln!("panel disconnected — waiting for it to come back…");
                while client.reconnect().is_err() {
                    std::thread::sleep(Duration::from_secs(1));
                }
                eprintln!("panel reconnected.");
            }
            Err(e) => panic!("send failed: {e}"),
        }

        std::thread::sleep(Duration::from_millis(500));
    }
}
