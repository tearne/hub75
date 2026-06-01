//! Minimal reference for sending frames to a HUB75 panel.
//!
//! The panel is 64x32. A frame is exactly 64*32 = 2048 `[r, g, b]` pixels, laid out
//! row by row starting top-left. You fill that array however you like, then hand it to
//! `client.send_frame_rgb(&frame)`. That is the whole protocol — everything below is
//! just an example of *what* to put in the array.
//!
//! This one keeps the last 20 random dots on screen, adding a fresh one twice a second.
//! If the panel is unplugged it logs the event, waits, and reconnects rather than
//! exiting, so it can be left running on a desk.

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
/// error at startup (e.g. a permissions problem) is fatal — there's nothing to wait for.
fn connect_panel() -> Hub75Client {
    let mut announced_wait = false;
    loop {
        match Hub75Client::open(None) {
            Ok(client) => {
                println!("panel connected.");
                return client;
            }
            Err(Error::Disconnected) => {
                if !announced_wait {
                    println!("waiting for a panel to be plugged in…");
                    announced_wait = true;
                }
                std::thread::sleep(Duration::from_secs(1));
            }
            Err(e) => panic!("cannot open panel: {e}"),
        }
    }
}

/// Block until the panel comes back, retrying the in-place reconnect once a second.
fn wait_for_panel(client: &mut Hub75Client) {
    println!("panel disconnected — waiting for it to come back…");
    while client.reconnect().is_err() {
        std::thread::sleep(Duration::from_secs(1));
    }
    println!("panel reconnected.");
}

fn main() {
    // Label the terminal window (OSC title — honoured by Windows Terminal and most
    // modern terminals, ignored elsewhere) and print a banner, so the console isn't a
    // blank mystery window. The version comes from Cargo at build time.
    let version = env!("CARGO_PKG_VERSION");
    print!("\x1b]0;dots-64x32 v{version} — HUB75 panel demo\x07");
    println!("dots-64x32 v{version}: streaming random dots to a 64x32 panel. Ctrl+C to quit.");

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

        match client.send_frame_rgb(&frame) {
            Ok(()) => {}
            // The expected recoverable case: the panel went away.
            Err(Error::Disconnected) => wait_for_panel(&mut client),
            // Anything else is unexpected on a desk demo. Log exactly what it was — this
            // is how we'd spot a host (e.g. Windows) reporting an unplug as something
            // other than `Disconnected` — but keep running and try to recover rather
            // than exiting and closing the window.
            Err(other) => {
                eprintln!("unexpected send error: {other:?} — trying to reconnect");
                wait_for_panel(&mut client);
            }
        }

        std::thread::sleep(Duration::from_millis(500));
    }
}
