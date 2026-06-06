//! Minimal reference for sending frames to a HUB75 panel.
//!
//! The panel is 64x32. A frame is exactly 64*32 = 2048 `[r, g, b]` pixels, laid out
//! row by row starting top-left. You fill that array however you like, then hand it to
//! `client.send_frame_rgb(&frame)`. That is the whole protocol — everything below is
//! just an example of *what* to put in the array.
//!
//! This one keeps the last 20 random dots on screen, adding a fresh one twice a second.
//! Press button A to run a panel bring-up test: the whole panel goes pure red, green,
//! then blue for a second each, then the dots resume. If the panel is unplugged it logs
//! the event, waits, and reconnects rather than exiting, so it can be left on a desk.

use hub75_client::{Error, Hub75Client};
use rand::Rng;
use std::collections::VecDeque;
use std::time::Duration;

const WIDTH: usize = 64;
const HEIGHT: usize = 32;
const DOT_COUNT: usize = 20;

/// Bit 0 of the firmware's packed button byte is button A (see `usb/README.md`).
const BUTTON_A_BIT: u8 = 0;

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

/// Send one frame, recovering transparently if the panel was unplugged so the program
/// keeps running. Any error other than `Disconnected` is unexpected on a desk demo, so
/// it's logged before we wait and reconnect rather than swallowed silently.
fn show(client: &mut Hub75Client, frame: &[[u8; 3]]) {
    match client.send_frame_rgb(frame) {
        Ok(()) => {}
        Err(Error::Disconnected) => wait_for_panel(client),
        Err(other) => {
            eprintln!("unexpected send error: {other:?} — trying to reconnect");
            wait_for_panel(client);
        }
    }
}

/// Drain pending button events and report whether button A was *newly* pressed (a 0→1
/// edge). The firmware sends a byte only when the button state changes, so most calls
/// just return false. A 1 ms timeout keeps this effectively non-blocking.
fn button_a_pressed(client: &mut Hub75Client, last_state: &mut u8) -> bool {
    let mut pressed = false;
    while let Ok(Some(byte)) = client.recv_event(Duration::from_millis(1)) {
        let newly_pressed = byte & !*last_state;
        *last_state = byte;
        if newly_pressed & (1 << BUTTON_A_BIT) != 0 {
            pressed = true;
        }
    }
    pressed
}

/// Fill the whole panel pure red, then green, then blue — one second each — as a quick
/// visual check that every pixel lights and each colour channel is wired correctly.
fn run_rgb_test(client: &mut Hub75Client) {
    println!("RGB test: red, green, blue (1s each)");
    for colour in [[255, 0, 0], [0, 255, 0], [0, 0, 255]] {
        show(client, &[colour; WIDTH * HEIGHT]);
        std::thread::sleep(Duration::from_secs(1));
    }
}

fn main() {
    // Label the terminal window (OSC title — honoured by Windows Terminal and most
    // modern terminals, ignored elsewhere) and print a banner, so the console isn't a
    // blank mystery window. The version comes from Cargo at build time.
    let version = env!("CARGO_PKG_VERSION");
    print!("\x1b]0;dots-64x32 v{version} — HUB75 panel demo\x07");
    println!("dots-64x32 v{version}: streaming random dots to a 64x32 panel. Press A for an RGB test. Ctrl+C to quit.");

    let mut client = connect_panel();

    // The dots currently on screen, oldest at the front so it falls off first.
    let mut dots: VecDeque<(usize, usize, [u8; 3])> = VecDeque::with_capacity(DOT_COUNT);
    let mut button_state = 0u8;

    loop {
        if button_a_pressed(&mut client, &mut button_state) {
            run_rgb_test(&mut client);
        }

        dots.push_back(make_random_dot());
        if dots.len() > DOT_COUNT {
            dots.pop_front();
        }

        // A frame is WIDTH*HEIGHT pixels. Start all-black, then paint each dot into it.
        let mut frame = [[0u8; 3]; WIDTH * HEIGHT];
        for &(x, y, rgb) in &dots {
            frame[y * WIDTH + x] = rgb;
        }
        show(&mut client, &frame);

        std::thread::sleep(Duration::from_millis(500));
    }
}
