//! Smoke test for the firmware's button → host event channel.
//!
//! Opens the panel, blocks on `recv_event`, and prints each new packed
//! state byte plus the per-button transitions since the previous one.
//!
//! Run: `cargo run --release --example buttons --features panel-64x32`
//! Press A and B on the Interstate 75 board; each press/release should
//! produce a line. Ctrl+C to quit.

use std::time::Duration;

use hub75_client::Hub75Client;

const BUTTON_A_BIT: u8 = 0;
const BUTTON_B_BIT: u8 = 1;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = Hub75Client::open(None)?;
    println!("connected. Press buttons on the panel. Ctrl+C to quit.");

    let mut last: u8 = 0;
    loop {
        // Long timeout so we mostly sit idle; the firmware only writes
        // on state change, so each return is a real event.
        match client.recv_event(Duration::from_secs(60))? {
            Some(packed) => {
                let changed = packed ^ last;
                describe_transitions(changed, packed);
                last = packed;
            }
            None => {} // timeout — keep listening
        }
    }
}

fn describe_transitions(changed: u8, current: u8) {
    let mut parts = Vec::new();
    if changed & (1 << BUTTON_A_BIT) != 0 {
        parts.push(format!("A {}", if current & (1 << BUTTON_A_BIT) != 0 { "pressed" } else { "released" }));
    }
    if changed & (1 << BUTTON_B_BIT) != 0 {
        parts.push(format!("B {}", if current & (1 << BUTTON_B_BIT) != 0 { "pressed" } else { "released" }));
    }
    println!("0b{:08b}  {}", current, parts.join(", "));
}
