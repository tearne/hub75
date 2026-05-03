//! sysmon2 — first-principles HUB75 system monitor.
//!
//! See `map.md` for the conceptual model.

mod bands;
mod cpu;
mod display;
mod presentation;
mod projection;
mod ram;
mod slate;
mod throughput;

use std::error::Error;
use std::thread;
use std::time::{Duration, Instant, SystemTime};

use hub75_client::Hub75Client;

use crate::cpu::CpuSampler;
use crate::presentation::{CORE_COUNT, LOGICAL_WIDTH};
use crate::projection::render_canvas;
use crate::ram::RamSampler;
use crate::slate::Slate;
use crate::throughput::{ThroughputSampler, disk_sampler, net_sampler};

#[derive(Clone, Copy)]
struct Mode {
    name: &'static str,
    sampling_rate: Duration,
}

const DEFAULT_MODE: Mode = Mode {
    name: "prod",
    sampling_rate: Duration::from_millis(500),
};

const FAST: Mode = Mode {
    name: "fast",
    sampling_rate: Duration::from_millis(50),
};

fn main() -> Result<(), Box<dyn Error>> {
    let mode = parse_mode_arg();
    let mut slate = Slate::new();
    let mut cpu = CpuSampler::new()?;
    let ram = RamSampler::new();
    let mut disk = disk_sampler()?;
    let mut net = net_sampler()?;
    let mut panel = Hub75Client::open_auto()?;

    println!(
        "sysmon2 [{}] connected. Sampling rate {} ms. Ctrl+C to stop.",
        mode.name,
        mode.sampling_rate.as_millis(),
    );

    let started_at = Instant::now();
    let initial_shift = random_initial_shift();
    loop {
        let cycle_start = Instant::now();
        sample_all(&mut cpu, &ram, &mut disk, &mut net, &mut slate);
        let elapsed_hours = started_at.elapsed().as_secs() / 3600;
        let shift = (initial_shift + elapsed_hours as usize) % LOGICAL_WIDTH;
        let canvas = render_canvas(&slate, shift);
        let frame = display::rotate_to_panel(&canvas);
        panel.send_frame_rgb(&frame)?;
        if let Some(rest) = mode.sampling_rate.checked_sub(cycle_start.elapsed()) {
            thread::sleep(rest);
        }
    }
}

/// Default mode is production (slow, lower CPU). `-f` selects fast
/// (development) — quicker scroll for visual tuning at the cost of CPU.
fn parse_mode_arg() -> Mode {
    if std::env::args().skip(1).any(|a| a == "-f") { FAST } else { DEFAULT_MODE }
}

/// Pick a random starting column offset for the screen-burn shift.
/// Different boots start at different positions in the 32-hour cycle,
/// so per-LED aging is averaged across reboots too. Seeded from
/// wall-clock nanoseconds — no proper RNG needed.
fn random_initial_shift() -> usize {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    (nanos as usize) % LOGICAL_WIDTH
}

fn sample_all(
    cpu: &mut CpuSampler,
    ram: &RamSampler,
    disk: &mut ThroughputSampler,
    net: &mut ThroughputSampler,
    slate: &mut Slate,
) {
    match cpu.sample() {
        Ok(busy) => {
            for core_idx in 0..CORE_COUNT {
                slate.cpu[core_idx].push_sample(busy[core_idx]);
            }
        }
        Err(e) => eprintln!("CPU sample failed: {e}"),
    }
    match ram.sample() {
        Ok(used) => slate.ram.push_sample(used),
        Err(e) => eprintln!("RAM sample failed: {e}"),
    }
    let now = Instant::now();
    match disk.sample(now) {
        Ok((read_frac, write_frac)) => {
            slate.disk_read.push_sample(read_frac);
            slate.disk_write.push_sample(write_frac);
        }
        Err(e) => eprintln!("Disk sample failed: {e}"),
    }
    match net.sample(now) {
        Ok((rx_frac, tx_frac)) => {
            slate.net_down.push_sample(rx_frac);
            slate.net_up.push_sample(tx_frac);
        }
        Err(e) => eprintln!("Net sample failed: {e}"),
    }
}
