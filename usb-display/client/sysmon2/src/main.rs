//! sysmon2 — first-principles HUB75 system monitor.
//!
//! See `map.md` for the conceptual model.

mod cpu;
mod display;
mod presentation;
mod projection;
mod ram;
mod slate;
mod throughput;

use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use hub75_client::Hub75Client;

use crate::cpu::CpuSampler;
use crate::presentation::{CORE_COUNT, RAM_WIDTH};
use crate::projection::{RAM_COLOUR, render_canvas, render_ram_row};
use crate::ram::RamSampler;
use crate::slate::{DISK_MULTIPLIER, NET_MULTIPLIER, RAM_MULTIPLIER, Slate};
use crate::throughput::{ThroughputSampler, disk_sampler, net_sampler};

#[derive(Clone, Copy)]
struct Mode {
    name: &'static str,
    master_sampling_rate: Duration,
    render_interval: Duration,
}

const DEFAULT_MODE: Mode = Mode {
    name: "prod",
    master_sampling_rate: Duration::from_millis(1000),
    render_interval: Duration::from_millis(200),
};

const FAST: Mode = Mode {
    name: "fast",
    master_sampling_rate: Duration::from_millis(50),
    render_interval: Duration::from_millis(50),
};

fn main() -> Result<(), Box<dyn Error>> {
    let mode = parse_mode_arg();
    let slate = Arc::new(Mutex::new(Slate::new()));
    spawn_sampler(slate.clone(), mode.master_sampling_rate);

    let mut panel = Hub75Client::open_auto()?;
    println!(
        "sysmon2 [{}] connected. Master sampling rate {} ms, render every {} ms. Ctrl+C to stop.",
        mode.name,
        mode.master_sampling_rate.as_millis(),
        mode.render_interval.as_millis(),
    );
    render_loop(&mut panel, slate, mode)
}

/// Default mode is production (slow, lower CPU). `-f` selects fast
/// (development) — quicker scroll for visual tuning at the cost of CPU.
fn parse_mode_arg() -> Mode {
    if std::env::args().skip(1).any(|a| a == "-f") { FAST } else { DEFAULT_MODE }
}

fn spawn_sampler(slate: Arc<Mutex<Slate>>, master_sampling_rate: Duration) {
    thread::spawn(move || {
        let mut cpu = match CpuSampler::new() {
            Ok(s) => s,
            Err(e) => { eprintln!("CPU sampler init failed: {e}"); return; }
        };
        let ram = RamSampler::new();
        let mut disk = match disk_sampler() {
            Ok(s) => s,
            Err(e) => { eprintln!("Disk sampler init failed: {e}"); return; }
        };
        let mut net = match net_sampler() {
            Ok(s) => s,
            Err(e) => { eprintln!("Net sampler init failed: {e}"); return; }
        };
        let mut sample_no: u64 = 0;
        loop {
            thread::sleep(master_sampling_rate);
            sample_no = sample_no.wrapping_add(1);
            push_cpu_sample(&mut cpu, &slate);
            if sample_no % RAM_MULTIPLIER  as u64 == 0 { push_ram_sample(&ram, sample_no, &slate); }
            if sample_no % NET_MULTIPLIER  as u64 == 0 { push_net_sample(&mut net, &slate); }
            if sample_no % DISK_MULTIPLIER as u64 == 0 { push_disk_sample(&mut disk, &slate); }
        }
    });
}

fn push_cpu_sample(cpu: &mut CpuSampler, slate: &Mutex<Slate>) {
    let busy = match cpu.sample() {
        Ok(b) => b,
        Err(e) => { eprintln!("CPU sample failed: {e}"); return; }
    };
    let now = Instant::now();
    let mut slate = slate.lock().unwrap();
    for core_idx in 0..CORE_COUNT {
        slate.cpu[core_idx].push(busy[core_idx]);
    }
    slate.last_cpu_sample_at = now;
}

fn push_ram_sample(ram: &RamSampler, sample_no: u64, slate: &Mutex<Slate>) {
    let used = match ram.sample() {
        Ok(v) => v,
        Err(e) => { eprintln!("RAM sample failed: {e}"); return; }
    };
    let now = Instant::now();
    let row = render_ram_row(used, RAM_WIDTH, RAM_COLOUR, ram_seed(sample_no));
    let mut slate = slate.lock().unwrap();
    slate.ram.push(row);
    slate.last_ram_sample_at = now;
}

fn ram_seed(sample_no: u64) -> u32 {
    (sample_no as u32) ^ ((sample_no >> 32) as u32).rotate_left(13) ^ 0xA1B2C3D4
}

fn push_disk_sample(disk: &mut ThroughputSampler, slate: &Mutex<Slate>) {
    let now = Instant::now();
    let (read_frac, write_frac) = match disk.sample(now) {
        Ok(v) => v,
        Err(e) => { eprintln!("Disk sample failed: {e}"); return; }
    };
    let mut slate = slate.lock().unwrap();
    slate.disk_read.push(read_frac);
    slate.disk_write.push(write_frac);
    slate.last_disk_sample_at = now;
}

fn push_net_sample(net: &mut ThroughputSampler, slate: &Mutex<Slate>) {
    let now = Instant::now();
    let (rx_frac, tx_frac) = match net.sample(now) {
        Ok(v) => v,
        Err(e) => { eprintln!("Net sample failed: {e}"); return; }
    };
    let mut slate = slate.lock().unwrap();
    slate.net_down.push(rx_frac);
    slate.net_up.push(tx_frac);
    slate.last_net_sample_at = now;
}

fn render_loop(panel: &mut Hub75Client, slate: Arc<Mutex<Slate>>, mode: Mode) -> Result<(), Box<dyn Error>> {
    loop {
        let frame_start = Instant::now();
        let canvas = {
            let slate = slate.lock().unwrap();
            render_canvas(&slate, mode.master_sampling_rate, frame_start)
        };
        let frame = display::rotate_to_panel(&canvas);
        panel.send_frame_rgb(&frame)?;
        if let Some(rest) = mode.render_interval.checked_sub(frame_start.elapsed()) {
            thread::sleep(rest);
        }
    }
}
