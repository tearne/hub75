//! sysmon2 — first-principles HUB75 system monitor.
//!
//! See `map.md` for the conceptual model. This vertical slice
//! implements only the four CPU-core metrics; other metrics are stubbed
//! and will follow once the architecture is validated on the panel.

mod cpu;
mod display;
mod presentation;
mod projection;
mod slate;

use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use hub75_client::Hub75Client;

use crate::cpu::CpuSampler;
use crate::presentation::CORE_COUNT;
use crate::projection::{CPU_COLOUR, render_canvas, render_row};
use crate::slate::Slate;

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
        let mut sample_no: u64 = 0;
        loop {
            thread::sleep(master_sampling_rate);
            sample_no = sample_no.wrapping_add(1);
            push_cpu_sample(&mut cpu, sample_no, &slate);
        }
    });
}

fn push_cpu_sample(cpu: &mut CpuSampler, sample_no: u64, slate: &Mutex<Slate>) {
    let busy = match cpu.sample() {
        Ok(b) => b,
        Err(e) => { eprintln!("CPU sample failed: {e}"); return; }
    };
    let now = Instant::now();
    let mut slate = slate.lock().unwrap();
    for core_idx in 0..CORE_COUNT {
        let seed = seed_for(sample_no, core_idx);
        let row = render_row(busy[core_idx], slate.cpu[core_idx].width(), CPU_COLOUR, seed);
        slate.cpu[core_idx].push(row);
    }
    slate.last_cpu_sample_at = now;
}

fn seed_for(sample_no: u64, core_idx: usize) -> u32 {
    let lo = sample_no as u32;
    let hi = (sample_no >> 32) as u32;
    lo ^ hi.rotate_left(13) ^ ((core_idx as u32).wrapping_mul(0x9e3779b9))
}

fn render_loop(panel: &mut Hub75Client, slate: Arc<Mutex<Slate>>, mode: Mode) -> Result<(), Box<dyn Error>> {
    loop {
        let frame_start = Instant::now();
        let canvas = {
            let slate = slate.lock().unwrap();
            let t = elapsed_fraction(frame_start, slate.last_cpu_sample_at, mode.master_sampling_rate);
            render_canvas(&slate, t)
        };
        let frame = display::rotate_to_panel(&canvas);
        panel.send_frame_rgb(&frame)?;
        if let Some(rest) = mode.render_interval.checked_sub(frame_start.elapsed()) {
            thread::sleep(rest);
        }
    }
}

fn elapsed_fraction(now: Instant, since: Instant, period: Duration) -> f32 {
    if now <= since { return 0.0; }
    let elapsed = now.duration_since(since).as_secs_f32();
    (elapsed / period.as_secs_f32()).clamp(0.0, 1.0)
}
