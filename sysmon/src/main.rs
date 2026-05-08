//! sysmon — HUB75 system monitor.
//!
//! See `map.md` for the conceptual model.

mod bands;
mod cpu;
mod display;
mod oklch;
mod presentation;
mod projection;
mod ram;
mod slate;
mod throughput;

use std::error::Error;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

use hub75_client::Hub75Client;

use crate::cpu::CpuSampler;
use crate::presentation::{CORE_COUNT, LOGICAL_WIDTH};
use crate::projection::{PALETTE_A, PALETTE_B, Renderer};
use crate::ram::RamSampler;
use crate::slate::Slate;
use crate::throughput::{ThroughputSampler, disk_sampler, net_sampler};

/// Toggle to compare two palettes side-by-side: every 5 seconds the
/// renderer flips between `PALETTE_A` and `PALETTE_B` and labels the
/// frame with `A`/`B`. When false, only `PALETTE_A` is used and the
/// label is blank. Edit `PALETTE_B` in `projection.rs` before enabling.
/// See `map.md` "A/B Palette Mode".
const AB_PALETTE_MODE: bool = false;

/// Default cycle duration when `-f` is not passed.
const PROD_CYCLE: Duration = Duration::from_millis(1000);

fn main() -> Result<(), Box<dyn Error>> {
    let cycle = parse_cycle_arg();
    let mut slate = Slate::new();
    let mut cpu = CpuSampler::new()?;
    let ram = RamSampler::new();
    let mut disk = disk_sampler()?;
    let mut net = net_sampler()?;
    let mut panel = Hub75Client::open(Some("sysmon"))?;

    let hz = 1000.0 / cycle.as_millis() as f64;
    println!(
        "sysmon connected. {} ms cycle (~{:.1} Hz). Ctrl+C to stop.",
        cycle.as_millis(),
        hz,
    );

    let started_at = Instant::now();
    let initial_shift = random_initial_shift();
    let mut renderer = Renderer::new();
    loop {
        let cycle_start = Instant::now();
        sample_all(&mut cpu, &ram, &mut disk, &mut net, &mut slate);
        let elapsed_secs = started_at.elapsed().as_secs();
        let elapsed_quarter_hours = elapsed_secs / 900;
        let shift = (initial_shift + elapsed_quarter_hours as usize) % LOGICAL_WIDTH;

        let (palette, label) = if AB_PALETTE_MODE {
            if (elapsed_secs / 5) % 2 == 0 {
                (&PALETTE_A, 'A')
            } else {
                (&PALETTE_B, 'B')
            }
        } else {
            (&PALETTE_A, ' ')
        };

        let frame = renderer.render(&slate, shift, palette, label);
        panel.send_frame_rgb(frame)?;
        match cycle.checked_sub(cycle_start.elapsed()) {
            Some(rest) => thread::sleep(rest),
            None => {
                // Cycle overran its budget. Rate-limit the warn
                // to once per second so we don't flood logs if it sticks.
                static LAST_WARN: AtomicU64 = AtomicU64::new(0);
                let now_secs = started_at.elapsed().as_secs();
                if now_secs != LAST_WARN.load(Ordering::Relaxed) {
                    LAST_WARN.store(now_secs, Ordering::Relaxed);
                    eprintln!(
                        "warn: cycle overran {} ms budget (took {} ms)",
                        cycle.as_millis(),
                        cycle_start.elapsed().as_millis()
                    );
                }
            }
        }
    }
}

/// `-f <hz>` runs at the given frame rate; without it, prod cadence
/// (1 Hz). Examples: `sysmon -f 20`, `sysmon -f 33`. Non-integer
/// rates are accepted; the cycle duration is rounded to the nearest
/// millisecond.
fn parse_cycle_arg() -> Duration {
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "-f" {
            let value = args.next().unwrap_or_else(|| usage_exit("-f needs a frame rate (Hz)"));
            let hz: f64 = value.parse().unwrap_or_else(|_| {
                usage_exit(&format!("-f: '{value}' is not a number"))
            });
            if !(hz > 0.0 && hz <= 10_000.0) {
                usage_exit(&format!("-f: rate must be between 0 and 10000 Hz, got {hz}"));
            }
            let ms = (1000.0 / hz).round().max(1.0) as u64;
            return Duration::from_millis(ms);
        }
    }
    PROD_CYCLE
}

fn usage_exit(msg: &str) -> ! {
    eprintln!("error: {msg}");
    eprintln!("usage: sysmon [-f <hz>]   (e.g. -f 20 for 20 Hz; default: 1 Hz)");
    std::process::exit(2);
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
            // For palette tuning without real disk activity, swap
            // `read_frac` for `synthetic_fraction(now)` to inject a
            // pseudo-random fraction. See `map.md` "A/B Palette Mode".
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

/// Pseudo-random fraction in [0, 1] derived from the sample instant —
/// hashes the nanosecond component so successive samples don't
/// correlate. Used as a temporary stand-in for absent disk-read
/// activity during palette comparison; kept available for future
/// development sessions. See `map.md` "A/B Palette Mode".
#[allow(dead_code)]
fn synthetic_fraction(now: Instant) -> f32 {
    let nanos = now.elapsed().as_nanos() as u32;
    let mut x = nanos.wrapping_mul(0x9e3779b9);
    x ^= x >> 16;
    x = x.wrapping_mul(0x85ebca6b);
    x ^= x >> 13;
    let hashed = x.wrapping_mul(0xc2b2ae35) ^ (x >> 16);
    (hashed as f32) / (u32::MAX as f32)
}
