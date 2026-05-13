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
use crate::projection::{PALETTE_A, Renderer};
use crate::ram::RamSampler;
use crate::slate::{LayoutMode, Slates};
use crate::throughput::{ThroughputSampler, disk_sampler, net_sampler};

/// Bit position of button A in the packed button-state byte the
/// firmware sends over the bulk IN endpoint (see `usb-serial/README.md`).
/// Button B is currently unused by sysmon.
const BUTTON_A_BIT: u8 = 0;

/// Adaptive frame rate bounds when `-f` is not passed: cycle duration
/// scales with peak CPU busy, fastest at full load, slowest at idle.
const ADAPTIVE_MIN_HZ: f64 = 0.5;
const ADAPTIVE_MAX_HZ: f64 = 50.0;

/// Inertia time-constant for the adaptive ramp. Each cycle the
/// driving signal eases toward the latest peak-busy with
/// `alpha = 1 - exp(-dt/TAU)`, so the rate change settles over
/// ~`ADAPTIVE_TAU_SECS` rather than snapping per-sample.
const ADAPTIVE_TAU_SECS: f64 = 3.0;

/// Curve exponent mapping smoothed peak-busy to the [0, 1] fraction
/// used between `ADAPTIVE_MIN_HZ` and `ADAPTIVE_MAX_HZ`. With `> 1`,
/// low busy values stay close to min — the panel sits near the floor
/// at routine idle-ish loads and only ramps up sharply when the host
/// is genuinely busy.
const ADAPTIVE_CURVE: f64 = 3.0;

/// How long sysmon waits between reconnection attempts when the panel
/// is unreachable. Fast enough to feel responsive on a brief outage,
/// slow enough that a long absence doesn't spam rusb enumeration.
const RECONNECT_INTERVAL: Duration = Duration::from_secs(2);

const PANEL_SERIAL: &str = "sysmon";

fn main() -> Result<(), Box<dyn Error>> {
    let fixed_cycle = parse_cycle_arg();
    let mut slates = Slates::new(LayoutMode::FastOnly);
    let mut cpu = CpuSampler::new()?;
    let ram = RamSampler::new();
    let mut disk = disk_sampler()?;
    let mut net = net_sampler()?;

    let (mut panel, mut disconnected_at): (Option<Hub75Client>, Option<Instant>) =
        match Hub75Client::open(Some(PANEL_SERIAL)) {
            Ok(p) => (Some(p), None),
            Err(e) => {
                eprintln!("panel not available at startup ({e}); will keep retrying");
                (None, Some(Instant::now()))
            }
        };
    let mut last_reconnect_attempt = Instant::now();

    match fixed_cycle {
        Some(cycle) => println!(
            "sysmon starting. {} ms cycle (~{:.1} Hz, fixed). Ctrl+C to stop.",
            cycle.as_millis(),
            1000.0 / cycle.as_millis() as f64,
        ),
        None => println!(
            "sysmon starting. Adaptive {:.1}–{:.0} Hz on CPU load. Button A toggles layout. Ctrl+C to stop.",
            ADAPTIVE_MIN_HZ, ADAPTIVE_MAX_HZ,
        ),
    }

    let started_at = Instant::now();
    let initial_shift = random_initial_shift();
    let mut renderer = Renderer::new();
    let mut peak_busy: f32 = 0.0;
    // Start the smoothed signal at the busy end of the range so the
    // adaptive ramp opens at max Hz and eases toward idle — the panel
    // animates immediately at startup instead of waiting on the EMA
    // to climb out of zero at idle CPU.
    let mut smoothed_busy: f32 = 1.0;
    let mut last_sample_at: Option<Instant> = None;
    let mut last_button_state: u8 = 0;
    loop {
        let cycle_start = Instant::now();

        if panel.is_none() && cycle_start.duration_since(last_reconnect_attempt) >= RECONNECT_INTERVAL {
            last_reconnect_attempt = cycle_start;
            match Hub75Client::open(Some(PANEL_SERIAL)) {
                Ok(p) => {
                    let downtime = disconnected_at
                        .map(|t| cycle_start.duration_since(t))
                        .unwrap_or_default();
                    println!("panel reconnected after {:.1}s", downtime.as_secs_f64());
                    panel = Some(p);
                    disconnected_at = None;
                    // Firmware sends button bytes only on state change;
                    // it doesn't replay the current state on reconnect.
                    // Resync the host view so any held button doesn't
                    // register as a press edge on the next genuine
                    // transition.
                    last_button_state = 0;
                }
                Err(_) => {} // quiet — next attempt in RECONNECT_INTERVAL
            }
        }

        if let Some(p) = panel.as_mut() {
            match drain_button_events(p, last_button_state) {
                Ok((new_state, pressed_edges)) => {
                    last_button_state = new_state;
                    if pressed_edges & (1 << BUTTON_A_BIT) != 0 {
                        let now_active = slates.toggle();
                        println!("layout → {:?}", now_active);
                    }
                }
                Err(e) => {
                    eprintln!("panel disconnected: {e}");
                    panel = None;
                    disconnected_at = Some(cycle_start);
                    last_reconnect_attempt = cycle_start;
                }
            }
        }

        sample_all(&mut cpu, &ram, &mut disk, &mut net, &mut slates, &mut peak_busy);
        let dt_secs = last_sample_at
            .map(|t| cycle_start.duration_since(t).as_secs_f64())
            .unwrap_or(0.0);
        last_sample_at = Some(cycle_start);
        let alpha = 1.0 - (-dt_secs / ADAPTIVE_TAU_SECS).exp();
        smoothed_busy += (alpha as f32) * (peak_busy - smoothed_busy);
        let cycle = fixed_cycle.unwrap_or_else(|| adaptive_cycle(smoothed_busy));
        let elapsed_secs = started_at.elapsed().as_secs();
        let elapsed_quarter_hours = elapsed_secs / 900;
        let shift = (initial_shift + elapsed_quarter_hours as usize) % LOGICAL_WIDTH;

        if let Some(p) = panel.as_mut() {
            let frame = renderer.render(slates.active(), shift, &PALETTE_A, ' ');
            if let Err(e) = p.send_frame_rgb(frame) {
                eprintln!("panel disconnected: {e}");
                panel = None;
                disconnected_at = Some(cycle_start);
                last_reconnect_attempt = cycle_start;
            }
        }
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

/// `-f <hz>` runs at the given fixed frame rate; without it, sysmon
/// adapts between `ADAPTIVE_MIN_HZ` and `ADAPTIVE_MAX_HZ` based on
/// peak CPU busy. Examples: `sysmon -f 20`, `sysmon -f 33`. Non-integer
/// rates are accepted; the cycle duration is rounded to the nearest
/// millisecond.
fn parse_cycle_arg() -> Option<Duration> {
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
            return Some(Duration::from_millis(ms));
        }
    }
    None
}

/// Cycle duration for the next frame given peak CPU busy in [0, 1].
/// Maps `busy` through `ADAPTIVE_CURVE` and interpolates Hz between
/// `ADAPTIVE_MIN_HZ` (idle) and `ADAPTIVE_MAX_HZ` (fully loaded).
fn adaptive_cycle(peak_busy: f32) -> Duration {
    let busy = peak_busy.clamp(0.0, 1.0) as f64;
    let weight = busy.powf(ADAPTIVE_CURVE);
    let hz = ADAPTIVE_MIN_HZ + (ADAPTIVE_MAX_HZ - ADAPTIVE_MIN_HZ) * weight;
    let ms = (1000.0 / hz).round().max(1.0) as u64;
    Duration::from_millis(ms)
}

/// Drain all pending button-state bytes from the firmware in this
/// cycle, returning the latest packed state and a bitmask of buttons
/// that newly became pressed since `prev_state`. Uses a 1 ms timeout —
/// effectively non-blocking; the firmware only writes on change, so
/// most cycles return `Ok((prev_state, 0))` immediately.
fn drain_button_events(
    panel: &mut Hub75Client,
    prev_state: u8,
) -> Result<(u8, u8), Box<dyn Error>> {
    let mut state = prev_state;
    let mut pressed_edges = 0u8;
    loop {
        match panel.recv_event(Duration::from_millis(1))? {
            Some(byte) => {
                pressed_edges |= byte & !state;
                state = byte;
            }
            None => return Ok((state, pressed_edges)),
        }
    }
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
    slates: &mut Slates,
    peak_busy: &mut f32,
) {
    match cpu.sample() {
        Ok(busy) => {
            let mut peak: f32 = 0.0;
            for slate in slates.each_mut() {
                for core_idx in 0..CORE_COUNT {
                    slate.cpu[core_idx].push_sample(busy[core_idx]);
                }
            }
            for core_idx in 0..CORE_COUNT {
                if busy[core_idx] > peak {
                    peak = busy[core_idx];
                }
            }
            *peak_busy = peak;
        }
        Err(e) => eprintln!("CPU sample failed: {e}"),
    }
    match ram.sample() {
        Ok(used) => {
            for slate in slates.each_mut() {
                slate.ram.push_sample(used);
            }
        }
        Err(e) => eprintln!("RAM sample failed: {e}"),
    }
    let now = Instant::now();
    match disk.sample(now) {
        Ok((read_frac, write_frac)) => {
            // For palette tuning without real disk activity, swap
            // `read_frac` for `synthetic_fraction(now)` to inject a
            // pseudo-random fraction. See `map.md` "A/B Palette Mode".
            for slate in slates.each_mut() {
                slate.disk_read.push_sample(read_frac);
                slate.disk_write.push_sample(write_frac);
            }
        }
        Err(e) => eprintln!("Disk sample failed: {e}"),
    }
    match net.sample(now) {
        Ok((rx_frac, tx_frac)) => {
            for slate in slates.each_mut() {
                slate.net_down.push_sample(rx_frac);
                slate.net_up.push_sample(tx_frac);
            }
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
