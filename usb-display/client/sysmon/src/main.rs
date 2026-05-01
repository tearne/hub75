//! At-a-glance system monitor for Linux hosts on the 64×32 panel rotated
//! into a 32×64 portrait view.
//!
//! Four vertical strips, left-to-right: Disk, CPU, RAM, Network.
//! Newest data appears at the top of each strip and slides downward as
//! samples age. CPU is split into 4 thin vertical sub-strips (one per Pi 5
//! core) so single-threaded workloads pinning a core are visible. Disk and
//! Net split horizontally into read/write and down/up halves.
//!
//! Each metric scrolls at its own pace — CPU is jumpy, RAM and Network
//! are moderate, Disk is the calmest. Disk and network use a fixed log
//! scale so quiet idle still shows and bursts don't peg out. Each lit dot
//! is a soft 5×5 Gaussian blob, blurred in both axes, so adjacent dots
//! within a strip overlap into a continuous lit field.
//!
//! Tested on Raspberry Pi 5; should work on any aarch64 Debian-based Linux
//! host with USB CDC support and the matching HUB75 firmware on a Pico.
//! See README.md for install instructions.
//!
//! Run:  cargo run --release  (or install via the deb package)

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("sysmon_linux is Linux-only — it reads /proc directly.");
}

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    linux::run()
}

#[cfg(target_os = "linux")]
mod linux {

use hub75_client::{Hub75Client, WIDTH, HEIGHT};
use std::collections::VecDeque;
use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// ── Geometry ───────────────────────────────────────────────────────

/// Logical canvas — physical 64×32 rotated 90° CCW into 32×64 portrait.
const LW: usize = 32;
const LH: usize = 64;

/// Strip layout: vertical columns, left-to-right.
const DISK_LEFT:  usize = 0;
const DISK_WIDTH: usize = 6;
const CPU_LEFT:   usize = 6;
const CPU_WIDTH:  usize = 12;
const RAM_LEFT:   usize = 18;
const RAM_WIDTH:  usize = 8;
const NET_LEFT:   usize = 26;
const NET_WIDTH:  usize = 6;

/// CPU column subdivided into 4 vertical sub-strips, one per Pi 5 core.
/// Each sub-strip runs the full canvas height; the per-core dot density
/// reads as that core's load. No separator gap between cores.
const CORE_COUNT: usize = 4;
const CORE_WIDTH: usize = 3;

/// Disk and Net columns split horizontally into two equal halves
/// (no separator gap; colour distinguishes the channels).
const LAYER_HALF_COLS: usize = 3;

const _: () = assert!(DISK_WIDTH == LAYER_HALF_COLS * 2, "disk strip must split evenly");
const _: () = assert!(NET_WIDTH  == LAYER_HALF_COLS * 2, "net strip must split evenly");
const _: () = assert!(DISK_LEFT + DISK_WIDTH + CPU_WIDTH + RAM_WIDTH + NET_WIDTH == LW,
                      "strip widths must fill the canvas");
const _: () = assert!(CORE_WIDTH * CORE_COUNT == CPU_WIDTH, "CPU sub-strips must fill the column");

/// Continuous time compression: every canvas row gets its own window into
/// a single shared buffer of raw samples. Top rows cover one or two raw
/// samples (fast motion); bottom rows cover many (slow motion). Window
/// sizes grow geometrically with row, so dots appear to slow down as they
/// fall — without any visible breakpoints between bands.
///
/// `TOTAL_ROWS` includes one imaginary row above the canvas (logical row
/// 0, slide-in for the newest sample) and one below (logical row `LH+1`,
/// slide-out for the oldest visible sample). Visible canvas rows 0..LH-1
/// correspond to logical rows 1..=LH.
const TOTAL_ROWS: usize = LH + 2;

/// Geometric base for window growth, scaled by 100 to keep the const-fn
/// computation in integer arithmetic. 106 / 100 = 1.06 → window size at
/// logical row r (for visible rows) is `ceil(1.06^(r-1))`.
const BLUR_BASE_NUM: u64 = 106;
const BLUR_BASE_DEN: u64 = 100;

const WINDOW_SIZES:  [usize; TOTAL_ROWS] = compute_window_sizes();
const WINDOW_STARTS: [usize; TOTAL_ROWS] = compute_window_starts();
const BUFFER_LEN:    usize               = WINDOW_STARTS[TOTAL_ROWS - 1] + WINDOW_SIZES[TOTAL_ROWS - 1];

const fn compute_window_sizes() -> [usize; TOTAL_ROWS] {
    let mut sizes = [1usize; TOTAL_ROWS];
    // sizes[0] = 1 (imaginary above), sizes[TOTAL_ROWS - 1] = 1 (imaginary below).
    // For visible rows logical_row = 1..=LH, window size = ceil(BASE^(logical_row - 1))
    // computed in fixed-point integer arithmetic to keep this const-evaluable.
    let scale: u64 = 1_000_000_000;
    let mut val: u64 = scale;          // BASE^0 = 1
    let mut k = 0;
    while k < LH {
        sizes[1 + k] = ((val + scale - 1) / scale) as usize;
        val = val * BLUR_BASE_NUM / BLUR_BASE_DEN;
        k += 1;
    }
    sizes
}

const fn compute_window_starts() -> [usize; TOTAL_ROWS] {
    let sizes = compute_window_sizes();
    let mut starts = [0usize; TOTAL_ROWS];
    let mut total = 0usize;
    let mut r = 0;
    while r < TOTAL_ROWS {
        starts[r] = total;
        total += sizes[r];
        r += 1;
    }
    starts
}

/// Upper bound on shuffle-buffer length, large enough for the widest
/// strip's dot-position pool (CPU at 14).
const STRIP_COLS_MAX: usize = 16;

// ── Per-metric tick periods and phases ─────────────────────────────
//
// CPU samples every tick; slower metrics push only every Nth tick, so each
// gets its own scroll speed. Multipliers are relative to the user-set
// `--update`, so changing `-u` rescales the whole panel proportionally.
//
// Phases stagger the slow metrics so they never push on the same tick.

const PERIOD_RAM:  u64 = 6;
const PERIOD_NET:  u64 = 6;
const PERIOD_DISK: u64 = 20;

const PHASE_RAM:  u64 = 0;
const PHASE_NET:  u64 = 4;
const PHASE_DISK: u64 = 11;

// ── Colour palettes ────────────────────────────────────────────────

const CPU_USER:    [u8; 3] = [50, 230, 230];   // bright cyan
const CPU_SYSTEM:  [u8; 3] = [0, 170, 120];    // teal — green-dominant to separate from iowait blue
const CPU_IOWAIT:  [u8; 3] = [80, 100, 255];   // brighter saturated blue

const RAM_USED:    [u8; 3] = [220, 60, 180];   // bright magenta — truly committed
const RAM_BUFFERS: [u8; 3] = [30, 20, 60];     // dim violet — reclaimable, shifted off the USED hue
const RAM_CACHED:  [u8; 3] = [10, 8, 22];      // very dim slate — reclaimable, almost background

const DISK_READ:   [u8; 3] = [255, 200, 50];   // bright gold-yellow
const DISK_WRITE:  [u8; 3] = [240, 90, 20];    // orange-red

const NET_DOWN:    [u8; 3] = [0, 220, 80];     // bright green
const NET_UP:      [u8; 3] = [160, 240, 30];   // lime

// ── Log-scale endpoints for unbounded throughput metrics ───────────

const LOG_MIN_BPS: f32 = 1_000.0;            // 1 KB/s — bottom of bar
const LOG_MAX_BPS: f32 = 1_000_000_000.0;    // 1 GB/s — top of bar

// ── Update interval bounds ─────────────────────────────────────────

const MIN_UPDATE_MS:     u64 = 33;       // ~30 fps ceiling
const MAX_UPDATE_MS:     u64 = 10_000;   // one frame per 10 s floor
const DEFAULT_UPDATE_MS: u64 = 1_000;

// ── Render-rate and smoothing constants ────────────────────────────

const RENDER_FPS:           u64 = 20;
const RENDER_INTERVAL:      Duration = Duration::from_millis(1000 / RENDER_FPS);
/// Gaussian kernel for the dot splat. `BLUR_HALF_PIX` sets the integer
/// iteration range (pixels checked on each side of the dot's centre);
/// `BLUR_SIGMA` sets the Gaussian standard deviation. Truncation at
/// `BLUR_HALF_PIX` clips the long tail; with σ ≈ 1.0 and half-pix = 2, the
/// weight at the boundary is ~0.14 (visible but small).
const BLUR_HALF_PIX: i32 = 2;
const BLUR_SIGMA:    f32 = 0.7;

// ── Run ────────────────────────────────────────────────────────────

pub fn run() -> Result<(), Box<dyn Error>> {
    let args = parse_args();
    let interval = Duration::from_millis(args.update_ms);
    let snapshot = start_sampler(interval);
    let mut client = open_panel(args.port.as_deref())?;
    println!("Connected. Update interval {} ms. Ctrl+C to stop.", args.update_ms);
    render_loop(&mut client, snapshot, interval)
}

struct Args {
    port: Option<String>,
    update_ms: u64,
}

fn parse_args() -> Args {
    let mut it = std::env::args().skip(1);
    let mut port: Option<String> = None;
    let mut update_ms = DEFAULT_UPDATE_MS;
    while let Some(a) = it.next() {
        match a.as_str() {
            "-u" | "--update" => {
                let raw = it.next().unwrap_or_else(|| die("--update needs a millisecond value"));
                let parsed: u64 = raw.parse().unwrap_or_else(|_| die(&format!("invalid --update value: {raw}")));
                update_ms = parsed.clamp(MIN_UPDATE_MS, MAX_UPDATE_MS);
                if parsed != update_ms {
                    eprintln!("note: clamped --update to {update_ms} ms (range {MIN_UPDATE_MS}–{MAX_UPDATE_MS})");
                }
            }
            "-h" | "--help" => die_help(),
            other if other.starts_with('-') => die(&format!("unknown flag: {other}")),
            other => port = Some(other.to_string()),
        }
    }
    Args { port, update_ms }
}

fn die(msg: &str) -> ! {
    eprintln!("sysmon_linux: {msg}");
    std::process::exit(2);
}

fn die_help() -> ! {
    eprintln!("Usage: sysmon_linux [PORT] [-u|--update MS]");
    eprintln!("  PORT          serial port (default: auto-detect)");
    eprintln!("  -u, --update  update interval in ms ({MIN_UPDATE_MS}–{MAX_UPDATE_MS}, default {DEFAULT_UPDATE_MS})");
    std::process::exit(0);
}

fn open_panel(port: Option<&str>) -> Result<Hub75Client, Box<dyn Error>> {
    match port {
        Some(p) => Ok(Hub75Client::open(p)?),
        None    => Ok(Hub75Client::open_auto()?),
    }
}

fn render_loop(
    client: &mut Hub75Client,
    snapshot: Arc<Mutex<Snapshot>>,
    interval: Duration,
) -> Result<(), Box<dyn Error>> {
    let interval_secs    = interval.as_secs_f32();
    let cpu_period_secs  = interval_secs;
    let ram_period_secs  = PERIOD_RAM  as f32 * interval_secs;
    let disk_period_secs = PERIOD_DISK as f32 * interval_secs;
    let net_period_secs  = PERIOD_NET  as f32 * interval_secs;

    loop {
        let frame_start = Instant::now();

        let snap = snapshot.lock().unwrap().clone();
        let now  = Instant::now();
        let cpu_t  = elapsed_fraction(now, snap.cpu_at,  cpu_period_secs);
        let ram_t  = elapsed_fraction(now, snap.ram_at,  ram_period_secs);
        let disk_t = elapsed_fraction(now, snap.disk_at, disk_period_secs);
        let net_t  = elapsed_fraction(now, snap.net_at,  net_period_secs);

        let mut canvas = [[0u16; 3]; LW * LH];
        draw_disk_strip(&mut canvas, &snap.disk, disk_t);
        draw_cpu_strip(&mut canvas,  &snap.cpu,  cpu_t);
        draw_ram_strip(&mut canvas,  &snap.ram,  ram_t);
        draw_net_strip(&mut canvas,  &snap.net,  net_t);

        let frame = pack_rotated_cw(&canvas);
        client.send_frame_rgb(&frame)?;

        if let Some(rest) = RENDER_INTERVAL.checked_sub(frame_start.elapsed()) {
            thread::sleep(rest);
        }
    }
}

fn elapsed_fraction(now: Instant, since: Instant, period_secs: f32) -> f32 {
    if period_secs <= 0.0 { return 0.0; }
    (now.duration_since(since).as_secs_f32() / period_secs).clamp(0.0, 1.0)
}

// ── Snapshot of latest sampled state ───────────────────────────────

#[derive(Clone)]
struct Snapshot {
    cpu:     VecDeque<CpuSample>,
    ram:     VecDeque<RamSample>,
    disk:    VecDeque<IoSample>,
    net:     VecDeque<IoSample>,
    cpu_at:  Instant,
    ram_at:  Instant,
    disk_at: Instant,
    net_at:  Instant,
}

impl Default for Snapshot {
    fn default() -> Self {
        let now = Instant::now();
        Snapshot {
            cpu: VecDeque::new(),
            ram: VecDeque::new(),
            disk: VecDeque::new(),
            net: VecDeque::new(),
            cpu_at: now,
            ram_at: now,
            disk_at: now,
            net_at: now,
        }
    }
}

#[derive(Clone, Copy, Default)]
struct CpuSample {
    cores: [CoreSample; CORE_COUNT],
    seed:  u32,
}

#[derive(Clone, Copy, Default)]
struct CoreSample {
    user:   f32,    // fraction of interval 0..1
    system: f32,
    iowait: f32,
}

#[derive(Clone, Copy, Default)]
struct RamSample {
    composition: RamComposition,
    seed: u32,
}

#[derive(Clone, Copy, Default)]
struct IoSample {
    a_bps: f32,     // read for disk, rx for network
    b_bps: f32,     // write for disk, tx for network
    seed: u32,
}

#[derive(Clone, Copy, Default)]
struct RamComposition {
    used_kb:    u64,
    buffers_kb: u64,
    cached_kb:  u64,
    total_kb:   u64,
}

// ── Sampler thread ─────────────────────────────────────────────────

fn start_sampler(interval: Duration) -> Arc<Mutex<Snapshot>> {
    let snapshot = Arc::new(Mutex::new(Snapshot::default()));
    let snapshot_writer = Arc::clone(&snapshot);
    thread::spawn(move || sample_loop(snapshot_writer, interval));
    snapshot
}

fn sample_loop(snapshot: Arc<Mutex<Snapshot>>, interval: Duration) {
    let mut prev_cpu       = read_cpu_times_all().unwrap_or_default();
    let mut prev_disk      = read_disk_bytes().unwrap_or_default();
    let mut prev_disk_tick = 0u64;
    let mut prev_net       = read_net_bytes().unwrap_or_default();
    let mut prev_net_tick  = 0u64;
    let mut tick           = 0u64;
    let interval_secs      = interval.as_secs_f32();

    loop {
        thread::sleep(interval);
        tick += 1;

        let cpu_sample = sample_cpu(&mut prev_cpu);

        let ram_sample = (tick % PERIOD_RAM == PHASE_RAM).then(|| RamSample {
            composition: read_ram_composition().unwrap_or_default(),
            seed: next_seed(),
        });

        let disk_sample = (tick % PERIOD_DISK == PHASE_DISK).then(|| {
            let elapsed = (tick - prev_disk_tick) as f32 * interval_secs;
            let s = sample_io(&mut prev_disk, read_disk_bytes, elapsed);
            prev_disk_tick = tick;
            s
        });

        let net_sample = (tick % PERIOD_NET == PHASE_NET).then(|| {
            let elapsed = (tick - prev_net_tick) as f32 * interval_secs;
            let s = sample_io(&mut prev_net, read_net_bytes, elapsed);
            prev_net_tick = tick;
            s
        });

        let push_at = Instant::now();
        let mut s = snapshot.lock().unwrap();
        push_history(&mut s.cpu, cpu_sample, BUFFER_LEN);
        s.cpu_at = push_at;
        if let Some(r) = ram_sample  { push_history(&mut s.ram,  r, BUFFER_LEN); s.ram_at  = push_at; }
        if let Some(d) = disk_sample { push_history(&mut s.disk, d, BUFFER_LEN); s.disk_at = push_at; }
        if let Some(n) = net_sample  { push_history(&mut s.net,  n, BUFFER_LEN); s.net_at  = push_at; }
    }
}

/// Newest sample at index 0; oldest at the back. Pop from the back when
/// the buffer reaches its cap.
fn push_history<T>(buf: &mut VecDeque<T>, v: T, cap: usize) {
    if buf.len() == cap { buf.pop_back(); }
    buf.push_front(v);
}

fn sample_cpu(prev: &mut [CpuTimes; CORE_COUNT]) -> CpuSample {
    let now = read_cpu_times_all().unwrap_or(*prev);
    let mut cores = [CoreSample::default(); CORE_COUNT];
    for i in 0..CORE_COUNT {
        let n = now[i];
        let p = prev[i];
        let dtotal = n.total().saturating_sub(p.total()).max(1) as f32;
        cores[i] = CoreSample {
            user:   (n.user.saturating_sub(p.user) + n.nice.saturating_sub(p.nice)) as f32 / dtotal,
            system: (n.system.saturating_sub(p.system)
                   + n.irq.saturating_sub(p.irq)
                   + n.softirq.saturating_sub(p.softirq)) as f32 / dtotal,
            iowait: n.iowait.saturating_sub(p.iowait) as f32 / dtotal,
        };
    }
    *prev = now;
    CpuSample { cores, seed: next_seed() }
}

/// Hash a monotonic counter through `mix32` so consecutive seeds are
/// uncorrelated.
fn next_seed() -> u32 {
    use std::sync::atomic::{AtomicU32, Ordering};
    static COUNTER: AtomicU32 = AtomicU32::new(0);
    mix32(0xa3c59ac3, COUNTER.fetch_add(1, Ordering::Relaxed))
}

fn sample_io<F>(prev: &mut (u64, u64), reader: F, elapsed: f32) -> IoSample
where
    F: Fn() -> std::io::Result<(u64, u64)>,
{
    let fresh = reader().unwrap_or(*prev);
    let sample = IoSample {
        a_bps: fresh.0.saturating_sub(prev.0) as f32 / elapsed.max(0.001),
        b_bps: fresh.1.saturating_sub(prev.1) as f32 / elapsed.max(0.001),
        seed:  next_seed(),
    };
    *prev = fresh;
    sample
}

// ── /proc/stat ─────────────────────────────────────────────────────

#[derive(Clone, Copy, Default)]
struct CpuTimes {
    user: u64, nice: u64, system: u64, idle: u64,
    iowait: u64, irq: u64, softirq: u64, steal: u64,
}

impl CpuTimes {
    fn total(&self) -> u64 {
        self.user + self.nice + self.system + self.idle
            + self.iowait + self.irq + self.softirq + self.steal
    }
}

/// Read per-core `cpuN` lines from `/proc/stat` (skipping the aggregate
/// `cpu` line). Cores beyond `CORE_COUNT` are silently ignored.
fn read_cpu_times_all() -> std::io::Result<[CpuTimes; CORE_COUNT]> {
    let text = std::fs::read_to_string("/proc/stat")?;
    let mut out = [CpuTimes::default(); CORE_COUNT];
    for line in text.lines() {
        let prefix = match line.split_whitespace().next() {
            Some(p) => p,
            None    => continue,
        };
        let n: usize = match prefix.strip_prefix("cpu") {
            Some(rest) if !rest.is_empty() => match rest.parse() {
                Ok(n) if n < CORE_COUNT => n,
                _ => continue,
            },
            _ => continue,
        };
        let fields: Vec<u64> = line.split_whitespace()
            .skip(1)
            .filter_map(|s| s.parse().ok())
            .collect();
        let g = |i: usize| fields.get(i).copied().unwrap_or(0);
        out[n] = CpuTimes {
            user: g(0), nice: g(1), system: g(2), idle: g(3),
            iowait: g(4), irq: g(5), softirq: g(6), steal: g(7),
        };
    }
    Ok(out)
}

// ── /proc/meminfo ──────────────────────────────────────────────────

fn read_ram_composition() -> std::io::Result<RamComposition> {
    let text = std::fs::read_to_string("/proc/meminfo")?;
    let mut total = 0u64;
    let mut free = 0u64;
    let mut available = 0u64;
    let mut buffers = 0u64;
    let mut cached = 0u64;
    let mut sreclaimable = 0u64;
    for line in text.lines() {
        let mut it = line.split_whitespace();
        let key = it.next().unwrap_or("");
        let value: u64 = it.next().and_then(|v| v.parse().ok()).unwrap_or(0);
        match key {
            "MemTotal:"     => total = value,
            "MemFree:"      => free = value,
            "MemAvailable:" => available = value,
            "Buffers:"      => buffers = value,
            "Cached:"       => cached = value,
            "SReclaimable:" => sreclaimable = value,
            _ => {}
        }
    }
    let cached_total = cached + sreclaimable;
    let used = total.saturating_sub(available.max(free + buffers + cached_total));
    Ok(RamComposition {
        used_kb: used,
        buffers_kb: buffers,
        cached_kb: cached_total,
        total_kb: total,
    })
}

// ── /proc/diskstats ────────────────────────────────────────────────

fn read_disk_bytes() -> std::io::Result<(u64, u64)> {
    let text = std::fs::read_to_string("/proc/diskstats")?;
    let mut read_sectors = 0u64;
    let mut write_sectors = 0u64;
    for line in text.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 10 { continue; }
        let name = f[2];
        if !is_real_block_device(name) { continue; }
        read_sectors  += f[5].parse::<u64>().unwrap_or(0);
        write_sectors += f[9].parse::<u64>().unwrap_or(0);
    }
    Ok((read_sectors * 512, write_sectors * 512))
}

fn is_real_block_device(name: &str) -> bool {
    !(name.starts_with("loop")
        || name.starts_with("ram")
        || name.starts_with("dm-"))
}

// ── /proc/net/dev ──────────────────────────────────────────────────

fn read_net_bytes() -> std::io::Result<(u64, u64)> {
    let text = std::fs::read_to_string("/proc/net/dev")?;
    let mut rx = 0u64;
    let mut tx = 0u64;
    for line in text.lines().skip(2) {
        let mut parts = line.splitn(2, ':');
        let iface = parts.next().unwrap_or("").trim();
        let rest  = parts.next().unwrap_or("");
        if iface == "lo" || iface.is_empty() { continue; }
        let nums: Vec<u64> = rest.split_whitespace()
            .filter_map(|s| s.parse().ok())
            .collect();
        rx += nums.first().copied().unwrap_or(0);
        tx += nums.get(8).copied().unwrap_or(0);
    }
    Ok((rx, tx))
}

// ── Drawing primitives ─────────────────────────────────────────────

/// Accumulator canvas — Gaussian splats add into u16 channels; clamped to
/// u8 when packed to the wire frame.
type Canvas = [[u16; 3]; LW * LH];

/// Add a 2-D Gaussian dot splat centred on integer column `x` and
/// sub-pixel row `y_subpix`. The kernel covers ±BLUR_HALF_PIX pixels in
/// each axis; weight at offset `(dx, dy)` is
/// `exp(-(dx² + dy²) / (2σ²)) × alpha` (separable: `w(dx) × w(dy)`).
/// Clips to canvas bounds only — blur extends freely across strip,
/// sub-strip, and layer boundaries.
fn splat_dot(canvas: &mut Canvas, x: usize, y_subpix: f32, colour: [u8; 3], alpha: f32) {
    if alpha <= 0.0 { return; }
    let two_sigma_sq = 2.0 * BLUR_SIGMA * BLUR_SIGMA;
    let cy = y_subpix.round() as i32;
    let cx = x as i32;
    for dy in -BLUR_HALF_PIX..=BLUR_HALF_PIX {
        let py = cy + dy;
        if py < 0 || py >= LH as i32 { continue; }
        let py = py as usize;
        let yd = py as f32 - y_subpix;
        let yw = (-yd * yd / two_sigma_sq).exp();
        for dx in -BLUR_HALF_PIX..=BLUR_HALF_PIX {
            let px = cx + dx;
            if px < 0 || px >= LW as i32 { continue; }
            let xd = dx as f32;
            let xw = (-xd * xd / two_sigma_sq).exp();
            let weight = xw * yw * alpha;
            let idx = py * LW + px as usize;
            for c in 0..3 {
                let add = (colour[c] as f32 * weight) as u16;
                canvas[idx][c] = canvas[idx][c].saturating_add(add);
            }
        }
    }
}

/// Light a row at sub-pixel `y_subpix` over `width` candidate column
/// positions starting at `left`. Each `(count, colour)` segment is
/// splatted at the next pseudo-random column from a per-sample shuffled
/// order, so densities sum cleanly and the pattern is stable across
/// renders for the same sample.
fn paint_dot_row(
    canvas: &mut Canvas,
    y_subpix: f32,
    left: usize,
    width: usize,
    seed: u32,
    alpha: f32,
    segments: &[(usize, [u8; 3])],
) {
    let mut shuffled = [0u8; STRIP_COLS_MAX];
    shuffle_indices(seed, &mut shuffled[..width]);
    let total: usize = segments.iter().map(|s| s.0).sum::<usize>().min(width);
    let mut idx = 0;
    for &(count, colour) in segments {
        for _ in 0..count {
            if idx >= total { return; }
            splat_dot(canvas, left + shuffled[idx] as usize, y_subpix, colour, alpha);
            idx += 1;
        }
    }
}

fn shuffle_indices(seed: u32, out: &mut [u8]) {
    let n = out.len();
    let mut keyed = [(0u32, 0u8); STRIP_COLS_MAX];
    for r in 0..n {
        keyed[r] = (mix32(seed, r as u32), r as u8);
    }
    keyed[..n].sort_unstable_by_key(|&(p, _)| p);
    for r in 0..n {
        out[r] = keyed[r].1;
    }
}

fn mix32(a: u32, b: u32) -> u32 {
    let mut x = a.wrapping_add(b.wrapping_mul(0x9e3779b9));
    x ^= x >> 16;
    x = x.wrapping_mul(0x85ebca6b);
    x ^= x >> 13;
    x.wrapping_mul(0xc2b2ae35) ^ (x >> 16)
}

/// Map a fraction in [0, 1] to a dot count over `width` candidate columns.
fn dots_for(fraction: f32, width: usize) -> usize {
    (fraction.clamp(0.0, 1.0) * width as f32).round() as usize
}

/// Map bytes-per-second to a fraction in [0, 1] using log10 between
/// `LOG_MIN_BPS` and `LOG_MAX_BPS`.
fn log_fraction(bps: f32) -> f32 {
    if bps <= LOG_MIN_BPS { return 0.0; }
    let logged   = bps.log10();
    let log_min  = LOG_MIN_BPS.log10();
    let log_max  = LOG_MAX_BPS.log10();
    ((logged - log_min) / (log_max - log_min)).clamp(0.0, 1.0)
}

// ── Strip renderers ────────────────────────────────────────────────
//
// Every sample in a metric's buffer is rendered individually at a sub-pixel
// y position derived from its window assignment. For a sample at buffer
// index `i`, find the logical row `r` whose window contains `i` (so
// `WINDOW_STARTS[r] <= i < WINDOW_STARTS[r] + WINDOW_SIZES[r]`) and let
// `w = i - WINDOW_STARTS[r]` be its position within that window. Then:
//
//     y     = (r as f32 - 1.0) + (w as f32 + t) / N(r) as f32
//     alpha = 1.0 / N(r) as f32
//
// The `-1.0` shift puts logical row 0 (imaginary above) off the top of the
// canvas. As `t` runs 0 → 1, every sample slides forward by `1/N(r)` of a
// pixel — fast in row 1 (1 px/period) and slow at the bottom (~1/12
// px/period for `b = 1.04`). Brightness compensation `1/N(r)` keeps
// summed per-row intensity consistent across the panel.

/// Visit every populated `(sample, logical_row, w_in_window)` triple in
/// the buffer in window order, calling `f` for each. Stops early if the
/// buffer hasn't accumulated enough samples to reach a row's window yet.
fn for_each_window_sample<T>(
    buffer: &VecDeque<T>,
    mut f: impl FnMut(&T, usize, usize),
) {
    for r in 0..TOTAL_ROWS {
        for w in 0..WINDOW_SIZES[r] {
            let i = WINDOW_STARTS[r] + w;
            match buffer.get(i) {
                Some(sample) => f(sample, r, w),
                None => return,
            }
        }
    }
}

fn window_y(r: usize, w: usize, t: f32) -> f32 {
    (r as f32 - 1.0) + (w as f32 + t) / WINDOW_SIZES[r] as f32
}

fn window_alpha(r: usize) -> f32 {
    // `1/N` gives mathematically uniform per-column expected brightness,
    // but at sparse activity the top reads as bright spots (high variance)
    // while the bottom reads as a dim uniform glow — perceived as a
    // dark-going-down gradient. `1/sqrt(N)` over-corrected, making the
    // bottom over-saturate. `1/N^0.7` is the compromise: bottom uniform
    // ≈ top peak, so neither side dominates. Plus a 0.5× global dim and a
    // halved-again multiplier on the imaginary edge rows.
    let n = WINDOW_SIZES[r] as f32;
    let base = if r == 0 || r == TOTAL_ROWS - 1 { 0.1 } else { 0.5 };
    base / n.powf(0.7)
}

fn draw_cpu_strip(canvas: &mut Canvas, buffer: &VecDeque<CpuSample>, t: f32) {
    for_each_window_sample(buffer, |sample, r, w| {
        let y = window_y(r, w, t);
        let alpha = window_alpha(r);
        for core_idx in 0..CORE_COUNT {
            let core_left = CPU_LEFT + core_idx * CORE_WIDTH;
            let core = sample.cores[core_idx];
            let segments = [
                (dots_for(core.user,   CORE_WIDTH), CPU_USER),
                (dots_for(core.system, CORE_WIDTH), CPU_SYSTEM),
                (dots_for(core.iowait, CORE_WIDTH), CPU_IOWAIT),
            ];
            let core_seed = mix32(sample.seed, core_idx as u32);
            paint_dot_row(canvas, y, core_left, CORE_WIDTH, core_seed, alpha, &segments);
        }
    });
}

fn draw_ram_strip(canvas: &mut Canvas, buffer: &VecDeque<RamSample>, t: f32) {
    for_each_window_sample(buffer, |sample, r, w| {
        let y = window_y(r, w, t);
        let alpha = window_alpha(r);
        let total = sample.composition.total_kb.max(1) as f32;
        let segments = [
            (dots_for(sample.composition.used_kb    as f32 / total, RAM_WIDTH), RAM_USED),
            (dots_for(sample.composition.buffers_kb as f32 / total, RAM_WIDTH), RAM_BUFFERS),
            (dots_for(sample.composition.cached_kb  as f32 / total, RAM_WIDTH), RAM_CACHED),
        ];
        paint_dot_row(canvas, y, RAM_LEFT, RAM_WIDTH, sample.seed, alpha, &segments);
    });
}

// Disk and Net put their input channel (read / download) on the inner
// edge of the canvas and their output channel (write / upload) on the
// outer edge. Since Disk is the leftmost strip, that means write on the
// left half and read on the right half — opposite of the natural
// (a_bps, b_bps) field order. Net is the rightmost strip so download
// already lands on the inner (left) half naturally.

fn draw_disk_strip(canvas: &mut Canvas, buffer: &VecDeque<IoSample>, t: f32) {
    draw_layered_strip(
        canvas, DISK_LEFT, LAYER_HALF_COLS, buffer, t,
        |s| s.b_bps, DISK_WRITE,   // outer (left): write
        |s| s.a_bps, DISK_READ,    // inner (right): read
    );
}

fn draw_net_strip(canvas: &mut Canvas, buffer: &VecDeque<IoSample>, t: f32) {
    draw_layered_strip(
        canvas, NET_LEFT, LAYER_HALF_COLS, buffer, t,
        |s| s.a_bps, NET_DOWN,     // inner (left): download
        |s| s.b_bps, NET_UP,       // outer (right): upload
    );
}

/// Two-channel throughput as two side-by-side layers within one strip.
/// `left_pick` / `right_pick` choose which sample field feeds each side,
/// so callers can map "input on the inside" regardless of which canvas
/// edge the strip sits on. Each layer scales independently against the log
/// endpoints, and gets its own derived seed so the two halves don't share
/// a permutation.
fn draw_layered_strip(
    canvas: &mut Canvas,
    left: usize,
    half_cols: usize,
    buffer: &VecDeque<IoSample>,
    t: f32,
    left_pick:  impl Fn(&IoSample) -> f32,
    left_colour: [u8; 3],
    right_pick: impl Fn(&IoSample) -> f32,
    right_colour: [u8; 3],
) {
    for_each_window_sample(buffer, |sample, r, w| {
        let y = window_y(r, w, t);
        let alpha = window_alpha(r);
        let n_left  = dots_for(log_fraction(left_pick(sample)),  half_cols);
        let n_right = dots_for(log_fraction(right_pick(sample)), half_cols);
        paint_dot_row(
            canvas, y, left, half_cols,
            sample.seed.wrapping_add(0xA1B2C3D4), alpha,
            &[(n_left, left_colour)],
        );
        paint_dot_row(
            canvas, y, left + half_cols, half_cols,
            sample.seed.wrapping_add(0x5E6F7081), alpha,
            &[(n_right, right_colour)],
        );
    });
}

// ── Rotation packing ───────────────────────────────────────────────

/// Map logical 32×64 portrait canvas → physical 64×32 wire frame, clamping
/// each accumulator channel to 0..=255. CW rotation: physical
/// (WIDTH-1-ly, lx) ← logical (lx, ly). Despite the user-facing
/// description being "90° CCW from landscape", this pack is the 180°
/// rotation of the strict CCW form — needed because the panel is
/// physically mounted such that the user-visible "top of view" maps to
/// the original landscape's right edge.
fn pack_rotated_cw(canvas: &Canvas) -> [[u8; 3]; WIDTH * HEIGHT] {
    let mut out = [[0u8; 3]; WIDTH * HEIGHT];
    for ly in 0..LH {
        for lx in 0..LW {
            let px  = WIDTH - 1 - ly;
            let py  = lx;
            let src = canvas[ly * LW + lx];
            out[py * WIDTH + px] = [
                src[0].min(255) as u8,
                src[1].min(255) as u8,
                src[2].min(255) as u8,
            ];
        }
    }
    out
}

} // mod linux
