//! Generic throughput sampler used for both Disk and Net. Each instance
//! tracks two cumulative byte counters (a, b), differences them per
//! sample to bytes/sec, and log-normalises against a per-channel
//! running peak so the panel scales with whatever the host's actual
//! traffic is rather than a fixed maximum.

use std::io;
use std::path::PathBuf;
use std::sync::OnceLock;
use std::time::Instant;

/// One-shot snapshot of "(a_bytes_total, b_bytes_total)" since boot.
pub type ByteReader = fn() -> io::Result<(u64, u64)>;

/// How the two channels share their normalisation peak.
///
/// - `Independent` — each channel has its own monotonic peak.
///   Activity in one direction doesn't dampen the visibility of the
///   other. Cost: the two bands aren't directly comparable in absolute
///   terms when peaks have diverged.
/// - `Shared` — both channels normalise against a single monotonic
///   peak. Bands are directly comparable: a taller band genuinely
///   means more bytes/sec. Cost: a heavily lopsided host (e.g. far
///   more rx than tx ever) can render the quieter direction
///   permanently small.
#[derive(Copy, Clone)]
pub enum PeakMode {
    Independent,
    Shared,
}

pub struct ThroughputSampler {
    prev_bytes: (u64, u64),
    prev_at: Instant,
    max_a_bps: f32,
    max_b_bps: f32,
    log_min_bps: f32,
    peak_mode: PeakMode,
    reader: ByteReader,
}

impl ThroughputSampler {
    pub fn new(
        reader: ByteReader,
        log_min_bps: f32,
        log_max_floor: f32,
        peak_mode: PeakMode,
    ) -> io::Result<Self> {
        let prev_bytes = reader()?;
        Ok(Self {
            prev_bytes,
            prev_at: Instant::now(),
            max_a_bps: log_max_floor,
            max_b_bps: log_max_floor,
            log_min_bps,
            peak_mode,
            reader,
        })
    }

    /// Returns `(a_fraction, b_fraction)` in [0, 1], log-scaled between
    /// `log_min_bps` and the running peak (per-channel or shared,
    /// depending on `peak_mode`).
    pub fn sample(&mut self, now: Instant) -> io::Result<(f32, f32)> {
        let curr = (self.reader)()?;
        let elapsed = now.duration_since(self.prev_at).as_secs_f32().max(0.001);
        let a_bps = curr.0.saturating_sub(self.prev_bytes.0) as f32 / elapsed;
        let b_bps = curr.1.saturating_sub(self.prev_bytes.1) as f32 / elapsed;
        match self.peak_mode {
            PeakMode::Independent => {
                self.max_a_bps = self.max_a_bps.max(a_bps);
                self.max_b_bps = self.max_b_bps.max(b_bps);
            }
            PeakMode::Shared => {
                let shared = self.max_a_bps.max(self.max_b_bps).max(a_bps).max(b_bps);
                self.max_a_bps = shared;
                self.max_b_bps = shared;
            }
        }
        self.prev_bytes = curr;
        self.prev_at = now;
        Ok((
            log_fraction(a_bps, self.log_min_bps, self.max_a_bps),
            log_fraction(b_bps, self.log_min_bps, self.max_b_bps),
        ))
    }
}

/// Map bytes-per-second to a fraction in [0, 1] using log10 between
/// `min_bps` and `max_bps`. Below the floor returns zero.
fn log_fraction(bps: f32, min_bps: f32, max_bps: f32) -> f32 {
    if bps <= min_bps { return 0.0; }
    let logged  = bps.log10();
    let log_min = min_bps.log10();
    let log_max = max_bps.log10();
    ((logged - log_min) / (log_max - log_min)).clamp(0.0, 1.0)
}

// ── Disk ───────────────────────────────────────────────────────────

pub fn disk_sampler() -> io::Result<ThroughputSampler> {
    ThroughputSampler::new(
        read_disk_bytes,
        DISK_LOG_MIN_BPS,
        DISK_LOG_MAX_FLOOR,
        PeakMode::Independent,
    )
}

const DISK_LOG_MIN_BPS: f32 = 1.0;
const DISK_LOG_MAX_FLOOR: f32 = 10_000.0;

/// Returns `(read_bytes_total, write_bytes_total)` summed across the
/// host's main physical disks. Devices are auto-detected by the
/// presence of `/sys/block/<name>/device` (a symlink the kernel
/// maintains for real backing devices: SD/eMMC `mmcblk*`, SCSI/SATA
/// `sd*`, NVMe `nvme*n*`). Virtuals (`loop`, `ram`, `zram`, `dm-*`)
/// have no `device` symlink and are skipped.
///
/// Discovery runs once and the resulting `stat` paths are cached
/// for the process lifetime; per-cycle work is just `read_to_string`
/// on the known files. USB hotplug after startup is not picked up.
///
/// Reads `/sys/block/<name>/stat`, avoiding the `/proc/diskstats`
/// seq_file path that formats every block device and partition on
/// the system. Field layout (whitespace-separated): reads,
/// read-merges, read-sectors, read-ticks, writes, write-merges,
/// write-sectors, …; we want fields 2 and 6.
fn read_disk_bytes() -> io::Result<(u64, u64)> {
    let mut read_sectors = 0u64;
    let mut write_sectors = 0u64;
    for stat_path in disk_stat_paths()? {
        let text = std::fs::read_to_string(stat_path)?;
        let mut fields = text.split_whitespace();
        let read = fields.nth(2).and_then(|s| s.parse::<u64>().ok());
        // .nth advances the iterator past index 2; we want the next
        // four fields (read-ticks, writes, write-merges, write-sectors).
        let write = fields.nth(3).and_then(|s| s.parse::<u64>().ok());
        if let (Some(r), Some(w)) = (read, write) {
            read_sectors += r;
            write_sectors += w;
        }
    }
    Ok((read_sectors * 512, write_sectors * 512))
}

fn disk_stat_paths() -> io::Result<&'static [PathBuf]> {
    static PATHS: OnceLock<Vec<PathBuf>> = OnceLock::new();
    if let Some(paths) = PATHS.get() {
        return Ok(paths);
    }
    let mut paths = Vec::new();
    for entry in std::fs::read_dir("/sys/block")? {
        let entry = entry?;
        if !entry.path().join("device").exists() { continue; }
        paths.push(entry.path().join("stat"));
    }
    Ok(PATHS.get_or_init(|| paths))
}

// ── Network ────────────────────────────────────────────────────────

pub fn net_sampler() -> io::Result<ThroughputSampler> {
    ThroughputSampler::new(
        read_net_bytes,
        NET_LOG_MIN_BPS,
        NET_LOG_MAX_FLOOR,
        PeakMode::Shared,
    )
}

const NET_LOG_MIN_BPS: f32 = 10_000.0;
const NET_LOG_MAX_FLOOR: f32 = 200_000.0;

/// Returns `(rx_bytes_total, tx_bytes_total)` summed across non-loopback
/// interfaces.
///
/// Uses `/proc/net/dev` rather than `/sys/class/net/<iface>/statistics/*`:
/// each per-counter `/sys` file triggers a full kernel `dev_get_stats()`
/// just to return one number, so a per-NIC pair of reads is more
/// expensive than parsing the whole `/proc/net/dev` table once.
fn read_net_bytes() -> io::Result<(u64, u64)> {
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
