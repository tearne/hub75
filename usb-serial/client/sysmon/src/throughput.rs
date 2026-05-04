//! Generic throughput sampler used for both Disk and Net. Each instance
//! tracks two cumulative byte counters (a, b), differences them per
//! sample to bytes/sec, and log-normalises against a per-channel
//! running peak so the panel scales with whatever the host's actual
//! traffic is rather than a fixed maximum.

use std::io;
use std::time::Instant;

/// One-shot snapshot of "(a_bytes_total, b_bytes_total)" since boot.
pub type ByteReader = fn() -> io::Result<(u64, u64)>;

pub struct ThroughputSampler {
    prev_bytes: (u64, u64),
    prev_at: Instant,
    max_a_bps: f32,
    max_b_bps: f32,
    log_min_bps: f32,
    reader: ByteReader,
}

impl ThroughputSampler {
    pub fn new(reader: ByteReader, log_min_bps: f32, log_max_floor: f32) -> io::Result<Self> {
        let prev_bytes = reader()?;
        Ok(Self {
            prev_bytes,
            prev_at: Instant::now(),
            max_a_bps: log_max_floor,
            max_b_bps: log_max_floor,
            log_min_bps,
            reader,
        })
    }

    /// Returns `(a_fraction, b_fraction)` in [0, 1], log-scaled between
    /// `log_min_bps` and the running per-channel peak.
    pub fn sample(&mut self, now: Instant) -> io::Result<(f32, f32)> {
        let curr = (self.reader)()?;
        let elapsed = now.duration_since(self.prev_at).as_secs_f32().max(0.001);
        let a_bps = curr.0.saturating_sub(self.prev_bytes.0) as f32 / elapsed;
        let b_bps = curr.1.saturating_sub(self.prev_bytes.1) as f32 / elapsed;
        self.max_a_bps = self.max_a_bps.max(a_bps);
        self.max_b_bps = self.max_b_bps.max(b_bps);
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
    ThroughputSampler::new(read_disk_bytes, DISK_LOG_MIN_BPS, DISK_LOG_MAX_FLOOR)
}

const DISK_LOG_MIN_BPS: f32 = 1.0;
const DISK_LOG_MAX_FLOOR: f32 = 10_000.0;

/// Returns `(read_bytes_total, write_bytes_total)` summed across whole
/// block devices only — virtuals (loop, ram, dm-*) and partitions are
/// skipped. Partitions are detected by the kernel-maintained
/// `/sys/class/block/<name>/partition` file, which only exists for
/// partition entries; whole devices don't have it.
fn read_disk_bytes() -> io::Result<(u64, u64)> {
    let text = std::fs::read_to_string("/proc/diskstats")?;
    let mut read_sectors = 0u64;
    let mut write_sectors = 0u64;
    for line in text.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 10 { continue; }
        let name = f[2];
        if !is_whole_block_device(name) { continue; }
        read_sectors  += f[5].parse::<u64>().unwrap_or(0);
        write_sectors += f[9].parse::<u64>().unwrap_or(0);
    }
    Ok((read_sectors * 512, write_sectors * 512))
}

fn is_whole_block_device(name: &str) -> bool {
    if name.starts_with("loop") || name.starts_with("ram") || name.starts_with("dm-") {
        return false;
    }
    let partition_marker = format!("/sys/class/block/{name}/partition");
    !std::path::Path::new(&partition_marker).exists()
}

// ── Network ────────────────────────────────────────────────────────

pub fn net_sampler() -> io::Result<ThroughputSampler> {
    ThroughputSampler::new(read_net_bytes, NET_LOG_MIN_BPS, NET_LOG_MAX_FLOOR)
}

const NET_LOG_MIN_BPS: f32 = 1.0;
const NET_LOG_MAX_FLOOR: f32 = 200_000.0;

/// Returns `(rx_bytes_total, tx_bytes_total)` summed across non-loopback
/// interfaces.
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
