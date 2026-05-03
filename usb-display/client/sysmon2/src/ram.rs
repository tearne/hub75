//! RAM device sampling. Reads `/proc/meminfo` and produces a "fraction
//! of total memory in use", computed as `(MemTotal − MemAvailable) /
//! MemTotal` — matches `btop` and `free`'s "used" reading.

use std::io;

pub struct RamSampler;

impl RamSampler {
    pub fn new() -> Self { Self }

    pub fn sample(&self) -> io::Result<f32> {
        let (used, total) = read_used_and_total()?;
        if total == 0 { return Ok(0.0); }
        Ok((used as f32 / total as f32).clamp(0.0, 1.0))
    }
}

fn read_used_and_total() -> io::Result<(u64, u64)> {
    let text = std::fs::read_to_string("/proc/meminfo")?;
    let mut total = 0u64;
    let mut available = 0u64;
    for line in text.lines() {
        let mut it = line.split_whitespace();
        let key = it.next().unwrap_or("");
        let value: u64 = it.next().and_then(|v| v.parse().ok()).unwrap_or(0);
        match key {
            "MemTotal:"     => total = value,
            "MemAvailable:" => available = value,
            _ => {}
        }
    }
    let used = total.saturating_sub(available);
    Ok((used, total))
}
