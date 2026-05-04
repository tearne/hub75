//! CPU device sampling. Reads `/proc/stat` and produces a per-core
//! "total busy" fraction (user + system + iowait) over the elapsed
//! tick interval.

use std::io;

use crate::presentation::CORE_COUNT;

#[derive(Clone, Copy, Default)]
struct CoreStat {
    user: u64,
    nice: u64,
    system: u64,
    idle: u64,
    iowait: u64,
    irq: u64,
    softirq: u64,
    steal: u64,
}

pub struct CpuSampler {
    prev: [CoreStat; CORE_COUNT],
}

impl CpuSampler {
    pub fn new() -> io::Result<Self> {
        let prev = read_core_stats()?;
        Ok(Self { prev })
    }

    /// Returns one fraction per core in [0, 1]: total busy time over the
    /// interval since the previous sample.
    pub fn sample(&mut self) -> io::Result<[f32; CORE_COUNT]> {
        let curr = read_core_stats()?;
        let busy = std::array::from_fn(|i| busy_fraction(&self.prev[i], &curr[i]));
        self.prev = curr;
        Ok(busy)
    }
}

fn busy_fraction(prev: &CoreStat, curr: &CoreStat) -> f32 {
    let active = curr.user.saturating_sub(prev.user)
        + curr.system.saturating_sub(prev.system)
        + curr.iowait.saturating_sub(prev.iowait);
    let total_busy = active
        + curr.nice.saturating_sub(prev.nice)
        + curr.irq.saturating_sub(prev.irq)
        + curr.softirq.saturating_sub(prev.softirq)
        + curr.steal.saturating_sub(prev.steal);
    let idle = curr.idle.saturating_sub(prev.idle);
    let total = total_busy + idle;
    if total == 0 { return 0.0; }
    (active as f32 / total as f32).clamp(0.0, 1.0)
}

fn read_core_stats() -> io::Result<[CoreStat; CORE_COUNT]> {
    let text = std::fs::read_to_string("/proc/stat")?;
    let mut cores = [CoreStat::default(); CORE_COUNT];
    for line in text.lines() {
        let mut it = line.split_whitespace();
        let key = it.next().unwrap_or("");
        if !key.starts_with("cpu") || key == "cpu" { continue; }
        let Ok(idx) = key[3..].parse::<usize>() else { continue };
        if idx >= CORE_COUNT { continue; }
        let g = |it: &mut std::str::SplitWhitespace, prev: u64| -> u64 {
            it.next().and_then(|v| v.parse().ok()).unwrap_or(prev)
        };
        let mut it = it;
        cores[idx] = CoreStat {
            user:    g(&mut it, 0),
            nice:    g(&mut it, 0),
            system:  g(&mut it, 0),
            idle:    g(&mut it, 0),
            iowait:  g(&mut it, 0),
            irq:     g(&mut it, 0),
            softirq: g(&mut it, 0),
            steal:   g(&mut it, 0),
        };
    }
    Ok(cores)
}
