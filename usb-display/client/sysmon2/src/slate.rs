//! The slate: nine per-metric ring buffers of scalar metric values.
//! Each entry stores the metric's normalised value (a fraction in
//! [0, 1]) at that sample. The pixel pattern is computed during
//! vertical mapping, not stored.

use std::collections::VecDeque;
use std::time::Instant;

use crate::presentation::{CORE_COUNT, TOTAL_ROWS};
use crate::projection::RING_LENGTH;

pub type Pixel = [u8; 3];

// Per-device sample-rate multipliers, in master sampling intervals.
// CPU samples on every master sample; slower metrics every Nth.
pub const CPU_MULTIPLIER:  u32 = 1;
pub const RAM_MULTIPLIER:  u32 = 10;
pub const NET_MULTIPLIER:  u32 = 6;
pub const DISK_MULTIPLIER: u32 = 20;

/// RAM uses a linear-time ring (one entry per panel row + edges) and
/// stores pre-laid random-dot pixel rows rather than scalar values.
pub const RAM_RING_LENGTH: usize = TOTAL_ROWS;

/// One metric's history of scalar values. Newest at the front.
pub struct Ring {
    values: VecDeque<f32>,
}

impl Ring {
    pub fn new() -> Self {
        Self { values: VecDeque::with_capacity(RING_LENGTH) }
    }

    pub fn push(&mut self, value: f32) {
        if self.values.len() == RING_LENGTH { self.values.pop_back(); }
        self.values.push_front(value);
    }

    pub fn get(&self, i: usize) -> Option<f32> {
        self.values.get(i).copied()
    }
}

/// RAM's special ring: stores pre-laid pixel rows (random dots) rather
/// than scalar values, with one entry per panel row + edge — no time
/// compression. An entry's panel position is its index in the ring.
pub struct RamRing {
    rows: VecDeque<Vec<Pixel>>,
}

impl RamRing {
    pub fn new() -> Self {
        Self { rows: VecDeque::with_capacity(RAM_RING_LENGTH) }
    }

    pub fn push(&mut self, row: Vec<Pixel>) {
        if self.rows.len() == RAM_RING_LENGTH { self.rows.pop_back(); }
        self.rows.push_front(row);
    }

    pub fn get(&self, i: usize) -> Option<&[Pixel]> {
        self.rows.get(i).map(|v| v.as_slice())
    }
}

/// The full slate — all nine metric streams plus per-device timestamps
/// of the most recent sample, used by Time Compression to compute
/// each metric's elapsed-fraction `t`.
pub struct Slate {
    pub cpu: [Ring; CORE_COUNT],
    pub ram: RamRing,
    pub disk_read: Ring,
    pub disk_write: Ring,
    pub net_down: Ring,
    pub net_up: Ring,

    pub last_cpu_sample_at: Instant,
    pub last_ram_sample_at: Instant,
    pub last_disk_sample_at: Instant,
    pub last_net_sample_at: Instant,
}

impl Slate {
    pub fn new() -> Self {
        let now = Instant::now();
        Self {
            cpu: std::array::from_fn(|_| Ring::new()),
            ram: RamRing::new(),
            disk_read: Ring::new(),
            disk_write: Ring::new(),
            net_down: Ring::new(),
            net_up: Ring::new(),
            last_cpu_sample_at: now,
            last_ram_sample_at: now,
            last_disk_sample_at: now,
            last_net_sample_at: now,
        }
    }
}
