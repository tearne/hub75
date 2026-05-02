//! The slate: nine per-metric ring buffers of pre-laid pixel rows.
//!
//! For the CPU vertical slice, only the four CPU-core rings exist.
//! Other metrics will be added as more devices come online.

use std::collections::VecDeque;
use std::time::Instant;

use crate::presentation::{CORE_COUNT, CORE_WIDTH};
use crate::projection::RING_LENGTH;

pub type Pixel = [u8; 3];

/// One metric's history of pre-laid pixel rows. Newest at the front.
pub struct Ring {
    width: usize,
    rows: VecDeque<Vec<Pixel>>,
}

impl Ring {
    pub fn new(width: usize) -> Self {
        Self { width, rows: VecDeque::with_capacity(RING_LENGTH) }
    }

    pub fn width(&self) -> usize { self.width }

    pub fn push(&mut self, row: Vec<Pixel>) {
        debug_assert_eq!(row.len(), self.width);
        if self.rows.len() == RING_LENGTH { self.rows.pop_back(); }
        self.rows.push_front(row);
    }

    /// Newest entry at index 0; older entries follow.
    pub fn get(&self, i: usize) -> Option<&[Pixel]> {
        self.rows.get(i).map(|v| v.as_slice())
    }
}

/// The full slate. CPU-only for this vertical slice.
pub struct Slate {
    pub cpu: [Ring; CORE_COUNT],
    /// Wall-clock instant of the most recent CPU sample. Used by Vertical
    /// Mapping to compute the elapsed-fraction `t` for sub-pixel sliding.
    pub last_cpu_sample_at: Instant,
}

impl Slate {
    pub fn new() -> Self {
        Self {
            cpu: std::array::from_fn(|_| Ring::new(CORE_WIDTH)),
            last_cpu_sample_at: Instant::now(),
        }
    }
}
