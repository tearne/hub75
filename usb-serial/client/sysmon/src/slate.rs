//! The slate: nine per-metric `MetricBands` instances. Each metric
//! has its own stack of bands (one per timescale), and samples
//! pushed to the slate cascade through the bands' aggregations.

use crate::bands::MetricBands;
use crate::presentation::{CORE_COUNT, CORE_WIDTH, HALF_WIDTH, RAM_WIDTH};

pub type Pixel = [u8; 3];

pub struct Slate {
    pub cpu: [MetricBands; CORE_COUNT],
    pub ram: MetricBands,
    pub disk_read:  MetricBands,
    pub disk_write: MetricBands,
    pub net_down: MetricBands,
    pub net_up:   MetricBands,
}

impl Slate {
    pub fn new() -> Self {
        Self {
            cpu: std::array::from_fn(|_| MetricBands::new(CORE_WIDTH)),
            ram:        MetricBands::new(RAM_WIDTH),
            disk_read:  MetricBands::new(HALF_WIDTH),
            disk_write: MetricBands::new(HALF_WIDTH),
            net_down:   MetricBands::new(HALF_WIDTH),
            net_up:     MetricBands::new(HALF_WIDTH),
        }
    }
}
