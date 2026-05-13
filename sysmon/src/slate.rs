//! The slate: nine per-metric `MetricBands` instances. Each metric
//! has its own stack of bands (one per timescale), and samples
//! pushed to the slate cascade through the bands' aggregations.
//!
//! `Slates` keeps one `Slate` per available layout, pushed in
//! parallel, so toggling layouts is just a pointer flip and both
//! views stay populated.

use crate::bands::{BandLayout, LAYOUT_BANDED, LAYOUT_FAST_ONLY, MetricBands};
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

/// Stable per-metric identity mixed into the dot-placement seed.
/// Distinct values matter; the specific numbers do not. Once assigned,
/// keep them stable so a given metric's pattern history is reproducible
/// across runs.
const METRIC_ID_CPU_BASE: u32 = 0x01;
const METRIC_ID_RAM:        u32 = 0x10;
const METRIC_ID_DISK_READ:  u32 = 0x20;
const METRIC_ID_DISK_WRITE: u32 = 0x21;
const METRIC_ID_NET_DOWN:   u32 = 0x30;
const METRIC_ID_NET_UP:     u32 = 0x31;

impl Slate {
    pub fn new(layout: &BandLayout) -> Self {
        Self {
            cpu: std::array::from_fn(|i| MetricBands::new(CORE_WIDTH, METRIC_ID_CPU_BASE + i as u32, layout)),
            ram:        MetricBands::new(RAM_WIDTH, METRIC_ID_RAM, layout),
            disk_read:  MetricBands::new(HALF_WIDTH, METRIC_ID_DISK_READ, layout),
            disk_write: MetricBands::new(HALF_WIDTH, METRIC_ID_DISK_WRITE, layout),
            net_down:   MetricBands::new(HALF_WIDTH, METRIC_ID_NET_DOWN, layout),
            net_up:     MetricBands::new(HALF_WIDTH, METRIC_ID_NET_UP, layout),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum LayoutMode {
    Banded,
    FastOnly,
}

/// Two `Slate`s — one per layout — kept fed in parallel so toggling
/// layouts is a pointer flip and both views stay populated.
pub struct Slates {
    pub banded: Slate,
    pub fast_only: Slate,
    pub active: LayoutMode,
}

impl Slates {
    pub fn new(initial: LayoutMode) -> Self {
        Self {
            banded: Slate::new(&LAYOUT_BANDED),
            fast_only: Slate::new(&LAYOUT_FAST_ONLY),
            active: initial,
        }
    }

    pub fn active(&self) -> &Slate {
        match self.active {
            LayoutMode::Banded => &self.banded,
            LayoutMode::FastOnly => &self.fast_only,
        }
    }

    pub fn each_mut(&mut self) -> [&mut Slate; 2] {
        [&mut self.banded, &mut self.fast_only]
    }

    pub fn toggle(&mut self) -> LayoutMode {
        self.active = match self.active {
            LayoutMode::Banded => LayoutMode::FastOnly,
            LayoutMode::FastOnly => LayoutMode::Banded,
        };
        self.active
    }
}
