//! Layout: which columns each metric occupies on the panel, and the
//! edge-row geometry consumed by window aggregation.

pub const LOGICAL_WIDTH: usize = 32;
pub const LOGICAL_HEIGHT: usize = 64;

pub const EDGE_ABOVE: usize = 1;
pub const EDGE_BELOW: usize = 1;
pub const TOTAL_ROWS: usize = LOGICAL_HEIGHT + EDGE_ABOVE + EDGE_BELOW;

pub const CORE_COUNT: usize = 4;
pub const CORE_WIDTH: usize = 3;

pub const CPU_LEFT: usize = 6;

pub fn cpu_core_left(core_idx: usize) -> usize {
    CPU_LEFT + core_idx * CORE_WIDTH
}
