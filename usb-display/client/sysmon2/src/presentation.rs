//! Layout: which columns each metric occupies on the panel, and the
//! edge-row geometry consumed by vertical mapping.

pub const LOGICAL_WIDTH: usize = 32;
pub const LOGICAL_HEIGHT: usize = 64;

pub const EDGE_ABOVE: usize = 1;
pub const EDGE_BELOW: usize = 1;
pub const TOTAL_ROWS: usize = LOGICAL_HEIGHT + EDGE_ABOVE + EDGE_BELOW;

pub const CORE_COUNT: usize = 4;
pub const CORE_WIDTH: usize = 4;
pub const HALF_WIDTH: usize = 3;
pub const RAM_WIDTH:  usize = 4;

// CPU cores are interleaved between the other metrics, left to right:
//   Disk write | CPU0 | Disk read | CPU1 | RAM | CPU2 | Net down | CPU3 | Net up
// CPU cores at width 4, Disk and Net halves at width 3, RAM at width 4
// — sums to 3+4+3+4+4+4+3+4+3 = 32.
pub const DISK_WRITE_LEFT: usize = 0;
pub const DISK_READ_LEFT:  usize = 7;
pub const RAM_LEFT:        usize = 14;
pub const NET_DOWN_LEFT:   usize = 22;
pub const NET_UP_LEFT:     usize = 29;

const CPU_LEFTS: [usize; CORE_COUNT] = [3, 10, 18, 25];

pub fn cpu_core_left(core_idx: usize) -> usize {
    CPU_LEFTS[core_idx]
}
