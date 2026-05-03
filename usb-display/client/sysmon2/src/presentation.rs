//! Layout: which columns each metric occupies on the panel.
//! In the banded scheme, panel rows correspond directly to band rows
//! (no edge rows, no time-compression curve).

pub const LOGICAL_WIDTH: usize = 32;
pub const LOGICAL_HEIGHT: usize = 64;

pub const CORE_COUNT: usize = 4;
pub const CORE_WIDTH: usize = 4;
pub const HALF_WIDTH: usize = 3;
pub const RAM_WIDTH:  usize = 4;

// CPU cores are interleaved between the other metrics, left to right:
//   Disk write | CPU0 | Disk read | CPU1 | Net down | CPU2 | Net up | CPU3 | RAM
// CPU cores at width 4, Disk and Net halves at width 3, RAM at width 4
// — sums to 3+4+3+4+3+4+3+4+4 = 32. RAM at the end so the screen-burn
// wrap-adjacency is RAM↔Disk-write rather than Net↔Disk.
pub const DISK_WRITE_LEFT: usize = 0;
pub const DISK_READ_LEFT:  usize = 7;
pub const NET_DOWN_LEFT:   usize = 14;
pub const NET_UP_LEFT:     usize = 21;
pub const RAM_LEFT:        usize = 28;

const CPU_LEFTS: [usize; CORE_COUNT] = [3, 10, 17, 24];

pub fn cpu_core_left(core_idx: usize) -> usize {
    CPU_LEFTS[core_idx]
}
