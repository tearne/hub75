//! BCM timing buffer for the Address SM.
//!
//! Encodes per-bitplane on/off cycle counts in pairs `[off, on, off, on, ...]`.
//! Bit 0 gets `BASE_CYCLES`; each subsequent bit doubles. Brightness scales
//! the on-time fraction within each plane.

use super::TIMING_BUF_WORDS;

/// On-time for bit-0 in PIO cycles. Each higher bit doubles.
pub const BASE_CYCLES: u32 = 25;

pub fn fill(buf: &mut [u32; TIMING_BUF_WORDS], brightness: u8) {
    for bit in 0..(TIMING_BUF_WORDS / 2) {
        let full_on = BASE_CYCLES << bit;
        let on_cycles = (full_on * brightness as u32) / 255;
        let off_cycles = (full_on - on_cycles) / 2 + 10;
        buf[bit * 2] = off_cycles;
        buf[bit * 2 + 1] = on_cycles;
    }
}
