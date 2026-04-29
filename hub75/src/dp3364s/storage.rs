//! Static storage for the panel's pixel and DMA buffers.
//!
//! Accessed from both cores: core 0 owns the pixel buffer when no pack
//! is in flight (caller writes via `frame_mut`); core 1 reads the pixel
//! buffer while packing into the spare frame buffer; the data DMA on
//! core 0 reads the active frame buffer concurrently with the pack on
//! core 1 (always the *other* frame buffer). The kick/done message
//! protocol via SIO FIFO enforces this disjointness.

use core::cell::UnsafeCell;

use crate::panel::Rgb;

use super::framing::{FRAME_WORDS, SCAN_LINES};
use super::{HEIGHT, WIDTH};

/// Scan buffer wrapped at 128-byte alignment so ring-mode DMA wraps on
/// a natural boundary.
#[repr(C, align(128))]
pub struct ScanBuf(pub [u32; SCAN_LINES]);

pub struct Storage {
    pub pixels: UnsafeCell<[[Rgb; WIDTH]; HEIGHT]>,
    pub frames: [UnsafeCell<[u32; FRAME_WORDS]>; 2],
    pub scan: UnsafeCell<ScanBuf>,
}

unsafe impl Sync for Storage {}

pub static STORAGE: Storage = Storage {
    pixels: UnsafeCell::new([[Rgb::BLACK; WIDTH]; HEIGHT]),
    frames: [
        UnsafeCell::new([0u32; FRAME_WORDS]),
        UnsafeCell::new([0u32; FRAME_WORDS]),
    ],
    scan: UnsafeCell::new(ScanBuf([0u32; SCAN_LINES])),
};
