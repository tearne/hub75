//! Core-1 pack worker.
//!
//! Loops forever: wait for a kick (target buf index) on the SIO FIFO,
//! pack the panel's pixel buffer into that frame buffer, signal back.
//! No embassy executor on core 1 — `wfe` keeps it asleep between kicks.
//!
//! Protocol (the kick/done word's low bit is the target buf index 0/1):
//! - Core 0 commit: pushes target_buf to FIFO, raises SEV.
//! - Core 1: wakes from wfe, packs, pushes the same target back.
//! - Core 0 scan loop: pops, swaps active buffer.

use embassy_rp::pac;

use super::pack::pack_pixels;
use super::sio::{fifo_read_blocking, fifo_write};
use super::storage::STORAGE;

pub fn pack_worker_loop() -> ! {
    // `embassy_rp::multicore::spawn_core1` unmasks SIO_IRQ_FIFO on core 1
    // for its pause feature. We don't bind a handler, so the first FIFO
    // write from core 0 would re-fire the IRQ forever. Mask it; the
    // wfe-loop in `fifo_read_blocking` still wakes on the SEV core 0
    // sends after each fifo_write.
    cortex_m::peripheral::NVIC::mask(pac::Interrupt::SIO_IRQ_FIFO);

    loop {
        let kick = fifo_read_blocking();
        let target = (kick & 1) as usize;

        // SAFETY: between the kick and the done message, the protocol
        // guarantees core 0 doesn't touch `pixels` (frame_mut waits)
        // or `frames[target]` (DMA reads `frames[1 - target]`).
        unsafe {
            let pixels = &*STORAGE.pixels.get();
            let frame = &mut *STORAGE.frames[target].get();
            pack_pixels(pixels, frame);
        }

        fifo_write(target as u32);
    }
}
