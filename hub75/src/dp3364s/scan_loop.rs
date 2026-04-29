//! Core-0 scan loop: drives the panel by alternating data and scan DMAs
//! through the unified PIO program, awaiting completion via DMA_IRQ_0
//! signals (so the task is `Pending` for ~99 % of every frame).

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;

use super::dma::DmaEngine;
use super::framing::{init_frame_headers, update_wr_cfg, CONFIG_REGS, DATA_END};
use super::pio::{self, PhaseSwap};
use super::scan::build_scan_buf;
use super::sio;
use super::storage::STORAGE;

/// Shared state between the scan loop, the application's `frame_mut`
/// / `commit` calls, and the pack worker on core 1.
pub struct PanelState {
    pub dma: DmaEngine,
    pub phase_swap: PhaseSwap,
    pub current_scan_cycles: AtomicU32,
    pub active_buf: AtomicU8,
    pub spare_buf: AtomicU8,
    /// Signalled by the scan loop when it consumes a pack-done message
    /// from core 1, telling `frame_mut` it's safe to take the borrow.
    pub frame_available: Signal<CriticalSectionRawMutex, ()>,
}

const FLUSH_FRAMES: u32 = 26;
const SYNC_FRAMES: u32 = 60;

#[embassy_executor::task]
pub async fn scan_loop_task(state: &'static PanelState) -> ! {
    initialise_buffers();
    boot_flush(state).await;
    boot_sync(state).await;
    state.dma.reset_signals();
    defmt::info!("hub75: scan loop running");

    let scan_buf_ptr = STORAGE.scan.get() as *const u32;

    loop {
        run_data_phase(state).await;
        run_scan_phase(state, scan_buf_ptr).await;
        absorb_pack_done(state);
    }
}

fn initialise_buffers() {
    unsafe {
        init_frame_headers(&mut *STORAGE.frames[0].get());
        init_frame_headers(&mut *STORAGE.frames[1].get());
        build_scan_buf(&mut (*STORAGE.scan.get()).0);
    }
}

async fn boot_flush(state: &PanelState) {
    let frame_a = STORAGE.frames[0].get();
    let scan_buf = STORAGE.scan.get() as *const u32;

    for flush in 0..FLUSH_FRAMES {
        if (flush as usize) < CONFIG_REGS.len() {
            unsafe { update_wr_cfg(&mut *frame_a, flush as usize) };
        }

        single_data_pass(state, frame_a as *const u32).await;
        single_scan_pass(state, scan_buf, 1).await;
    }
}

async fn boot_sync(state: &PanelState) {
    let frame_a = STORAGE.frames[0].get() as *const u32;
    let scan_buf = STORAGE.scan.get() as *const u32;

    pio::set_clkdiv(3);
    state.phase_swap.to_scan();
    state.dma.start_scan_endless(scan_buf);

    for _ in 0..SYNC_FRAMES {
        Timer::after_millis(10).await;

        state.dma.stop_scan();
        pio::wait_txstall();

        single_data_pass(state, frame_a).await;

        pio::set_clkdiv(3);
        state.phase_swap.to_scan();
        state.dma.start_scan_endless(scan_buf);
    }

    state.dma.stop_scan();
    pio::wait_txstall();

    // Seed FRAME_BUF_B's WR_CFG payload so both buffers carry identical
    // WR_CFG content in the main loop.
    unsafe {
        update_wr_cfg(&mut *STORAGE.frames[1].get(), CONFIG_REGS.len() - 1);
    }
}

async fn single_data_pass(state: &PanelState, buf: *const u32) {
    pio::set_clkdiv(2);
    state.phase_swap.to_data();
    state.dma.start_data(buf, DATA_END as u32);
    state.dma.wait_data().await;
    pio::wait_txstall();
}

async fn single_scan_pass(state: &PanelState, scan_buf: *const u32, n_cycles: u32) {
    pio::set_clkdiv(3);
    state.phase_swap.to_scan();
    state.dma.start_scan_bounded(scan_buf, n_cycles);
    state.dma.wait_scan().await;
    pio::wait_txstall();
}

async fn run_data_phase(state: &PanelState) {
    let active = state.active_buf.load(Ordering::Acquire) as usize;
    let buf = STORAGE.frames[active].get() as *const u32;

    pio::set_clkdiv(2);
    pio::enable_sm();
    state.phase_swap.to_data();
    state.dma.start_data(buf, DATA_END as u32);
    state.dma.wait_data().await;
    pio::wait_txstall();
    pio::disable_sm();
}

async fn run_scan_phase(state: &PanelState, scan_buf: *const u32) {
    let cycles = state.current_scan_cycles.load(Ordering::Relaxed);

    pio::set_clkdiv(3);
    pio::enable_sm();
    state.phase_swap.to_scan();
    state.dma.start_scan_bounded(scan_buf, cycles);
    state.dma.wait_scan().await;
    pio::wait_txstall();
    pio::disable_sm();
}

fn absorb_pack_done(state: &PanelState) {
    if let Some(done) = sio::fifo_try_read() {
        let packed = (done & 1) as u8;
        let prev_active = state.active_buf.load(Ordering::Relaxed);
        state.active_buf.store(packed, Ordering::Release);
        state.spare_buf.store(prev_active, Ordering::Release);
        state.frame_available.signal(());
    }
}

