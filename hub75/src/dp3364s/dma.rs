//! DMA channel setup and IRQ-driven completion signalling.
//!
//! Two channels:
//! - **Data channel** — one-shot, streams a frame buffer through the
//!   PIO TX FIFO. Triggered once per display frame.
//! - **Scan channel** — ring-mode, wraps a 32-word `SCAN_BUF` and feeds
//!   the same words into the PIO TX FIFO repeatedly. Bounded mode is
//!   used for the main loop (`n_cycles × 32` transfers, then stop and
//!   IRQ); endless mode for the boot sync phase (manually aborted).
//!
//! Both channels share `DMA_IRQ_0`. The crate exposes [`DmaIrqHandler`]
//! which the application binds via `bind_interrupts!`. Other DMA
//! channels also routing through DMA_IRQ_0 can be used by the
//! application as long as their bits do not collide with the panel's.

use core::sync::atomic::{AtomicU8, Ordering};

use embassy_rp::Peri;
use embassy_rp::dma::ChannelInstance;
use embassy_rp::interrupt;
use embassy_rp::interrupt::typelevel::Interrupt as _;
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TransCountMode, TreqSel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use super::framing::SCAN_LINES;

const PIO0_TXF0: u32 = 0x5020_0010;

// ── Completion signalling ───────────────────────────────────────────

static DATA_DMA_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static SCAN_DMA_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

static DATA_CH_NO: AtomicU8 = AtomicU8::new(0);
static SCAN_CH_NO: AtomicU8 = AtomicU8::new(0);

/// IRQ handler the application binds via `bind_interrupts!`:
///
/// ```ignore
/// embassy_rp::bind_interrupts!(struct Irqs {
///     DMA_IRQ_0 => hub75::DmaIrqHandler;
/// });
/// ```
pub struct DmaIrqHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::DMA_IRQ_0> for DmaIrqHandler {
    unsafe fn on_interrupt() {
        let dma = pac::DMA;
        let pending = dma.ints(0).read();
        dma.ints(0).write_value(pending);

        let data_bit = 1u32 << DATA_CH_NO.load(Ordering::Relaxed);
        let scan_bit = 1u32 << SCAN_CH_NO.load(Ordering::Relaxed);

        if pending & data_bit != 0 {
            DATA_DMA_DONE.signal(());
        }
        if pending & scan_bit != 0 {
            SCAN_DMA_DONE.signal(());
        }
    }
}

// ── Engine ──────────────────────────────────────────────────────────

pub struct DmaEngine {
    data_ch: u8,
    scan_ch: u8,
}

impl DmaEngine {
    pub fn new<DmaData, DmaScan>(
        _data: Peri<'static, DmaData>,
        _scan: Peri<'static, DmaScan>,
        _irq: impl interrupt::typelevel::Binding<interrupt::typelevel::DMA_IRQ_0, DmaIrqHandler>,
    ) -> Self
    where
        DmaData: ChannelInstance,
        DmaScan: ChannelInstance,
    {
        let data_ch = DmaData::number();
        let scan_ch = DmaScan::number();

        DATA_CH_NO.store(data_ch, Ordering::Relaxed);
        SCAN_CH_NO.store(scan_ch, Ordering::Relaxed);

        let dma = pac::DMA;
        dma.inte(0).modify(|w| *w |= (1u32 << data_ch) | (1u32 << scan_ch));

        interrupt::typelevel::DMA_IRQ_0::unpend();
        unsafe { interrupt::typelevel::DMA_IRQ_0::enable() };

        Self { data_ch, scan_ch }
    }

    /// Start a one-shot data-phase DMA: read `count` u32s from `buf`
    /// into the PIO TX FIFO. Raises DMA_IRQ_0 on completion.
    pub fn start_data(&self, buf: *const u32, count: u32) {
        let ch = pac::DMA.ch(self.data_ch as usize);
        ch.read_addr().write_value(buf as u32);
        ch.write_addr().write_value(PIO0_TXF0);
        ch.trans_count().write(|w| {
            w.set_count(count);
            w.set_mode(TransCountMode::NORMAL);
        });
        ch.ctrl_trig().write(|w| {
            w.set_en(true);
            w.set_data_size(DataSize::SIZE_WORD);
            w.set_incr_read(true);
            w.set_incr_write(false);
            w.set_chain_to(self.data_ch);
            w.set_treq_sel(TreqSel::from_bits(0));
        });
    }

    /// Start a bounded ring-mode scan DMA: replay `SCAN_BUF` for
    /// `n_cycles` complete passes, wrapping the read address every 32
    /// words. Raises DMA_IRQ_0 on completion.
    pub fn start_scan_bounded(&self, scan_buf: *const u32, n_cycles: u32) {
        let count = n_cycles * SCAN_LINES as u32;
        self.configure_scan_ring(scan_buf, count, TransCountMode::NORMAL);
    }

    /// Start an endless ring-mode scan DMA — runs forever until
    /// `stop_scan` is called. Used during the boot sync phase. Does
    /// not raise an IRQ on its own (it never completes).
    pub fn start_scan_endless(&self, scan_buf: *const u32) {
        self.configure_scan_ring(scan_buf, SCAN_LINES as u32, TransCountMode::ENDLESS);
    }

    fn configure_scan_ring(&self, scan_buf: *const u32, count: u32, mode: TransCountMode) {
        let ch = pac::DMA.ch(self.scan_ch as usize);
        ch.read_addr().write_value(scan_buf as u32);
        ch.write_addr().write_value(PIO0_TXF0);
        ch.trans_count().write(|w| {
            w.set_count(count);
            w.set_mode(mode);
        });
        ch.ctrl_trig().write(|w| {
            w.set_en(true);
            w.set_data_size(DataSize::SIZE_WORD);
            w.set_incr_read(true);
            w.set_incr_write(false);
            w.set_chain_to(self.scan_ch);
            w.set_treq_sel(TreqSel::from_bits(0));
            w.set_ring_size(7); // 2^7 bytes = 32 u32 words
            w.set_ring_sel(false); // wrap read addresses
        });
    }

    /// Abort the running scan DMA and wait for the abort to complete.
    pub fn stop_scan(&self) {
        let dma = pac::DMA;
        let mask = 1u16 << self.scan_ch;
        dma.chan_abort().write(|w| w.set_chan_abort(mask));
        while dma.chan_abort().read().chan_abort() & mask != 0 {
            core::hint::spin_loop();
        }
    }

    pub async fn wait_data(&self) {
        DATA_DMA_DONE.wait().await;
    }

    pub async fn wait_scan(&self) {
        SCAN_DMA_DONE.wait().await;
    }

    /// Discard any pending completion signals. Used after the boot
    /// flush/sync phases, whose DMAs have set INTS0 bits that would
    /// make the first main-loop wait return spuriously.
    pub fn reset_signals(&self) {
        DATA_DMA_DONE.reset();
        SCAN_DMA_DONE.reset();
        let dma = pac::DMA;
        dma.ints(0).write_value(0xFFFF_FFFF);
    }
}
