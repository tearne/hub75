//! Shift-register HUB75 panel — the "regular" / "generic" HUB75 family.
//!
//! These panels use 74HC595-style shift registers as column drivers and
//! discrete address decoders for row selection. They have no on-chip
//! memory, no on-chip PWM: the host clocks one bitplane at a time and
//! the address SM holds OE on for a fraction of the line proportional
//! to the bitplane's weight (binary-code modulation).
//!
//! Two PIO state machines run autonomously, paced by a four-channel
//! self-feeding DMA chain. Once started, scanning runs forever with no
//! per-frame CPU involvement; the only host work is packing pixels
//! into the bitplane DMA buffer when [`ShiftPanel::commit`] is called.

mod dma;
mod pack;
mod pio;
mod timing;

use embassy_rp::Peri;
use embassy_rp::dma::ChannelInstance;
use embassy_rp::interrupt;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio as epio;
use embassy_rp::pio::Pio;

use crate::dp3364s::InterstatePins;
use crate::panel::{Panel, Rgb};

pub const COLOR_DEPTH: usize = 8;
pub const TIMING_BUF_WORDS: usize = COLOR_DEPTH * 2;

/// Default brightness on construction. Caller can override via
/// [`ShiftPanel::set_brightness`].
const DEFAULT_BRIGHTNESS: u8 = 64;

#[repr(C)]
pub struct ShiftStorage<const W: usize, const H: usize> {
    pub(crate) pixels: [[Rgb; W]; H],
    pub(crate) dma: [[[u32; W]; H]; 2],
    pub(crate) timing: [u32; TIMING_BUF_WORDS],
    /// Pointer the data-reload DMA channel reads to find the currently
    /// active bitplane buffer. Updated in `commit`.
    pub(crate) active_buf_ptr: u32,
    /// Pointer the time-reload DMA channel reads. Constant after init,
    /// but stored alongside so the address is `'static`.
    pub(crate) timing_ptr: u32,
}

impl<const W: usize, const H: usize> ShiftStorage<W, H> {
    pub const fn new() -> Self {
        Self {
            pixels: [[Rgb::BLACK; W]; H],
            dma: [[[0u32; W]; H]; 2],
            timing: [0u32; TIMING_BUF_WORDS],
            active_buf_ptr: 0,
            timing_ptr: 0,
        }
    }
}

impl<const W: usize, const H: usize> Default for ShiftStorage<W, H> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct ShiftPanel<const W: usize, const H: usize> {
    storage: &'static mut ShiftStorage<W, H>,
    spare: u8,
}

impl<const W: usize, const H: usize> ShiftPanel<W, H> {
    pub fn new<DmaData, DmaDataReload, DmaTime, DmaTimeReload, Irqs>(
        storage: &'static mut ShiftStorage<W, H>,
        pio0: Peri<'static, PIO0>,
        irqs: Irqs,
        pins: InterstatePins,
        _dma_data: Peri<'static, DmaData>,
        _dma_data_reload: Peri<'static, DmaDataReload>,
        _dma_time: Peri<'static, DmaTime>,
        _dma_time_reload: Peri<'static, DmaTimeReload>,
    ) -> Self
    where
        DmaData: ChannelInstance,
        DmaDataReload: ChannelInstance,
        DmaTime: ChannelInstance,
        DmaTimeReload: ChannelInstance,
        Irqs: interrupt::typelevel::Binding<
                interrupt::typelevel::PIO0_IRQ_0,
                epio::InterruptHandler<PIO0>,
            > + 'static,
    {
        // Compile-time sanity for the const generics.
        const { assert!(W >= 4 && W % 4 == 0, "W must be a multiple of 4") };
        const { assert!(H >= 2 && H % 2 == 0, "H must be even") };

        let scan_lines = (H / 2) as u8;
        let scan_minus_one = scan_lines - 1;
        let addr_pin_count = scan_lines.trailing_zeros() as u8; // log2

        // Fill timing + initial buffer pointers before the DMA chain
        // starts reading from them.
        timing::fill(&mut storage.timing, DEFAULT_BRIGHTNESS);
        storage.timing_ptr = storage.timing.as_ptr() as u32;
        storage.active_buf_ptr = storage.dma[0].as_flattened().as_ptr() as u32;

        let Pio { mut common, mut sm0, mut sm1, .. } = Pio::new(pio0, irqs);

        let r0 = common.make_pio_pin(pins.r0);
        let g0 = common.make_pio_pin(pins.g0);
        let b0 = common.make_pio_pin(pins.b0);
        let r1 = common.make_pio_pin(pins.r1);
        let g1 = common.make_pio_pin(pins.g1);
        let b1 = common.make_pio_pin(pins.b1);
        let addr_a = common.make_pio_pin(pins.addr_a);
        let addr_b = common.make_pio_pin(pins.addr_b);
        let addr_c = common.make_pio_pin(pins.addr_c);
        let addr_d = common.make_pio_pin(pins.addr_d);
        let addr_e = common.make_pio_pin(pins.addr_e);
        let clk = common.make_pio_pin(pins.clk);
        let lat = common.make_pio_pin(pins.lat);
        let oe = common.make_pio_pin(pins.oe);

        let progs = pio::assemble(scan_minus_one);
        let data_prog = common.load_program(&progs.data);
        let addr_prog = common.load_program(&progs.address);

        // ── Data SM (SM0) ────────────────────────────────────────
        let mut data_cfg = epio::Config::default();
        data_cfg.use_program(&data_prog, &[&clk, &lat]);
        data_cfg.set_out_pins(&[&r0, &g0, &b0, &r1, &g1, &b1]);
        data_cfg.shift_out = epio::ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: epio::ShiftDirection::Right,
        };
        data_cfg.fifo_join = epio::FifoJoin::TxOnly;
        data_cfg.clock_divider = 4u16.into();

        sm0.set_config(&data_cfg);
        sm0.set_pin_dirs(
            epio::Direction::Out,
            &[&r0, &g0, &b0, &r1, &g1, &b1, &clk, &lat],
        );

        // Push (W - 1) — the per-row pixel counter. The PIO program
        // pulls it into Y once at startup, before DMA takes over.
        sm0.tx().push((W - 1) as u32);

        // ── Address SM (SM1) ─────────────────────────────────────
        // All 5 address pins go in set_out_pins; we then override
        // PINCTRL.OUT_COUNT to addr_pin_count so `mov PINS, ~X`
        // writes only the bits the panel actually uses.
        let mut addr_cfg = epio::Config::default();
        addr_cfg.use_program(&addr_prog, &[&oe]);
        addr_cfg.set_out_pins(&[&addr_a, &addr_b, &addr_c, &addr_d, &addr_e]);
        addr_cfg.set_set_pins(&[&addr_a, &addr_b, &addr_c, &addr_d, &addr_e]);
        addr_cfg.shift_out = epio::ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: epio::ShiftDirection::Right,
        };
        addr_cfg.fifo_join = epio::FifoJoin::TxOnly;
        addr_cfg.clock_divider = 2u16.into();

        sm1.set_config(&addr_cfg);
        sm1.set_pin_dirs(
            epio::Direction::Out,
            &[&addr_a, &addr_b, &addr_c, &addr_d, &addr_e, &oe],
        );
        override_addr_out_count(addr_pin_count);

        // ── DMA chain ─────────────────────────────────────────────
        let txf_data_addr = sm0.tx_fifo_ptr() as u32;
        let txf_time_addr = sm1.tx_fifo_ptr() as u32;
        let data_words = (H as u32) * (W as u32);

        dma::start_chains(dma::ChainConfig {
            data_ch: <DmaData as ChannelInstance>::number(),
            data_reload_ch: <DmaDataReload as ChannelInstance>::number(),
            time_ch: <DmaTime as ChannelInstance>::number(),
            time_reload_ch: <DmaTimeReload as ChannelInstance>::number(),
            active_buf_ptr_addr: core::ptr::addr_of!(storage.active_buf_ptr) as u32,
            timing_buf_addr: core::ptr::addr_of!(storage.timing_ptr) as u32,
            data_words,
            txf_data_addr,
            txf_time_addr,
        });

        // ── Start ─────────────────────────────────────────────────
        sm0.set_enable(true);
        sm1.set_enable(true);
        core::mem::forget(sm0);
        core::mem::forget(sm1);

        Self { storage, spare: 1 }
    }

    /// Set global brightness (0–255). Takes effect on the next
    /// address-SM timing cycle without DMA reconfiguration.
    pub fn set_brightness(&mut self, level: u8) {
        timing::fill(&mut self.storage.timing, level);
    }
}

impl<const W: usize, const H: usize> Panel for ShiftPanel<W, H> {
    type Frame = [[Rgb; W]; H];
    const WIDTH: usize = W;
    const HEIGHT: usize = H;

    async fn frame_mut(&mut self) -> &mut Self::Frame {
        &mut self.storage.pixels
    }

    fn commit(&mut self) {
        let target = self.spare as usize;
        // SAFETY: we're about to flip `active_buf_ptr` to point at
        // `dma[target]`, but right now the DMA chain is reading from
        // `dma[1 - target]`. So `dma[target]` is exclusive to us.
        pack::pack_pixels(&self.storage.pixels, &mut self.storage.dma[target]);
        let new_ptr = self.storage.dma[target].as_flattened().as_ptr() as u32;
        // The next data-reload DMA cycle (one bitplane buffer worth
        // away) picks up the new pointer.
        self.storage.active_buf_ptr = new_ptr;
        self.spare = 1 - self.spare;
    }
}

fn override_addr_out_count(out_count: u8) {
    // PIO0 SM1 PINCTRL — bits 25:20 = OUT_COUNT.
    const SM1_PINCTRL: *mut u32 = (0x5020_0000 + 0x0F4) as *mut u32;
    unsafe {
        let mut v = SM1_PINCTRL.read_volatile();
        v &= !(0x3F << 20);
        v |= ((out_count as u32) & 0x3F) << 20;
        SM1_PINCTRL.write_volatile(v);
    }
}
