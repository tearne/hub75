//! DP3364S concrete panel — Pimoroni Interstate 75 W (RP2350A) + 64×128 panel.

mod dma;
mod framing;
mod pack;
mod pack_worker;
mod pio;
mod scan;
mod scan_loop;
mod sio;
mod storage;

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::dma::ChannelInstance;
use embassy_rp::interrupt;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::{
    CORE1, PIO0,
    PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5,
    PIN_6, PIN_7, PIN_8, PIN_9, PIN_10,
    PIN_11, PIN_12, PIN_13,
};
use embassy_rp::pio as epio;
use embassy_rp::pio::Pio;
use embassy_sync::signal::Signal;
use static_cell::StaticCell;

pub use dma::DmaIrqHandler;

use crate::panel::{Panel, Rgb};
use scan_loop::PanelState;
use storage::STORAGE;

pub const WIDTH: usize = 128;
pub const HEIGHT: usize = 64;

pub type Frame = [[Rgb; WIDTH]; HEIGHT];

/// All 14 GPIOs the DP3364S panel uses, in their fixed roles on the
/// Interstate 75 W board.
pub struct InterstatePins {
    pub r0: Peri<'static, PIN_0>,
    pub g0: Peri<'static, PIN_1>,
    pub b0: Peri<'static, PIN_2>,
    pub r1: Peri<'static, PIN_3>,
    pub g1: Peri<'static, PIN_4>,
    pub b1: Peri<'static, PIN_5>,
    pub addr_a: Peri<'static, PIN_6>,
    pub addr_b: Peri<'static, PIN_7>,
    pub addr_c: Peri<'static, PIN_8>,
    pub addr_d: Peri<'static, PIN_9>,
    pub addr_e: Peri<'static, PIN_10>,
    pub clk: Peri<'static, PIN_11>,
    pub lat: Peri<'static, PIN_12>,
    pub oe: Peri<'static, PIN_13>,
}

pub struct Dp3364sPanel {
    state: &'static PanelState,
    pack_pending: bool,
}

impl Dp3364sPanel {
    pub const WIDTH: usize = WIDTH;
    pub const HEIGHT: usize = HEIGHT;

    /// Take all the peripherals the panel needs and start it scanning.
    /// On return the panel is alive: the scan task is running on core 0
    /// and the pack worker is running on core 1.
    pub fn new<DmaData, DmaScan, Irqs, const STACK_SIZE: usize>(
        spawner: Spawner,
        irqs: Irqs,
        pio0: Peri<'static, PIO0>,
        pins: InterstatePins,
        dma_data: Peri<'static, DmaData>,
        dma_scan: Peri<'static, DmaScan>,
        core1: Peri<'static, CORE1>,
        core1_stack: &'static mut Stack<STACK_SIZE>,
    ) -> Self
    where
        DmaData: ChannelInstance,
        DmaScan: ChannelInstance,
        Irqs: Copy
            + interrupt::typelevel::Binding<
                interrupt::typelevel::PIO0_IRQ_0,
                epio::InterruptHandler<PIO0>,
            >
            + interrupt::typelevel::Binding<interrupt::typelevel::DMA_IRQ_0, DmaIrqHandler>
            + 'static,
    {
        let phase_swap = configure_pio_sm(pio0, pins, irqs);
        configure_lat_pulldown();

        let dma_engine = dma::DmaEngine::new(dma_data, dma_scan, irqs);

        let state = init_panel_state(dma_engine, phase_swap);

        spawn_core1(core1, core1_stack, || pack_worker::pack_worker_loop());
        spawner.spawn(scan_loop::scan_loop_task(state).unwrap());

        Self { state, pack_pending: false }
    }

    /// Scan-cycle count per display frame. 50 → ~64 Hz / ~17 % dark
    /// (brightness sweet spot); 20 → ~128 Hz / ~34 % dark (refresh-rate
    /// sweet spot, default). Above ~57 the refresh enters perceptible
    /// flicker; below ~20 brightness loss is noticeable.
    pub fn set_scan_cycles(&mut self, cycles: u32) {
        self.state.current_scan_cycles.store(cycles, Ordering::Relaxed);
    }
}

impl Panel for Dp3364sPanel {
    type Frame = Frame;
    const WIDTH: usize = WIDTH;
    const HEIGHT: usize = HEIGHT;

    async fn frame_mut(&mut self) -> &mut Self::Frame {
        if self.pack_pending {
            self.state.frame_available.wait().await;
            self.pack_pending = false;
        }
        unsafe { &mut *STORAGE.pixels.get() }
    }

    fn commit(&mut self) {
        let target = self.state.spare_buf.load(Ordering::Acquire);
        self.state.frame_available.reset();
        self.pack_pending = true;
        sio::fifo_write(target as u32);
    }
}

// ── Setup helpers ───────────────────────────────────────────────────

fn configure_pio_sm<Irqs>(
    pio0: Peri<'static, PIO0>,
    pins: InterstatePins,
    irqs: Irqs,
) -> pio::PhaseSwap
where
    Irqs: interrupt::typelevel::Binding<
            interrupt::typelevel::PIO0_IRQ_0,
            epio::InterruptHandler<PIO0>,
        > + 'static,
{
    let Pio { mut common, mut sm0, .. } = Pio::new(pio0, irqs);

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

    let assembled = pio::assemble();
    let loaded = common.load_program(&assembled.program);
    let prog_offset = loaded.origin as u32;

    let mut cfg = epio::Config::default();
    cfg.use_program(&loaded, &[&clk]);
    cfg.set_out_pins(&[
        &r0, &g0, &b0, &r1, &g1, &b1, &addr_a, &addr_b, &addr_c, &addr_d, &addr_e,
    ]);
    cfg.set_set_pins(&[&lat, &oe]);
    cfg.shift_out = epio::ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: epio::ShiftDirection::Right,
    };
    cfg.fifo_join = epio::FifoJoin::TxOnly;
    cfg.clock_divider = 3u16.into();

    sm0.set_config(&cfg);
    sm0.set_pin_dirs(
        epio::Direction::Out,
        &[
            &r0, &g0, &b0, &r1, &g1, &b1,
            &addr_a, &addr_b, &addr_c, &addr_d, &addr_e,
            &clk, &lat, &oe,
        ],
    );
    sm0.set_enable(true);

    let base_execctrl = read_sm0_execctrl();
    core::mem::forget(sm0); // prevent Drop from disabling

    pio::PhaseSwap::new(&assembled, prog_offset, base_execctrl)
}

fn read_sm0_execctrl() -> u32 {
    const SM0_EXECCTRL: *const u32 = (0x5020_0000 + 0x0CC) as *const u32;
    unsafe { SM0_EXECCTRL.read_volatile() }
}

fn configure_lat_pulldown() {
    // Pull-down on LAT (GPIO 12) for safety during startup. The PIO
    // drives the pin actively once the SM is running, but until the
    // first phase-swap a stray transient on LAT could latch garbage.
    const PADS_BANK0: u32 = 0x4003_8000;
    const GPIO12_PAD: *mut u32 = (PADS_BANK0 + 0x04 + 12 * 4) as *mut u32;
    unsafe {
        let val = GPIO12_PAD.read_volatile();
        GPIO12_PAD.write_volatile(val | (1 << 2));
    }
}

fn init_panel_state(dma: dma::DmaEngine, phase_swap: pio::PhaseSwap) -> &'static PanelState {
    static PANEL_STATE: StaticCell<PanelState> = StaticCell::new();
    PANEL_STATE.init(PanelState {
        dma,
        phase_swap,
        current_scan_cycles: AtomicU32::new(20),
        active_buf: AtomicU8::new(0),
        spare_buf: AtomicU8::new(1),
        frame_available: Signal::new(),
    })
}
