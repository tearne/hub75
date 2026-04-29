//! USB-driven HUB75 display firmware.
//!
//! Receives pixel frames over USB CDC and pushes them to a HUB75 panel
//! via `hub75::shift::ShiftPanel` (shift-register family) or
//! `hub75::Dp3364sPanel` (S-PWM family). The panel feature selected at
//! build time decides which family is used; both run autonomously on
//! PIO + DMA, so this firmware is just glue between the USB endpoint
//! and `panel.commit()`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp::usb;
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use static_cell::StaticCell;

use hub75::{InterstatePins, Panel};

#[cfg(any(feature = "panel-shift-64x64", feature = "panel-shift-64x32"))]
use hub75::shift::{ShiftPanel, ShiftStorage};

#[cfg(feature = "panel-spwm-128x64")]
use embassy_rp::multicore::Stack;
#[cfg(feature = "panel-spwm-128x64")]
use hub75::{DmaIrqHandler, Dp3364sPanel};

mod display;
use display::{FrameReceiver, ReceiveBuffer, HEIGHT, WIDTH};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    #[cfg(feature = "panel-spwm-128x64")]
    DMA_IRQ_0 => DmaIrqHandler;
});

/// Cross-task channel from USB rx → panel writer. The USB rx task fills
/// a static `ReceiveBuffer` and then signals; the panel task copies
/// from `RX_BUF` into `panel.frame_mut()` and commits.
static FRAME_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static mut RX_BUF: ReceiveBuffer = ReceiveBuffer::new();

#[cfg(any(feature = "panel-shift-64x64", feature = "panel-shift-64x32"))]
type ActivePanel = ShiftPanel<WIDTH, HEIGHT>;
#[cfg(feature = "panel-spwm-128x64")]
type ActivePanel = Dp3364sPanel;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let pins = InterstatePins {
        r0: p.PIN_0, g0: p.PIN_1, b0: p.PIN_2,
        r1: p.PIN_3, g1: p.PIN_4, b1: p.PIN_5,
        addr_a: p.PIN_6, addr_b: p.PIN_7, addr_c: p.PIN_8,
        addr_d: p.PIN_9, addr_e: p.PIN_10,
        clk: p.PIN_11, lat: p.PIN_12, oe: p.PIN_13,
    };

    #[cfg(any(feature = "panel-shift-64x64", feature = "panel-shift-64x32"))]
    let panel: ActivePanel = {
        static STORAGE: StaticCell<ShiftStorage<WIDTH, HEIGHT>> = StaticCell::new();
        let storage = STORAGE.init(ShiftStorage::new());
        ShiftPanel::new(
            storage,
            p.PIO0,
            Irqs,
            pins,
            p.DMA_CH0,
            p.DMA_CH1,
            p.DMA_CH2,
            p.DMA_CH3,
        )
    };

    #[cfg(feature = "panel-spwm-128x64")]
    let panel: ActivePanel = {
        static CORE1_STACK: StaticCell<Stack<4096>> = StaticCell::new();
        let core1_stack = CORE1_STACK.init(Stack::new());
        Dp3364sPanel::new(
            spawner,
            Irqs,
            p.PIO0,
            pins,
            p.DMA_CH0,
            p.DMA_CH1,
            p.CORE1,
            core1_stack,
        )
    };

    spawner.spawn(panel_task(panel).unwrap());
    spawner.spawn(usb_task(p.USB).unwrap());

    defmt::info!("usb-display firmware running");
}

#[embassy_executor::task]
async fn panel_task(mut panel: ActivePanel) {
    loop {
        FRAME_READY.wait().await;
        // SAFETY: the USB task only writes RX_BUF when FRAME_READY is
        // un-signalled. It signals after writing, then yields. This
        // task consumes the signal and copies RX_BUF before yielding,
        // so RX_BUF accesses are serialised.
        let frame = panel.frame_mut().await;
        unsafe {
            *frame = (*core::ptr::addr_of!(RX_BUF)).pixels;
        }
        panel.commit();
    }
}

#[embassy_executor::task]
async fn usb_task(usb_periph: Peri<'static, USB>) {
    let driver = usb::Driver::new(usb_periph, Irqs);

    let mut config = embassy_usb::Config::new(0x16c0, 0x27dd);
    config.manufacturer = Some("tearne");
    config.product = Some("hub75");
    config.serial_number = Some("001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64);
    let mut usb = builder.build();
    let (_, mut receiver) = class.split();

    defmt::info!("USB task started — waiting for frames");

    let mut rx = FrameReceiver::new();
    let mut frames_this_sec: u32 = 0;
    let mut last_report = embassy_time::Instant::now();

    let usb_fut = usb.run();
    let rx_fut = async {
        let mut buf = [0u8; 64];
        loop {
            match receiver.read_packet(&mut buf).await {
                Ok(n) if n > 0 => {
                    let rx_buf = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUF) };
                    if rx.feed(&buf[..n], rx_buf) {
                        FRAME_READY.signal(());
                        frames_this_sec += 1;

                        let now = embassy_time::Instant::now();
                        if now.duration_since(last_report) >= embassy_time::Duration::from_secs(1)
                        {
                            defmt::info!("{} fps received", frames_this_sec);
                            frames_this_sec = 0;
                            last_report = now;
                        }
                    }
                }
                _ => {}
            }
        }
    };

    embassy_futures::join::join(usb_fut, rx_fut).await;
}
