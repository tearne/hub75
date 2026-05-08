//! USB-driven HUB75 display firmware.
//!
//! Receives pixel frames over a vendor-class USB bulk endpoint and
//! pushes them to a HUB75 panel via `hub75::shift::ShiftPanel`
//! (shift-register family) or `hub75::Dp3364sPanel` (S-PWM family).
//! The panel feature selected at build time decides which family is
//! used; both run autonomously on PIO + DMA, so this firmware is just
//! glue between the USB endpoint and `panel.commit()`.
//!
//! USB descriptor: vendor-class (`0xFF`), one bulk OUT endpoint for
//! pixel frames + one bulk IN endpoint reserved for future telemetry
//! (declared but unused — adding it now avoids a future re-flash if
//! buttons or status reporting land later).

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
use embassy_usb::driver::{Endpoint, EndpointOut};
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

    defmt::info!("usb-serial firmware running");
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

/// pid.codes VID. PID 0x7575 chosen as a free entry; matches the
/// "hub75" project name as a digit pair. Optionally register at
/// pid.codes when convenient — not load-bearing for operation.
const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x7575;

/// Vendor-specific USB class triplet (no defined subclass/protocol).
const VENDOR_CLASS: u8 = 0xFF;

/// USB Full-Speed bulk endpoint max packet size.
const BULK_MAX_PACKET: u16 = 64;

/// USB serial-number string the firmware advertises.
///
/// Build with `PANEL_NAME=<name>` to bake in a friendly identifier;
/// otherwise the firmware emits the low 32 bits of the RP2350 OTP
/// chip ID as 8 lowercase hex chars, which is unique per board.
fn panel_serial_number() -> &'static str {
    if let Some(name) = option_env!("PANEL_NAME") {
        return name;
    }
    static BUF: StaticCell<[u8; 8]> = StaticCell::new();
    let buf = BUF.init([0u8; 8]);
    let low = embassy_rp::otp::get_chipid().unwrap_or(0) as u32;
    for i in 0..8 {
        let nibble = ((low >> ((7 - i) * 4)) & 0xF) as u8;
        buf[i] = if nibble < 10 { b'0' + nibble } else { b'a' + (nibble - 10) };
    }
    core::str::from_utf8(buf).unwrap()
}

#[embassy_executor::task]
async fn usb_task(usb_periph: Peri<'static, USB>) {
    let driver = usb::Driver::new(usb_periph, Irqs);

    let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("tearne");
    config.product = Some("hub75");
    config.serial_number = Some(panel_serial_number());
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    // Vendor-class at device level. We're a single-function device
    // and don't need Interface Association Descriptors, so disable
    // the IAD-composite default — otherwise embassy-usb panics
    // unless we set the IAD-composite class triplet (0xEF/0x02/0x01).
    config.device_class = VENDOR_CLASS;
    config.device_sub_class = 0;
    config.device_protocol = 0;
    config.composite_with_iads = false;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    // Single vendor-class function with one interface and two bulk
    // endpoints: OUT for pixel frames, IN reserved for future
    // telemetry (declared but never written to in this firmware).
    let (mut bulk_out, _bulk_in) = {
        let mut function = builder.function(VENDOR_CLASS, 0, 0);
        let mut interface = function.interface();
        let mut alt = interface.alt_setting(VENDOR_CLASS, 0, 0, None);
        let bulk_out = alt.endpoint_bulk_out(None, BULK_MAX_PACKET);
        let bulk_in = alt.endpoint_bulk_in(None, BULK_MAX_PACKET);
        drop(function);
        (bulk_out, bulk_in)
    };

    let mut usb = builder.build();

    let mut rx = FrameReceiver::new();
    let mut frames_this_period: u32 = 0;
    let mut last_report = embassy_time::Instant::now();

    let usb_fut = usb.run();
    let rx_fut = async {
        let mut buf = [0u8; BULK_MAX_PACKET as usize];
        loop {
            bulk_out.wait_enabled().await;
            match bulk_out.read(&mut buf).await {
                Ok(n) if n > 0 => {
                    let rx_buf = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUF) };
                    if rx.feed(&buf[..n], rx_buf) {
                        FRAME_READY.signal(());
                        frames_this_period += 1;

                        let now = embassy_time::Instant::now();
                        // 10 s averaging — frequent enough to see a
                        // hung host quickly, sparse enough not to spam.
                        if now.duration_since(last_report) >= embassy_time::Duration::from_secs(10)
                        {
                            defmt::info!("{} fps (10s avg)", frames_this_period / 10);
                            frames_this_period = 0;
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
