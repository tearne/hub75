//! Shared firmware library for USB-driven HUB75 displays.
//!
//! Product-specific binaries (the default firmware in `../firmware/`,
//! and `sysmon/firmware/`) supply panel construction, IRQ bindings, and
//! a [`FirmwareConfig`] with their USB identity. This crate carries
//! everything else: USB descriptor setup, the binary frame protocol on
//! the bulk OUT endpoint, the panel-feeder loop, and button polling on
//! the bulk IN endpoint.
//!
//! The USB device class is chosen by Cargo feature: `usb-class-vendor`
//! (default) advertises a vendor-specific interface with raw bulk
//! endpoints; `usb-class-cdc` advertises CDC ACM so the device appears
//! as `/dev/ttyACM*` on Linux or a COM port on Windows via the in-box
//! driver. The two features are mutually exclusive.
//!
//! Typical use from a binary:
//!
//! ```ignore
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) {
//!     let p = embassy_rp::init(Default::default());
//!     let panel = /* construct via hub75 */;
//!     let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);
//!     let config = FirmwareConfig { serial_number: "sysmon" };
//!     embassy_futures::join::join(
//!         run_panel(panel),
//!         run_usb_and_buttons(driver, p.PIN_14, p.PIN_15, &config),
//!     ).await;
//! }
//! ```

#![no_std]

#[cfg(all(feature = "usb-class-vendor", feature = "usb-class-cdc"))]
compile_error!("usb-class-vendor and usb-class-cdc are mutually exclusive");

#[cfg(not(any(feature = "usb-class-vendor", feature = "usb-class-cdc")))]
compile_error!("enable one of: usb-class-vendor, usb-class-cdc");

pub mod display;

#[cfg(feature = "usb-class-vendor")]
mod class_vendor;
#[cfg(feature = "usb-class-vendor")]
pub use class_vendor::run_usb_and_buttons;

#[cfg(feature = "usb-class-cdc")]
mod class_cdc;
#[cfg(feature = "usb-class-cdc")]
pub use class_cdc::run_usb_and_buttons;

pub use display::{FrameReceiver, ReceiveBuffer, HEIGHT, WIDTH};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use hub75::Panel;

/// pid.codes VID. PID 0x7575 chosen as a free entry; matches the
/// "hub75" project name as a digit pair. Optionally register at
/// pid.codes when convenient — not load-bearing for operation.
pub const USB_VID: u16 = 0x1209;
pub const USB_PID: u16 = 0x7575;

/// USB Full-Speed bulk endpoint max packet size.
pub const BULK_MAX_PACKET: u16 = 64;

/// Per-product configuration the binary supplies.
pub struct FirmwareConfig {
    /// USB serial number string. Bake an identifier per product
    /// (e.g. `"sysmon"`) so hosts can target it by name.
    pub serial_number: &'static str,
}

/// Cross-task channel from USB rx → panel feeder. The USB task fills
/// `RX_BUF` and signals; the panel feeder copies into `panel.frame_mut()`
/// and commits.
pub static FRAME_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static mut RX_BUF: ReceiveBuffer = ReceiveBuffer::new();

/// Feed completed frames from `RX_BUF` into the panel as they arrive.
/// Runs forever; await this from the binary's main.
pub async fn run_panel<P>(mut panel: P)
where
    P: Panel<Frame = [[hub75::Rgb; WIDTH]; HEIGHT]>,
{
    loop {
        FRAME_READY.wait().await;
        // SAFETY: the USB task only writes RX_BUF before signalling
        // FRAME_READY. This task consumes the signal and copies RX_BUF
        // before yielding, so RX_BUF accesses are serialised.
        let frame = panel.frame_mut().await;
        unsafe {
            *frame = (*core::ptr::addr_of!(RX_BUF)).pixels;
        }
        panel.commit();
    }
}

/// Bit positions in the packed button-state byte written on bulk IN.
pub const BUTTON_A_BIT: u8 = 0;
pub const BUTTON_B_BIT: u8 = 1;
/// Poll cadence for the buttons. With 3-sample debounce this gives a
/// 15 ms minimum press detection window — well under perceptual delay,
/// well above mechanical bounce on the I75 tactile switches.
pub(crate) const BUTTON_POLL_MS: u64 = 5;
pub(crate) const BUTTON_DEBOUNCE_SAMPLES: u8 = 3;

/// N-sample debouncer. The stable state flips only after
/// `BUTTON_DEBOUNCE_SAMPLES` consecutive matching readings.
pub(crate) struct Debouncer {
    stable: bool,
    candidate: bool,
    streak: u8,
}

impl Debouncer {
    pub(crate) fn new() -> Self {
        Self { stable: false, candidate: false, streak: 0 }
    }

    pub(crate) fn update(&mut self, sample: bool) -> bool {
        if sample == self.candidate {
            if self.streak < BUTTON_DEBOUNCE_SAMPLES {
                self.streak += 1;
            }
            if self.streak >= BUTTON_DEBOUNCE_SAMPLES {
                self.stable = sample;
            }
        } else {
            self.candidate = sample;
            self.streak = 1;
        }
        self.stable
    }
}
