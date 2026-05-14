//! Shared firmware library for USB-driven HUB75 displays.
//!
//! Product-specific binaries (the default firmware in `../firmware/`,
//! and `sysmon/firmware/`) supply panel construction, IRQ bindings, and
//! a [`FirmwareConfig`] with their USB identity. This crate carries
//! everything else: USB descriptor setup, the binary frame protocol on
//! the bulk OUT endpoint, the panel-feeder loop, and button polling on
//! the bulk IN endpoint.
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

pub mod display;

pub use display::{FrameReceiver, ReceiveBuffer, HEIGHT, WIDTH};

use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::{PIN_14, PIN_15};
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_usb::driver::{Driver, Endpoint, EndpointIn, EndpointOut};
use hub75::Panel;
use static_cell::StaticCell;

/// pid.codes VID. PID 0x7575 chosen as a free entry; matches the
/// "hub75" project name as a digit pair. Optionally register at
/// pid.codes when convenient — not load-bearing for operation.
pub const USB_VID: u16 = 0x1209;
pub const USB_PID: u16 = 0x7575;

/// Vendor-specific USB class triplet (no defined subclass/protocol).
pub const VENDOR_CLASS: u8 = 0xFF;

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
const BUTTON_POLL_MS: u64 = 5;
const BUTTON_DEBOUNCE_SAMPLES: u8 = 3;

/// Configure the vendor-class USB descriptor, wire up the bulk OUT
/// receiver (frames) and bulk IN emitter (button events), and run
/// the lot. Returns never.
pub async fn run_usb_and_buttons<D: Driver<'static>>(
    driver: D,
    button_a: Peri<'static, PIN_14>,
    button_b: Peri<'static, PIN_15>,
    config: &FirmwareConfig,
) {
    let mut usb_config = embassy_usb::Config::new(USB_VID, USB_PID);
    usb_config.manufacturer = Some("tearne");
    usb_config.product = Some("hub75");
    usb_config.serial_number = Some(config.serial_number);
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;
    // Vendor-class at device level. We're a single-function device
    // and don't need Interface Association Descriptors, so disable
    // the IAD-composite default — otherwise embassy-usb panics
    // unless we set the IAD-composite class triplet (0xEF/0x02/0x01).
    usb_config.device_class = VENDOR_CLASS;
    usb_config.device_sub_class = 0;
    usb_config.device_protocol = 0;
    usb_config.composite_with_iads = false;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        usb_config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    // Single vendor-class function with one interface and two bulk
    // endpoints: OUT for pixel frames, IN for button events.
    let (mut bulk_out, mut bulk_in) = {
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

    let buttons_fut = button_emit_loop(button_a, button_b, &mut bulk_in);

    embassy_futures::join::join3(usb_fut, rx_fut, buttons_fut).await;
}

/// Poll the two buttons at `BUTTON_POLL_MS` cadence with an
/// `BUTTON_DEBOUNCE_SAMPLES`-deep debouncer per button. Each time the
/// packed state changes, write a single byte to the host on the
/// bulk IN endpoint. Buttons pull the line low when pressed.
async fn button_emit_loop(
    pin_a: Peri<'static, PIN_14>,
    pin_b: Peri<'static, PIN_15>,
    bulk_in: &mut impl EndpointIn,
) {
    bulk_in.wait_enabled().await;

    let input_a = Input::new(pin_a, Pull::Up);
    let input_b = Input::new(pin_b, Pull::Up);

    let mut state_a = Debouncer::new();
    let mut state_b = Debouncer::new();
    let mut last_sent: Option<u8> = None;

    loop {
        let pressed_a = state_a.update(input_a.is_low());
        let pressed_b = state_b.update(input_b.is_low());
        let packed = (pressed_a as u8) << BUTTON_A_BIT | (pressed_b as u8) << BUTTON_B_BIT;
        if last_sent != Some(packed) {
            // Best-effort: if the host isn't draining, drop the event
            // rather than block the poll loop. `write` returning Err
            // generally means the endpoint stalled or disconnected;
            // we'll resync on the next change.
            let _ = bulk_in.write(&[packed]).await;
            last_sent = Some(packed);
        }
        Timer::after(Duration::from_millis(BUTTON_POLL_MS)).await;
    }
}

/// N-sample debouncer. The stable state flips only after
/// `BUTTON_DEBOUNCE_SAMPLES` consecutive matching readings.
struct Debouncer {
    stable: bool,
    candidate: bool,
    streak: u8,
}

impl Debouncer {
    fn new() -> Self {
        Self { stable: false, candidate: false, streak: 0 }
    }

    fn update(&mut self, sample: bool) -> bool {
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
