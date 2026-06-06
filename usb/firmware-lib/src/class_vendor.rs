//! Vendor-class USB transport: a single function with one interface and
//! two raw bulk endpoints (OUT for pixel frames, IN for button events).
//! Host clients claim the interface via libusb-style APIs and do bulk
//! I/O directly — no kernel TTY layer.

use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::{PIN_14, PIN_15};
use embassy_rp::Peri;
use embassy_time::{Duration, Timer};
use embassy_usb::driver::{Driver, Endpoint, EndpointIn, EndpointOut};
use static_cell::StaticCell;

use crate::{
    Debouncer, FirmwareConfig, FrameReceiver, BULK_MAX_PACKET, BUTTON_A_BIT, BUTTON_B_BIT,
    BUTTON_POLL_MS, FRAME_READY, RX_BUF, USB_PID, USB_VID,
};

/// Vendor-specific USB class triplet (no defined subclass/protocol).
const VENDOR_CLASS: u8 = 0xFF;

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
    usb_config.product = Some(crate::PRODUCT);
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
