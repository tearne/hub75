//! CDC ACM USB transport: appears as `/dev/ttyACM*` on Linux/macOS and
//! as a COM port on Windows via the in-box `usbser.sys` driver — no
//! admin install needed. Slower than vendor-class bulk because each
//! host OS interposes a serial/TTY abstraction; intended for hosts
//! where the WinUSB-style driver association isn't allowed.

use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::{PIN_14, PIN_15};
use embassy_rp::Peri;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Sender, State};
use embassy_usb::driver::Driver;
use static_cell::StaticCell;

use crate::{
    Debouncer, FirmwareConfig, FrameReceiver, BULK_MAX_PACKET, BUTTON_A_BIT, BUTTON_B_BIT,
    BUTTON_POLL_MS, FRAME_READY, RX_BUF, USB_PID, USB_VID,
};

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
    // IAD-composite device class triplet: Multi-Interface Function /
    // Common Class / Interface Association Descriptor. Required when
    // composite_with_iads is true (embassy-usb panics otherwise). Both
    // Linux's cdc_acm and Windows's usbser.sys dispatch per-interface
    // when this triplet is set at device level.
    usb_config.device_class = 0xEF;
    usb_config.device_sub_class = 0x02;
    usb_config.device_protocol = 0x01;
    usb_config.composite_with_iads = true;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        usb_config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    let cdc = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), BULK_MAX_PACKET);
    let (mut sender, mut receiver) = cdc.split();

    let mut usb = builder.build();

    let mut rx = FrameReceiver::new();
    let mut frames_this_period: u32 = 0;
    let mut last_report = embassy_time::Instant::now();

    let usb_fut = usb.run();
    let rx_fut = async {
        let mut buf = [0u8; BULK_MAX_PACKET as usize];
        loop {
            receiver.wait_connection().await;
            match receiver.read_packet(&mut buf).await {
                Ok(n) if n > 0 => {
                    let rx_buf = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUF) };
                    if rx.feed(&buf[..n], rx_buf) {
                        FRAME_READY.signal(());
                        frames_this_period += 1;

                        let now = embassy_time::Instant::now();
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

    let buttons_fut = button_emit_loop(button_a, button_b, &mut sender);

    embassy_futures::join::join3(usb_fut, rx_fut, buttons_fut).await;
}

async fn button_emit_loop<'d, D: Driver<'d>>(
    pin_a: Peri<'static, PIN_14>,
    pin_b: Peri<'static, PIN_15>,
    sender: &mut Sender<'d, D>,
) {
    sender.wait_connection().await;

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
            let _ = sender.write_packet(&[packed]).await;
            last_sent = Some(packed);
        }
        Timer::after(Duration::from_millis(BUTTON_POLL_MS)).await;
    }
}
