//! Firmware for the panel that sysmon drives.
//!
//! Thin shell over `usb-firmware-lib`. The only product-specific
//! decisions live here: USB serial number is `"sysmon"` (so the host
//! sysmon process can target it by name) and the panel is the 64×32
//! shift-register family. Everything else — USB descriptor, frame
//! protocol, button polling — comes from the library.
//!
//! Build and flash:
//!   cargo run --release        (via probe-rs)
//! or:
//!   cargo build --release
//!   picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/sysmon-firmware

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp::usb;
use static_cell::StaticCell;

use hub75::shift::{ShiftPanel, ShiftStorage};
use hub75::InterstatePins;
use usb_firmware_lib::{run_panel, run_usb_and_buttons, FirmwareConfig, HEIGHT, WIDTH};

const PANEL_SERIAL: &str = "sysmon";

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

type ActivePanel = ShiftPanel<WIDTH, HEIGHT>;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let pins = InterstatePins {
        r0: p.PIN_0, g0: p.PIN_1, b0: p.PIN_2,
        r1: p.PIN_3, g1: p.PIN_4, b1: p.PIN_5,
        addr_a: p.PIN_6, addr_b: p.PIN_7, addr_c: p.PIN_8,
        addr_d: p.PIN_9, addr_e: p.PIN_10,
        clk: p.PIN_11, lat: p.PIN_12, oe: p.PIN_13,
    };

    let panel: ActivePanel = {
        static STORAGE: StaticCell<ShiftStorage<WIDTH, HEIGHT>> = StaticCell::new();
        let storage = STORAGE.init(ShiftStorage::new());
        ShiftPanel::new(storage, p.PIO0, Irqs, pins, p.DMA_CH0, p.DMA_CH1, p.DMA_CH2, p.DMA_CH3)
    };

    let driver = usb::Driver::new(p.USB, Irqs);
    let config = FirmwareConfig {
        serial_number: PANEL_SERIAL,
    };

    defmt::info!("sysmon-firmware running");
    embassy_futures::join::join(
        run_panel(panel),
        run_usb_and_buttons(driver, p.PIN_14, p.PIN_15, &config),
    )
    .await;
}
