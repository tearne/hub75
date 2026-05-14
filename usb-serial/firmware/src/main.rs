//! Default USB-driven HUB75 firmware. Thin shell over
//! `usb-serial-firmware-lib`. Identity is supplied via the `PANEL_NAME`
//! env var at build time, falling back to the RP2350 chip ID — fine
//! for ad-hoc boards and the `life` / `clock` host examples.
//!
//! Long-lived hosts (like `sysmon`) ship their own firmware binary
//! with the identity baked in — see `sysmon/firmware/`.

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

use hub75::InterstatePins;
use usb_serial_firmware_lib::{run_panel, run_usb_and_buttons, FirmwareConfig, HEIGHT, WIDTH};

#[cfg(any(feature = "panel-shift-64x64", feature = "panel-shift-64x32"))]
use hub75::shift::{ShiftPanel, ShiftStorage};

#[cfg(feature = "panel-spwm-128x64")]
use embassy_rp::multicore::Stack;
#[cfg(feature = "panel-spwm-128x64")]
use hub75::{DmaIrqHandler, Dp3364sPanel};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    #[cfg(feature = "panel-spwm-128x64")]
    DMA_IRQ_0 => DmaIrqHandler;
});

#[cfg(any(feature = "panel-shift-64x64", feature = "panel-shift-64x32"))]
type ActivePanel = ShiftPanel<WIDTH, HEIGHT>;
#[cfg(feature = "panel-spwm-128x64")]
type ActivePanel = Dp3364sPanel;

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

    #[cfg(any(feature = "panel-shift-64x64", feature = "panel-shift-64x32"))]
    let panel: ActivePanel = {
        static STORAGE: StaticCell<ShiftStorage<WIDTH, HEIGHT>> = StaticCell::new();
        let storage = STORAGE.init(ShiftStorage::new());
        ShiftPanel::new(storage, p.PIO0, Irqs, pins, p.DMA_CH0, p.DMA_CH1, p.DMA_CH2, p.DMA_CH3)
    };

    #[cfg(feature = "panel-spwm-128x64")]
    let panel: ActivePanel = {
        static CORE1_STACK: StaticCell<Stack<4096>> = StaticCell::new();
        let core1_stack = CORE1_STACK.init(Stack::new());
        Dp3364sPanel::new(_spawner, Irqs, p.PIO0, pins, p.DMA_CH0, p.DMA_CH1, p.CORE1, core1_stack)
    };

    let driver = usb::Driver::new(p.USB, Irqs);
    let config = FirmwareConfig {
        serial_number: panel_serial_number(),
    };

    defmt::info!("usb-serial firmware running");
    embassy_futures::join::join(
        run_panel(panel),
        run_usb_and_buttons(driver, p.PIN_14, p.PIN_15, &config),
    )
    .await;
}

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
