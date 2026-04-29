//! Smoke test for the shift-register panel driver.
//!
//! Solid R / G / B / W fills (3 s each) followed by a scrolling diagonal
//! rainbow forever. Targets a 64×32 shift-register HUB75 panel — change
//! the `<64, 32>` const-generic argument on the `ShiftPanel` type to
//! match a different physical panel size.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_time::Timer;
use static_cell::StaticCell;

use hub75::shift::{ShiftPanel, ShiftStorage};
use hub75::{InterstatePins, Panel, Rgb};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let _ = spawner;
    let p = embassy_rp::init(Default::default());

    static STORAGE: StaticCell<ShiftStorage<64, 32>> = StaticCell::new();
    let storage = STORAGE.init(ShiftStorage::new());

    let pins = InterstatePins {
        r0: p.PIN_0, g0: p.PIN_1, b0: p.PIN_2,
        r1: p.PIN_3, g1: p.PIN_4, b1: p.PIN_5,
        addr_a: p.PIN_6, addr_b: p.PIN_7, addr_c: p.PIN_8,
        addr_d: p.PIN_9, addr_e: p.PIN_10,
        clk: p.PIN_11, lat: p.PIN_12, oe: p.PIN_13,
    };

    let mut panel: ShiftPanel<64, 32> = ShiftPanel::new(
        storage,
        p.PIO0,
        Irqs,
        pins,
        p.DMA_CH0,
        p.DMA_CH1,
        p.DMA_CH2,
        p.DMA_CH3,
    );

    defmt::info!("hub75 shift_rainbow example running");

    show_solid_fills(&mut panel).await;
    show_scrolling_rainbow(&mut panel).await;
}

async fn show_solid_fills(panel: &mut ShiftPanel<64, 32>) {
    let palette = [
        Rgb::new(255, 0, 0),
        Rgb::new(0, 255, 0),
        Rgb::new(0, 0, 255),
        Rgb::new(255, 255, 255),
    ];

    for color in palette {
        let frame = panel.frame_mut().await;
        for row in frame.iter_mut() {
            for px in row.iter_mut() {
                *px = color;
            }
        }
        panel.commit();
        Timer::after_secs(3).await;
    }
}

async fn show_scrolling_rainbow(panel: &mut ShiftPanel<64, 32>) -> ! {
    let mut offset: u8 = 0;
    loop {
        let frame = panel.frame_mut().await;
        for (row_idx, row) in frame.iter_mut().enumerate() {
            for (col_idx, px) in row.iter_mut().enumerate() {
                let hue = (row_idx as u16 + col_idx as u16 + offset as u16) as u8;
                *px = hsv(hue);
            }
        }
        panel.commit();
        offset = offset.wrapping_add(1);
        Timer::after_millis(15).await;
    }
}

fn hsv(hue: u8) -> Rgb {
    let h = hue as u16;
    let region = h / 43;
    let remainder = (h - region * 43) * 6;
    let q = (255 - remainder) as u8;
    let t = remainder as u8;
    match region {
        0 => Rgb::new(255, t, 0),
        1 => Rgb::new(q, 255, 0),
        2 => Rgb::new(0, 255, t),
        3 => Rgb::new(0, q, 255),
        4 => Rgb::new(t, 0, 255),
        _ => Rgb::new(255, 0, q),
    }
}
