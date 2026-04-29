//! Test-pattern cycle for any supported HUB75 panel.
//!
//! Drives the panel through a deliberate sequence so a user bringing a
//! new physical panel can quickly characterise its wiring:
//!
//! 1. Solid R, G, B, W (~2 s each) — confirms each channel lights and
//!    white balance is sane.
//! 2. Solid Y, C, M (~2 s each) — confirms channel order via two-channel
//!    mixes (yellow renders as magenta if G/B are swapped, etc.).
//! 3. A "sunset" scene: blue sky on top, orange near the horizon, dark
//!    ground on the bottom, and a bright yellow sun in the upper-left
//!    quadrant. Recognisable orientation: where the sun appears tells
//!    you both column 0 (sun on left) and row 0 (sky at top), and the
//!    sun's corner directly indicates the panel's rotation.
//! 4. A "bounce" animation: a white ball dropped from the top, falling
//!    under gravity and bouncing off the floor with damping. Removes
//!    any ambiguity about which direction is "down" — the ball settles
//!    on the bottom edge.
//!
//! Each step is logged via `defmt`; the cycle then loops forever.
//!
//! Build with one of the panel-selection features:
//! - `panel-shift-64x64`
//! - `panel-shift-64x32`
//! - `panel-spwm-128x64`

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

use hub75::{InterstatePins, Panel, Rgb};

#[cfg(any(feature = "panel-shift-64x64", feature = "panel-shift-64x32"))]
use hub75::shift::{ShiftPanel, ShiftStorage};

#[cfg(feature = "panel-spwm-128x64")]
use embassy_rp::multicore::Stack;
#[cfg(feature = "panel-spwm-128x64")]
use hub75::{DmaIrqHandler, Dp3364sPanel};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

#[cfg(not(any(
    feature = "panel-shift-64x64",
    feature = "panel-shift-64x32",
    feature = "panel-spwm-128x64",
)))]
compile_error!(
    "Select a panel: --features panel-shift-64x64 (or panel-shift-64x32 or panel-spwm-128x64)"
);

#[cfg(any(
    all(feature = "panel-shift-64x64", feature = "panel-shift-64x32"),
    all(feature = "panel-shift-64x64", feature = "panel-spwm-128x64"),
    all(feature = "panel-shift-64x32", feature = "panel-spwm-128x64"),
))]
compile_error!("Only one panel feature may be enabled");

#[cfg(feature = "panel-shift-64x64")]
const W: usize = 64;
#[cfg(feature = "panel-shift-64x64")]
const H: usize = 64;

#[cfg(feature = "panel-shift-64x32")]
const W: usize = 64;
#[cfg(feature = "panel-shift-64x32")]
const H: usize = 32;

#[cfg(feature = "panel-spwm-128x64")]
const W: usize = 128;
#[cfg(feature = "panel-spwm-128x64")]
const H: usize = 64;

#[cfg(any(feature = "panel-shift-64x64", feature = "panel-shift-64x32"))]
type ActivePanel = ShiftPanel<W, H>;
#[cfg(feature = "panel-spwm-128x64")]
type ActivePanel = Dp3364sPanel;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    #[cfg(feature = "panel-spwm-128x64")]
    DMA_IRQ_0 => DmaIrqHandler;
});

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
    let mut panel: ActivePanel = {
        let _ = spawner;
        static STORAGE: StaticCell<ShiftStorage<W, H>> = StaticCell::new();
        let storage = STORAGE.init(ShiftStorage::new());
        ShiftPanel::new(
            storage, p.PIO0, Irqs, pins,
            p.DMA_CH0, p.DMA_CH1, p.DMA_CH2, p.DMA_CH3,
        )
    };

    #[cfg(feature = "panel-spwm-128x64")]
    let mut panel: ActivePanel = {
        static CORE1_STACK: StaticCell<Stack<4096>> = StaticCell::new();
        let core1_stack = CORE1_STACK.init(Stack::new());
        Dp3364sPanel::new(
            spawner, Irqs, p.PIO0, pins,
            p.DMA_CH0, p.DMA_CH1, p.CORE1, core1_stack,
        )
    };

    defmt::info!("test_pattern cycling on {}×{}", W, H);

    loop {
        for (label, color) in &[
            ("RED",     Rgb::new(255, 0, 0)),
            ("GREEN",   Rgb::new(0, 255, 0)),
            ("BLUE",    Rgb::new(0, 0, 255)),
            ("WHITE",   Rgb::new(255, 255, 255)),
            ("YELLOW",  Rgb::new(255, 255, 0)),
            ("CYAN",    Rgb::new(0, 255, 255)),
            ("MAGENTA", Rgb::new(255, 0, 255)),
        ] {
            defmt::info!("solid {}", label);
            fill_with_label(&mut panel, *color, label).await;
            Timer::after_secs(2).await;
        }

        defmt::info!("sunset (sky top, sun in upper-left quadrant, dark ground bottom)");
        sunset(&mut panel).await;
        Timer::after_secs(5).await;

        defmt::info!("bounce (ball drops to bottom under gravity)");
        bounce(&mut panel).await;
    }
}

async fn fill_with_label(panel: &mut ActivePanel, color: Rgb, label: &str) {
    let frame = panel.frame_mut().await;
    for row in frame.iter_mut() {
        for px in row.iter_mut() {
            *px = color;
        }
    }
    let text_w = text_width(label);
    let x = W.saturating_sub(text_w) / 2;
    let y = H.saturating_sub(GLYPH_H + 2);
    draw_text(frame, x, y, label, Rgb::new(0, 0, 0));
    panel.commit();
}

async fn sunset(panel: &mut ActivePanel) {
    let horizon = (H / 2) as i32;
    let sun_cx = (W / 4) as i32;
    let sun_cy = horizon / 2;
    let sun_r = (core::cmp::min(W, H) / 8) as i32;
    let sun_r2 = sun_r * sun_r;

    let frame = panel.frame_mut().await;
    for (y, row) in frame.iter_mut().enumerate() {
        let yy = y as i32;
        for (x, px) in row.iter_mut().enumerate() {
            let xx = x as i32;

            // Sun: bright yellow disc in the upper-left quadrant.
            let dx = xx - sun_cx;
            let dy = yy - sun_cy;
            if dx * dx + dy * dy <= sun_r2 {
                *px = Rgb::new(255, 240, 80);
                continue;
            }

            // Sky vs ground gradient.
            *px = if yy < horizon {
                // Sky: deep blue at top → orange at horizon.
                lerp(yy, 0, horizon - 1, Rgb::new(20, 30, 120), Rgb::new(255, 80, 0))
            } else {
                // Ground: dark green near horizon → black at bottom.
                lerp(yy, horizon, H as i32 - 1, Rgb::new(30, 60, 20), Rgb::new(0, 0, 0))
            };
        }
    }
    panel.commit();
}

async fn bounce(panel: &mut ActivePanel) {
    let radius = (core::cmp::min(W, H) / 16).max(2) as i32;
    let mut cx_q8: i32 = ((W as i32) / 2) << 8;       // fixed-point: 1 px = 256 units
    let mut cy_q8: i32 = radius << 8;
    let mut vy_q8: i32 = 0;
    let gravity_q8: i32 = 96;                          // ~0.375 px/frame²
    let elasticity_pct: i32 = 82;                      // 82% retained on bounce

    let floor_q8 = ((H as i32 - 1 - radius) << 8);
    let frames = 30 * 5;                               // ~5 s at 30 fps

    for _ in 0..frames {
        vy_q8 += gravity_q8;
        cy_q8 += vy_q8;
        if cy_q8 >= floor_q8 {
            cy_q8 = floor_q8;
            vy_q8 = -vy_q8 * elasticity_pct / 100;
            // Stop when bounces get tiny.
            if vy_q8.abs() < 64 {
                vy_q8 = 0;
            }
        }

        let cxi = cx_q8 >> 8;
        let cyi = cy_q8 >> 8;
        let r2 = radius * radius;

        let frame = panel.frame_mut().await;
        for (y, row) in frame.iter_mut().enumerate() {
            for (x, px) in row.iter_mut().enumerate() {
                let dx = x as i32 - cxi;
                let dy = y as i32 - cyi;
                *px = if dx * dx + dy * dy <= r2 {
                    Rgb::new(255, 255, 255)
                } else {
                    Rgb::new(0, 0, 0)
                };
            }
        }
        panel.commit();
        Timer::after_millis(33).await;
    }
}

fn lerp(t: i32, t0: i32, t1: i32, c0: Rgb, c1: Rgb) -> Rgb {
    let span = (t1 - t0).max(1);
    let frac = ((t - t0).clamp(0, span) * 256) / span;
    Rgb::new(
        ((c0.r as i32 * (256 - frac) + c1.r as i32 * frac) / 256) as u8,
        ((c0.g as i32 * (256 - frac) + c1.g as i32 * frac) / 256) as u8,
        ((c0.b as i32 * (256 - frac) + c1.b as i32 * frac) / 256) as u8,
    )
}

// ── Tiny bitmap font (4 px wide × 5 px tall) ────────────────────────
// Each row is the low 4 bits of a u8, MSB first. Only uppercase
// letters used in our colour names are defined; missing glyphs are
// rendered as blank space.

const GLYPH_W: usize = 4;
const GLYPH_H: usize = 5;
const GLYPH_ADVANCE: usize = GLYPH_W + 1; // 1 px inter-character gap

fn glyph_for(c: char) -> Option<[u8; GLYPH_H]> {
    Some(match c {
        'A' => [0b0110, 0b1001, 0b1111, 0b1001, 0b1001],
        'B' => [0b1110, 0b1001, 0b1110, 0b1001, 0b1110],
        'C' => [0b0111, 0b1000, 0b1000, 0b1000, 0b0111],
        'D' => [0b1110, 0b1001, 0b1001, 0b1001, 0b1110],
        'E' => [0b1111, 0b1000, 0b1110, 0b1000, 0b1111],
        'G' => [0b0111, 0b1000, 0b1011, 0b1001, 0b0111],
        'H' => [0b1001, 0b1001, 0b1111, 0b1001, 0b1001],
        'I' => [0b1111, 0b0110, 0b0110, 0b0110, 0b1111],
        'L' => [0b1000, 0b1000, 0b1000, 0b1000, 0b1111],
        'M' => [0b1001, 0b1111, 0b1111, 0b1001, 0b1001],
        'N' => [0b1001, 0b1101, 0b1111, 0b1011, 0b1001],
        'O' => [0b0110, 0b1001, 0b1001, 0b1001, 0b0110],
        'R' => [0b1110, 0b1001, 0b1110, 0b1010, 0b1001],
        'T' => [0b1111, 0b0110, 0b0110, 0b0110, 0b0110],
        'U' => [0b1001, 0b1001, 0b1001, 0b1001, 0b1111],
        'W' => [0b1001, 0b1001, 0b1001, 0b1111, 0b1111],
        'Y' => [0b1001, 0b1001, 0b1111, 0b0110, 0b0110],
        _ => return None,
    })
}

fn text_width(text: &str) -> usize {
    let mut w: usize = 0;
    for _ in text.chars() {
        w += GLYPH_ADVANCE;
    }
    w.saturating_sub(1)
}

fn draw_text(frame: &mut [[Rgb; W]; H], x: usize, y: usize, text: &str, color: Rgb) {
    let mut cx = x;
    for ch in text.chars() {
        let upper = ch.to_ascii_uppercase();
        if let Some(glyph) = glyph_for(upper) {
            for (row_idx, row_bits) in glyph.iter().enumerate() {
                for col in 0..GLYPH_W {
                    if (row_bits >> (GLYPH_W - 1 - col)) & 1 == 1 {
                        let px = cx + col;
                        let py = y + row_idx;
                        if px < W && py < H {
                            frame[py][px] = color;
                        }
                    }
                }
            }
        }
        cx += GLYPH_ADVANCE;
    }
}
