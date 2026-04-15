//! Per-pixel colour via bitplane packing (CPU-driven scanning).
//!
//! Builds on `minimal_cpu` by adding a framebuffer, 8-bit colour depth
//! via Binary Code Modulation (BCM), and per-pixel drawing.
//! A filled circle smoothly cycles through hues to demonstrate.
//!
//! Run: cargo run --release --example bitplane_cpu

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const WIDTH: usize = 64;
const HEIGHT: usize = 64;
const ROW_PAIRS: usize = HEIGHT / 2;
const COLOR_DEPTH: usize = 8;
const WORDS_PER_ROW: usize = WIDTH / 4;

const ADDR_MASK: u32 = 0x1F << 6;
const LAT: u32 = 1 << 12;
const OE: u32 = 1 << 13;

// ── Simple RGB type ─────────────────────────────────────────────────

#[derive(Copy, Clone)]
struct Rgb {
    r: u8,
    g: u8,
    b: u8,
}

impl Rgb {
    const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
    const BLACK: Rgb = Rgb::new(0, 0, 0);
}

/// Simple HSV to RGB conversion (hue 0–255, full saturation and value).
fn hsv(hue: u8) -> Rgb {
    let h = hue as u16;
    let region = h / 43;
    let remainder = (h - region * 43) * 6;

    let p = 0u8;
    let q = (255 - remainder) as u8;
    let t = remainder as u8;

    match region {
        0 => Rgb::new(255, t, p),
        1 => Rgb::new(q, 255, p),
        2 => Rgb::new(p, 255, t),
        3 => Rgb::new(p, q, 255),
        4 => Rgb::new(t, p, 255),
        _ => Rgb::new(255, p, q),
    }
}

// ── Framebuffer ─────────────────────────────────────────────────────

struct Framebuffer {
    pixels: [[Rgb; WIDTH]; HEIGHT],
    packed: [[[u32; WORDS_PER_ROW]; ROW_PAIRS]; COLOR_DEPTH],
}

impl Framebuffer {
    fn new() -> Self {
        Self {
            pixels: [[Rgb::BLACK; WIDTH]; HEIGHT],
            packed: [[[0u32; WORDS_PER_ROW]; ROW_PAIRS]; COLOR_DEPTH],
        }
    }

    fn set_pixel(&mut self, x: usize, y: usize, color: Rgb) {
        if x < WIDTH && y < HEIGHT {
            self.pixels[y][x] = color;
        }
    }

    fn clear(&mut self) {
        for row in self.pixels.iter_mut() {
            for px in row.iter_mut() {
                *px = Rgb::BLACK;
            }
        }
    }

    /// Pack RGB pixels into bitplanes for scanning.
    fn commit(&mut self) {
        for bit in 0..COLOR_DEPTH {
            for row in 0..ROW_PAIRS {
                for word_idx in 0..WORDS_PER_ROW {
                    let mut word: u32 = 0;
                    for pix in 0..4 {
                        let col = word_idx * 4 + pix;
                        let upper = &self.pixels[row][col];
                        let lower = &self.pixels[row + ROW_PAIRS][col];

                        let r0 = ((upper.r >> bit) & 1) as u32;
                        let g0 = ((upper.g >> bit) & 1) as u32;
                        let b0 = ((upper.b >> bit) & 1) as u32;
                        let r1 = ((lower.r >> bit) & 1) as u32;
                        let g1 = ((lower.g >> bit) & 1) as u32;
                        let b1 = ((lower.b >> bit) & 1) as u32;

                        let pixel_data = r0 | (g0 << 1) | (b0 << 2)
                            | (r1 << 3) | (g1 << 4) | (b1 << 5);
                        word |= pixel_data << (pix * 8);
                    }
                    self.packed[bit][row][word_idx] = word;
                }
            }
        }
    }
}

// ── Entry point ─────────────────────────────────────────────────────

const ALL_RGB: u32 = 0x3F; // bits 0-5
const CLK: u32 = 1 << 11;

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000,
        pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS,
    );

    // All HUB75 pins as SIO outputs (CPU-driven, no PIO)
    let _pins = (
        pins.gpio0.into_push_pull_output(),
        pins.gpio1.into_push_pull_output(),
        pins.gpio2.into_push_pull_output(),
        pins.gpio3.into_push_pull_output(),
        pins.gpio4.into_push_pull_output(),
        pins.gpio5.into_push_pull_output(),
        pins.gpio6.into_push_pull_output(),
        pins.gpio7.into_push_pull_output(),
        pins.gpio8.into_push_pull_output(),
        pins.gpio9.into_push_pull_output(),
        pins.gpio10.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        pins.gpio12.into_push_pull_output(),
        pins.gpio13.into_push_pull_output(),
    );

    let sio = unsafe { &(*hal::pac::SIO::ptr()) };
    sio.gpio_out_set().write(|w| unsafe { w.bits(OE) });

    defmt::info!("Circle demo running");

    let mut fb = Framebuffer::new();
    let mut hue: u8 = 0;
    let mut frame_count: u32 = 0;

    let cx = WIDTH as i32 / 2;
    let cy = HEIGHT as i32 / 2;
    let radius_sq = 31i32 * 31;

    loop {
        // Update colour every 4 scan frames
        if frame_count % 4 == 0 {
            let color = hsv(hue);
            fb.clear();
            for y in 0..HEIGHT {
                for x in 0..WIDTH {
                    let dx = x as i32 - cx;
                    let dy = y as i32 - cy;
                    if dx * dx + dy * dy <= radius_sq {
                        fb.set_pixel(x, y, color);
                    }
                }
            }
            fb.commit();
            hue = hue.wrapping_add(1);
        }

        // Scan one frame: 8 bitplanes × 32 row-pairs
        for bit in 0..COLOR_DEPTH {
            let delay_cycles: u32 = 25 << bit;

            for row in 0..ROW_PAIRS as u32 {
                sio.gpio_out_set().write(|w| unsafe { w.bits(OE) });

                // Clock 64 pixels using packed bitplane data
                let row_data = &fb.packed[bit][row as usize];
                for &word in row_data {
                    for pix in 0..4 {
                        let bits = (word >> (pix * 8)) & 0x3F;
                        sio.gpio_out_clr().write(|w| unsafe { w.bits(ALL_RGB) });
                        sio.gpio_out_set().write(|w| unsafe { w.bits(bits) });
                        sio.gpio_out_set().write(|w| unsafe { w.bits(CLK) });
                        cortex_m::asm::nop();
                        sio.gpio_out_clr().write(|w| unsafe { w.bits(CLK) });
                    }
                }

                // Set row address, latch, enable
                sio.gpio_out_clr().write(|w| unsafe { w.bits(ADDR_MASK) });
                sio.gpio_out_set().write(|w| unsafe { w.bits(row << 6) });
                sio.gpio_out_set().write(|w| unsafe { w.bits(LAT) });
                cortex_m::asm::nop();
                sio.gpio_out_clr().write(|w| unsafe { w.bits(LAT) });
                sio.gpio_out_clr().write(|w| unsafe { w.bits(OE) });
                cortex_m::asm::delay(delay_cycles);
            }
        }
        sio.gpio_out_set().write(|w| unsafe { w.bits(OE) });

        frame_count = frame_count.wrapping_add(1);
    }
}
