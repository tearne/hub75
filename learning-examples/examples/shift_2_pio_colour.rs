//! Per-pixel colour via BCM bitplane packing (CPU-driven scanning).
//!
//! Second in the three-part shift-register progression. Builds on
//! `shift_1_cpu`.
//!
//! **What this step adds:** an RGB framebuffer with 8-bit colour depth
//! via Binary Code Modulation (BCM). Each frame is scanned 8 times
//! (one per bitplane), with OE display time doubling for each higher
//! bit — giving 256 perceived brightness levels per channel.
//! Displays a scrolling HSV rainbow.
//!
//! **What the CPU still does:** everything — bitplane packing, pixel
//! clocking, row select, and BCM timing are all CPU-driven.
//!
//! ## Run
//!
//! ```sh
//! cargo run --release --example shift_2_pio_colour
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// ── HUB75 pin mapping ───────────────────────────────────────────────
const ALL_RGB: u32 = 0x3F;
const ADDR_MASK: u32 = 0x1F << 6;
const CLK: u32 = 1 << 11;
const LAT: u32 = 1 << 12;
const OE: u32 = 1 << 13;

// ── Panel geometry ──────────────────────────────────────────────────
const WIDTH: usize = 64;
const HEIGHT: usize = 64;
const ROW_PAIRS: usize = HEIGHT / 2;
const COLOR_DEPTH: usize = 8;
const WORDS_PER_ROW: usize = WIDTH / 4;

// ── Pixel helpers ───────────────────────────────────────────────────

#[derive(Copy, Clone)]
struct Rgb { r: u8, g: u8, b: u8 }

impl Rgb {
    const fn new(r: u8, g: u8, b: u8) -> Self { Self { r, g, b } }
    const BLACK: Rgb = Rgb::new(0, 0, 0);
}

fn hsv(hue: u8) -> Rgb {
    let h = hue as u16;
    let region = h / 43;
    let remainder = (h - region * 43) * 6;
    let q = (255 - remainder) as u8;
    let t = remainder as u8;
    match region {
        0 => Rgb::new(255, t, 0),  1 => Rgb::new(q, 255, 0),
        2 => Rgb::new(0, 255, t),  3 => Rgb::new(0, q, 255),
        4 => Rgb::new(t, 0, 255),  _ => Rgb::new(255, 0, q),
    }
}

// ── Framebuffer with BCM bitplane packing ───────────────────────────

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
            for px in row.iter_mut() { *px = Rgb::BLACK; }
        }
    }

    fn commit(&mut self) {
        for bit in 0..COLOR_DEPTH {
            for row in 0..ROW_PAIRS {
                for word_idx in 0..WORDS_PER_ROW {
                    let mut word: u32 = 0;
                    for pix in 0..4 {
                        let col = word_idx * 4 + pix;
                        let upper = &self.pixels[row][col];
                        let lower = &self.pixels[row + ROW_PAIRS][col];
                        let bits = ((upper.r >> bit) & 1) as u32
                            | (((upper.g >> bit) & 1) as u32) << 1
                            | (((upper.b >> bit) & 1) as u32) << 2
                            | (((lower.r >> bit) & 1) as u32) << 3
                            | (((lower.g >> bit) & 1) as u32) << 4
                            | (((lower.b >> bit) & 1) as u32) << 5;
                        word |= bits << (pix * 8);
                    }
                    self.packed[bit][row][word_idx] = word;
                }
            }
        }
    }
}

// ── Entry point ─────────────────────────────────────────────────────

#[hal::entry]
fn main() -> ! {
    // ── 1. Boot ──────────────────────────────────────────────────
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).unwrap();

    // ── 2. Pins ──────────────────────────────────────────────────
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS,
    );
    let _pins = (
        pins.gpio0.into_push_pull_output(),  pins.gpio1.into_push_pull_output(),
        pins.gpio2.into_push_pull_output(),  pins.gpio3.into_push_pull_output(),
        pins.gpio4.into_push_pull_output(),  pins.gpio5.into_push_pull_output(),
        pins.gpio6.into_push_pull_output(),  pins.gpio7.into_push_pull_output(),
        pins.gpio8.into_push_pull_output(),  pins.gpio9.into_push_pull_output(),
        pins.gpio10.into_push_pull_output(), pins.gpio11.into_push_pull_output(),
        pins.gpio12.into_push_pull_output(), pins.gpio13.into_push_pull_output(),
    );
    let sio = unsafe { &(*hal::pac::SIO::ptr()) };
    sio.gpio_out_set().write(|w| unsafe { w.bits(OE) });

    defmt::info!("BCM scrolling rainbow");

    // ── 3. Animation state ───────────────────────────────────────
    let mut fb = Framebuffer::new();
    let mut offset: u8 = 0;

    // ── 4. Main loop ─────────────────────────────────────────────
    loop {
        // Fill with scrolling HSV rainbow
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                let hue = (x as u32 * 255 / (WIDTH as u32 - 1)) as u8;
                fb.set_pixel(x, y, hsv(hue.wrapping_add(offset)));
            }
        }
        fb.commit();
        offset = offset.wrapping_add(2);

        // Scan: 8 bitplanes × 32 row-pairs, OE time doubles per bit
        for bit in 0..COLOR_DEPTH {
            let delay_cycles: u32 = 25 << bit;
            for row in 0..ROW_PAIRS as u32 {
                sio.gpio_out_set().write(|w| unsafe { w.bits(OE) });

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
    }
}
