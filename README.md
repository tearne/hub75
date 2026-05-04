# hub75

Rust firmware for driving HUB75 LED matrix panels from the [Pimoroni Interstate 75 W](https://shop.pimoroni.com/products/interstate-75-w) (RP2350A).

## Structure

| Directory | Description |
|-----------|-------------|
| [`hub75/`](hub75/) | Library crate — embassy-rp HUB75 panel driver. Two panel families (S-PWM and shift-register). |
| [`learning-examples/`](learning-examples/) | Self-contained bare-metal examples, progressing from CPU bit-bang to fully-autonomous PIO + DMA scanning. |
| [`usb-serial/`](usb-serial/) | USB CDC firmware: receives frames from a host program over USB serial. + Python and Rust host clients. |
| [`usb-drop/`](usb-drop/) | USB Mass Storage firmware: drag a frame file onto the Pico's drive to render it. |

## Getting started

- [`SETUP.md`](SETUP.md) — toolchain, probe-rs, udev rules, hardware list. One-time per machine.
- [`FLASHING.md`](FLASHING.md) — three workflows: probe + `cargo run`, BOOTSEL + `picotool`, build-once-share-ELF.

Each subdir has its own README with what's specific to that crate.

## Acknowledgements

The autonomous DMA scanning architecture (shift-register family) is based on [dgrantpete/Pi-Pico-Hub75-Driver](https://github.com/dgrantpete/Pi-Pico-Hub75-Driver).
