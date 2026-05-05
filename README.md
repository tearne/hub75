# hub75

Rust firmware and host tooling for driving HUB75 LED matrix panels from the [Pimoroni Interstate 75 W](https://shop.pimoroni.com/products/interstate-75-w) (RP2350A).

## Structure

```
.
├ hub75/                  embassy-rp HUB75 panel driver library (S-PWM + shift-register)
├ learning-examples/      bare-metal examples, CPU bit-bang → autonomous PIO + DMA
├ usb-serial/             USB CDC firmware + Rust/Python host clients
├ usb-drop/               USB Mass Storage firmware: drop a file onto the Pico's drive
└ sysmon/                 Linux host system monitor (deb + systemd service)
```

Each crate stands alone — there is no top-level Cargo workspace. Build from inside the crate's own directory.

## Getting started

- [`SETUP.md`](SETUP.md) — toolchain, probe-rs, udev rules, hardware list. One-time per machine.
- [`FLASHING.md`](FLASHING.md) — three workflows: probe + `cargo run`, BOOTSEL + `picotool`, build-once-share-ELF.

Each subdir has its own README with what's specific to it.

## Acknowledgements

The autonomous DMA scanning architecture (shift-register family) is based on [dgrantpete/Pi-Pico-Hub75-Driver](https://github.com/dgrantpete/Pi-Pico-Hub75-Driver).
