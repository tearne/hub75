# hub75

Rust firmware for driving HUB75 LED matrix panels from the [Pimoroni Interstate 75 W](https://shop.pimoroni.com/products/interstate-75-w) (RP2350A).

## Structure

| Directory | Description |
|-----------|-------------|
| [`learning-examples/`](learning-examples/) | Self-contained bare-metal examples progressing from CPU bit-bang to fully autonomous PIO+DMA scanning. Covers two families of HUB75 driver chip (shift-register BCM and S-PWM). |
| [`usb-display/`](usb-display/) | USB-driven display firmware (Embassy) + Python/Rust host clients |

## Prerequisites

```bash
# Rust with the ARM Cortex-M33 target
rustup target add thumbv8m.main-none-eabihf

# probe-rs for flashing and debugging via SWD
cargo install probe-rs-tools
```

### USB permissions (Linux)

```bash
# Debug probe (probe-rs)
curl -sL https://probe.rs/files/69-probe-rs.rules | sudo tee /etc/udev/rules.d/69-probe-rs.rules > /dev/null

# RP2350 BOOTSEL mode (picotool)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-pico.rules > /dev/null

sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Acknowledgements

The autonomous DMA scanning architecture (shift-register examples) is based on [dgrantpete/Pi-Pico-Hub75-Driver](https://github.com/dgrantpete/Pi-Pico-Hub75-Driver).
