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

## Flashing

### With a debug probe (default)

The cargo runner in `learning-examples/.cargo/config.toml` is set to `probe-rs run --chip RP235x`, so `cargo run --release --example <name>` builds, flashes and streams `defmt` logs over SWD in one go. This is the smoothest workflow — use it whenever the probe is connected.

### Without a debug probe (USB BOOTSEL)

The RP2350A on the Interstate 75 W has USB BOOTSEL built in: hold **BOOT** while tapping **RESET** (or plug in USB with BOOT held) and the board becomes a flashable USB device. `cargo run` won't work — you need to build the ELF and transfer it separately. No `defmt` log output in this mode.

Use Raspberry Pi's official **`picotool`**. It reads the ELF directly, handles the RP2350 image format correctly, and loads over USB.

> **Don't use `elf2uf2-rs`.** It's a port of the RP2040-only `elf2uf2` and has no RP2350 support. It will happily transfer a UF2 to a BOOTSEL-mounted RP2350 board, but the bootloader silently rejects the file (wrong family ID `0xe48bff56` vs. the required `0xe48bff57`) and the board stays in BOOTSEL. Easy to waste hours on.

**Install picotool (prebuilt Linux x86_64 binary):**

```bash
curl -LO https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.2.0-3/picotool-2.2.0-a4-x86_64-lin.tar.gz
tar -xzf picotool-2.2.0-a4-x86_64-lin.tar.gz
sudo cp picotool/picotool /usr/local/bin/
```

Check the [latest release](https://github.com/raspberrypi/pico-sdk-tools/releases/latest) for newer versions. For other platforms (macOS, Windows, aarch64 Linux) grab the matching asset from that page.

**Build and flash:**

```bash
cd learning-examples
cargo build --release --example shift_1_cpu
```

Put the board in BOOTSEL (hold BOOT, tap RESET), then:

```bash
picotool load -v -x -t elf ../target/thumbv8m.main-none-eabihf/release/examples/shift_1_cpu
```

- `-v` verify after write
- `-x` execute (reboot into the new firmware)
- `-t elf` input is an ELF, not a UF2

`picotool info -a` is useful if you want to confirm which device it sees before loading.

## Acknowledgements

The autonomous DMA scanning architecture (shift-register examples) is based on [dgrantpete/Pi-Pico-Hub75-Driver](https://github.com/dgrantpete/Pi-Pico-Hub75-Driver).
