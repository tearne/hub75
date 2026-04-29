# Setup

One-time setup for working on this project. Re-do these steps when bringing up a new machine; otherwise skip to [`FLASHING.md`](FLASHING.md) for the day-to-day workflow.

## Hardware

- [Pimoroni Interstate 75 W](https://shop.pimoroni.com/products/interstate-75-w) (RP2350A version).
- A HUB75 LED matrix panel. Two driver-chip families are supported by the [`hub75`](hub75/) crate; see that crate's README for the current per-panel feature list.
- Optional: a debug probe (Raspberry Pi Debug Probe or any CMSIS-DAP SWD probe) for `defmt` logs and one-step build-and-flash. Without one, you can still flash via USB BOOTSEL — see [`FLASHING.md`](FLASHING.md).

## Rust toolchain

```bash
rustup target add thumbv8m.main-none-eabihf
```

The workspace's per-crate `.cargo/config.toml` files set this as the build target.

## Tools

```bash
# probe-rs — flash + stream defmt logs over SWD (needs a probe)
cargo install probe-rs-tools
```

`picotool` (for BOOTSEL flashing without a probe) is installed separately; instructions live in [`FLASHING.md`](FLASHING.md).

## Linux USB permissions

Udev rules so non-root users can talk to the probe and to the board in BOOTSEL mode:

```bash
# Debug probe (probe-rs)
curl -sL https://probe.rs/files/69-probe-rs.rules | sudo tee /etc/udev/rules.d/69-probe-rs.rules > /dev/null

# RP2350 BOOTSEL mode (picotool)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-pico.rules > /dev/null

sudo udevadm control --reload-rules && sudo udevadm trigger
```
