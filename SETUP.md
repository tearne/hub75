# Setup

One-time setup and day-to-day flashing for this project. Pick a [flashing path](#flashing-paths-at-a-glance) first, then install only what that path needs.

## Hardware

- [Pimoroni Interstate 75 W](https://shop.pimoroni.com/products/interstate-75-w) (RP2350A version).
- A HUB75 LED matrix panel. Two driver-chip families are supported by the [`hub75`](./hub75/) crate; see that crate's README for the current per-panel feature list.
- Optional: a debug probe (Raspberry Pi Debug Probe or any CMSIS-DAP SWD probe) for `defmt` logs and one-step build-and-flash.

## Rust toolchain

Both flashing paths need this.

```bash
rustup target add thumbv8m.main-none-eabihf
```

The embedded crates' `.cargo/config.toml` files set this as the default build target.

## Flashing paths at a glance

| | BOOTSEL + `picotool` | Debug probe + `probe-rs` |
|---|---|---|
| Extra hardware | None | SWD debug probe |
| Flashing | Hold BOOT, tap RESET, run `picotool load` | `cargo run` builds, flashes, runs |
| `defmt` logs | No | Yes, streamed live |
| Best for | Occasional flashing, sharing a built ELF | Active development |

## Flashing via BOOTSEL

No extra hardware. Put the board in BOOTSEL — hold **BOOT** while tapping **RESET** (or plug in USB with BOOT held) — then build + flash in one go.

### Install `picotool` (Linux x86_64)

```sh
curl -LO https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.2.0-3/picotool-2.2.0-a4-x86_64-lin.tar.gz
tar -xzf picotool-2.2.0-a4-x86_64-lin.tar.gz
sudo cp picotool/picotool /usr/local/bin/
```

Check the [latest release](https://github.com/raspberrypi/pico-sdk-tools/releases/latest) for newer versions. For other platforms (macOS, Windows, aarch64 Linux) grab the matching asset from that page.

### Udev rule (Linux)

So non-root users can talk to the board in BOOTSEL mode:

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-pico.rules > /dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Flashing

```sh
cd <crate>
cargo build --release --example <name> [--features <...>] && \
  picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/examples/<name>
```

For the firmware binary in `usb-serial/firmware/`, use the binary name instead of `examples/<name>`:

```sh
cd usb-serial/firmware
cargo build --release --features panel-shift-64x32 && \
  picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/usb-serial-firmware
```

`picotool` flags:

- `-v` verify after write
- `-x` execute (reboot into the new firmware)
- `-t elf` input is an ELF, not a UF2

`picotool info -a` confirms which device is in BOOTSEL before loading.

No `defmt` log output in this workflow.

> **Don't use `elf2uf2-rs`.** It's the RP2040-only `elf2uf2`. Its UF2s have the wrong family ID for the RP2350 (`0xe48bff56` vs the required `0xe48bff57`); the bootloader silently rejects them and the board sits in BOOTSEL doing nothing. Easy to waste hours on.

### Flashing a pre-built ELF

The `picotool load` half is all you need to flash an ELF that someone else built (or that you built earlier and saved). Skip the `cargo build` step:

```sh
picotool load -v -x -t elf path/to/your-binary.elf
```

This is also how you'd ship a binary to someone for them to flash without giving them the source.

## Flashing via debug probe

Fastest iteration. Requires a debug probe wired up. Builds, flashes over SWD, streams `defmt` logs back, all in one go.

### Install `probe-rs`

```bash
cargo install probe-rs-tools
```

### Udev rule (Linux)

```bash
curl -sL https://probe.rs/files/69-probe-rs.rules | sudo tee /etc/udev/rules.d/69-probe-rs.rules > /dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Flashing

```sh
cd <crate>             # e.g. hub75/, learning-examples/, usb-serial/firmware/
cargo run --release --example <name> [--features <...>]
```

Ctrl+C disconnects without halting the firmware. The cargo runner is set per-crate in `.cargo/config.toml` to `probe-rs run --chip RP235x`.

## Driving the panel from a host

The `usb-serial` firmware speaks a vendor-class USB bulk protocol (VID `0x1209`, PID `0x7575` from pid.codes). Hosts talk to it via `libusb`, not as a serial port — there's no `/dev/ttyACM*`.

### libusb runtime (Linux)

```bash
sudo apt install libusb-1.0-0
```

For Python clients, add `libusb-1.0-0-dev` if you build wheels from source.

### Udev rule (Linux)

So non-root users can open the panel:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="7575", GROUP="dialout", MODE="0660"' \
  | sudo tee /etc/udev/rules.d/99-hub75-panel.rules > /dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Most desktop Linux installs (including Pi OS) put logins in `dialout` by default. Check with `groups`; if not, `sudo usermod -aG dialout $USER` and re-login.

> The sysmon `.deb` installs this rule automatically and reloads udev in its `postinst`. Skip the manual step if you're using the deb.
