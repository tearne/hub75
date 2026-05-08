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

## Targeting a specific panel

Each panel is uniquely identified by **two different IDs**, depending on whether it's running firmware or sitting in BOOTSEL:

| State | ID source | Visible via |
|---|---|---|
| Running firmware | RP2350 OTP chip ID | `lsusb -v` (`iSerial`) |
| In BOOTSEL | SPI flash chip's unique ID | `picotool info -a` |

For single-panel use, you can ignore both — there's only one device, no disambiguation needed. The rest of this section is for users juggling multiple panels.

### Friendly-name override (firmware build)

Bake a human-readable name into the firmware so it advertises that instead of the chip ID:

```sh
PANEL_NAME=living-room cargo build --release --features panel-shift-64x64
# (or PANEL_NAME=… cargo run … via the debug-probe path)
```

The name appears as `iSerial` in `lsusb -v` and is what host-side clients use to target this specific panel.

### Flash-time targeting (BOOTSEL workflow)

If multiple boards are in BOOTSEL at once, picotool will pick one — possibly not the one you mean. List all attached boards and target deliberately:

```sh
picotool info -a              # find the flash unique ID of each board
picotool load -v -x -t elf --ser <flash-id> path/to/firmware.elf
```

The debug-probe workflow targets physically (one probe wired to one board), so this isn't needed there.

### Runtime selectors (host clients)

The Rust client's `Hub75Client::open` takes an `Option<&str>` — `Some(serial)` to target a specific panel by `iSerial`, `None` for first match. The Python client takes a `serial=` kwarg.

The `life` and `clock` examples accept the serial as an optional positional argument:

```sh
cargo run --release --example life --features panel-shift-64x64 -- living-room
```

`sysmon` is hardcoded to open the panel whose `iSerial` is `sysmon` — i.e. a panel flashed with `PANEL_NAME=sysmon`. It will refuse to start if no such panel is attached.

## Driving the panel from a host

The `usb-serial` firmware speaks a vendor-class USB bulk protocol — see [`usb-serial/README.md`](usb-serial/README.md#usb-descriptor) for the full descriptor (VID/PID, endpoints, strings). Hosts talk to it via `libusb`, not as a serial port — there's no `/dev/ttyACM*`.

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

## Troubleshooting

### Panel display corrupts, USB devices drop unexpectedly

Symptoms vary by panel — random flashing pixels, garbled regions, frozen output — sometimes paired with the panel disappearing from `lsusb`, or unrelated USB devices (e.g. a debug probe) dropping at the same moment.

Likely USB **over-current**. The Pi's per-port current budget is small; HUB75 panels (especially larger ones) plus a debug probe and other accessories can exceed it. When the controller trips, port power cycles, the panel's internal state corrupts, and other devices on the bus drop with it.

Diagnose:

```sh
dmesg | grep over-current | tail -20
```

If you see `usbN-portM: over-current change #...` events — especially with counters in the hundreds or thousands — that's the cause. Software workarounds aren't reliable here. Mitigations:

- Powered USB hub between the Pi and the panels (most reliable).
- External power to the Pico via its `VSYS` / `VBUS` pin so it doesn't draw through the Pi's USB port.
- Higher-current Pi PSU and/or fewer USB peripherals on the same bus.
