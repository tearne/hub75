# usb

Receives RGB frames from a host computer over USB and displays them on a HUB75 panel. The panel is scanned autonomously by hardware via the [`hub75`](../hub75/) crate; this firmware is just glue between the USB endpoint and the panel.

Two USB-class transports are selected at firmware build time, with matching client builds:

| Transport | Firmware feature | Client feature / script | When to pick it |
|---|---|---|---|
| **Vendor-class bulk** (default) | `usb-class-vendor` | Rust `transport-vendor` (default), Python `hub75_client_vendor.py` | Max throughput. Needs a WinUSB-style driver association on Windows (typically admin). The default everywhere else. |
| **CDC ACM** | `usb-class-cdc` | Rust `transport-cdc`, Python `hub75_client_cdc.py` | Plug-and-play on locked-down Windows 11 (uses the in-box `usbser.sys` — no admin install). Slower; capped at 15 fps client-side until a target host is measured. |

The two USB classes are mutually exclusive at compile time — the firmware advertises one or the other, and clients pick the matching transport.

## USB descriptor

- **VID:** `0x1209` (pid.codes)
- **PID:** `0x7575`
- **Strings:** `manufacturer = "tearne"`, `product = "hub75"`, `serial_number = "001"` (or per-board `PANEL_NAME`/chip-ID)

Vendor-class build:
- **Class:** vendor-specific (`0xFF`)
- **Endpoints:** one bulk OUT for pixel frames (`0x01`), one bulk IN for button events (`0x81`)

CDC ACM build:
- **Class:** `0x02` (CDC) at device level, with the standard CDC ACM interface descriptors (one comms interface + one data interface, grouped by IAD)
- **Endpoints:** the CDC data interface's bulk pair carries pixel frames host→device and button events device→host

Setup and flashing: [`SETUP.md`](../SETUP.md). See [`hub75/`](../hub75/) for the panel-driver implementations and architecture.

## Panel options

Pick a firmware feature for the panel you have, build the firmware and Rust client with the matching size feature, and pass matching dimensions to the Python client.

| Family | Physical panel | Firmware feature | Rust client feature | Python `--width --height` |
|---|---|---|---|---|
| Shift-register | 64×64 | `panel-shift-64x64` | `panel-64x64` | `--width 64 --height 64` |
| Shift-register | 64×32 | `panel-shift-64x32` | `panel-64x32` | `--width 64 --height 32` |
| S-PWM (DP3364S) | 128×64 | `panel-spwm-128x64` | `panel-128x64` | `--width 128 --height 64` |

Firmware features include the family because the on-device driver code differs per family. Client features are size-only because the wire protocol is just bytes — the host doesn't care about the on-panel chip. Firmware and host must agree on size.

The firmware panel feature also enables any per-panel wiring corrections in [`hub75`](../hub75/) (e.g. G/B channel swap on the 64×32 shift panel; column-direction reversal on the 128×64 S-PWM panel). For new physical panels, run `cargo run --example test_pattern --features <panel>` from `hub75/` to characterise the wiring, then add corrections in the relevant `hub75/src/<family>/pack.rs` cfg-gated by the panel feature.

To add a new size, declare a new feature in `firmware/Cargo.toml` (forwarded to the matching `hub75/<feature>`) and `client/rust/Cargo.toml`, add matching `cfg` blocks for `WIDTH`/`HEIGHT`, and wire it through `firmware/src/main.rs`'s panel construction blocks.

## Firmware variants

The firmware is split into a shared library plus one binary per product:

| Crate | Role |
|---|---|
| [`firmware-lib/`](firmware-lib/) | Library with USB descriptor setup, frame protocol, button polling. No `main`. Class chosen by `usb-class-vendor` (default) / `usb-class-cdc` feature. |
| [`firmware/`](firmware/) | **Default firmware.** Thin binary using the lib. Identity from `PANEL_NAME` env var (fallback: chip ID). For ad-hoc boards and the `life` / `clock` host examples. |
| [`../sysmon/firmware/`](../sysmon/firmware/) | **sysmon firmware.** Thin binary using the lib. USB serial `"sysmon"` baked in, `panel-shift-64x32` selected. Vendor class only (sysmon's panel is dedicated and the host can install the driver). |

Long-lived hosts that target a panel by serial should ship their own binary crate in their directory rather than relying on the default firmware's env var.

## Flash the firmware

Vendor-class (default):
```sh
cd firmware
cargo run --release --features panel-shift-64x32      # or panel-shift-64x64, panel-spwm-128x64
```

CDC ACM:
```sh
cd firmware
cargo run --release --no-default-features --features usb-class-cdc,panel-shift-64x32
```

sysmon firmware:
```sh
cd ../sysmon/firmware
cargo run --release
```

For BOOTSEL + `picotool` (no probe), see [`SETUP.md`](../SETUP.md#flashing-via-bootsel).

## Client examples

### Python — `client/python/`

Runs via [uv](https://docs.astral.sh/uv/) (dependencies install automatically). Pass `--width` / `--height` matching the firmware.

| Script / pattern | What it draws |
|---|---|
| `hub75_client_vendor.py --pattern rainbow` (default) | Vendor transport; hue-shifting rainbow that scrolls horizontally over time |
| `hub75_client_vendor.py --pattern solid-red` (or `solid-green` / `solid-blue`) | Whole panel a single colour |
| `hub75_client_vendor.py --pattern gradient` | Red-to-blue horizontal gradient |
| `hub75_client_cdc.py --pattern gradient` | CDC transport; gradient (or solid-{r,g,b}). Capped at 15 fps. |
| `scanline_test.py` | Vendor transport; scanline alternating horizontal red and vertical cyan |

```sh
./client/python/hub75_client_vendor.py --width 64 --height 32
./client/python/hub75_client_cdc.py    --width 64 --height 32
./client/python/scanline_test.py       --width 64 --height 32
```

### Rust — `client/rust/`

```rust
use hub75_client::{Hub75Client, WIDTH, HEIGHT};

let mut client = Hub75Client::open(None)?;
let frame = vec![[255, 0, 0]; WIDTH * HEIGHT];  // solid red
client.send_frame_rgb(&frame)?;
```

The active transport is a build-time feature on the client crate:

```sh
cd client/rust
# Vendor (default)
cargo run --release --example life --features panel-64x32
# CDC
cargo run --release --example life --no-default-features --features panel-64x32,transport-cdc
```

| Example | What it draws |
|---|---|
| `life` | Conway's Game of Life with per-cell hues; stagnation-detect injects random particles every few generations when ≥95 % of pixels match the state from 6 generations ago. |
| `clock` | Mondaine-style Swiss Railway Clock — analogue face with a smoothly sweeping red second hand, fed from local time. |
| `buttons` | Smoke test for the button event channel. Prints each packed state byte. |

On Linux, the CDC build needs `libudev-dev` installed (build-time only; the `serialport` crate uses udev to enumerate USB-attached TTYs by VID/PID):

```sh
sudo apt install libudev-dev
```

Windows and macOS need no system packages for the CDC build.

### Sysmon

Linux host system monitor that consumes this protocol — promoted to its own top-level crate. See [`../sysmon/`](../sysmon/).

## Protocol

Each frame is a single transfer (bulk OUT for vendor class, serial write for CDC) carrying:

| Field | Size | Description |
|-------|------|-------------|
| Magic | 4 bytes | `HB75` (`0x48 0x42 0x37 0x35`) |
| Sequence | 1 byte | Wrapping counter (0–255) for dropped-frame detection |
| Pixels | `WIDTH × HEIGHT × 3` bytes | RGB, row-major, top-left origin (e.g. 6,144 bytes for 64×32, 12,288 bytes for 64×64) |

The client matches the device by VID/PID (`0x1209:0x7575`). The vendor client also sanity-checks the manufacturer/product strings (`tearne`/`hub75`); the CDC client matches on VID/PID alone.

### Button events

The firmware polls the two Interstate 75 buttons (A on GP14, B on GP15) every 5 ms with a 3-sample debounce. Each time the packed state changes, it writes one byte back to the host (on the vendor bulk IN endpoint, or down the CDC stream):

| Bit | Button | Meaning when set |
|-----|--------|------------------|
| 0   | A      | pressed          |
| 1   | B      | pressed          |
| 2–7 | —      | reserved (zero)  |

State, not edges: the host can recover the press/release of each button by xor-diffing successive bytes. A host that misses a packet or reconnects will resync on the next change. Writes are best-effort — if the host isn't draining, the firmware drops the event rather than block the poll loop.

The Rust client exposes this via `Hub75Client::recv_event(timeout)` → `Ok(Some(byte))` on event, `Ok(None)` on timeout. See `client/rust/examples/buttons.rs` for a worked example.
