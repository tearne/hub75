# usb-display

Receives RGB frames from a host computer over USB and displays them on a HUB75 panel. The panel is scanned autonomously by hardware via the [`hub75`](../hub75/) crate; this firmware is just glue between the USB endpoint and the panel.

Setup and flashing: [`SETUP.md`](../SETUP.md), [`FLASHING.md`](../FLASHING.md). See [`hub75/`](../hub75/) for the panel-driver implementations and architecture.

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

## Flash the firmware

```sh
cd firmware
cargo run --release --features panel-shift-64x32      # or panel-shift-64x64, panel-spwm-128x64
```

For BOOTSEL + `picotool` (no probe), see [`FLASHING.md`](../FLASHING.md).

## Client examples

### Python — `client/python/`

Runs via [uv](https://docs.astral.sh/uv/) (dependencies install automatically). Pass `--width` / `--height` matching the firmware.

| Script / pattern | What it draws |
|---|---|
| `hub75_client.py --pattern rainbow` (default) | Hue-shifting rainbow that scrolls horizontally over time |
| `hub75_client.py --pattern solid-red` (or `solid-green` / `solid-blue`) | Whole panel a single colour |
| `hub75_client.py --pattern gradient` | Red-to-blue horizontal gradient |
| `scanline_test.py` | Scanline alternating horizontal red and vertical cyan, used to compare USB throughput vs on-device generation |

```sh
./client/python/hub75_client.py --width 64 --height 32                          # rainbow
./client/python/hub75_client.py --width 64 --height 32 --pattern solid-red       # solid
./client/python/hub75_client.py --width 64 --height 32 --fps 15 /dev/ttyACM1     # explicit fps + port
./client/python/scanline_test.py  --width 64 --height 32                         # scanline
```

### Rust — `client/rust/`

```rust
use hub75_client::{Hub75Client, WIDTH, HEIGHT};

let mut client = Hub75Client::open_auto()?;
let frame = vec![[255, 0, 0]; WIDTH * HEIGHT];  // solid red
client.send_frame_rgb(&frame)?;
```

| Example | What it draws |
|---|---|
| `life` | Conway's Game of Life. Each cell carries its own hue; new births inherit a shifted average of their neighbours' hues. Compares each generation against the state from 6 generations ago: when ≥95 % of pixels match (which catches period-1/2/3 cycles), a handful of random "particles" is injected to perturb the field back into evolution. |
| `clock` | Mondaine-style Swiss Railway Clock — analogue face with a smoothly sweeping red second hand, fed from local time. |

```sh
cd client/rust
cargo run --release --example life  --features panel-64x32
cargo run --release --example clock --features panel-64x32
```

## Protocol

Each frame is a binary packet over USB CDC serial:

| Field | Size | Description |
|-------|------|-------------|
| Magic | 4 bytes | `HB75` (`0x48 0x42 0x37 0x35`) |
| Sequence | 1 byte | Wrapping counter (0–255) for dropped-frame detection |
| Pixels | `WIDTH × HEIGHT × 3` bytes | RGB, row-major, top-left origin (e.g. 6,144 bytes for 64×32, 12,288 bytes for 64×64) |

The client auto-detects the device by USB manufacturer (`tearne`) and product (`hub75`).
