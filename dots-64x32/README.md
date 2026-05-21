# dots-64x32

Minimal reference example for sending frames to a 64×32 HUB75 panel over USB CDC ACM. Every second it places a new random-coloured dot at a random position, keeps the most recent 20, and re-renders.

The entire example is in [`src/main.rs`](src/main.rs) — ~50 lines, one helper function, no abstractions. Open that file first to see how data reaches the panel.

The frame shape is the only protocol detail to know: the panel is 64 columns × 32 rows, and each frame is a `64 * 32` array of `[r, g, b]` triples (row-major, top-left origin), pushed via `client.send_frame_rgb(&frame)`.

## Run on Linux

The panel must be flashed with the `usb-class-cdc` firmware (see [`../usb/README.md`](../usb/README.md)) and show up as `/dev/ttyACM*`.

```sh
cargo run --release
```

## Cross-compile for Windows 11 x64 (from Linux)

One-time setup:

```sh
sudo apt install mingw-w64
rustup target add x86_64-pc-windows-gnu
```

Build:

```sh
cargo build --release --target x86_64-pc-windows-gnu
```

The executable lands at:

```
target/x86_64-pc-windows-gnu/release/dots-64x32.exe
```

It's self-contained — copy that single `.exe` to any Windows 11 host. The CDC firmware appears as a COM port via the in-box `usbser.sys` driver (no admin install needed), and the example opens the first matching panel automatically.

The mingw-w64 linker (`x86_64-w64-mingw32-gcc`) is wired up in [`.cargo/config.toml`](.cargo/config.toml); no environment variables needed.

## Using as a starter outside this workspace

`Cargo.toml` depends on `hub75-client` via a relative path (`../usb/client/rust`). If you copy this directory somewhere else, change that line to a git reference, e.g.:

```toml
hub75-client = { git = "https://github.com/tearne/hub75", default-features = false, features = ["panel-64x32", "transport-cdc"] }
```
