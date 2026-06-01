# dots-64x32

The smallest useful "how do I send data to the panel" example. Every second it drops a random-coloured dot at a random position on a 64×32 panel, keeping the last 20 on screen.

## What it does

All the logic is in [`src/main.rs`](src/main.rs) — one short file. The one thing to take away: a frame is exactly `64 * 32` `[r, g, b]` pixels in a flat array (row by row, top-left first), and you send it with `client.send_frame_rgb(&frame)`. Everything else in the file is just deciding what colour to make each pixel.

The panel size is hardcoded at 64×32 to keep the example focused on the data flow rather than on generality.

## Flashing the panel firmware

This example talks **CDC** — the panel appears as a serial port (`/dev/ttyACM*`), which needs no admin driver install on any host. If your unit isn't already running the CDC firmware, flash it from [`usb/firmware/`](../usb/firmware/) with the matching 64×32 panel feature.

With a debug probe attached:

```sh
cd ../usb/firmware
cargo run --release --no-default-features --features usb-class-cdc,panel-shift-64x32
```

No probe? Flash over BOOTSEL with `picotool` instead — see [`SETUP.md`](../SETUP.md#flashing-via-bootsel) for the toolchain and steps. Full firmware details and the vendor-vs-CDC trade-off are in [`usb/README.md`](../usb/README.md).

## Running on Linux

With a CDC panel plugged in:

```sh
cargo run --release
```

`Hub75Client::open(None)` grabs the first panel it finds, so there's nothing to configure.

## Cross-compiling for Windows 11 x64

CDC needs no admin driver install on Windows, so a single cross-compiled `.exe` runs on a stock Windows 11 machine. Build it from Linux with the mingw-w64 toolchain:

```sh
sudo apt install mingw-w64                          # the cross-linker (Debian/Ubuntu)
rustup target add x86_64-pc-windows-gnu             # the Rust std for the target
cargo build --release --target x86_64-pc-windows-gnu
```

The `.exe` lands at:

```
target/x86_64-pc-windows-gnu/release/dots-64x32.exe
```

Copy that single file to the Windows host by any normal means (USB stick, network share, `scp` — the panel itself is only a display device and can't ferry it) and run it — mingw produces a binary that depends only on standard Windows runtime DLLs already present on Windows 11. The linker is wired up in [`.cargo/config.toml`](.cargo/config.toml), so no environment variables are needed.

## Using this as a starter outside the workspace

This crate depends on `hub75-client` by **relative path** (`../usb/client/rust`), which only resolves inside this repository. If you copy `dots-64x32/` somewhere else as a starting point, switch that dependency in `Cargo.toml` to a git reference — Cargo finds the `hub75-client` crate inside the repo by name:

```toml
hub75-client = { git = "https://github.com/tearne/hub75", default-features = false, features = ["panel-64x32", "transport-cdc"] }
```

That pulls the latest commit on the default branch. To pin a specific version, add `tag = "..."` or `rev = "..."`.
