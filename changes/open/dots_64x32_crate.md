# `dots-64x32` example crate + Linux→Windows cross-compile guide

**Mode:** Formal

## Intent

Two outcomes:

1. **A reference example crate, `dots-64x32`**, that places a random-coloured dot at a random position every second, keeps a rolling buffer of the last 20, and re-renders on each tick. Purpose: the most obvious possible "how do I send data to the panel" reference — no abstractions beyond what the API itself requires, panel size hardcoded at 64×32 to keep the reader's attention on the data flow. Optimised for *time-to-comprehension*, not Rust idioms.

2. **A short, self-contained guide for cross-compiling the crate from Linux to Windows 11 x64**, in the same crate's `README.md`. Tight scope: target install, toolchain prereqs, the exact `cargo build` command, where the .exe lands. Specifically the CDC transport, since vendor-class on Windows usually requires an admin driver install that defeats cross-compiled distribution.

The crate sits at the top level alongside `sysmon/`, `usb/`, `usb-drop/`. It is its own standalone Cargo crate (not an in-tree example), so it demonstrates the realistic shape — `Cargo.toml` declaring `hub75-client` as a dependency with the right features pinned — that anyone copying it as a starter would write.

## Approach

### Layout

A single new top-level directory `dots-64x32/` with:

- `Cargo.toml` — declares the crate, depends on `hub75-client` via path (`../usb/client/rust`), pins `default-features = false, features = ["panel-64x32", "transport-cdc"]`, and depends on `rand` for picking dot positions/colours.
- `src/main.rs` — the whole example, single file.
- `README.md` — a brief explanation of what the example does, how to run it on Linux for local sanity-checking, and the cross-compile guide.
- `.cargo/config.toml` — sets the mingw-w64 linker for the `x86_64-pc-windows-gnu` target so `cargo build --target` works without env vars.

### Source shape

`src/main.rs` is ~50 lines:

- Open the panel with `Hub75Client::open(None)`.
- A `VecDeque<(x, y, [u8; 3])>` capacity 20 holding the rolling dot list.
- Loop: every second, push a new random dot; if at capacity, drop the oldest; clear the frame buffer; paint each dot; `send_frame_rgb`.
- No abstractions, no traits, no helper functions beyond a single `make_random_dot()`.

The single most important comment in the file is at the top: "the panel is 64×32; each frame is `64*32` `[r, g, b]` triples; send via `client.send_frame_rgb(&frame)`". Everything else is explanatory of *Rust syntax*, not of the protocol.

### Cross-compile toolchain choice

Two viable Linux→Windows paths:

- **`x86_64-pc-windows-gnu` + mingw-w64.** Available as a Debian/Ubuntu package (`mingw-w64`), no MSVC licensing dance. Produces a `.exe` that runs on stock Windows. The serialport crate compiles cleanly under this target (its Windows backend uses `windows-sys`, which works under either gnu or msvc).
- **`x86_64-pc-windows-msvc` + `cargo-xwin`.** Closer to what Windows-native developers produce, but requires downloading MSVC tooling.

Going with `windows-gnu`. The instructions become two `apt` lines + `rustup target add` + one `cargo build` invocation; nothing else.

### README structure

The README inside `dots-64x32/` has three sections:

- **What it does** — one paragraph, with an indication of which file to open.
- **Running on Linux** (for sanity-checking before shipping a Windows build). Single command using the CDC firmware on a `/dev/ttyACM*` device. References the parent project's `SETUP.md` for prerequisites.
- **Cross-compiling for Windows 11 x64.** The three commands needed, where the .exe lands, and one note about how to ship it to a Windows host (single .exe; mingw produces a binary that depends only on standard Windows runtime DLLs already present on Windows 11).

### Workspace caveat

The crate uses a path dependency on `../usb/client/rust`, so copying just `dots-64x32/` to a fresh location won't build standalone — the relative path breaks. The README notes this and tells anyone using this as a starter outside the workspace to switch the dep to a git reference (or to publish a tag of `hub75-client` first). Acceptable trade-off: an in-workspace example shouldn't pretend to be a real published-crate setup, and the path dep is what lets the example be exercised against unreleased changes to the client.

## Plan

- [x] Create `dots-64x32/` at the workspace root.
- [x] `dots-64x32/Cargo.toml`: name `dots-64x32`, version 0.1.0, `hub75-client` via path `../usb/client/rust` with `default-features = false, features = ["panel-64x32", "transport-cdc"]`, `rand = "0.9"`.
- [x] `dots-64x32/.cargo/config.toml` setting `linker = "x86_64-w64-mingw32-gcc"` for the `x86_64-pc-windows-gnu` target.
- [x] `dots-64x32/src/main.rs`: open panel, `VecDeque` of last 20 dots, 1 Hz loop, clear+paint+send each tick. Single `make_random_dot` helper. Header comment names the panel size and frame shape.
- [x] `dots-64x32/README.md`: what it does, run on Linux, cross-compile for Windows 11 x64 (mingw install line, `rustup target add`, `cargo build --release --target x86_64-pc-windows-gnu`, where the .exe lands, a note on dependency-on-workspace-path).
- [x] Build verify on Linux: `cargo build --release` from `dots-64x32/`.
- [x] Install `mingw-w64` and add the windows-gnu target, then cross-compile verify: `cargo build --release --target x86_64-pc-windows-gnu`. Confirm `dots-64x32.exe` lands at the documented path.
- [ ] Smoke-test on Linux against the CDC firmware: `cargo run --release` → dots appear on the panel, rolling 20.
- [x] Update top-level `README.md` `## Structure` table to include `dots-64x32/`.

## Log

- Cross-compile to `x86_64-pc-windows-gnu` failed initially because hub75-client used `SerialPortBuilder::exclusive()`, which is Unix-only (Windows has no flock/TIOCEXCL concept and the method isn't exposed there). Fixed in hub75-client 0.6.3 by cfg-gating the call to `#[cfg(unix)]`. Cross-build now produces a 421 KB self-contained `dots-64x32.exe`.
- Build is complete except for the on-hardware smoke test, which requires the panel attached and the CDC firmware flashed. Paused at the user's request, to be resumed potentially on a different machine.

### Resuming the smoke test

State at pause: every Plan task is ticked except the final "Smoke-test on Linux" item. `active.md` still points to this change. No source edits remain — the smoke test only exercises what's already on disk.

Prerequisites on the resuming machine:

- A 64×32 HUB75 panel attached, flashed with the `usb-class-cdc` firmware (`usb-firmware` 0.9.2 or later). If reflashing is needed, from `usb/firmware/`: `cargo build --release --no-default-features --features usb-class-cdc,panel-shift-64x32` then load via `picotool` or `probe-rs`.
- Linux build prereqs: `libudev-dev` (for the CDC client; see `usb/README.md`). No mingw or windows-gnu target needed unless re-running the cross-compile step.
- The panel must enumerate as `/dev/ttyACM*` (confirm with `ls /dev/ttyACM*`). If nothing else holds the port (`fuser /dev/ttyACM0` should be empty), it's ready.

To run:

```sh
cd dots-64x32
cargo run --release
```

Pass condition: a single new random-coloured pixel appears on the panel every ~1 second, with at most 20 visible at a time; once the queue is full each new pixel evicts the oldest. Ctrl+C stops cleanly.

After confirmation, the build is complete and the change can move to Conclusion + archive.
