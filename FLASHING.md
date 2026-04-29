# Flashing

Three workflows. Pick whichever fits the situation. Setup steps are in [`SETUP.md`](SETUP.md).

## Path convention

`cargo build` runs inside a crate, but the workspace shares one `target/` at the **workspace root**. To avoid juggling `../target/...` vs `../../target/...` based on crate depth, **always quote `picotool` paths from the workspace root** — i.e. just `target/thumbv8m.main-none-eabihf/release/...`. Run `picotool` from the workspace root regardless of which crate you cd'd into to build.

The chained one-liner pattern parenthesises the `cd` so the second command runs from the workspace root:

```sh
(cd hub75 && cargo build --release --example test_pattern --features panel-shift-64x32) && \
  picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/examples/test_pattern
```

## Workflow A — probe + `cargo run` (fastest iteration)

For development. Requires a debug probe wired up. Builds, flashes over SWD, streams `defmt` logs back, all in one go.

```sh
cd <crate>             # e.g. hub75/, learning-examples/, usb-display/firmware/
cargo run --release --example <name> [--features <...>]
```

Ctrl+C disconnects without halting the firmware. The cargo runner is set per-crate in `.cargo/config.toml` to `probe-rs run --chip RP235x`.

## Workflow B — BOOTSEL + `picotool` (no probe needed)

For boards without a probe attached. Put the board in BOOTSEL — hold **BOOT** while tapping **RESET** (or plug in USB with BOOT held) — then build + flash in one go:

```sh
(cd <crate> && cargo build --release --example <name> [--features <...>]) && \
  picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/examples/<name>
```

For the firmware binary in `usb-display/firmware/`, use the binary name instead of `examples/<name>`:

```sh
(cd usb-display/firmware && cargo build --release --features panel-shift-64x32) && \
  picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/usb-display-firmware
```

`picotool` flags:

- `-v` verify after write
- `-x` execute (reboot into the new firmware)
- `-t elf` input is an ELF, not a UF2

`picotool info -a` confirms which device is in BOOTSEL before loading.

No `defmt` log output in this workflow.

> **Don't use `elf2uf2-rs`.** It's the RP2040-only `elf2uf2`. Its UF2s have the wrong family ID for the RP2350 (`0xe48bff56` vs the required `0xe48bff57`); the bootloader silently rejects them and the board sits in BOOTSEL doing nothing. Easy to waste hours on.

### Flashing a pre-built ELF

The `picotool load ...` half of Workflow B is all you need to flash an ELF that someone else built (or that you built earlier and saved). Skip the `cargo build` step and run `picotool` directly with the path to the ELF:

```sh
picotool load -v -x -t elf path/to/your-binary.elf
```

This is also how you'd ship a binary to someone for them to flash without giving them the source.

## Installing `picotool` (Linux x86_64)

```sh
curl -LO https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.2.0-3/picotool-2.2.0-a4-x86_64-lin.tar.gz
tar -xzf picotool-2.2.0-a4-x86_64-lin.tar.gz
sudo cp picotool/picotool /usr/local/bin/
```

Check the [latest release](https://github.com/raspberrypi/pico-sdk-tools/releases/latest) for newer versions. For other platforms (macOS, Windows, aarch64 Linux) grab the matching asset from that page.
