# sysmon-firmware

Firmware for the panel that `sysmon` drives. Thin shell over [`usb-serial-firmware-lib`](../../usb-serial/firmware-lib/) — the bulk of the firmware logic (USB descriptor, frame protocol, button polling) lives in the library; this crate only supplies the sysmon-specific bits:

- USB serial number is `"sysmon"` so the host process can target the panel by name.
- Panel selected is the 64×32 shift-register family (`panel-shift-64x32` via the lib's feature gates).

## Build and flash

With a debug probe attached:

```sh
cargo run --release
```

Or via BOOTSEL + `picotool`:

```sh
cargo build --release
picotool load -v -x -t elf target/thumbv8m.main-none-eabihf/release/sysmon-firmware
```

No env vars or feature flags to remember — the identity and panel are baked into the binary.
