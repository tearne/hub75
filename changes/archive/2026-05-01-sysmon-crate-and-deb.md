# Sysmon as its own crate + deb package + boot service

## Intent

Promote `sysmon_linux` from an example inside the `hub75-client` crate to a first-class binary in its own crate. Package it as a Debian `.deb` so the user can `apt install` (or `dpkg -i`) it on a Raspberry Pi and have a systemd service automatically run it on boot. Include user-facing documentation in the new crate explaining how to flash the matching firmware to the Pico, install the package, and check the service status.

The goal is "plug the Pi 5 into a fresh Raspberry Pi OS, install the deb and the firmware, plug in the Pico, and the panel comes alive on every reboot" — without the user having to run `cargo` themselves.

Crate name: `sysmon` (short, matches the binary). The README will be explicit that only Raspberry Pi 5 has been tested.

## Approach

### New crate at `usb-display/client/sysmon/`

A sibling to `usb-display/client/rust/`. Name: `sysmon`. Single binary `src/main.rs` (the existing `sysmon_linux.rs` body, renamed). Path dependency on `hub75-client` so changes to the client library propagate. Hardcodes `hub75-client/panel-64x32` as the only panel feature — one deb per panel geometry; if other sizes are needed later, additional packages.

### Excluded from the workspace, like the existing rust client

The new crate is host-side; embedded toolchain settings in the workspace would conflict. Add `usb-display/client/sysmon` to the existing `exclude = [...]` list in the root `Cargo.toml`. Each host-side crate has its own `.cargo/config.toml` pinning its build target.

### Build target: aarch64

`.cargo/config.toml` in the new crate sets `target = "aarch64-unknown-linux-gnu"` so `cargo build` on the Pi produces the right binary. The README will document that the deb is built and tested only on the Pi 5.

### Deb packaging via `cargo-deb`

Add `[package.metadata.deb]` to the new crate's `Cargo.toml`. Run `cargo deb` from inside the crate to produce a `.deb` under `target/aarch64-unknown-linux-gnu/debian/`. The metadata covers the maintainer, summary, dependencies (`libudev1` if needed by `serialport`), and the file mappings:

- `target/.../release/sysmon` → `/usr/bin/sysmon`
- `debian/sysmon.service` → `/lib/systemd/system/sysmon.service`
- `README.md` → `/usr/share/doc/sysmon/README.md`

### Service: systemd, runs as root, auto-restart, auto-start on boot

Simplest possible service file:

```
[Unit]
Description=HUB75 panel system monitor
After=multi-user.target

[Service]
ExecStart=/usr/bin/sysmon
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Runs as root — avoids fiddling with udev rules and group membership for `/dev/ttyACM*` access. Trade-off accepted because this is a hobby panel on a single-user Pi.

A maintainer-script hook (`postinst`) enables and starts the service after install:

```
systemctl daemon-reload
systemctl enable sysmon.service
systemctl start sysmon.service
```

### Move sysmon out of the rust client crate

Once the new crate ships its own binary, the `examples/sysmon_linux.rs` and `run-sysmon.sh` in `usb-display/client/rust/` are duplicates and can go. The `usb-display/README.md` table updates to mention sysmon now lives in its own subdirectory.

### README in the new crate

User-facing install guide:

1. **Hardware compatibility** — only tested on Raspberry Pi 5. Should work on any aarch64 Debian-based system with USB CDC support.
2. **Flash the Pico firmware** — short walkthrough that points to the existing `FLASHING.md` for full detail. Specifically the BOOTSEL + `picotool load` flow with the `panel-shift-64x32` feature.
3. **Install udev rules** — point to `SETUP.md` for the existing `99-pico.rules` snippet so the running firmware's USB CDC interface is accessible (if the service ever runs as non-root in future).
4. **Install the deb** — `sudo dpkg -i sysmon_<version>_arm64.deb`. The postinst enables and starts the service.
5. **Verify** — `systemctl status sysmon`, `journalctl -u sysmon -f`.
6. **Override flags** — if the user wants `-u 5000` or a specific port, edit `/etc/systemd/system/sysmon.service.d/override.conf`.

### Versioning

The new crate starts at `0.1.0`. It's separately versioned from `hub75-client`, since the API surface is different (one crate is a library, the other is a service binary).

## Plan

The new crate starts at `0.1.0`. No version bump on `hub75-client`.

- [x] Create `usb-display/client/sysmon/` with the standard layout: `Cargo.toml`, `src/main.rs`, `.cargo/config.toml`, `README.md`, `debian/`
- [x] Move the body of `usb-display/client/rust/examples/sysmon_linux.rs` into `usb-display/client/sysmon/src/main.rs`
- [x] Add `Cargo.toml` with `[package.metadata.deb]` (maintainer, depends, asset mappings)
- [x] Add `.cargo/config.toml` pinning `target = "aarch64-unknown-linux-gnu"` (later removed — see Log)
- [x] Add `debian/sysmon.service`
- [x] Add `debian/postinst` (daemon-reload + enable + start)
- [x] Add `debian/postrm` (stop + disable on remove/purge)
- [x] Write `usb-display/client/sysmon/README.md`
- [x] Update root `Cargo.toml` workspace `exclude` list
- [x] Delete `usb-display/client/rust/examples/sysmon_linux.rs` and `usb-display/client/rust/run-sysmon.sh`
- [x] Update `usb-display/README.md` (drop sysmon_linux row, add pointer to client/sysmon/)
- [x] Add `build.sh` (executable) — checks `cargo-deb`, runs build, prints output `.deb` path
- [x] `cargo build --release` succeeds in the new crate
- [x] `./build.sh` produces a valid `.deb` (verified locally with `dpkg-deb -I/-c`); user runs install on the Pi and confirms the service starts on boot

## Log

- `cargo-deb` rejected the asset path `target/aarch64-unknown-linux-gnu/release/sysmon`. It treats `target/release/...` as a placeholder and resolves the actual path itself based on whether the build is native or cross-compile (via `--target`). Switched the asset path to the placeholder.
- Initially worked around by also adding `--target aarch64-unknown-linux-gnu` to `cargo deb` (so cargo-deb knew where to look, since the original `.cargo/config.toml` forced cross-compile mode even on aarch64 hosts). Then realised the `.cargo/config.toml` itself was unnecessary — building natively on the Pi already produces an aarch64 binary because the Pi *is* aarch64. Removed the `.cargo/config.toml` and the `--target` flag; `cargo build --release` now lands the binary at the standard `target/release/sysmon` and `cargo deb` finds it directly.
- `build.sh` uses `find` to locate the produced `.deb` so it works whether the build was native or cross-target.
- `cargo-deb` auto-generates `/usr/share/doc/sysmon/copyright` from the `license` field — no manual file needed.

## Conclusion

Shipped sysmon as its own crate at `usb-display/client/sysmon/`, packaged as a `.deb` via `cargo-deb`, runnable as a systemd service that auto-starts on boot. The build is reduced to a single command:

```
cd usb-display/client/sysmon && ./build.sh
```

…and a single install:

```
sudo apt install ./target/debian/sysmon_0.1.0-1_arm64.deb
```

The `postinst` does daemon-reload + enable + start. `postrm` reverses on remove/purge.

Notable choices captured in the Approach: aarch64-only build (Pi 5 tested), single-panel-size build (`panel-shift-64x32`), service runs as root (no udev/group setup), `cargo-deb` install is a one-time `cargo install` step the user runs themselves (referenced in `build.sh` and the README, not automated).

`hub75-client` unchanged at **0.2.6**. New crate `sysmon` starts at **0.1.0**. No project changelog, no map. README files updated: new crate has its own; `usb-display/README.md` table updated to point at the new crate.


