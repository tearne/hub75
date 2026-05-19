# USB CDC ACM crate, and rename `usb-serial` → `usb-vendor`

**Mode:** Formal

## Intent

`usb-serial` uses a vendor-class device with bulk endpoints. On Windows that requires a WinUSB-style driver association, which typically needs administrator rights to install. For hosts where admin rights aren't available — notably Windows 11 in locked-down environments — that's a blocker.

Earlier versions of this firmware (pre-0.5.0) used USB CDC ACM, which appears as a TTY on Linux/macOS and as a COM port on Windows via the in-box `usbser.sys` driver — no admin install needed. The move to vendor-class bulk was a deliberate performance choice (bypassing the kernel TTY/line-discipline layer that dominated host CPU when streaming frames), and that remains the right default for high-throughput streaming.

We want two outcomes from this change:

1. A new top-level crate, `usb-cdc`, that brings back the CDC ACM approach as a peer to the current crate. Same product shape — firmware + clients — different USB class. The trade-off is host CPU for plug-and-play on locked-down Windows hosts.

2. Rename the current `usb-serial` crate to `usb-vendor`. The "serial" name is historical and now misleading; the crate is no longer a TTY/serial device. Renaming makes the family shape legible — `usb-vendor` (vendor class, fast, needs driver install) and `usb-cdc` (CDC class, slower, plug-and-play) — at the directory level.

Both pieces land together so the family naming is consistent at every commit. Shared code between the two variants is preferred where it doesn't distort the design — the question of how much sharing is right is settled in the Approach.

## Approach

### One top-level crate, class-selected by Cargo feature

Inspecting `usb-serial/firmware-lib/`: the USB-class-specific code is ~30 lines (descriptor + interface + endpoint construction inside `run_usb_and_buttons`). Everything else — `FrameReceiver`, the panel-feed task, button debouncer and bulk-IN poll loop, the cross-task `FRAME_READY` signal — is class-agnostic. Splitting into two top-level crates would duplicate ~200 lines of working code and force every future protocol/button change to land twice. A third "common" crate behind two thin class-specific crates would avoid the duplication but adds a third crate, a third version number, and a third place to look.

Instead: keep a single top-level crate, with the USB class chosen by Cargo feature inside `firmware-lib`. The class-specific section of `run_usb_and_buttons` becomes a small adapter behind a feature flag (`usb-class-vendor` default, `usb-class-cdc` opt-in). The firmware binaries pick the class at build time the same way they pick the panel today.

### Rename, but to a class-neutral name

`usb-serial` is misleading today (no TTY) and would be misleading after the merge (it'd cover both vendor and CDC). `usb-vendor` works for the current crate in isolation but stops working the moment CDC lives in the same directory. The crate name should describe *what the thing does* (stream pixel frames over USB), not *which class it uses*.

Proposed name: **`usb`**. Short, unambiguous in this workspace, parallels `hub75` (the panel driver) at the directory level. The two USB-class implementations are an internal feature flag, not a directory split.

### Client side

Vendor and CDC clients can't share much: `rusb`/`pyusb` claim a vendor interface and do bulk I/O; CDC clients open a serial port via `serialport-rs` / `pyserial`. Frame encoding (the byte format) is identical but lives on the firmware side in `display.rs` and on the host as trivial RGB packing — not worth a third shared crate.

Approach: keep `client/rust/` and `client/python/` as today, with the transport selected at build/install time per example, not at runtime. Each example is single-purpose — it knows which device it talks to — so runtime polymorphism would just drag both transport libraries into every binary for no gain.

- Rust client: one library crate with two mutually-exclusive Cargo features, `transport-vendor` (default) and `transport-cdc`. Same shape as the existing panel-size features. Each example binary picks one; only the matching transport library (`rusb` or `serialport-rs`) is linked.
- Python client: two scripts, `hub75_client_vendor.py` and `hub75_client_cdc.py` (the current `hub75_client.py` is renamed to `_vendor`). Each imports only `pyusb` or `pyserial`.

### CDC frame-rate cap

CDC throughput is OS-dependent (Linux/macOS kernel TTY overhead vs Windows `usbser.sys` overhead) and not measurable in this change without a Windows 11 host on hand. The CDC client transport caps sends at 15 fps as a conservative ceiling; the firmware just consumes what arrives. The cap lives in the CDC client transport, not in the firmware, so it can be raised later by editing one place once real numbers exist.

### Knock-on: `sysmon/firmware/` dependency

`sysmon/firmware/` depends on `usb-serial-firmware-lib`. The rename updates that path and the crate name reference. No behavioural change. `sysmon` keeps using the default (vendor) class — `sysmon` is wired to a fixed host that's allowed to install the driver, and the per-frame CPU cost matters.

### Cargo crate names

The directory rename forces internal package-name renames too:
- `usb-serial-firmware` → `usb-firmware`
- `usb-serial-firmware-lib` → `usb-firmware-lib`
- `hub75-client` (already class-neutral) stays as is.

### Version bump

`usb-serial-firmware` is at 0.8.0 today. The rename + new class is a clear minor bump (no behaviour change for existing vendor-class users beyond the directory/crate-name rename). Propose `0.9.0`. `firmware-lib` is at 0.1.0 and unpublished — bump only if useful for the changelog, otherwise leave alone.

### Documentation

`usb-serial/README.md` becomes `usb/README.md`, rewritten to lead with the family shape (two USB classes, when to pick each) rather than the "serial is historical" caveat. Top-level `README.md` and `SETUP.md` get the path/name updates. `sysmon/README.md` and `sysmon/PROFILING.md` get the dependency path update.

## Plan

**Rename and restructure (stays on vendor only)**

- [x] `git mv usb-serial usb`.
- [x] Rename crate `usb-serial-firmware` → `usb-firmware` in `usb/firmware/Cargo.toml`; bump version to 0.9.0.
- [x] Rename crate `usb-serial-firmware-lib` → `usb-firmware-lib` in `usb/firmware-lib/Cargo.toml`.
- [x] Update dep in `usb/firmware/Cargo.toml` to the new lib name.
- [x] Update dep path and name in `sysmon/firmware/Cargo.toml`.
- [x] Update every `usb-serial` / `usb_serial_firmware_lib` reference in `sysmon/firmware/src/main.rs` and `usb/firmware/src/main.rs`.
- [x] Build verify `usb/firmware/` (default features).
- [x] Build verify `sysmon/firmware/`.

**CDC class in firmware-lib**

- [x] Add mutually-exclusive features `usb-class-vendor` (default) and `usb-class-cdc` in `usb/firmware-lib/Cargo.toml`.
- [x] Extract the class-specific section of `run_usb_and_buttons` into a `mod class_vendor` module behind `usb-class-vendor`.
- [x] Add `mod class_cdc` behind `usb-class-cdc`: CDC ACM descriptor setup via `embassy_usb::class::cdc_acm`, returning the same bulk-out/bulk-in pair shape `run_usb_and_buttons` expects.
- [x] Forward `usb-class-cdc` from `usb/firmware/Cargo.toml` as a passthrough feature.
- [x] Build verify `usb/firmware/ --features usb-class-cdc,panel-shift-64x64`.
- [x] Smoke-test CDC firmware on hardware: device enumerates as `/dev/ttyACM*` on Linux.

**Rust client transport**

- [x] Add mutually-exclusive features `transport-vendor` (default) and `transport-cdc` to `usb/client/rust/Cargo.toml`; gate `rusb` behind vendor and add `serialport` behind cdc.
- [x] Refactor `usb/client/rust/src/lib.rs` to expose a feature-gated `Transport` (one impl per crate compilation), keeping `Hub75Client` as the public façade.
- [x] Implement the 15 fps send-rate cap inside `transport_cdc::Transport::send_bytes`.
- [x] Examples (`clock.rs`, `buttons.rs`, `life.rs`) work unchanged — they use the `Hub75Client` façade and don't care about transport.
- [x] Bump `hub75-client` to 0.6.0.
- [x] Build verify `usb/client/rust/ --features transport-vendor` and `--features transport-cdc` (lib and examples).
- [x] Run one example against the CDC firmware end-to-end.

**Python client transport**

- [x] Rename `usb/client/python/hub75_client.py` → `hub75_client_vendor.py`.
- [x] Update `list_panels.py` and `scanline_test.py` imports to `hub75_client_vendor`.
- [x] Add `usb/client/python/hub75_client_cdc.py` using `pyserial`, with the 15 fps cap.
- [x] Smoke-test `hub75_client_cdc.py` against the CDC firmware end-to-end.

**Documentation**

- [x] Rewrite `usb/README.md` to lead with the vendor-vs-CDC family shape; document the 15 fps CDC cap and the Windows-no-admin rationale.
- [x] Update top-level `README.md` and `SETUP.md` for the directory rename.
- [x] Update `sysmon/README.md`, `sysmon/PROFILING.md`, `sysmon/firmware/README.md`, `sysmon/Cargo.toml`, `sysmon/src/main.rs` for path/name changes. (sysmon/src/display.rs, projection.rs, map.md only referenced `hub75-client` — crate name unchanged.)
- [x] Update the stray `usb-serial` reference in `hub75/src/shift/pack.rs`.

## Log

- Linux build of `serialport` (transport-cdc) needs `libudev-dev` installed for VID/PID-based port enumeration. Not a runtime cost — only required to compile the CDC client on Linux. Documented in the README; alternative would be to drop libudev support in serialport and match by port name only, losing the auto-VID/PID-filter ergonomics.
- Plan task list said the Rust client needed a `Transport` trait with two impls; the actual shape ended up simpler — one `Transport` struct per feature compiled in (not a trait), with `Hub75Client` as the public façade. The user-facing API didn't change and examples build unchanged, which is what the original "single Transport-trait" decision was trying to achieve.
- Plan listed `sysmon/src/display.rs`, `sysmon/src/projection.rs`, `sysmon/map.md` as needing path/name updates. They only referenced the `hub75-client` *crate name* (unchanged), not the directory path; no edits needed there.
- First CDC firmware flash enumerated as `1209:7575` but no `/dev/ttyACM*` appeared — Linux's `cdc_acm` didn't bind. Root cause: I'd set `device_class = 0x02` *and* `composite_with_iads = true`, which conflict.
- Second attempt (0.9.1: `device_class = 0`, `composite_with_iads = true`) didn't enumerate at all — embassy-usb panics on init when composite_with_iads is set unless the device triplet is exactly `0xEF/0x02/0x01` (the vendor module's existing comment had warned of this). 0.9.2 sets the IAD triplet explicitly. CDC enumeration confirmed working; Python smoke test passed.
- Rust CDC example failed first run with "Unable to acquire exclusive lock on serial port". serialport-rs defaults to exclusive=true on Unix (TIOCEXCL + flock); pyserial does no locking at all (why Python worked). 0.6.1 tried `TTYPort::set_exclusive(false)` after open — too late, since `TTYPort::open` itself calls TIOCEXCL. 0.6.2 uses the builder's `.exclusive(false)` so the open path takes only a shared flock; this is the right shape for the client and we keep it.
- After the 0.6.2 fix the error became "Unable to acquire **shared** lock on serial port" and I assumed ModemManager was the holder, adding a `ID_MM_DEVICE_IGNORE` udev rule + SETUP.md note. Wrong diagnosis. `fuser /dev/ttyACM0` revealed the real holder: a running `platformio` upload to a different board on the same TTY. The udev change has been reverted from SETUP.md; the 0.6.2 client change stays.

## Conclusion

Shipped as `usb-firmware` 0.9.2 + `hub75-client` 0.6.2.

Deviations from the plan worth noting:

- The original Intent framed this as "a new top-level crate, like `usb-serial`, using CDC ACM". Approach turned it into a single merged `usb/` crate with the class selected by Cargo feature, since firmware-lib was almost entirely class-agnostic. Less code, less duplication, and the same user-facing options.
- The plan said the Rust client would expose a `Transport` trait with two impls. Final shape is a single feature-gated `Transport` struct per build (vendor or cdc), with `Hub75Client` as the public façade. Same compile-time selection, simpler types, examples build unchanged.
- Firmware patched twice mid-build (0.9.0 → 0.9.1 → 0.9.2) to chase the right CDC descriptor shape: the right device-class triplet is `0xEF/0x02/0x01` (IAD-composite), and that's load-bearing — embassy-usb panics on init if `composite_with_iads = true` is set without it. The existing vendor module's comment had warned of this; I should have read it more carefully first.
- Client patched twice (0.6.0 → 0.6.1 → 0.6.2) to land non-exclusive serial open via the builder's `.exclusive(false)` (Linux-only race with anything else briefly touching `/dev/ttyACM*`).
- One wrong-diagnosis dead-end: a "shared lock" failure was misattributed to ModemManager and a `ID_MM_DEVICE_IGNORE` udev rule was added to `SETUP.md`. `fuser` then showed the holder was an unrelated `pio` upload. The SETUP.md change was reverted; the client `.exclusive(false)` change stayed (it's still the correct shape).
- Linux build of the CDC client needs `libudev-dev` (build-time only). Documented in `usb/README.md`. Not required on Windows or macOS.

Docs touched: top-level `README.md`, `SETUP.md`, `usb/README.md` (rewritten), `sysmon/README.md`, `sysmon/PROFILING.md`, `sysmon/firmware/README.md`, `hub75/src/shift/pack.rs` (stray comment). One pre-existing typo found and fixed during the doc audit (`scanline_test.py` error message). No project-level changelog exists; version bumps in the Cargo manifests are the durable record.

