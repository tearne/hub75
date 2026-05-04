# Spike: Pico as a populated USB MSC drive

## Intent

The parent change [`usb_file_drop_rendering`](usb_file_drop_rendering.md) (parked in plan mode) hinges on us being able to make the Pico appear as a USB mass-storage drive with a populated FAT12 filesystem — the same trick the RP2350 BOOTSEL ROM uses when it serves up an `RP2350` drive containing `INFO_UF2.TXT`. The hard part is implementing the MSC class against `embassy-usb` (no in-tree class for it) and getting `fatfs` running against a RAM-backed block device in `no_std`. Either could turn out harder than expected, and discovering that mid-build of the parent change wastes time.

This is a focused, throw-effort-not-scope spike: get the Pico to enumerate as a USB MSC drive, format it FAT12 from firmware on boot, populate it with a single `README.TXT`, and confirm the host OS mounts it and shows the file *without* prompting to format. That's all. No file-event detection, no JSON parsing, no rendering, no host clients.

If this works, the resulting code is the foundation for the parent change; we'll extend it with file detection + JSON parsing + render. If it doesn't work in a reasonable spike timebox, we re-plan the parent change with the new information.

The output of this change: a `usb-drop/firmware/` crate that boots, enumerates as USB MSC, and shows up on the host as a small drive containing `README.TXT`.

## Approach

### Use the `usb-device` ecosystem for this firmware

The spike's first attempt against `embassy-usb` revealed that prerequisite APIs for MSC (per upstream issue #2837) are missing. Rather than block on contributing them upstream, switch this firmware off the embassy-usb stack and onto the [`usb-device`](https://github.com/rust-embedded-community/usb-device) ecosystem so we can use the existing [`usbd_mass_storage`](https://docs.rs/usbd_mass_storage/) crate directly.

Consequences:
- The firmware drops `embassy-rp` / `embassy-executor` / `embassy-usb` and uses [`rp235x-hal`](https://crates.io/crates/rp235x-hal) for board init and the synchronous USB peripheral driver — same HAL that `learning-examples/` already uses.
- The firmware is plain synchronous embedded-Rust, not async. That's fine; it has nothing async to do (no panel work, no inter-task coordination).
- The codebase ends up with two firmware architectures: `usb-serial/firmware/` on embassy, `usb-drop/firmware/` on rp235x-hal + usb-device. They share no code, so the duplication is contained.

### Brand-new crate at the eventual production location

`usb-drop/firmware/` is added as a new workspace member, alongside (the still-named) `usb-display/`. The parent change handles the `usb-display` → `usb-serial` rename; the spike doesn't touch existing crates.

The spike's code is the seed of the eventual production firmware. If the spike succeeds, the parent change extends it with file detection + JSON parsing + render.

### Crate stack

- [`rp235x-hal`](https://crates.io/crates/rp235x-hal) — board init, clock, USB peripheral driver. Provides `UsbBus` implementing `usb_device::bus::UsbBus`.
- [`usb-device`](https://crates.io/crates/usb-device) — USB device framework. Provides `UsbDevice<UsbBus>` with poll-based event loop.
- [`usbd_mass_storage`](https://crates.io/crates/usbd_mass_storage) — MSC class implementation against `usb-device`. Implements BOT + SCSI; expects a `BlockDevice` trait we provide.
- [`fatfs`](https://crates.io/crates/fatfs) — FAT12 filesystem. Used at boot to format the volume and create `README.TXT`.

### RAM-backed block device

Static `[u8; 65536]` (64 KB) backing store, 512-byte sectors. Implements both `fatfs`'s storage trait (for boot-time format + write README) and `usbd_mass_storage`'s `BlockDevice` trait (for serving the host's reads/writes).

### Boot sequence

1. Init clocks + USB peripheral via `rp235x-hal`.
2. Init the static buffer to zeros.
3. `fatfs::format_volume` → FAT12.
4. Mount; create `README.TXT` with a placeholder `&'static str`.
5. Hand the buffer to the MSC class.
6. `loop { usb_dev.poll(...); }` — sync event loop, no executor.

### Out of scope for the spike

No file detection, no JSON parsing, no panel, no `hub75` dependency, no host-side anything. Just "does the USB+MSC+FAT stack land".

## Plan

- [x] Replace `usb-drop/firmware/Cargo.toml` deps with the new stack: `rp235x-hal`, `usb-device`, `usbd_mass_storage`, `fatfs`, `cortex-m`, `cortex-m-rt`, `defmt`, `defmt-rtt`, `panic-probe`. Drop the embassy crates. Keep `build.rs`, `memory.x`, `.cargo/config.toml` from the embassy-spike scaffolding.
- [x] Replace `src/main.rs` with a synchronous skeleton: `rp235x-hal` clocks + USB peripheral init, then a stub `loop { usb_dev.poll(...); }` against a bare `UsbDevice` (no class yet) — proof that the new dependency tree builds and enumerates.
- [x] Implement a RAM-backed block device (static `[u8; 65536]`, 512-byte sectors). Provide `fatfs`'s storage trait *and* `usbd_mass_storage`'s `BlockDevice` trait against the same buffer.
- [x] On boot: format the buffer as FAT12 via `fatfs::format_volume`; mount; create `README.TXT` with a placeholder string.
- [x] Register `usbd_mass_storage`'s class against the `UsbDevice`, hand it the block device.
- [x] Build verify.
- [x] Hardware verify: flash, plug into a host, drive enumerates as a removable drive (named `HUB75DROP` or similar), `README.TXT` is visible in the file manager, no format prompt. At least one host OS.

## Unresolved

(none.)

## Log

- Scaffolded `usb-drop/firmware/` (workspace member, Cargo.toml mirroring `usb-display/firmware`'s deps, build.rs / memory.x / .cargo/config.toml). Crate builds and the board enumerates as a CDC device (proof of life that the toolchain and embassy-usb stack work in the new crate).
- Stopped before attempting MSC. No embassy-usb-MSC crate exists; the closest reference is `usbd_mass_storage` (built against `usb-device`, not embassy-usb). Hand-rolling MSC is ~400-600 lines of USB/SCSI protocol code and not realistic to write from memory in a single session without spec references in front of the agent. Hardware verify confirmed: the board enumerates (as CDC), but does not appear as a drive.
- After replan to `usb-device` + `usbd-storage`: found `usbd-storage` v2 (apohrebniak fork) which provides high-level `ScsiCommand` matching with the BOT state machine handled internally. Its `examples/rp2040/` is a 335-line reference for exactly this use case. Adapted to RP2350 by swapping `rp2040-hal` → `rp235x-hal`; replaced flash-backed storage with a static RAM buffer.
- `fatfs` 0.3.6 turned out to be unusable in `no_std` on stable Rust: its `core_io` feature pulls in an unmaintained crate that panics during its build script on modern compilers; its `alloc` feature uses `#![feature(alloc)]` which requires nightly. Resolution: use `fatfs` only on the host side via `build.rs` (where `std` works fine), generate a 64 KB FAT12 disk image at build time with `README.TXT` already in the root directory, and `include_bytes!` it into the firmware. Firmware seeds the RAM disk by copying from this static image at boot. No FAT layer in the embedded code at all — it just shovels bytes between the host's reads/writes and the static buffer.

## Conclusion

Spike succeeded. Hardware confirms the Pico enumerates as a 64 KB removable drive labelled `HUB75DROP` containing `README.TXT`, with no format prompt on the host (verified on at least one OS).

Outcome of the spike's central question (*is making the Pico look like a drag-drop USB drive feasible?*): yes, via `usb-device` + `usbd-storage` on `rp235x-hal`, with `fatfs` confined to a build-time helper. The resulting `usb-drop/firmware/` is the foundation for the [parent change](usb_file_drop_rendering.md), which extends it with file detection, JSON parsing, and rendering through `hub75`.

Architectural cost: this firmware doesn't share the embassy-rp / async ecosystem the rest of the project uses. Acceptable — `usb-drop/firmware/` has nothing async to do, and the duplication is contained.

## Feedback

- **Status:** partially implemented. Plan tasks 1 (scaffold) done, plus an unplanned CDC-enumeration proof-of-life. MSC tasks (2–6) not started; hardware verify (8) failed in the sense intended (no drive on host).
- **Notes:** the spike's question — "is it tractable to hand-roll MSC against embassy-usb in a session" — has been answered no, at least not without authoritative references. Replan options to consider:
  - Move MSC to a multi-session effort with USB MSC class spec + SCSI primary commands + a working reference implementation (e.g. `usbd_mass_storage`'s source as a porting target) loaded in.
  - Switch this firmware off embassy-usb onto `usb-device` so we can use `usbd_mass_storage` directly. Cost: lose embassy-rp's executor integration and async pattern; this firmware would look quite different from `usb-display/firmware`.
  - Find or fund an embassy-usb-MSC crate (search more thoroughly, ask in embassy's chat, check forks).
- **Documentation impact:** none beyond this change document. The `usb-drop/firmware/` skeleton is left in place; if we abandon the approach, deleting it is one `rm -rf` away. The workspace `Cargo.toml` `members` entry is the only project-file change.
