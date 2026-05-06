# Reduce USB write cost

**Mode:** Formal

## Intent

After the `reduce-sysmon-sampling-cost` change, sysmon's host CPU breakdown shifts: USB write is now the largest single stage at ~24% (84M of 351M weighted samples), and the dominant cost in a frame's hot path. The cost is the kernel TTY / line-discipline machinery used by CDC ACM: pixel bytes go `serialport::write` → `tty_write` → `n_tty_write` → bulk endpoint, all per-frame.

A flamegraph slice of the 24% confirms the cost is structural to the TTY layer, not the USB stack itself: ~89% TTY/line-discipline (`n_tty_write`, `tty_wait_until_sent`, `tty_poll`), ~11% generic VFS/syscall, ~0% USB host controller. Two cost components: per-syscall fixed overhead (entering line discipline, locking, polling for write space) and per-byte processing (n_tty_write copying through the discipline buffer).

## Options

Numbered by descending architectural impact, not desirability.

### 1. Vendor-class bulk USB

Firmware drops the CDC ACM pretence and exposes a vendor-defined bulk endpoint. Host opens the device with `rusb`/`libusb` and writes pixel data straight into URBs, bypassing the kernel TTY stack entirely.

- **Estimated saving:** ~21% of total host CPU (the full TTY layer goes away; bulk transfer itself is essentially free).
- **Pros:** biggest single win. Targets the right kernel layer cleanly. No format changes needed.
- **Cons:**
  - Loses "it's just a serial port" affordance — no more `cat /dev/ttyACM*`, `screen`, or any tool that speaks serial.
  - Python client (`pyserial`) breaks; needs rewrite onto `pyusb` plus `libusb-1.0` system package.
  - Backwards compatibility break: old firmware ↔ new client (or vice versa) won't talk. Every Pico needs reflashing in lockstep with the host upgrade.
  - Windows: needs WCID / Microsoft-OS descriptors in firmware to be driver-free, otherwise users install WinUSB via Zadig.
  - Adds `rusb`/`libusb` host dependency — new system library on every host that wants to drive the panel.
  - `embassy-usb` vendor-class examples are thinner than CDC ACM; modest implementation friction.
- **Linux permissions are not really a downside:** a udev rule with `GROUP="dialout" MODE="0660"` puts the device in the same group most desktop users (including Pi OS) already belong to, so access is automatic. The sysmon `.deb` can ship the rule in `/etc/udev/rules.d/` and run `udevadm control --reload-rules && udevadm trigger` from `debian/postinst` (idempotent, no reboot or replug). Non-deb users get a one-liner in `SETUP.md` matching the existing BOOTSEL rule. macOS/Windows have separate stories.

#### Migration shape

Three ways to roll out vendor-class:

- **A — True dual-mode firmware.** Composite USB device exposing both CDC ACM and vendor-class bulk simultaneously. Host picks. *Pros:* no lockstep break — old clients (Python `pyserial`, `cat /dev/ttyACM*`) keep working; gradual client migration. *Cons:* composite descriptors are more involved in `embassy-usb`; both paths must be implemented, tested, and maintained forever; need a policy for which endpoint drives the panel when both are written to.

- **B — Compile-time feature flag.** One firmware binary at a time, `--features cdc-acm` vs `--features vendor-bulk`. Each Pico is committed to one mode at flash time. *Pros:* clean firmware (one active path); easy to fall back if vendor-class hits a snag; can flash one Pico of each to A/B compare. *Cons:* switching modes on a given Pico needs reflash; no help for ad-hoc `cat /dev/ttyACM*` mid-session on a vendor-class Pico.

- **C — Single mode, just switch.** Drop CDC, ship vendor-class only. *Pros:* simplest firmware; no legacy paths. *Cons:* full lockstep upgrade; CDC tooling gone permanently.

For a single-user project where every Pico can be flashed in lockstep, **(B)** is the recommended middle: keeps a fallback during validation, no permanent dual-path complexity. **(A)** is right if the project ever gains other users running CDC tooling. **(C)** is fine if confidence is high.
- **Scope:** firmware USB descriptor + client-lib transport rewrite. Two crates. Sysmon picks up the new API for free.

### 2. Move rendering onto the Pico

Pico becomes the artist: sysmon sends only the ~9 sampled metric values per cycle (~40 bytes), firmware does banding, pattern flow, palette mapping, rotation, screen-burn shift, and paints pixels.

- **Estimated saving:** ~30% of total host CPU (USB write almost gone; render gone too).
- **Pros:** biggest absolute win. Wire format becomes trivial. Panel becomes a generic "render these N values" device — any host (Mac, Pi, ESP32) could drive it by sending tiny number packets.
- **Cons:**
  - **Visual iteration cost goes way up.** Today: edit Rust on the host, `cargo run`, see the change in seconds. After: every colour or band tweak needs a Pico reflash.
  - **Loses host-side rendering flexibility.** Today the host owns the look — palette comparison, A/B testing, presentation tweaks all live in `presentation.rs` / `projection.rs` and iterate cheaply. After, the look lives in firmware.
  - Most of `bands.rs`, `projection.rs`, `oklch.rs`, palette code migrates into `no_std`. Heap-using helpers (e.g. `flow_pattern`'s small `Vec`s) need rewriting. Debugging gets harder (no `println`, only `defmt` over a probe).
  - Wire protocol becomes structured (typed values per metric); adding a new metric is now a coordinated firmware + host change with version bump.
  - Big migration; substantial test surface.
- **Scope:** large. Most of sysmon's visual code moves to firmware; new wire protocol on both sides.

### 3. RLE / frame-buffer compression

Most pixels are black; RLE or similar compresses a typical frame to a fraction of the 6 KB. Adds encode CPU on host, decode CPU on firmware.

- **Estimated saving:** ~5–15% of total CPU, highly content-dependent (good when bands are sparse, much worse when busy).
- **Pros:** keeps CDC ACM and current tooling intact. No protocol-class shift.
- **Cons:** awkward middle ground. Adds host encode + firmware decode work. Worst-case (busy panel) can be worse than uncompressed. Implementation is fiddly to get right (delimiter/escape handling).
- **Scope:** firmware decoder + client-lib encoder. Sysmon API surface unchanged.

### 4. RGB565 pixel packing

Pack each pixel into 16 bits (5R / 6G / 5B) instead of 24 bits. Halves payload (~6 KB → ~3 KB).

- **Estimated saving:** ~8–10% of total CPU (cuts the per-byte TTY cost roughly in half; per-syscall fixed cost unchanged).
- **Pros:** trivial encode (bit shifts) and trivial decode. Fixed-format — no escaping or framing headaches. CDC ACM stays. Python client just learns one new format. Negligible visible loss for sysmon's saturated palette.
- **Cons:** smaller win than vendor-class. Backwards-compatibility break (firmware and clients must agree on format), but a much smaller one — same kind of versioning thinking as adding a new metric.
- **Scope:** trivial code change in client lib (encode) + firmware (decode). No new dependencies, no system libs, no udev changes.

### 5. Indexed-palette pixel packing

Sysmon already uses a finite palette. Send a palette index per pixel; firmware looks up RGB at paint time. 1 byte/pixel = 3× reduction; 4-bit packed = 6× if palette fits in 16 colours.

- **Estimated saving:** ~12–15% of total CPU.
- **Pros:** bigger win than RGB565, similar character. CDC ACM stays. Very small encode CPU. Wire format becomes more compact and arguably cleaner.
- **Cons:**
  - Couples the firmware to the host palette: firmware needs the palette table, sent at startup or compiled in. Host palette tweaks now need either a re-flash (compiled in) or a startup handshake (sent fresh).
  - 16-colour limit (for 4-bit) constrains future palette work; 256-colour limit (for 8-bit) is roomy.
  - Same compatibility-break shape as RGB565 but with the added handshake/compilation question.
- **Scope:** moderate — encode in client lib, palette table in firmware (and maybe startup handshake), wire format bump.

### 6. TTY raw mode (free tweak)

Force `cfmakeraw()` on the serial port if `serialport-rs` isn't already doing it. Disables cooked-mode processing (newline translation, signal characters) for every byte.

- **Estimated saving:** unknown; possibly small, possibly meaningful — depends on what processing is currently active. May already be the default.
- **Pros:** trivial. No format changes. No compatibility break. Worth checking whether or not we go further.
- **Cons:** none of substance.
- **Scope:** one-line tweak in client lib (or just confirmation that it's already done).

### 7. Single-syscall write (free tweak)

`send_frame_rgb` currently does `write_all(header)` + `write_all(pixels)` + `flush`. Concat into one buffer, one `write_all`. Eliminates one syscall's worth of TTY overhead per frame.

- **Estimated saving:** small (<2%). Only saves the per-syscall fixed cost on one extra syscall per frame.
- **Pros:** tiny code change, no compatibility implications, free to combine with anything else.
- **Cons:** none of substance.
- **Scope:** trivial.

### 8. Drop the post-write flush

`port.flush()` calls `tty_wait_until_sent`, which sleeps until the TTY output buffer drains. Removing it doesn't reduce CPU directly but reduces wall-clock blocking, freeing the host thread to do other work sooner.

- **Estimated saving:** doesn't change CPU% in absolute terms, but reduces sysmon's apparent CPU% (less time blocked) and improves frame-pacing headroom.
- **Pros:** trivial removal. Next frame queues behind this one anyway.
- **Cons:** need to confirm we're not relying on the flush for backpressure (probably not — the next write would block on full buffer).
- **Scope:** trivial.

## Composing options

Options 4–8 compose with each other and don't touch the USB protocol class. Combined estimate: **12–18% of total CPU saved**, in the same ballpark as vendor-class (~21%) but without vendor-class's costs (Python client unchanged, `/dev/ttyACM*` unchanged, no re-flash compatibility break, no new system libs, no Windows driver story).

Vendor-class is the bigger single win, but the contained-tweaks bundle is most of the way there for a fraction of the cost and disruption.

On-Pico rendering is in a different category — a long-term architectural redirection, not a perf tweak.

## Summary

| Option | Saving | Effort | Compatibility break |
|---|---|---|---|
| 1. Vendor-class bulk | ~21% | medium | yes (tooling, Python, firmware/host lockstep, system libs) |
| 2. On-Pico rendering | ~30% | very large | yes (wire protocol; loses host visual iteration) |
| 3. RLE | ~5–15% | medium | minor (firmware/client format) |
| 4. RGB565 | ~8–10% | small | minor (firmware/client format) |
| 5. Indexed palette | ~12–15% | small–medium | minor + palette coupling |
| 6. TTY raw mode | ? | trivial | none |
| 7. Single-syscall write | <2% | trivial | none |
| 8. Drop flush | (wall-clock) | trivial | none (probably) |

## Conclusion

Completed. Vendor-class bulk USB now drives the panel; CDC ACM dropped. The Rust client uses an async-libusb transfer reused across frames; the Python client moved to `pyusb`. Linux setup is automated by the sysmon `.deb` (udev rule + `udevadm reload` in `postinst`). PID `0x7575` chosen under VID `0x1209` (pid.codes); registration PR is non-blocking and listed as a follow-up.

Per-frame USB write cost dropped 63% from the CDC baseline (84M → 31M weighted samples). Sysmon's overall wall-clock CPU at fast mode is ~0.5%, down from ~0.9% across both changes. The architectural cleanup (no TTY layer, reserved IN endpoint for future buttons/telemetry) carries forward independently of the perf delta.

Two incidental fixes rode along: a duplicate `debug = true` in `hub75/Cargo.toml` left over from earlier profiling work, and a pre-existing seq-counter double-increment in `display.rs` that produced spurious "dropped 255 frames" warnings on every frame. Both are noted in the Log.

Documentation impact: `SETUP.md` gained a "Driving the panel from a host" section describing the libusb runtime dep and the udev rule. `usb-serial/README.md` gained a "USB descriptor" section documenting the VID/PID and endpoint layout.

## Log

- Discovered a duplicate `debug = true` in `hub75/Cargo.toml` left over from earlier profiling work (my own PROFILING.md guide had instructed adding it; the duplicate slipped through when the guide was simplified). Removed it.
- First firmware build had `device_class = 0xFF` at the Config level. The Pico flashed and rebooted but never re-enumerated on USB. Initial guess (composite-device shape) was wrong; second guess swapped to `device_class = 0x00`, also failed. Real root cause emerged from a probe-attached `cargo run`: embassy-usb 0.6 defaults `Config::composite_with_iads = true`, which requires the IAD-composite class triplet (`0xEF/0x02/0x01`) — the constructor panics otherwise. Fixed by setting `composite_with_iads = false` (single-function device, no IADs needed) and restoring `device_class = 0xFF`.
- First-pass profile of vendor-class with `rusb`'s synchronous `write_bulk` showed only ~4% total CPU drop and ~17% drop on USB write — far below the ~21% projection. Cause: kernel TTY work was eliminated, but `rusb` allocates a fresh `libusb_transfer` struct per frame and polls completion synchronously, replacing most of the saving with userspace libusb work. Switched to manual async via `rusb::ffi`: a single `libusb_transfer` reused across frames, `libusb_handle_events_completed` polled until our atomic-flag callback fires. Per-frame USB write absolute cost: 84M (CDC) → 70M (sync vendor) → 31M (async vendor), a 63% drop from CDC.
- Discovered an unrelated pre-existing bug in `display.rs`: `expected_seq` was incremented twice per frame (once on receiving the seq byte, once after pixels). Result: every frame reported as "1 dropped" (= 255 in u8 wrapping arithmetic), spamming defmt warnings on power-of-2 event counts. Fixed by removing the second increment.
- `pidstat` not installed; used `top -b -p $(pidof sysmon)` instead. sysmon's wall-clock CPU at fast mode is ~0–2% — small in absolute terms. The 658M → 337M weighted-sample reduction across both changes corresponds to roughly 0.9% → 0.5% CPU. Composition of the slice changed substantially even where the absolute number is small.

## Approach

### Pick option 1C: vendor-class bulk USB, single mode, lockstep flash

Biggest single win (~21% of host CPU) and the cleanest target — full TTY layer eliminated. Single-mode (no CDC fallback) since this is a single-user project where every Pico can be flashed in one go.

### Endpoints

Bulk OUT for pixels. Bulk IN endpoint reserved with no defined payload — adding it now avoids a future re-flash if/when buttons or telemetry land. Host ignores it for now.

### Identification

VID `0x1209` (pid.codes), self-picked unused PID. Retain `manufacturer = "tearne"`, `product = "hub75"` strings. Optionally populate `serial_number` from the RP2350 chip ID for per-board uniqueness. Client matches on VID/PID with strings as a sanity check. Submitting a pid.codes registration PR is non-blocking.

### Public client API stays put

`Hub75Client::open_auto()` and `send_frame_rgb()` keep their signatures; only internals change (`serialport`/string-match → `rusb`/VID-PID). Sysmon recompiles unchanged.

### Linux permissions ship with the deb

Sysmon's `.deb` adds `/etc/udev/rules.d/99-hub75-panel.rules` (`GROUP="dialout" MODE="0660"`) and a `debian/postinst` that runs `udevadm control --reload-rules && udevadm trigger`. Non-deb users get the same one-liner in `SETUP.md` next to the BOOTSEL rule.

### Python client ported in-change

`pyserial` → `pyusb`. Two scripts to update; uv-script dep header changes. Keeps host-side tooling in lockstep with the firmware change.

### Versioning

Breaking wire-format change. Bump:
- `usb-serial-firmware` 0.4.0 → 0.5.0
- `hub75-client` 0.2.6 → 0.3.0
- `sysmon` 0.3.2 → 0.4.0

## Plan

- [x] Pick an unused PID under VID `0x1209` from the pid.codes list; record in firmware + client constants.
- [x] Implement vendor-class USB descriptor in `usb-serial/firmware/`: drop CDC ACM, declare bulk OUT (pixels) + bulk IN (reserved). Wire OUT bytes into the existing `FrameReceiver` state machine; leave IN endpoint declared but unused.
- [x] Replace `serialport` transport in `usb-serial/client/rust/` with `rusb`: open by VID/PID, sanity-check `manufacturer`/`product` strings, send frames via bulk OUT. Keep `open_auto()` and `send_frame_rgb()` signatures unchanged.
- [x] Port `usb-serial/client/python/{hub75_client.py,scanline_test.py}` from `pyserial` to `pyusb`; update uv-script dependency headers.
- [x] Add `/etc/udev/rules.d/99-hub75-panel.rules` to `sysmon/Cargo.toml`'s `[package.metadata.deb] assets`; add `sysmon/debian/postinst` that reloads udev rules.
- [x] Update `SETUP.md`: add the udev one-liner for non-deb users; note the wire-format break and lockstep flash requirement.
- [x] Update `usb-serial/README.md`: document the new VID/PID and protocol.
- [x] Bump versions: `usb-serial-firmware` → 0.5.0, `hub75-client` → 0.3.0, `sysmon` → 0.4.0.
- [x] Re-flash the Pico, restart sysmon, confirm panel updates correctly under `cargo run` and via the deb.
- [x] Re-profile per `sysmon/PROFILING.md`; report the delta against the post-sampling-fix baseline (351M weighted samples; USB write 24%).
- [ ] (Non-blocking) Submit pid.codes registration PR with the chosen PID `0x7575`.
