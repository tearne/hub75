# hub75-client: typed disconnect error + in-place reconnect

**Mode:** Formal

## Intent

When a panel is temporarily unplugged, a consumer application currently can't handle it gracefully without guesswork. `send_frame_rgb` (and `open`) return `Result<(), Box<dyn std::error::Error>>`, so the only thing an app can inspect is `.is_err()` — a yanked cable, a write timeout, and a genuine caller bug (wrong pixel count) are all the same opaque box. The app is forced into "any failure → reconnect", which both loses flexibility (can't distinguish transient from fatal) and is burdensome: it must construct a whole new `Hub75Client`, re-supplying the serial selector and losing the `seq` counter and frame buffer.

The library is the right place to fix this: it already abstracts two transport backends (rusb for vendor, `serialport` for CDC) that report disconnects in completely different, per-OS ways — that knowledge currently leaks up to every consumer. Both current consumers hit this: the `dots-64x32` example and `sysmon`, which hand-rolls a full reconnect state machine and exhibits the same over-broad "any error = disconnect" handling.

## Approach

### A single public `Error` type

Replace `Box<dyn std::error::Error>` across the public API with a fully-owned, closed `Error` enum (no `dyn` anywhere) implementing `std::error::Error` + `Display`, plus `pub type Result<T> = core::result::Result<T, Error>`. Variants:

- `Disconnected` — no usable device present; reconnectable.
- `Timeout` — operation didn't complete in time.
- `PermissionDenied` — missing udev rule (vendor) or user not in `dialout` (CDC); actionable first-run failure.
- `Busy` — another process holds the panel.
- `InvalidFrame { expected, got }` — caller passed a wrong-sized frame; never retry.
- `Other(String)` — uncategorised tail, carrying the underlying error's `Display` as a human-readable message, not a type-erased object.

A closed enum keeps `Error` `Send + Sync + 'static` and `Debug`-derivable, and free of the opacity this change removes; the only cost versus boxing a source is programmatic `source()`-chaining, which no caller of a hardware client needs. Implementing `std::error::Error` keeps existing `?`-into-`Box<dyn Error>` callers (sysmon's `main`, the `life`/`clock`/`buttons` examples) compiling, so the break is confined to code that pattern-matches the error — which today none does.

### Transports own the error mapping

Each transport module maps its native failures into `Error` before returning, because only the transport knows its own vocabulary and this is exactly the per-OS knowledge that shouldn't leak to consumers:

- Vendor: `LIBUSB_ERROR_NO_DEVICE` / `LIBUSB_TRANSFER_NO_DEVICE` → `Disconnected`; `LIBUSB_TRANSFER_TIMED_OUT` → `Timeout`; `LIBUSB_ERROR_ACCESS` → `PermissionDenied`; `LIBUSB_ERROR_BUSY` (incl. a failed `claim_interface`) → `Busy`.
- CDC: `io::ErrorKind::{BrokenPipe, NotFound, UnexpectedEof}` → `Disconnected`; `TimedOut` → `Timeout`; `PermissionDenied` → `PermissionDenied`; serialport's in-use/busy → `Busy`.
- `open`'s "could not find a panel" → `Disconnected`; anything unrecognised → `Other`.

### In-place `reconnect()`, single attempt

`Hub75Client::reconnect(&mut self) -> Result<()>` re-runs `Transport::open` with a stored serial selector (a new `serial: Option<String>` field on the client) and swaps `self.transport` in place, preserving `buffer` and `seq`.

One attempt, not a blocking retry loop: sysmon must keep sampling and animating elsewhere during downtime and reconnect on its own interval — a blocking retry inside the library would steal that control. The caller owns the loop and cadence; the library owns the transport-specific *how* and the `seq` continuity.

### Scope: foundation now, resilient wrapper deferred

Ship typed errors + `reconnect()` in this change. Defer the opt-in `ResilientClient` / policy / callback layer to a follow-up: the breaking part (the error type) should land and stabilise first, the wrapper is purely additive and non-breaking, and neither current consumer wants it (sysmon needs non-blocking; dots is a two-line loop).

### Consumer updates

Update `dots-64x32` and `sysmon` to the new API: use `reconnect()` instead of rebuilding the client, and match `Error::Disconnected` so a genuine fault isn't silently treated as an unplug. This keeps the flagship example and the real app modelling the intended pattern and fixes the latent over-broad error handling in both.

## Plan

- [x] Define the `Error` enum (`Disconnected`, `Timeout`, `PermissionDenied`, `Busy`, `InvalidFrame { expected, got }`, `Other(String)`) and `pub type Result<T>` in `lib.rs`, with `Display` and `std::error::Error` impls.
- [x] Map vendor-transport failures to `Error` in `transport_vendor.rs` (open, `send_bytes`, `recv_event`, `find_device`, `list_panels`).
- [x] Map CDC-transport failures to `Error` in `transport_cdc.rs` (open, `send_bytes`, `recv_event`, `find_port`, `list_panels`).
- [x] Switch `Hub75Client`'s public methods to `Result<_, Error>` and store the serial selector as a `serial: Option<String>` field.
- [x] Add `Hub75Client::reconnect()` — re-open with the stored serial, swap the transport in place, preserve `buffer` and `seq`.
- [x] Update `dots-64x32` to use `reconnect()` and match `Error::Disconnected`.
- [x] Update `sysmon` to use `reconnect()` and match `Error::Disconnected`, replacing the client rebuild and the broad "any error = disconnect" handling.
- [x] Build-verify: client under both transport features, plus `dots-64x32`, `sysmon`, and the `life`/`clock`/`buttons` examples.

## Log

- `find_device`/`find_port` previously emitted a detailed "could not find … VID/PID …" message; collapsed to `Error::Disconnected` (fixed `Display`), so that diagnostic detail is gone. Acceptable — `Disconnected` is the matchable signal and the VID/PID are compile-time constants.
- `sysmon` restructured so the client is *retained* across a disconnect (so `reconnect()` can be called on it) rather than dropped to `None`; connectivity is now tracked by `disconnected_at`, and active use (button drain + send) is gated on it. Non-`Disconnected` send/read errors are now logged-and-continued rather than treated as a disconnect — fixing the old "any error tears down the panel" behaviour.
- `sysmon` imports the client error as `PanelError` to avoid clashing with its existing `use std::error::Error`.
- Pre-existing dead-code warnings in the `clock` example (`set_disk`, `draw_face`) are unrelated to this change.
- Hardware test: `sysmon` (vendor, Linux) confirmed working — deb rebuilt, service restarted, renders. `dots` (CDC) renders correctly on Windows 11 but **exited on unplug instead of reconnecting** — strong evidence Windows reports a CDC unplug as an `io::ErrorKind` we don't map to `Disconnected` (likely access-denied or a generic OS error), so dots hit its `panic!` arm and the console window closed.
- `dots` 0.1.6 made resilient regardless of the exact error kind: any send error now logs and routes through `wait_for_panel()` (reconnect loop) rather than exiting; the unexpected-error arm logs `{error:?}` so the next Windows run reveals what the unplug actually maps to. Also added a startup banner and an OSC terminal-title (addresses the blank-console and "which app is this" niggles). Banner/title are dots UX beyond the pure disconnect scope.
- **Pending:** once the Windows diagnostic log identifies the real error kind, fix the CDC transport mapping in `transport_cdc.rs` so a Windows unplug classifies as `Error::Disconnected` at the source (benefiting every consumer, not just dots). Library mapping deliberately not guessed without the data.
