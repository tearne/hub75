# Fix stale `Hub75Client::open` call in the `clock` example

**Mode:** Wander

## Intent

The `clock.rs` example still calls `Hub75Client::open(&p)` (lines 21–24), an API that was removed when the client moved to libusb-based device discovery in commit `3cbc886`. The `life` example had the same bug and was fixed alongside `changes/archive/2026-05-07-life_drop_rgb_primer.md`; `clock` was missed. Apply the same fix: drop the optional port arg and call `open_auto()` directly.

## Conclusion

Replaced the `port`-arg `match` block in `clock.rs:20–25` with a single `Hub75Client::open_auto()` call, mirroring the fix applied to `life.rs` in `2026-05-07-life_drop_rgb_primer.md`. Verified all three panel-size features build, and `clock` ran on the 128x64 panel after a power cycle.

Surprise: panel display became corrupted after running `clock`; cleared on power cycle. Cause unknown — parked as `changes/open/panel_corruption_after_clock.md`.
