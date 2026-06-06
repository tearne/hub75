# dots: button-A RGB bring-up test

**Mode:** Formal

## Intent

Augment the `dots-64x32` example so pressing **button A** runs a panel bring-up test: fill the whole panel pure red, then pure green, then pure blue, one second each, then resume the normal dots animation. Purpose: a quick visual check that every pixel lights and each colour channel is wired correctly when bringing up a new panel — without reaching for a separate tool.

The example already streams frames over CDC; this adds reading the panel's A button (the firmware already emits button events over CDC) and a fixed colour sequence on press. It nudges dots from a pure minimal-send reference toward a small test tool — acceptable since it's explicitly the "test program".

## Approach

### All in `src/main.rs`

Keep the whole feature in the example's single file — no new modules — per the one-place goal; revisit only if it bloats the file.

### Detect button A by rising edge, polling once per loop

Each loop iteration, drain pending button bytes via `recv_event` with a 1 ms timeout and detect a rising edge on bit 0 (button A). The firmware emits a button byte only on state change, so a press is a 0→1 transition; the short timeout keeps the dot cadence intact. Press latency is bounded by the 500 ms loop tick — fine for a manual trigger.

### RGB sweep on press, then resume

On an A press, send a full-panel frame of pure red, then green, then blue — send + 1 s sleep each — then fall back into the dots loop. The rolling buffer is untouched, so the dots simply resume. One cycle per press.

### One send path

Route every frame — dots and sweep — through a single `show()` helper that sends and, on `Disconnected` (or any other error), waits and reconnects. Keeps the unplug-resilience consistent and avoids duplicating the recovery match (also holds the line count down).

## Plan

- [x] Factor frame-send + error-recovery into a `show()` helper and route the dots loop through it, replacing the inline match.
- [x] Add a `button_a_pressed()` helper that drains `recv_event` (1 ms timeout) and reports a rising edge on bit 0, tracking last state.
- [x] On an A press in the main loop, run a full-panel red → green → blue sweep (via `show`, 1 s each) before continuing the dots.
- [x] Build-verify with `cargo build` from `dots-64x32/`.

## Log

- `main.rs` is now 137 lines (was ~100). Still one file, as intended; on the longer side for a "minimal reference" but reasonable for the test program. Split-out remains the fallback if it grows further.

## Conclusion

Completed and confirmed working — button A runs the full-panel red → green → blue sweep, then the dots resume. `dots-64x32` 0.1.8, kept entirely in `main.rs` (137 lines, single file by choice). Went to plan; no `map.md` or project changelog to update.
