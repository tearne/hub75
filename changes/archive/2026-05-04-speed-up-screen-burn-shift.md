# Speed up screen-burn shift

## Intent

The per-hour 1-column screen-burn shift has been observed in action and the rotation is working correctly — when reviewed, the previously-flagged "RAM split 3+1" turned out to be the algorithm operating as designed (any column-level shift through a multi-column metric briefly splits it across the wrap).

But the cadence is too slow. Speed up to **1 column every 15 minutes** so the full 32-column cycle completes in 8 hours instead of 32. Split-metric transient is accepted; metrics stay clean for ~12 minutes per shift before the next column boundary crosses them.

## Approach

Change the shift cadence in `main.rs` from `elapsed_hours` to `elapsed_quarter_hours`. One-line edit. No other code touched.

## Plan

- [x] In `usb-display/client/sysmon2/src/main.rs`, change the shift computation from `started_at.elapsed().as_secs() / 3600` to `started_at.elapsed().as_secs() / 900` and rename the local variable accordingly.
- [x] `cargo build --release`.

## Conclusion

Started life as a suspected rotation bug; investigation showed the algorithm was working correctly (a column-level shift necessarily splits multi-column metrics across the wrap during transitions). The change pivoted from "fix bug" to "tighten cadence" — 1 column every 15 minutes instead of 60. Full 32-column cycle now takes 8 hours instead of 32.

File renamed on archive to `speed-up-screen-burn-shift.md` to reflect the actual landed change.
