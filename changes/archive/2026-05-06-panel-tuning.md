# Panel tuning

**Mode:** Wander

## Intent

Experiment freely with sysmon's visual elements: frame rate, column layouts, number of bands, palette tweaks via the A/B mode (`AB_PALETTE_MODE` toggle in `main.rs`, `PALETTE_B` in `projection.rs`), and anything else that crops up.

For frame rate specifically: measure host CPU as we push the rate up, so the visual decision is informed by the cost. The cycle-overrun warning added in the previous change is the natural floor signal; `top`/`PROFILING.md` captures the rest.

No fixed target — exploration to find what looks good (and what it costs). The change name will be updated to reflect where the work lands.

## Conclusion

The wander pulled together five layers of refinement: frame-rate tuning (now CLI-parameterised), palette enrichment, layout reorganisation, and a couple of small bug fixes. Headline items:

**Frame rate parameterised.** `-f` is now `-f <hz>` — set any rate. The default stays at 1 Hz prod cadence. Documented the cost curve in `sysmon/README.md` (linear ~0.025 % CPU per Hz above 20 Hz; practical hardware ceiling ~125 Hz on a Pi 5, USB-FS bulk transfer is the limit).

**Palette refined.** Green and blue both pushed to max OKLCh saturation and lower lightness (rich `[0,153,0]` and `[0,51,153]`). Red dimmed slightly to `[220,50,50]`. Lime/NetUp untouched in the end (the candidate dimmer lime didn't ship). Below-mean rows now use OKLCh-based desaturation (preserves hue at low chroma — fixes a "dark blue reads as purple" drift) with a higher chroma factor (0.4 → 0.7) so dim rows stay recognisable as the metric's colour.

**Column layout: RAM as divider.** Original layout (`DiskW CPU0 DiskR CPU1 NetD CPU2 NetU CPU3 RAM`) became `DiskW CPU0 [RAM] DiskR CPU1 [RAM] NetD CPU2 [RAM] NetU CPU3 [RAM]` — RAM's 4 columns now act as single-column accents bookending each non-CPU/CPU pair. RAM still occupies 4 columns total, just spread out. Implemented via a static `structured_column_map` in `projection.rs`.

**Bands.** Briefly explored a 4-band golden-ratio split with extended 12× slowdown ratio. Initial factor table miscomputed (would have made band 2 commit every 28 minutes); caught and corrected. Reverted to the original 3-band layout — 4 bands didn't add visible value.

**Bug fixes.**
- A/B label was being drawn on the canvas before the screen-burn shift, which could split it across the wrap boundary. Now drawn directly into the rotated frame at fixed panel coordinates.
- Label position now signals palette: A bottom-left, B bottom-right (in user-view orientation, accounting for the panel's 90° rotated mounting).

**Versioning.** `sysmon` 0.4.1 → 0.4.2 (visible visual changes, no protocol or API shift).

**Change renamed.** `visual-tweaks.md` → `panel-tuning.md` to better reflect the scope of work.

## Log

- `-f` is now `-f <hz>` — runs at any positive rate up to 10 000 Hz, rounded to the nearest millisecond. Without `-f`: prod cadence (1 Hz). Default-when-bare removed; `-f` alone now prints a usage hint and exits.
- `Mode` struct dropped; main loop holds a `Duration` directly. Startup line now reads `sysmon connected. {ms} ms cycle (~{hz} Hz). Ctrl+C to stop.`
- `sysmon/README.md` configure example bumped to `-f 20`.
- Frame rate / CPU sweep on the Pi 5 (perf stat -I 5000, task-clock per 5 s window):
  - 1 Hz: 1.85 ms = 0.04% of one core (0.37 ms/cycle).
  - 10 Hz: 15.3 ms = 0.31% (0.31 ms/cycle).
  - 20 Hz: 25 ms = 0.5% (0.25 ms/cycle).
  - 30 Hz: 41 ms = 0.82% (0.27 ms/cycle).
  - 40 Hz: 52 ms = 1.04% (0.26 ms/cycle).
  - 60 Hz: 62 ms = 1.24% (0.21 ms/cycle).
  - 70 Hz: 73 ms = 1.46% (0.21 ms/cycle).
  - Linear scaling above 20 Hz at ~0.025% CPU per Hz. Per-cycle on-CPU work drops slightly at higher rates (tighter loops, friendlier cache).
  - 100 Hz: 101 ms = 2.0% (0.20 ms/cycle). Clean.
  - 143 Hz (`-f 150` → 7 ms cycle): overran immediately at 8/7 ms.
  - On-CPU ceiling is academic; the practical limit is wall-clock USB-FS bulk transfer time (~6 ms for a 6 KB frame). Real ceiling: **~125 Hz**, where the 8 ms wall-clock work fits in the budget.
