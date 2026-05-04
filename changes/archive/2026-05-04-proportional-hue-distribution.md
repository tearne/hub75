# Proportional hue distribution

## Intent

In `sysmon2`, replace the current evenly-anchored hue layout with one where each metric is allocated an arc of the OKLCh hue circle proportional to its column count on the panel. CPU dominates the panel (4 cores × 4 columns = 16 of 32 columns), so giving CPU half the hue circle expresses that visual prominence in colour space.

The center of each metric's arc becomes its primary hue. The warm/cool variant offsets are scaled per-metric to the metric's arc width — so CPU's variants can swing a lot while a 3-column metric's variants stay small. CPU's hue then carries more internal variation per painted row, compensating for being repeated across four cores and four columns each. Smaller metrics keep tighter colour identity.

Walking around the wheel preserves the current visual ordering (red → yellow → green → cyan → blue → magenta).

## Approach

### Arc allocation walks the wheel in metric order

Walk around the hue circle starting at 0° and assign each distinct metric a contiguous arc whose width is `(metric_columns / total_columns) × 360°`. The four CPU cores collapse into a single CPU allocation (16 columns); each other metric counts its own columns. Order around the wheel: Disk write → Disk read → Net down → CPU → Net up → RAM.

| Metric     | Cols | Arc width | Arc start | Arc end  | Center   |
|------------|------|-----------|-----------|----------|----------|
| Disk write | 3    | 33.75°    | 0°        | 33.75°   | 16.875°  |
| Disk read  | 3    | 33.75°    | 33.75°    | 67.5°    | 50.625°  |
| Net down   | 3    | 33.75°    | 67.5°     | 101.25°  | 84.375°  |
| CPU        | 16   | 180°      | 101.25°   | 281.25°  | 191.25°  |
| Net up     | 3    | 33.75°    | 281.25°   | 315°     | 298.125° |
| RAM        | 4    | 45°       | 315°      | 360°     | 337.5°   |

Centers are pinned by allocation, not search. No per-metric wiggle this time — the column count drives placement.

### Per-metric variant offsets, scaled to arc

Each metric's warm/cool variant offset is a fraction of its arc half-width:

```
offset_metric = (arc_width_metric / 2) × FILL
```

`FILL = 0.7` is shared across all metrics, so variants always leave a small buffer between adjacent metrics' arcs (no two metrics' rendered hues coincide). The walk starts at 0° (Disk write straddles red). Concrete values:

- Disk write / Disk read / Net down / Net up: `±5.9°` variants.
- CPU: `±63°` variants — green at the cool end, deep blue at the warm end.
- RAM: `±15.75°` variants.

CPU's wide variant range is the deliberate point: 16 columns of one metric become visually richer per row, while small-arc metrics keep tighter colour identity.

### `(L, C)` re-optimised against the new hue set

The 18 rendered hues (six centers × {−offset, 0, +offset}) are entirely different from the previous palette — CPU's ±63° variants land far from cyan; small metrics cluster more tightly. Re-run the existing `optimal_lc()` sweep against this new set and pick the new `(L, C)` automatically. The mechanism stays exactly as in the previous change; only the hue inputs change.

### Code shape

`projection.rs`:

- Replace the six fixed `HUE_*` constants and `ALL_HUES` with a `METRIC_HUES: [(metric, center, offset); 6]` table derived from the column-count allocation.
- `variants(hue)` becomes `variants(metric_idx)` (or accepts the per-metric offset directly).
- `render_canvas` passes per-metric `(center, offset)` into `render_metric` via the existing `ColourPair`.
- `optimal_lc()` builds its target set by walking the new table.

The `oklch.rs` module is unchanged.

## Plan

- [x] In `projection.rs`, replace the six `HUE_*` constants and `ALL_HUES`/`HUE_SHIFT_DEGREES` with a `METRIC_HUES: [(center: f32, offset: f32); 6]` table populated from the proportional allocation (centers `[16.875, 50.625, 84.375, 191.25, 298.125, 337.5]`, offsets `[5.91, 5.91, 5.91, 63.0, 5.91, 15.75]` — i.e. `arc_width / 2 × 0.7`). Add a comment block explaining the derivation.
- [x] Add a per-metric index → table-row mapping (`enum MetricKind { DiskWrite, DiskRead, NetDown, Cpu, NetUp, Ram }`, or a constant index per metric).
- [x] Refactor `variants()` to take `(center, offset)` and return the warm/cool `Pixel` pair using the OKLCh round-trip.
- [x] Update `render_canvas` to look each metric's `(center, offset)` from the table and pass into `render_metric`.
- [x] Update `optimal_lc()` to build its target set from `METRIC_HUES` (each row contributes `[center − offset, center, center + offset]`).
- [x] `cargo build --release` and eyeball.

## Conclusion

The Approach didn't survive contact with the panel. Started as planned (proportional arc allocation + per-metric variant offsets scaled to arc width), but eyeballing surfaced problems immediately:

- CPU's wide ±63° offset pushed the warm variant into deep-blue gamut-poor territory; "blue is dull".
- Small-metric arcs gave variant offsets so tight (~±5.9°) that warm/cool were visually indistinguishable.
- Once offsets were uniformised to ±12° to fix the disparity, hue-rotation as the deviation signal became visually weak.

The eventual landing point dropped most of the original Approach: variants no longer rotate hue at all. Instead, the cool variant sits at the metric's centre with the shared uniform `(L, C)`, and the warm variant pushes chroma to the per-hue gamut frontier (×0.95 safety factor). Above-mean rows pop with saturation rather than hue shift.

The hue centres were also re-picked by hand, abandoning proportional walk-the-wheel — the user wanted visually well-defined exemplars (a real green, a clearly-blue not-purple Net up). Final values: `[25°, 65°, 150°, 200°, 250°, 330°]` — red, yellow, green, blue-cyan, blue, magenta.

User flagged reservations about OKLCh as the working palette space — a perceptually uniform space may not be what they want for this display. Parked for future exploration; no proposal spawned yet.

Map: `Row Rendering` node still describes the old `value × colour` rule. Compounding catch-up debt with the previous change; will need a single per-node negotiation when picked up.
