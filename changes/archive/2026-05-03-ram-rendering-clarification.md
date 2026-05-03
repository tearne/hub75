# RAM rendering clarification

## Intent

Step through how RAM rendering works in its current form, build shared understanding, develop the vocabulary to describe it cleanly (the "sample slate" / "panel buffer" distinction surfaced at the end of the previous change), make any algorithmic adjustments that fall out of the discussion, and finally update the map and refactor the code so the names match the concepts.

Cadence agile.

## Working notes

### Key new concepts

- **Sample slate** — data-space. A ring of historical metric values. All nine metrics have one. (Currently just called "Slate" in the map.)
- **Panel buffer** — panel-coordinate space. Per-panel-row buffer of rendering state, carried between render frames. Only RAM has one today; the bar metrics render statelessly from their sample slate alone.

### Conceptual flow

```
sampling → sample slate → render path → panel
                            │
                            └─ for RAM: writes/reads a panel buffer
                               as stateful intermediate
```

### RAM's panel buffer (as currently implemented)

- 66 entries: one per visible row + one above-edge + one below-edge.
- Each entry holds:
  - `pattern` — bitmask of currently-lit columns in RAM's 4-wide slice.
  - `frac_offset` ∈ [0, 1) — accumulator that ticks toward the next inheritance from the row above. Increments by `1/N(r)` per RAM sample.
  - `intensity` — cached `V[r]` (window-aggregated value from the sample slate) for render-time brightness.

### Open naming decisions (not yet acted on)

- Existing "Slate" map node → rename to "Sample Slate"? Or leave "Slate" as umbrella term?
- Where does "Panel Slate" sit in the map tree? Sibling of Time Compression? Child of it? Hanging off RAM?
- `RamStream` struct in code: split into sample-side `Ring` (matching other metrics) and a separate render-side struct, or keep bundled and rename?

### Open algorithmic questions

- Is the cascade-shift behaviour (rows shift independently at their own rates, intermediate top patterns "lost" if downstream rows haven't shifted) the right one? Or do we want carry-over so every top pattern eventually propagates somewhere?
- Should the panel buffer generalise — could other metrics benefit from one too?

## Conclusion

What landed during the discussion:

- **Inertia in adjustments.** The seed for `adjust_pattern` changed from `(sample_no, row)` to a stable per-row seed. Toggling target N up and down now reversibly toggles the same column rather than picking different ones each sample.
- **Inherited-column protection.** Each `RamRow` now carries an `inherited: u8` mask. When a row inherits from above (`pattern = snap[r-1]`), `inherited` records what came in. Adjustments prefer to drop *non-inherited* columns first; inherited columns are only forced out when target N falls below `popcount(inherited)`, at which point they're cleared from the mask too.

What did not land:

- The "sample slate" / "panel buffer" naming distinction was clarified verbally (and surfaced the conceptual error in the map), but neither the map nor the code was refactored.
- The cascade-shift "lost patterns" issue was identified but not solved. Inheritance is event-based: when a row above shifts (e.g. row 0 reseeds every sample) and the row below hasn't reached its own inheritance event yet, the intermediate pattern is discarded. The next downstream inheritance picks up the *latest* upstream state, not whatever was "in transit". Visually: streaks vanish at row boundaries where cells have different inheritance rates.

Three sketches of a "smarter inherit" were enumerated (inheritance queues; smearing during transit; particle-based rendering with no per-row buffer), but the user changed direction before any was chosen — to be picked up in the next change with a different approach entirely.
