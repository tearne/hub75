# Render once per sample

## Intent

Drop the separate render rate. With the band scheme, the panel only changes when a sample arrives (no sub-pixel sliding between samples), so rendering between samples produces identical frames — wasted CPU and pointless USB traffic. Collapse the architecture to a single loop that samples and renders together at the sampling rate.

Concretely:

- Remove `Mode.render_interval` and the separate render thread.
- Sampler becomes the only loop: sleep `sampling_rate` → sample all metrics → render canvas → send frame.
- `Slate` no longer needs the `Mutex` (single-threaded ownership).
- `Modes` simplifies to `(name, sampling_rate)` — drop the render-rate column.
- Update Display and Modes nodes in the map accordingly.

This is a code-side simplification motivated by the band rewrite. The benefit: lower CPU, simpler code, cleaner concept (one cadence governs everything).

Cadence agile.

## Conclusion

What shipped:

- `Mode` is now `(name, sampling_rate)` — the `render_interval` field is gone.
- `main.rs` collapsed to a single loop: sample → render → send → sleep. No spawned thread, no `Arc<Mutex<Slate>>`, no separate render function. Slate is owned directly by main.
- The unused per-metric multiplier constants (`RAM_MULTIPLIER`, `NET_MULTIPLIER`, `DISK_MULTIPLIER`) and their gating logic were dropped. All metrics sample on every cycle uniformly.
- Map's Display Detail rewritten: render runs once per sampling cycle. Modes Detail rewritten: simplified table (no render-rate column), depth formula explicit.

Net result: simpler code (fewer threads, no synchronisation), simpler concept (one rate governs everything), and the panel content is consistent with the data — no rendering of identical frames between samples.
