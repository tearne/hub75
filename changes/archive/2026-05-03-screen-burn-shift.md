# Screen-burn shift

## Intent

Mitigate per-LED differential aging by sliding the rendered canvas horizontally by 1 logical column every wall-clock hour, with wrap. After 32 hours the content has cycled through all column positions, so every LED has spent roughly equal time as every metric.

Implementation:
- Track `Instant::now()` at startup as `started_at`.
- Each render: `shift = (started_at.elapsed().as_secs() / 3600) as usize % LOGICAL_WIDTH`.
- Apply as a post-process to the canvas before rotation: each pixel at logical `(x, y)` is read from `(x − shift mod LOGICAL_WIDTH, y)`.

Cadence agile.

## Conclusion

Shipped. `render_canvas` takes a `shift: usize` parameter and post-processes the canvas with a horizontal wrap. Main loop computes `(initial_shift + elapsed_hours) % LOGICAL_WIDTH` per render. Initial offset seeded from `SystemTime::now()` nanoseconds, so reboots start at different positions in the 32-hour cycle.
