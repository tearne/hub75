# RAM at layout end

## Intent

Move RAM from the middle of the column layout to the end, so the metric that's adjacent to Disk write across the screen-burn wrap is RAM rather than Net up. Keeps the Disk pair and Net pair grouped within the linear order; RAM becomes the visual buffer at the wrap.

## Approach

New order, left to right (logical columns): Disk write | CPU0 | Disk read | CPU1 | Net down | CPU2 | Net up | CPU3 | RAM. CPU cores at width 4, Disk and Net halves at width 3, RAM at width 4 — sums to 32 unchanged.

| Metric     | Cols  | Width |
|------------|-------|-------|
| Disk write | 0–2   | 3     |
| CPU0       | 3–6   | 4     |
| Disk read  | 7–9   | 3     |
| CPU1       | 10–13 | 4     |
| Net down   | 14–16 | 3     |
| CPU2       | 17–20 | 4     |
| Net up     | 21–23 | 3     |
| CPU3       | 24–27 | 4     |
| RAM        | 28–31 | 4     |

Strict CPU/non-CPU alternation preserved inside the linear layout. Only adjacency that's two non-CPUs is the wrap (RAM ↔ Disk write), which the structural constraint (5 non-CPU + 4 CPU in 9 slots) makes unavoidable somewhere.

## Plan

- [x] Update `presentation.rs`: `DISK_WRITE_LEFT`, `DISK_READ_LEFT`, `NET_DOWN_LEFT`, `NET_UP_LEFT`, `RAM_LEFT`, `CPU_LEFTS`.
- [x] Update Presentation node in `map.md`: layout description and column-widths table.

## Conclusion

Completed. Layout changed in `presentation.rs` and the Presentation node in `map.md`. No other code touched.
