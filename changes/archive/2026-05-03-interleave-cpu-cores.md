# Interleave CPU cores

## Intent

Reshuffle the panel layout so the four CPU cores are interleaved between the other metrics (Disk write, Disk read, RAM, Net down, Net up) rather than clustered together. Pattern, left to right: Disk write, CPU0, Disk read, CPU1, RAM, CPU2, Net down, CPU3, Net up.

Cadence agile.

## Conclusion

Layout updated in `presentation.rs`: `CPU_LEFT` removed (cores no longer contiguous), replaced with a `CPU_LEFTS: [usize; CORE_COUNT] = [4, 11, 18, 25]` lookup. Non-CPU `_LEFT` constants reflowed to the interleaved positions. Map's Presentation Detail table and Layout prose updated to match. Sums to 32 with no gaps; no other code touched.
