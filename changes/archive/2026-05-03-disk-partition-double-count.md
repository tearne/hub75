# Disk partition double-count

## Intent

Fix sysmon2's disk byte reader, which currently sums both whole block devices and their partitions from `/proc/diskstats`. The whole-device counters already aggregate partition activity, so the displayed Disk read/write rates are roughly doubled.

Fix: skip entries whose `/sys/class/block/<name>/partition` file exists (the kernel only creates that file for partitions, not whole devices). This handles all naming conventions — `sdaN`, `mmcblk0pN`, `nvme0n1pN`, etc. — without baked-in regex assumptions.

Cadence agile.

## Conclusion

`is_real_block_device` in `throughput.rs` renamed to `is_whole_block_device` and extended to skip partitions by checking `/sys/class/block/<name>/partition`. Verified on this Pi: filter keeps `mmcblk0` and `zram0`, drops `mmcblk0p1` and `mmcblk0p2`. Map untouched (no map node mentions partition handling).
