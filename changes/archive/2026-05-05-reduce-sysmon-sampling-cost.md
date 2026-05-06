# Reduce sysmon sampling cost

**Mode:** Exploration

## Intent

Sysmon's host-CPU cost is dominated by reading and re-parsing `/proc` pseudofiles every cycle: at fast-mode (50 ms) ~83% of CPU is sampling, with `/proc/diskstats` alone at 38%; at 20 ms the picture barely shifts (~75% sampling). The original premise that USB framing was the bottleneck doesn't hold — USB writes are 12–15%. Reduce the sampling cost so refresh rate can rise without hurting the host.

In scope: the four `/proc` reads (`/proc/diskstats`, `/proc/stat`, `/proc/net/dev`, `/proc/meminfo`) at `sysmon/src/cpu.rs:57`, `ram.rs:20`, `throughput.rs:78`, `throughput.rs:112`, and the cadence at which they are called from the main loop. Out of scope: USB protocol, render pipeline, on-Pico rendering — these are smaller wins and can be revisited later.

## Approach

### Switch from `/proc` aggregate files to `/sys` per-device files

`/proc/diskstats` formats every block device and partition through `seq_file` text formatting; we then re-parse and discard most of it. `/sys/block/<dev>/stat` is one tiny single-line file per device — minimal kernel work, minimal parsing. Same story for `/proc/net/dev` vs `/sys/class/net/<iface>/statistics/{rx,tx}_bytes` (each is a single decimal — no parsing at all). At fast-mode rates these reads are effectively free.

### Sample cadence stays at frame rate

Sample cadence drives band scroll: each `push_sample` advances a row through the bands, so the visual flow rate equals the sample rate. Decoupling sampling from rendering would freeze a band for many frames between scrolls. Keep sampling every frame; rely on the cheaper source to absorb the cost.

### Disk and net scope: physical devices, auto-detected

Cover the host's main physical disks and NICs — SD on a Pi 4/5, USB-attached SSD on USB-boot Pis, NVMe on Pi 5 + HAT; on-board eth + wifi plus any USB wifi dongle. Detect via the `/sys/.../device` symlink: present for physical devices (`mmcblk*`, `sd*`, `nvme*n*`, `eth*`, `wlan*`, `wlx*`), absent for virtuals (`loop`, `ram`, `zram`, `dm-*`, `lo`, `docker*`, `br-*`, `veth*`, `tun*`). Sum across detected devices, same as today. Small behaviour change: `zram0` (currently summed into disk) and any docker/VPN interfaces (currently summed into net) become excluded.

### CPU and RAM sampling left as-is

`/proc/stat` has no clean `/sys` equivalent for per-core busy fractions, and at "only" ~22% it's a smaller fish. `/proc/meminfo` at 4–8% is a single small file; field-targeted parsing might shave it but isn't urgent. Both can be revisited in a later change if the post-fix profile still shows them as the dominant cost.

## Plan

### Topics

- Replace the disk source: enumerate physical block devices via `/sys/block/*/device` symlinks, read `/sys/block/<name>/stat` for each, sum sectors. Drop the `/proc/diskstats` parse and the partition-marker check.
- Replace the net source: enumerate physical NICs via `/sys/class/net/*/device` symlinks, read `statistics/rx_bytes` and `statistics/tx_bytes` for each, sum. Drop the `/proc/net/dev` parse.
- Re-profile under the same conditions as Runs 1 & 2; report the delta against the original breakdown.

### Done when

A re-profile shows the dominant `/proc` sampling cost meaningfully reduced. The user judges sufficiency from the new breakdown rather than a fixed target.

## Conclusion

Completed. Disk and net `/proc` reads replaced (disk → cached `/sys/block/<n>/stat` per discovered physical device; net stayed on `/proc/net/dev` after a regression with `/sys/class/net/.../statistics/*` exposed `dev_get_stats()` per-counter cost). Total sysmon CPU work cut by ~47%; sampling dropped from 83.5% to 64.4% of host CPU. USB write is now the dominant single stage at 24% — to be picked up in a follow-up change.

`sysmon/PROFILING.md` was added to support this work and is reusable for future tuning. The `audit-markdown-accuracy.md` change should cover it on its sweep.

## Log

- Wrote `sysmon/PROFILING.md` as a step-by-step guide. User runs the profiling themselves (sudo + live hardware needed). Two runs requested: fast mode (50 ms) and an amplified-bottleneck mode (cycle pushed below sustainable, e.g. 20 ms via temporary `FAST` constant edit).
- Switched the guide from `cargo flamegraph` SVGs to a `perf record` → `inferno-collapse-perf` → `flamelens` (TUI) pipeline on user request. Static SVG is still available as an optional inferno-flamegraph step.
- Run 1 results (50 ms fast mode, 30 s capture): `/proc` sampling dominates host CPU. Disk 38.2%, CPU 22.4%, Net 14.5%, RAM 8.4% — total 83.5% of CPU is reading `/proc` text and re-parsing. USB write only 12.1%. Render 2.4%. Inverts the change's premise: the three Intent options (vendor-class USB, on-Pico render, RLE) all target the 12% slice. Real win is sampling-side.
- Run 2 (20 ms cycle, ~1B weighted samples): same picture confirmed. Disk 34.1%, CPU 21.8%, Net 15.3%, USB 15.3%, RAM 4.3%, Render 5.3%. USB and Render shares grow modestly with frame rate but `/proc` reads still dominate (~75%). User confirmed redirect to sampling-cost attack.
- Change renamed from `usb-protocol-optimisation` to `reduce-sysmon-sampling-cost` and Intent/Approach/Plan rewritten to match the redirected aim.
- First implementation pass swapped both disk and net to `/sys`. Re-profile showed disk down (38% → 30%) but **net regressed** (14.5% → 22.8%): each `/sys/class/net/<iface>/statistics/<counter>` file read triggers a full kernel `dev_get_stats()` that returns one number, so 4 reads/cycle (rx+tx × 2 NICs) costs more than parsing `/proc/net/dev` once. Also new kernfs path-walk cost from re-enumerating `/sys/block/` every cycle.
- Refinement: cached disk device discovery in a `OnceLock<Vec<PathBuf>>` (USB hotplug after startup not picked up — acceptable). Reverted net to `/proc/net/dev` with a comment recording why `/sys/class/net/.../statistics/*` is a trap.
- Final re-profile (50 ms fast mode, 30 s capture): total weighted samples 658M → 351M (**−47% CPU work**). Disk 251M → 65M (−74% absolute). Net 95M → 54M (−43%). RAM 55M → 15M (−73%). CPU 147M → 90M (−39%). USB write flat at 84M but proportional share grew from 12% to 24% — now the largest single stage. Sampling total dropped from 83.5% to 64.4% of CPU.

