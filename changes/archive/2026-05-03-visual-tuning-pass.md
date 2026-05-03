# Visual tuning pass

## Intent

A loose tuning session over sysmon2's visual rendering — started narrowly as "centered soft bars" but expanded as we iterated on the panel. The unifying theme is "small targeted changes to make the panel read better at a glance"; rather than scoping each tweak as its own change, they were bundled here.

Cadence agile.

## Conclusion

What shipped, in the order it landed:

- **Centered soft bars** (the original intent). Bar fill in vertical mapping reworked: bar grows symmetrically outward from the centre of each metric's column slice, with per-column intensity = `overlap × v × colour`. So the bar is both spatially shorter *and* visually dimmer at low values, instead of maxing out one column at a time.

- **Layout reweighted**: CPU cores 3 → 4 columns wide; Disk and Net halves 4 → 3 columns wide; RAM unchanged at 4. Sums to 32 with no gaps. Gives CPU more visual presence, since on a desktop workload the four cores are the most active and most differentiated metrics.

- **Map cleanup**: renamed *Vertical Mapping* → *Time Compression* throughout the map (anchor, references, tree). Trimmed the node's prose substantially (~2,900 chars → ~1,230) — same content, less verbose.

- **Time compression turned up**: window-curve geometric base 1.07 → 1.10. Bottom of CPU column now reaches ~55 min back in production mode (was ~17 min); Disk reaches ~18 hours. Ring length grows from ~1,185 to ~3,340 entries per metric — still well under 1 MB total memory across all nine rings.

- **Net floor raised**: `NET_LOG_MAX_FLOOR` 10 KB/s → 200 KB/s. Net bars now read less hot at typical idle (background DNS, sync chatter) and only fill meaningfully on real network activity. Disk floor unchanged at 10 KB/s.

No version bump; sysmon2 still v0.1.0.

Going forward: tweaks like the curve base and throughput floors should ideally be their own small changes rather than bundled. This change is the one-time exception covering the early-iteration drift.
