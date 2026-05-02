# Visual experiments

## Intent

Open a session for small visual experiments on the sysmon panel — tuning the look and feel rather than rebuilding any one piece. Things like default update rate, colour adjustments, blur tuning, and screen-burn-mitigating drift are illustrative of the kinds of tweaks in scope; the actual list will emerge as we iterate. Each item is independent and small; acceptable to ship some and abandon others.

## Approach

### Work scope: `sysmon` crate only

All experiments touch the standalone `sysmon` crate at `usb-display/client/sysmon/`. Nothing in the underlying `hub75-client` library moves — these are pure presentation tweaks.

### Iteration loop: edit, build, observe, decide

Each experiment is a quick code change → `cargo build --release` → user runs the binary on the Pi → user reports what they see → keep, tune, or revert. No batching. No formal sub-plans.

### Out-of-scope escalation

If an experiment turns out to need significant infrastructure (e.g. screen-burn drift requires substantial layout rework), it gets pulled out of this session into its own change. We commit to an experiment for this session only when it's a single-constant or small-helper-function change.

### Versioning

Bump `sysmon` from `0.1.0` → `0.1.1` once at the end of the session covering whatever tweaks survive. No per-experiment version churn.

### Documentation

The crate's README isn't touched unless an experiment changes user-visible CLI behaviour. The change document itself logs what was tried and the outcome.

## Plan

**Topics** (illustrative — actual experiments emerge in conversation):

- Default update rate.
- Per-strip palette balance / contrast.
- Blur sigma and kernel size relative to current time-compression density.
- Horizontal drift for LED-wear mitigation.
- Whatever else surfaces while looking at the panel.

**Done when** the user signals satisfaction with the current visual and asks to wrap up.

A version bump task is queued for end-of-session: bump `sysmon` to `0.1.1`.

## Log

- Bumped `BLUR_SIGMA` from 0.5 → 0.7 (softer vertical Gaussian).
- Tuned `window_alpha` exponent: 0.7 → 0.75 → 0.65. The detour to 0.75 went the wrong way (toward mean-uniform / dimmer bottom); 0.65 leans toward variance-uniform / brighter bottom.
- Introduced colour gradient on every strip: each splat's colour interpolates between a `_LOW` and `_HIGH` anchor by the same fraction that drives `dots_for`. Committed as `e9feb10 intensity colour gradient`.
- For CPU specifically, the three sub-segments (user/system/iowait) all share one gradient driver — total core activity — rather than each sub-fraction. Hue still discriminates the kind of work; gradient intensity tracks overall busyness. Without this, system and iowait sat at their LOW ends almost permanently because their fractions rarely exceed ~0.2.
- Reworked palette into a colour-wheel spread: CPU clusters in the blue band (cyan/azure/deep-blue), RAM in magenta, DISK on the warm side (red/amber), NET in green (forest/lime). Replaced the older teal/forest CPU_SYSTEM that collided with NET_DOWN's hue.
- Latent bug surfaced and fixed: RAM column was painting black on a 16GB Pi. Cause: `dots_for(used_frac, 8)` rounded to 0 for any usage below 6.25%. On this Pi the existing `used = total - max(available, free + buffers + cached_total)` formula gave ~5.5% (cache reserve causes `free + buffers + cached_total` to slightly exceed `available`). Two-part fix: (a) `dots_for` now floors at 1 dot for any positive fraction; (b) RAM "used" now uses the simpler `total - available` (matches btop), giving ~6.6% on the same machine. Both changes apply to all strips, not just RAM — any low-but-nonzero CPU/disk/net moment now stays visible too.

## Conclusion

Completed. Version bumped 0.1.0 → 0.1.1 as planned. Scope held to the `sysmon` crate; `hub75-client` was not touched. The latent RAM-zero-dots bug was outside the plan's stated topics but fell within "whatever else surfaces while looking at the panel" and was small enough to fit the session's single-helper-function bar.

