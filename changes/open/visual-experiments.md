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

