# Process feedback

## 2026-05-05 — Versioning applies regardless of mode

Context: Wander change `sysmon-dot-dimming` shipped a behaviour change to a versioned crate (sysmon, 0.3.0) without a version bump, because Wander has no Plan stage and PROCESS.md only attaches the bump task to the Plan.

User's rule: if versioning is in effect, every time code is handed back to the user the version is to be bumped by an appropriate amount, irrespective of the change mode. The bump should happen at hand-back time, not be tied to having a Plan.
