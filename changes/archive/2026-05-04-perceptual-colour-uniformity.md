# Perceptual colour uniformity

## Intent

Make sysmon2's panel colours perceptually uniform along two axes:

1. **Hue-shift variants** (the warm/cool pair per metric, currently produced by HSL rotation in `projection.rs::shift_hue`) should preserve perceptual brightness across the rotation. HSL's `L` is geometric, not perceptual — rotating in HSL produces variants of unequal apparent luminance, and for cyan and green metrics actually reverses the warm-vs-cool brightness ordering. Switch to OKLCh (or a similar perceptually uniform space) for the rotation so above/below variants read at matched luminance.

2. **Base metric colours** themselves (`CPU_COLOUR`, `RAM_COLOUR`, etc. in `projection.rs`) are not currently matched on perceptual brightness — yellow-lime, amber-gold and cyan all read brighter than red and magenta at the same nominal RGB scale. Bring them onto a common perceptual-luminance target so no single metric visually dominates the panel by virtue of its hue.

Both are display-tuning concerns separate from data semantics.

## Approach

### Use OKLCh, gamma-aware

Replace the HSL helpers in `projection.rs` with conversions through OKLCh — a perceptually uniform polar form of OKLab. Pipeline is the standard one: sRGB (8-bit) → linear sRGB → OKLab → OKLCh → manipulate → OKLab → linear sRGB → sRGB. Gamma matters; skipping the linearisation step is the usual reason these conversions look wrong.

OKLCh's `L` is perceptual lightness, `C` is chroma, `h` is hue in degrees. Hue rotation is a straight `h += degrees`. Lightness equalisation is `L = target_L`.

### Pick `(L, C)` to maximise uniform chroma across all hues

All six base metric colours, *and* their warm/cool ±30° rotations, land on a common `(L, C)` point — only `h` differs. Both perceptual lightness and chroma are uniform across the panel. The existing RGB values' chromas are discarded; only their hues survive.

The chosen `(L, C)` is the one that maximises the largest `C` simultaneously achievable in the sRGB gamut at every hue we render. Concretely, the relevant hue set is 18 angles: 6 base hues × {−30°, 0°, +30°}. For a candidate `L`, the maximum representable `C` at a hue is computed via Björn Ottosson's analytical gamut-frontier formula (closed-form, no iteration). We sweep `L` (200 steps from 0 to 1) and pick the one that maximises the worst-case (bottleneck) `C` across the 18 hues.

Outcome: rich, equal chroma across every painted dot; the panel reads as one coherent palette differing only in hue. As far as I know this is what "uniform perceptual rendering" gets you.

### Lightness and chroma preserved through hue rotation

Inside `shift_hue`, after rotating `h` we deliberately do not change `L` or `C`. The point of OKLCh is that constant `L` and `C` mean constant perceptual brightness and constant colourfulness. The warm/cool variant pair derived from a base colour inherits that base's `(L, C)` automatically — and since `(L, C)` was chosen so the rotated hues are still in-gamut, no clipping happens.

### Module split

The OKLCh maths goes in a new `oklch.rs`. Conversions are pure functions, not tied to anything in `projection.rs`. `projection.rs` calls into it for the rotation helper and for the startup-time equalisation pass.

### Re-derive RGB constants from hue alone

The six metric colours live as their **hues** (one `f32` each, in degrees), not as RGB triplets. At startup the equalisation pass computes `(L, C)` once and converts `(L, C, h_i)` → sRGB for each hue. Result: a `[Pixel; 6]` lookup populated via `OnceLock`. The hand-tuned chroma of the original colours is discarded.

Hues are extracted once from the current RGB constants and hard-coded in degrees. Re-tuning a metric's hue is then a single number edit.

### Equal hue spacing

Six metrics, 60° apart on the OKLCh hue circle. CPU pinned at 180° (cyan); RAM at 300° (magenta, the closest tick to its current 325°). Remaining four slots filled to keep the disk pair adjacent and the net pair flanking CPU:

| Metric     | Hue   |
|------------|-------|
| Disk write | 0°    |
| Disk read  | 60°   |
| Net down   | 120°  |
| CPU        | 180°  |
| Net up     | 240°  |
| RAM        | 300°  |

Net up shifts substantially (lime → blue); the rest stay close to their current hues.

## Plan

- [x] Create `usb-display/client/sysmon2/src/oklch.rs` with pure conversions: sRGB ↔ linear sRGB (gamma), linear sRGB ↔ OKLab (matrix), OKLab ↔ OKLCh (cartesian/polar). Also `max_chroma(L, h) -> f32` using Ottosson's analytical gamut-frontier formula.
- [x] Add `oklch.rs` to `main.rs` module list.
- [x] In `projection.rs`, replace the six `*_COLOUR` `Pixel` constants with a `HUES: [(metric, degrees)]` table at the per-metric mapping. Move `CPU_COLOUR` etc. to `OnceLock<Pixel>` populated from the equalised `(L, C, h)`.
- [x] Add a `compute_lc()` function: sweeps L in 200 steps from 0 to 1, computes the worst-case `max_chroma` across the 18 target hues (6 base × {−30°, 0°, +30°}) at each L, returns the `(L, C)` maximising that worst case. Caches the result via `OnceLock` and prints the chosen `(L, C)` to stderr on first computation so the values can later be inspected and hard-coded if desired.
- [x] Replace the existing `shift_hue` (HSL) implementation with an OKLCh version: convert sRGB → OKLCh, add `degrees` to `h`, convert back. Remove `rgb_to_hsl`, `hsl_to_rgb`, `hue_to_rgb` helpers.
- [x] `cargo build --release` and eyeball.

## Conclusion

OKLCh-based palette in place: six metric hues placed on a roughly-60°-spaced grid with ±20° wiggle, all sharing the same equalised `(L, C)` chosen to maximise the worst-case in-gamut chroma across the 18 rendered hues (six bases × {−30°, 0°, +30°}). Final values: hues `[2°, 74°, 104°, 185°, 253°, 319°]`; `(L, C) = (0.6400, 0.1128)`.

`oklch.rs` carries the conversions (sRGB ↔ linear ↔ OKLab ↔ OKLCh) and Ottosson's analytical `max_chroma`. The unused but useful sRGB→OKLCh direction is marked `#[allow(dead_code)]` for future use.

Eyeball confirmed the result was acceptable; superseded immediately by the new proposal in `changes/open/proportional-hue-distribution.md`, which redistributes the hue circle proportionally to per-metric column count.
