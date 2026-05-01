# 2-axis dot blur

## Intent

Replace the current 1-D vertical splat with a 2-D blurred dot. Each lit dot becomes a small soft blob, blurred in both axes — so dots within a strip merge into a continuous lit field instead of reading as a column of vertical streaks. Adjacent samples already overlap vertically; this change adds horizontal overlap so neighbouring columns within a strip blend together too.

Blur extends freely across all visual boundaries — strip-to-strip, CPU sub-strip-to-sub-strip, Disk/Net layer-to-layer. Where two adjacent strips' dots overlap, their colours add naturally.

## Approach

### Kernel: separable 2-D raised cosine

Extend the existing raised-cosine kernel to 2-D as a separable product: `weight(dx, dy) = w(dx) × w(dy)` where `w(d) = 0.5 × (1 + cos(π·d/R))` for `|d| < R`, zero otherwise. Same `R = BLUR_HALF_PIX = 2` for both axes, so dots become a roughly 5 × 5 soft blob (corners weighted near zero, peak in the middle).

### No internal clipping; canvas bounds only

The splat skips any pixel outside the canvas (`px ∉ [0, LW)` or `py ∉ [0, LH)`) but does not clip to strip boundaries. Adjacent strips with different colour palettes can blend at their shared column, and per-core CPU sub-strips blend with their neighbours — both desired.

### Drop the y-clip parameters

`splat_gaussian`'s `y_min` / `y_max` arguments existed to keep horizontal CPU sub-band blur from bleeding into the next sub-band — irrelevant now that CPU is vertical sub-strips and no callers needed those bounds for anything other than the full canvas. Remove the parameters; the function clips to `[0, LH)` directly.

### Rename to reflect the new shape

`splat_gaussian` is misleading on two counts (it's raised cosine, and was 1-D). Rename to `splat_dot` to match what it now does.

### Cost

5 × 5 = 25 pixel updates per dot, vs 5 before. Per frame, the busy strips peak around ~64 visible-row × ~3-dot worst case ≈ 200 dots, ≈ 5000 pixel writes. At 20 fps that's 100 k/s — negligible on a Pi.

## Plan

Version bump: `hub75-client` patch — **0.2.4 → 0.2.5**. Public API unchanged.

- [x] Bump `usb-display/client/rust/Cargo.toml` to `0.2.5`
- [x] Rename `splat_gaussian` → `splat_dot`; update call sites and doc comment
- [x] Drop the `y_min` / `y_max` parameters from the splat function and all call sites; clip to `[0, LH)` internally
- [x] Add the horizontal raised-cosine factor: nest a `for dx in -BLUR_HALF_PIX..=BLUR_HALF_PIX` inside the existing y loop, multiplying the per-pixel weight by `0.5 × (1 + cos(π·|dx|/R))`
- [x] Swap `CPU_USER` and `CPU_SYSTEM` colour values back: `CPU_USER` returns to bright cyan `[50, 230, 230]`; `CPU_SYSTEM` returns to teal `[0, 170, 120]`
- [x] `cargo build --example sysmon_linux --features panel-64x32` succeeds; user runs and visually confirms blob-shaped dots, smoother fill, and the colour swap

## Log

- Initial 5×5 raised-cosine blob (R=2) felt too wide. Iterated through narrower radii: R=1 reverted horizontal blur to nothing (cosine reaches zero at integer ±1, so dx=±1 has weight 0), R=1.5 was smaller but reintroduced vertical judder because a small kernel concentrates brightness in fewer pixels and the centre-of-mass jumps more abruptly between integer rows during sub-pixel slides. R=1.7/1.8 split the difference.
- Decoupled `BLUR_HALF_PIX` (iteration range, integer) from the kernel-shape parameter so we can shrink the visible kernel without losing the wider iteration needed for smooth motion.
- Final kernel: switched from raised cosine to **Gaussian** with σ=1.0 over the same ±2 px iteration. Gaussian's broader-tailed shape (vs cosine's quick drop near boundary) gives smoother sub-pixel motion at a similar visual size. Truncation at the kernel boundary (~14% weight at d=2) is small enough to be invisible. `BLUR_RADIUS` constant retired in favour of `BLUR_SIGMA`.

## Conclusion

Shipped 2-D dot blur: each lit dot is now a soft 5×5 Gaussian blob (σ=1.0) instead of a 1-D vertical streak. Adjacent dots within a strip overlap horizontally, so the field reads as continuous lit area rather than discrete columns. Blur extends freely across all visual boundaries — strip-to-strip, CPU sub-strip-to-sub-strip, Disk/Net layer-to-layer.

Notable deviations beyond the Plan, captured in the Log:

- Kernel shape ended up Gaussian (σ=1.0) rather than the raised cosine the Approach inherited. Iterated through several R values for raised cosine before deciding the broader Gaussian tail tracked sub-pixel motion more smoothly at the same visual size.
- `BLUR_HALF_PIX` and the kernel-shape parameter (`BLUR_RADIUS` → `BLUR_SIGMA`) decoupled so the iteration range and the kernel shape can be tuned independently.

Plan side-task done: `CPU_USER` and `CPU_SYSTEM` colours swapped back (user = bright cyan, system = teal).

`hub75-client` bumped to **0.2.5**. No project changelog, no map. Nothing queued.

