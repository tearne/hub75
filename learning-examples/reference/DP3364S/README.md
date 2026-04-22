# DP3364S reference

Datasheet and extracted register notes for the **Depuw DP3364S** 16-channel
S-PWM constant-current LED driver — the chip used on the 64×128 HUB75
panel this project is tested against (`learning-examples/README.md`).

## Files

- `DP3364S_Rev1.0_EN.pdf` — official English datasheet, Rev 1.0, 2024-04-15.
- `DP3364S_Rev1.0_EN.txt` — plain-text extract for grep-ability. Tables and
  register-map images do not extract cleanly; refer to the PDF for those.

## Provenance

Datasheet sourced from
<https://github.com/hzeller/rpi-rgb-led-matrix/issues/1866> (SPWM panel
collation thread), attachment URL
<https://github.com/user-attachments/files/25687538/DP3364S.pdf>.
Additional register-field notes for HUB320-variant panels are in
<https://github.com/hzeller/rpi-rgb-led-matrix/issues/1821#issuecomment-3506532993>
(images only — not extracted here).

## Key facts the text extract surfaces

- **§6.1 — Max clock frequency is 25 MHz.** The project currently runs
  DCLK at 37.5 MHz (50 % over spec). Above-spec operation is flagged in
  the datasheet as risking permanent damage.
- **15 valid register addresses** (§10.5); the project's `CONFIG_REGS`
  array writes 0x01–0x0D (13 of them). 0x0E and 0x0F are left at
  power-on defaults.
- **No soft-reset register / command.** Recovery from a latched-bad
  state requires a panel power-cycle.
- **Four PWM display modes** (§10.7). The project uses mode 2 —
  High-Gray Data Independent Refresh Synchronous — via
  `reg0x0C[7:6] = 10`.
- **Power-on mode selection** (§10.3): SDR (single-edge) vs DDR
  (double-edge). The chip must be sent the SDR/DDR command after
  power-on; the project relies on single-edge being the power-on
  default.
