# HUB75 scan-line echo — mechanism & minimum viable fix

*Investigation dates: 2026-04-21 – 2026-04-24*

## Summary

The "echo" artifact — a faint repeated image of scan_line 31 appearing at
scan_line 0 — is triggered in the DP3364S driver chips when GPIO pins 6–10
(the `ADDR[4:0]` bus) are driven to `0b00000` during the data-shifting
phase. The trigger originates in how the RP2350 PIO executes
`out PINS, 6` when `PINCTRL.OUT_COUNT > 6`: pins beyond the instruction's
bit-count are zero-filled on each execution.

With `OUT_COUNT = 11`, every one of the ~130 000 `out PINS, 6` instructions
per frame hammers the ADDR pins to 0, repeatedly presenting the chip with
`ADDR = 0` while LAT pulses propagate shift-register state into the
driver's SRAM. With `OUT_COUNT = 6`, pins 6–10 retain the last value
driven — `scan_line = 31 = 0b11111` from the end of the preceding scan
phase — and no echo appears.

A second, independent echo trigger had been found earlier: RGB=0 on
every scan word. Both triggers produce the same visible symptom and both
fixes (RGB=0x3F scan words + `OUT_COUNT` swap) are applied in the current
working architecture.

## Minimum viable echo-free firmware

`spwm_30_pinctrl_out_only.rs`. One PIO state machine running one unified
program. At each phase transition the CPU writes three registers:

| Register | Data → Scan change |
|----------|--------------------|
| `PINCTRL` | `OUT_COUNT`: 6 → 11 |
| `EXECCTRL` | `WRAP_TOP`/`WRAP_BOTTOM`: data wrap → scan wrap |
| `SM0_INSTR` | `JMP data_cmd` → `JMP scan_entry` |

All other PINCTRL fields — `SIDESET_BASE/COUNT`, `SET_BASE/COUNT`,
`OUT_BASE` — stay constant across the phase boundary.

This is simpler than the prior working baseline (`spwm_17_combined.rs`),
which swapped the full PINCTRL register via a two-program install. The
two-program architecture happened to swap OUT_COUNT as a side effect of
installing programs with different `out_pins()` configs; that side effect
was load-bearing, but all the other PINCTRL changes it made were
incidental.

## Evidence

Four variants, identical except for the indicated PINCTRL fields:

| File | `OUT_COUNT` data/scan | `SET_BASE/COUNT` data/scan | Echo |
|------|-----------------------|-----------------------------|------|
| `spwm_27_execctrl_swap` | 11 / 11 | 12,2 / 12,2 | yes |
| `spwm_28_pinctrl_swap` | **6 / 11** | **12,2 / 13,1** | no |
| `spwm_29_pinctrl_set_only` | 11 / 11 | **12,2 / 13,1** | yes |
| `spwm_30_pinctrl_out_only` | **6 / 11** | 12,2 / 12,2 | **no** |

Between 27 and 30 the only PINCTRL register bit that changes during
data phase is `OUT_COUNT` (bits 25:20), from 11 to 6. That single change
eliminates the echo.

## Mechanism hypothesis (not yet confirmed at the pin level)

The RP2350 PIO spec and `rp235x-hal`'s `out_pins` doc are ambiguous
about `out PINS, N` behaviour when `N < OUT_COUNT`. The empirical
evidence here is consistent with the hardware zero-filling pins
`OUT_BASE + N` through `OUT_BASE + OUT_COUNT - 1`. Direct confirmation
would require a scope/logic-analyser capture of pins 6–10 during the
data phase under both configurations.

## Reconciliation with prior findings

- `spwm_21` (two-program, `OUT_COUNT` already swapped correctly) still
  echoed when scan-word RGB bits were 0. `spwm_22` (same architecture,
  RGB=0x3F) did not. That identified a scan-phase trigger independent
  of data-phase ADDR behaviour.
- The current working builds (both `spwm_17` and `spwm_30`) push
  RGB=0x3F in every scan word and drive `OUT_COUNT = 6` during data,
  so both triggers are suppressed simultaneously.
- Whether these are two manifestations of a single DP3364S internal
  glitch or two genuinely independent chip bugs is not yet known.

## Related firmware sidenote

`pio-0.2.1`'s `Assembler::assemble_with_wrap` (`src/lib.rs:574`)
internally does `source -= 1` on the wrap-source label, because
`bind()` labels the *next* instruction slot. Runtime code that builds
`EXECCTRL.WRAP_TOP` from the label value must account for this — use
the instruction PC (label − 1) directly, not the label value.
`spwm_27` initially hung because it used the label value as
`WRAP_TOP`; fixed 2026-04-24.

## Open questions

1. **Is the zero-fill theory right?** Confirm by driving the data path
   with `out PINS, 11` and explicit `ADDR = 0b11111` bits in the OSR —
   if that also kills echoes with `OUT_COUNT = 11`, the mechanism is
   confirmed.
2. **Why does `ADDR = 0` during data cause the DP3364S to echo?**
   Chip-side question. Possibly related to the PWM sync mode
   configured in register `0x0C[7:6]` (currently `0b10`, not the
   datasheet-documented `0b00` General Frame Sync).
3. **Are the two echo triggers one bug or two?** Would become clearer
   from a logic-analyser capture of the scan_line 31 → 0 boundary
   under varying RGB/ADDR conditions.

## Files referenced

All in `learning-examples/examples/`:

- `spwm_17_combined.rs` — prior baseline (echo-free, two-program swap)
- `spwm_21_rgb_drive.rs` — scan-RGB=0 trigger demonstration
- `spwm_22_rgb_ones.rs` — scan-RGB=1 fix demonstration
- `spwm_27_execctrl_swap.rs` — unified + EXECCTRL swap only → echo
- `spwm_28_pinctrl_swap.rs` — unified + full PINCTRL swap → no echo
- `spwm_29_pinctrl_set_only.rs` — SET swap alone → echo
- `spwm_30_pinctrl_out_only.rs` — **OUT_COUNT swap alone → no echo**
