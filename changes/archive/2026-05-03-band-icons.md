# Band icons

## Intent

Add small dim icons that scroll along with each band, indicating what the band is.

- Disk read / Net down: `^` (incoming)
- Disk write / Net up: `v` (outgoing)
- CPU / RAM: small centred square / dot

Icons are 2 rows tall, appear every 4 commits, painted at ~15% of the metric's colour. Dot pixels rendered on the same row overwrite the icon — the icon is a background hint, the dots take precedence.

Cadence agile.

## Conclusion

Tried, didn't work. Implemented icons as 2-row glyphs (`^` for incoming, `v` for outgoing, square for CPU/RAM) at 15% colour intensity, every 4 commits per band. The icon machinery added a `BandRow.icon` field, an `IconShape` enum, per-band commit counter, and `icon_shape` parameter on `MetricBands::new`.

Cycle of 4 was too dense visually; tried 32 too — still didn't read as intended icons, just looked like noise.

Reverted entirely. Replaced with a solid dim-colour background fill behind the dot pattern (ultimately gated behind a `BACKGROUND_ENABLED` toggle, currently off) which gives a similar "metric is here" cue without trying to encode metric type into a glyph the eye couldn't really parse at 3- or 4-pixel widths.
