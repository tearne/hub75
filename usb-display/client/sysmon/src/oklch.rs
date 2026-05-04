#![allow(dead_code)]
//! OKLab / OKLCh colour-space conversions and sRGB-gamut max-chroma
//! lookup, after Björn Ottosson (https://bottosson.github.io/posts/oklab/).
//!
//! All conversions are gamma-aware: sRGB byte triplets ↔ linear sRGB
//! ↔ OKLab ↔ OKLCh. Chroma here is in OKLab units (typically 0–0.4
//! for displayable colours).
//!
//! `max_chroma(L, h)` returns the largest `C` such that the OKLCh
//! triplet `(L, C, h)` round-trips into in-gamut sRGB. Implementation
//! follows Ottosson's analytical gamut-frontier formula (with one
//! Halley refinement step in the upper half).

use crate::slate::Pixel;

// ── Public API ─────────────────────────────────────────────────────

#[allow(dead_code)]
pub fn srgb_to_oklch(rgb: Pixel) -> (f32, f32, f32) {
    let (l, a, b) = oklab_from_linear(linear_from_srgb(rgb));
    let c = (a * a + b * b).sqrt();
    let h = b.atan2(a).to_degrees().rem_euclid(360.0);
    (l, c, h)
}

pub fn oklch_to_srgb(l: f32, c: f32, h: f32) -> Pixel {
    let h_rad = h.to_radians();
    let a = c * h_rad.cos();
    let b = c * h_rad.sin();
    srgb_from_linear(linear_from_oklab(l, a, b))
}

/// Largest `C` such that `(L, C, h_degrees)` maps to in-gamut sRGB.
pub fn max_chroma(l: f32, h_degrees: f32) -> f32 {
    let h_rad = h_degrees.to_radians();
    let a = h_rad.cos();
    let b = h_rad.sin();
    let cusp = find_cusp(a, b);
    find_gamut_intersection(a, b, l, 1.0, l, cusp)
}

// ── Gamma (sRGB ↔ linear) ──────────────────────────────────────────

#[allow(dead_code)]
fn linear_from_srgb(rgb: Pixel) -> [f32; 3] {
    [
        srgb_decode(rgb[0] as f32 / 255.0),
        srgb_decode(rgb[1] as f32 / 255.0),
        srgb_decode(rgb[2] as f32 / 255.0),
    ]
}

fn srgb_from_linear(lin: [f32; 3]) -> Pixel {
    [
        (srgb_encode(lin[0]) * 255.0).clamp(0.0, 255.0).round() as u8,
        (srgb_encode(lin[1]) * 255.0).clamp(0.0, 255.0).round() as u8,
        (srgb_encode(lin[2]) * 255.0).clamp(0.0, 255.0).round() as u8,
    ]
}

#[allow(dead_code)]
fn srgb_decode(c: f32) -> f32 {
    if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
}

fn srgb_encode(c: f32) -> f32 {
    let c = c.clamp(0.0, 1.0);
    if c <= 0.0031308 { 12.92 * c } else { 1.055 * c.powf(1.0 / 2.4) - 0.055 }
}

// ── OKLab forward / inverse (linear sRGB ↔ OKLab) ──────────────────

#[allow(dead_code)]
fn oklab_from_linear(lin: [f32; 3]) -> (f32, f32, f32) {
    let r = lin[0]; let g = lin[1]; let b = lin[2];
    let l = 0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b;
    let m = 0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b;
    let s = 0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b;
    let l_ = l.cbrt(); let m_ = m.cbrt(); let s_ = s.cbrt();
    (
        0.2104542553 * l_ + 0.7936177850 * m_ - 0.0040720468 * s_,
        1.9779984951 * l_ - 2.4285922050 * m_ + 0.4505937099 * s_,
        0.0259040371 * l_ + 0.7827717662 * m_ - 0.8086757660 * s_,
    )
}

fn linear_from_oklab(big_l: f32, a: f32, b: f32) -> [f32; 3] {
    let l_ = big_l + 0.3963377774 * a + 0.2158037573 * b;
    let m_ = big_l - 0.1055613458 * a - 0.0638541728 * b;
    let s_ = big_l - 0.0894841775 * a - 1.2914855480 * b;
    let l = l_ * l_ * l_;
    let m = m_ * m_ * m_;
    let s = s_ * s_ * s_;
    [
         4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s,
        -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s,
        -0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s,
    ]
}

// ── Ottosson's gamut frontier ──────────────────────────────────────

/// `(a, b)` must be unit-norm (cos h, sin h).
fn compute_max_saturation(a: f32, b: f32) -> f32 {
    let (k0, k1, k2, k3, k4, wl, wm, ws) =
        if -1.88170328 * a - 0.80936493 * b > 1.0 {
            // Red component limits first
            (1.19086277, 1.76576728, 0.59662641, 0.75515197, 0.56771245,
              4.0767416621, -3.3077115913, 0.2309699292)
        } else if 1.81444104 * a - 1.19445276 * b > 1.0 {
            // Green
            (0.73956515, -0.45954404, 0.08285427, 0.12541070, 0.14503204,
             -1.2684380046, 2.6097574011, -0.3413193965)
        } else {
            // Blue
            (1.35733652, -0.00915799, -1.15130210, -0.50559606, 0.00692167,
             -0.0041960863, -0.7034186147, 1.7076147010)
        };

    let mut s = k0 + k1 * a + k2 * b + k3 * a * a + k4 * a * b;

    // One Halley step
    let k_l =  0.3963377774 * a + 0.2158037573 * b;
    let k_m = -0.1055613458 * a - 0.0638541728 * b;
    let k_s = -0.0894841775 * a - 1.2914855480 * b;

    let l_ = 1.0 + s * k_l;
    let m_ = 1.0 + s * k_m;
    let s_ = 1.0 + s * k_s;

    let l = l_ * l_ * l_;
    let m = m_ * m_ * m_;
    let ss = s_ * s_ * s_;

    let l_ds = 3.0 * k_l * l_ * l_;
    let m_ds = 3.0 * k_m * m_ * m_;
    let s_ds = 3.0 * k_s * s_ * s_;

    let l_ds2 = 6.0 * k_l * k_l * l_;
    let m_ds2 = 6.0 * k_m * k_m * m_;
    let s_ds2 = 6.0 * k_s * k_s * s_;

    let f  = wl * l    + wm * m    + ws * ss;
    let f1 = wl * l_ds + wm * m_ds + ws * s_ds;
    let f2 = wl * l_ds2 + wm * m_ds2 + ws * s_ds2;

    s = s - f * f1 / (f1 * f1 - 0.5 * f * f2);
    s
}

#[derive(Clone, Copy)]
struct Cusp { l: f32, c: f32 }

fn find_cusp(a: f32, b: f32) -> Cusp {
    let s_cusp = compute_max_saturation(a, b);
    let rgb = linear_from_oklab(1.0, s_cusp * a, s_cusp * b);
    let max_rgb = rgb[0].max(rgb[1]).max(rgb[2]).max(1e-9);
    let l_cusp = (1.0 / max_rgb).cbrt();
    Cusp { l: l_cusp, c: l_cusp * s_cusp }
}

/// Intersection along the line `L = L0*(1−t) + t*L1`, `C = t*C1`.
fn find_gamut_intersection(a: f32, b: f32, l1: f32, c1: f32, l0: f32, cusp: Cusp) -> f32 {
    let lower_half = (l1 - l0) * cusp.c - (cusp.l - l0) * c1 <= 0.0;
    if lower_half {
        cusp.c * l0 / (c1 * cusp.l + cusp.c * (l0 - l1))
    } else {
        let mut t = cusp.c * (l0 - 1.0) / (c1 * (cusp.l - 1.0) + cusp.c * (l0 - l1));
        // One Halley iteration in OKLab → linear sRGB
        let dl = l1 - l0;
        let dc = c1;
        let k_l =  0.3963377774 * a + 0.2158037573 * b;
        let k_m = -0.1055613458 * a - 0.0638541728 * b;
        let k_s = -0.0894841775 * a - 1.2914855480 * b;

        let l_dt = dl + dc * k_l;
        let m_dt = dl + dc * k_m;
        let s_dt = dl + dc * k_s;

        let big_l = l0 * (1.0 - t) + t * l1;
        let big_c = t * c1;
        let l_ = big_l + big_c * k_l;
        let m_ = big_l + big_c * k_m;
        let s_ = big_l + big_c * k_s;

        let l = l_ * l_ * l_;
        let m = m_ * m_ * m_;
        let s = s_ * s_ * s_;

        let ldt = 3.0 * l_dt * l_ * l_;
        let mdt = 3.0 * m_dt * m_ * m_;
        let sdt = 3.0 * s_dt * s_ * s_;
        let ldt2 = 6.0 * l_dt * l_dt * l_;
        let mdt2 = 6.0 * m_dt * m_dt * m_;
        let sdt2 = 6.0 * s_dt * s_dt * s_;

        // Each channel: r = wlr*l + wmr*m + wsr*s. Compute t adjustment
        // for whichever channel currently exceeds 1 by the most.
        let channels: [(f32, f32, f32); 3] = [
            ( 4.0767416621, -3.3077115913,  0.2309699292),
            (-1.2684380046,  2.6097574011, -0.3413193965),
            (-0.0041960863, -0.7034186147,  1.7076147010),
        ];
        let mut best_t = t;
        let mut best_excess = -1.0f32;
        for (wr, wg, wb) in channels {
            let f  = wr * l   + wg * m   + wb * s   - 1.0;
            let f1 = wr * ldt + wg * mdt + wb * sdt;
            let f2 = wr * ldt2 + wg * mdt2 + wb * sdt2;
            if f.abs() > best_excess {
                best_excess = f.abs();
                best_t = t - f * f1 / (f1 * f1 - 0.5 * f * f2);
            }
        }
        t = best_t;
        t
    }
}
