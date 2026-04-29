//! HUB75 LED matrix panel driver for the Pimoroni Interstate 75 W (RP2350A).
//!
//! Two panel families are supported:
//!
//! - [`Dp3364sPanel`] — DP3364S S-PWM column-driver chips with on-chip
//!   PWM (the "smart" family). Hardware-tied 64×128.
//! - [`shift::ShiftPanel`] — generic shift-register HUB75 panels with
//!   host-side BCM timing (the "regular" family). Const-generic over
//!   width and height (e.g. 64×64, 64×32).

#![no_std]

mod dp3364s;
mod panel;
pub mod shift;

pub use dp3364s::{DmaIrqHandler, Dp3364sPanel, Frame, InterstatePins};
pub use panel::{Panel, Rgb};
