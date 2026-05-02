//! Display output: rotate the 32×64 portrait canvas to the panel's
//! 64×32 native landscape frame and emit RGB bytes over hub75-client.

use hub75_client::{HEIGHT as PANEL_H, WIDTH as PANEL_W};

use crate::presentation::{LOGICAL_HEIGHT, LOGICAL_WIDTH};
use crate::slate::Pixel;

/// Logical pixel `(x, y)` in the portrait canvas maps to native pixel
/// `(W - 1 - y, x)` in the panel's landscape frame — a 90° clockwise
/// rotation that puts the user's top-of-view at the panel's right edge.
pub fn rotate_to_panel(canvas: &[Pixel]) -> Vec<Pixel> {
    debug_assert_eq!(canvas.len(), LOGICAL_WIDTH * LOGICAL_HEIGHT);
    debug_assert_eq!(PANEL_W, LOGICAL_HEIGHT);
    debug_assert_eq!(PANEL_H, LOGICAL_WIDTH);
    let mut out = vec![[0u8; 3]; PANEL_W * PANEL_H];
    for ly in 0..LOGICAL_HEIGHT {
        for lx in 0..LOGICAL_WIDTH {
            let px = PANEL_W - 1 - ly;
            let py = lx;
            out[py * PANEL_W + px] = canvas[ly * LOGICAL_WIDTH + lx];
        }
    }
    out
}
