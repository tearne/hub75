//! Display output: rotate the 32×64 portrait canvas to the panel's
//! 64×32 native landscape frame and emit RGB bytes over hub75-client.

use hub75_client::{HEIGHT as PANEL_H, WIDTH as PANEL_W};

use crate::presentation::{LOGICAL_HEIGHT, LOGICAL_WIDTH};
use crate::slate::Pixel;

/// Apply screen-burn shift and 90° clockwise rotation in a single
/// pass. Logical pixel `(lx, ly)` from `canvas`, after shifting `lx`
/// by `shift` columns (with wrap), is written to native panel pixel
/// `(W - 1 - ly, lx_shifted)`. One pass over the canvas, no
/// intermediate buffer.
pub fn shift_and_rotate(canvas: &[Pixel], frame: &mut [Pixel], shift: usize) {
    debug_assert_eq!(canvas.len(), LOGICAL_WIDTH * LOGICAL_HEIGHT);
    debug_assert_eq!(frame.len(), PANEL_W * PANEL_H);
    debug_assert_eq!(PANEL_W, LOGICAL_HEIGHT);
    debug_assert_eq!(PANEL_H, LOGICAL_WIDTH);
    let s = shift % LOGICAL_WIDTH;
    for ly in 0..LOGICAL_HEIGHT {
        let px = PANEL_W - 1 - ly;
        let row_start = ly * LOGICAL_WIDTH;
        for lx in 0..LOGICAL_WIDTH {
            let lx_shifted = (lx + s) % LOGICAL_WIDTH;
            frame[lx_shifted * PANEL_W + px] = canvas[row_start + lx];
        }
    }
}
