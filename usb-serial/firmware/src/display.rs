//! USB frame protocol receiver for the host → firmware pixel stream.
//!
//! Pixel pack, gamma, and DMA setup all live in `hub75`. This module
//! only owns the binary frame protocol that USB CDC carries.

use hub75::Rgb;

#[cfg(not(any(
    feature = "panel-shift-64x64",
    feature = "panel-shift-64x32",
    feature = "panel-spwm-128x64",
)))]
compile_error!(
    "Select a panel: --features panel-shift-64x64 (or panel-shift-64x32 or panel-spwm-128x64)"
);

#[cfg(any(
    all(feature = "panel-shift-64x64", feature = "panel-shift-64x32"),
    all(feature = "panel-shift-64x64", feature = "panel-spwm-128x64"),
    all(feature = "panel-shift-64x32", feature = "panel-spwm-128x64"),
))]
compile_error!("Only one panel feature may be enabled");

#[cfg(feature = "panel-shift-64x64")]
pub const WIDTH: usize = 64;
#[cfg(feature = "panel-shift-64x64")]
pub const HEIGHT: usize = 64;

#[cfg(feature = "panel-shift-64x32")]
pub const WIDTH: usize = 64;
#[cfg(feature = "panel-shift-64x32")]
pub const HEIGHT: usize = 32;

#[cfg(feature = "panel-spwm-128x64")]
pub const WIDTH: usize = 128;
#[cfg(feature = "panel-spwm-128x64")]
pub const HEIGHT: usize = 64;

const FRAME_MAGIC: [u8; 4] = *b"HB75";
const FRAME_PIXEL_BYTES: usize = WIDTH * HEIGHT * 3;

/// Pixel receive buffer. `FrameReceiver` writes incoming USB data here.
pub struct ReceiveBuffer {
    pub pixels: [[Rgb; WIDTH]; HEIGHT],
}

impl ReceiveBuffer {
    pub const fn new() -> Self {
        Self {
            pixels: [[Rgb::BLACK; WIDTH]; HEIGHT],
        }
    }
}

enum RxState {
    SyncMagic { pos: usize },
    ReadSeq,
    ReadPixels { offset: usize },
}

/// Receiver state machine for the binary frame protocol.
pub struct FrameReceiver {
    state: RxState,
    expected_seq: u8,
    dropped_total: u32,
    drop_events: u32,
}

impl FrameReceiver {
    pub fn new() -> Self {
        Self {
            state: RxState::SyncMagic { pos: 0 },
            expected_seq: 0,
            dropped_total: 0,
            drop_events: 0,
        }
    }

    /// Feed incoming bytes. Returns true when a complete frame has been
    /// written into the receive buffer.
    pub fn feed(&mut self, data: &[u8], buf: &mut ReceiveBuffer) -> bool {
        let mut got_frame = false;

        for &b in data {
            match &mut self.state {
                RxState::SyncMagic { pos } => {
                    if b == FRAME_MAGIC[*pos] {
                        *pos += 1;
                        if *pos == FRAME_MAGIC.len() {
                            self.state = RxState::ReadSeq;
                        }
                    } else if b == FRAME_MAGIC[0] {
                        self.state = RxState::SyncMagic { pos: 1 };
                    } else {
                        *pos = 0;
                    }
                }

                RxState::ReadSeq => {
                    if b != self.expected_seq {
                        let dropped = b.wrapping_sub(self.expected_seq);
                        self.dropped_total = self.dropped_total.saturating_add(dropped as u32);
                        self.drop_events += 1;
                        if self.drop_events.count_ones() == 1 {
                            defmt::warn!(
                                "dropped {} frames ({} total in {} events)",
                                dropped,
                                self.dropped_total,
                                self.drop_events
                            );
                        }
                    }
                    self.expected_seq = b.wrapping_add(1);
                    self.state = RxState::ReadPixels { offset: 0 };
                }

                RxState::ReadPixels { offset } => {
                    let pixel_idx = *offset / 3;
                    let channel = *offset % 3;
                    let x = pixel_idx % WIDTH;
                    let y = pixel_idx / WIDTH;

                    if pixel_idx < WIDTH * HEIGHT {
                        let px = &mut buf.pixels[y][x];
                        match channel {
                            0 => px.r = b,
                            1 => px.g = b,
                            _ => px.b = b,
                        }
                    }

                    *offset += 1;
                    if *offset >= FRAME_PIXEL_BYTES {
                        self.expected_seq = self.expected_seq.wrapping_add(1);
                        got_frame = true;
                        self.state = RxState::SyncMagic { pos: 0 };
                    }
                }
            }
        }

        got_frame
    }
}
