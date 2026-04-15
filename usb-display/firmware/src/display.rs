//! HUB75 display types for DMA-driven autonomous scanning.
//!
//! Contains pixel types, gamma correction, bitplane packing, the USB frame
//! protocol receiver, and BCM timing generation.

pub const WIDTH: usize = 64;
pub const HEIGHT: usize = 64;
pub const ROW_PAIRS: usize = HEIGHT / 2;
pub const COLOR_DEPTH: usize = 8;
pub const WORDS_PER_ROW: usize = WIDTH / 4;

pub const OE: u32 = 1 << 13;

// ── Pixel type ──────────────────────────────────────────────────────

#[derive(Copy, Clone, Default)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const BLACK: Rgb = Rgb::new(0, 0, 0);
}

// ── Gamma correction ────────────────────────────────────────────────

static GAMMA_LUT: [u8; 256] = {
    let mut lut = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        let isqrt = {
            let mut x = i;
            let mut y = (x + 1) / 2;
            while y < x { x = y; y = (x + i / x) / 2; }
            x
        };
        let v = (i * isqrt + 8) / 16;
        lut[i] = if v > 255 { 255 } else { v as u8 };
        i += 1;
    }
    lut
};

fn gamma(color: Rgb) -> Rgb {
    Rgb::new(
        GAMMA_LUT[color.r as usize],
        GAMMA_LUT[color.g as usize],
        GAMMA_LUT[color.b as usize],
    )
}

// ── DMA buffer ──────────────────────────────────────────────────────

const WORDS_PER_BITPLANE: usize = ROW_PAIRS * WORDS_PER_ROW;
const PIXEL_DATA_WORDS: usize = COLOR_DEPTH * WORDS_PER_BITPLANE;

pub const DMA_BUF_WORDS: usize = PIXEL_DATA_WORDS;
pub const TIMING_BUF_WORDS: usize = COLOR_DEPTH * 2;

/// Contiguous buffer for DMA-driven autonomous scanning.
///
/// Layout: `[bitplane 0: row0..row31] [bitplane 1: row0..row31] ... [bitplane 7]`
///
/// Packed bitplane data laid out contiguously so DMA can stream the entire
/// buffer without CPU intervention.
pub struct DmaBuffer {
    pub pixels: [u32; DMA_BUF_WORDS],
}

impl DmaBuffer {
    pub const fn new() -> Self {
        Self { pixels: [0u32; DMA_BUF_WORDS] }
    }

    /// Pack RGB pixel data (with gamma correction) into the DMA buffer.
    pub fn load_from_rgb(&mut self, src: &[[Rgb; WIDTH]; HEIGHT]) {
        for bit in 0..COLOR_DEPTH {
            for row in 0..ROW_PAIRS {
                for word_idx in 0..WORDS_PER_ROW {
                    let mut word: u32 = 0;
                    for pix in 0..4 {
                        let col = word_idx * 4 + pix;
                        let upper = gamma(src[row][col]);
                        let lower = gamma(src[row + ROW_PAIRS][col]);

                        let r0 = ((upper.r >> bit) & 1) as u32;
                        let g0 = ((upper.g >> bit) & 1) as u32;
                        let b0 = ((upper.b >> bit) & 1) as u32;
                        let r1 = ((lower.r >> bit) & 1) as u32;
                        let g1 = ((lower.g >> bit) & 1) as u32;
                        let b1 = ((lower.b >> bit) & 1) as u32;

                        let pixel_data = r0 | (g0 << 1) | (b0 << 2)
                            | (r1 << 3) | (g1 << 4) | (b1 << 5);
                        word |= pixel_data << (pix * 8);
                    }
                    let idx = bit * WORDS_PER_BITPLANE + row * WORDS_PER_ROW + word_idx;
                    self.pixels[idx] = word;
                }
            }
        }
    }

    pub fn as_ptr(&self) -> *const u32 {
        self.pixels.as_ptr()
    }
}

// ── USB frame protocol ──────────────────────────────────────────────

const FRAME_MAGIC: [u8; 4] = *b"HB75";
const FRAME_PIXEL_BYTES: usize = WIDTH * HEIGHT * 3;

/// Pixel receive buffer. FrameReceiver writes incoming USB data here.
pub struct ReceiveBuffer {
    pub pixels: [[Rgb; WIDTH]; HEIGHT],
}

impl ReceiveBuffer {
    pub const fn new() -> Self {
        Self { pixels: [[Rgb::BLACK; WIDTH]; HEIGHT] }
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
                        if self.drop_events.count_ones() == 1 { // log at 1, 2, 4, 8, 16...
                            defmt::warn!(
                                "dropped {} frames ({} total in {} events)",
                                dropped, self.dropped_total, self.drop_events
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

// ── BCM timing ──────────────────────────────────────────────────────

/// Generate BCM timing values for the Address SM.
///
/// Returns [off, on, off, on, ...] cycle counts, two per bitplane.
/// `base_cycles` is the display time for bit 0; each subsequent bit doubles.
/// `brightness` is 0–255 (255 = full brightness).
pub fn generate_timing_buffer(base_cycles: u32, brightness: u8) -> [u32; TIMING_BUF_WORDS] {
    let mut buf = [0u32; TIMING_BUF_WORDS];
    for bit in 0..COLOR_DEPTH {
        let full_on = base_cycles << bit;
        let on_cycles = (full_on * brightness as u32) / 255;
        let off_cycles = (full_on - on_cycles) / 2 + 10;
        buf[bit * 2] = off_cycles;
        buf[bit * 2 + 1] = on_cycles;
    }
    buf
}

/// Estimate display refresh rate in Hz.
pub fn estimate_refresh_rate(
    pio_clock_hz: u32,
    pixels_per_row: u32,
    pio_cycles_per_pixel: u32,
    base_cycles: u32,
    brightness: u8,
) -> u32 {
    let data_cycles = pixels_per_row * pio_cycles_per_pixel;
    let mut total: u32 = 0;
    for bit in 0..COLOR_DEPTH {
        let full_on = base_cycles << bit;
        let on_cycles = (full_on * brightness as u32) / 255;
        let off_cycles = (full_on - on_cycles) / 2 + 10;
        let addr_cycles = off_cycles + on_cycles + off_cycles;
        total += (data_cycles + addr_cycles) * ROW_PAIRS as u32;
    }
    if total == 0 { 0 } else { pio_clock_hz / total }
}
