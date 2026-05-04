//! 64×64 shift-register HUB75 panel driver, sync `rp235x-hal`.
//!
//! Vendored from `learning-examples/shift_3_dual_pio.rs`. Once
//! [`Panel::start`] is called the panel scans autonomously via two
//! PIO state machines and four chained DMA channels — no further
//! CPU involvement. [`Panel::set_pixels`] swaps the displayed frame
//! by packing the new RGB grid into the inactive bitplane buffer
//! and pointing the data DMA chain at it.

use rp235x_hal as hal;
use hal::pio::PIOExt;

pub const WIDTH: usize = 64;
pub const HEIGHT: usize = 64;
const ROW_PAIRS: usize = HEIGHT / 2;
const COLOR_DEPTH: usize = 8;
const WORDS_PER_ROW: usize = WIDTH / 4;
const WORDS_PER_BITPLANE: usize = ROW_PAIRS * WORDS_PER_ROW;
const PIXEL_DATA_WORDS: usize = COLOR_DEPTH * WORDS_PER_BITPLANE;
const TIMING_BUF_WORDS: usize = COLOR_DEPTH * 2;

#[derive(Copy, Clone)]
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

static GAMMA_LUT: [u8; 256] = {
    let mut lut = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        let isqrt = {
            let mut x = i;
            let mut y = (x + 1) / 2;
            while y < x {
                x = y;
                y = (x + i / x) / 2;
            }
            x
        };
        let v = (i * isqrt + 8) / 16;
        lut[i] = if v > 255 { 255 } else { v as u8 };
        i += 1;
    }
    lut
};

fn gamma(c: Rgb) -> Rgb {
    Rgb::new(
        GAMMA_LUT[c.r as usize],
        GAMMA_LUT[c.g as usize],
        GAMMA_LUT[c.b as usize],
    )
}

#[repr(C, align(4))]
struct DmaBuffer {
    pixels: [u32; PIXEL_DATA_WORDS],
}

impl DmaBuffer {
    const fn new() -> Self {
        Self {
            pixels: [0u32; PIXEL_DATA_WORDS],
        }
    }
    fn as_ptr(&self) -> *const u32 {
        self.pixels.as_ptr()
    }

    fn pack(&mut self, src: &[[Rgb; WIDTH]; HEIGHT]) {
        for bit in 0..COLOR_DEPTH {
            for row in 0..ROW_PAIRS {
                for word_idx in 0..WORDS_PER_ROW {
                    let mut word: u32 = 0;
                    for pix in 0..4 {
                        let col = word_idx * 4 + pix;
                        let upper = gamma(src[row][col]);
                        let lower = gamma(src[row + ROW_PAIRS][col]);
                        let bits = ((upper.r >> bit) & 1) as u32
                            | (((upper.g >> bit) & 1) as u32) << 1
                            | (((upper.b >> bit) & 1) as u32) << 2
                            | (((lower.r >> bit) & 1) as u32) << 3
                            | (((lower.g >> bit) & 1) as u32) << 4
                            | (((lower.b >> bit) & 1) as u32) << 5;
                        word |= bits << (pix * 8);
                    }
                    self.pixels[bit * WORDS_PER_BITPLANE + row * WORDS_PER_ROW + word_idx] = word;
                }
            }
        }
    }
}

fn generate_timing(base: u32) -> [u32; TIMING_BUF_WORDS] {
    let mut buf = [0u32; TIMING_BUF_WORDS];
    for bit in 0..COLOR_DEPTH {
        buf[bit * 2] = 10;
        buf[bit * 2 + 1] = base << bit;
    }
    buf
}

static mut BUF_A: DmaBuffer = DmaBuffer::new();
static mut BUF_B: DmaBuffer = DmaBuffer::new();
static mut ACTIVE_BUF_PTR: u32 = 0;
static mut TIMING_BUF: [u32; TIMING_BUF_WORDS] = [0u32; TIMING_BUF_WORDS];
static mut TIMING_BUF_PTR: u32 = 0;

const DMA_BASE: u32 = 0x5000_0000;
const PIO0_TXF0: u32 = 0x5020_0010;
const PIO0_TXF1: u32 = 0x5020_0014;

fn dma_ch_base(n: u32) -> u32 {
    DMA_BASE + n * 0x40
}

fn dma_write(ch: u32, offset: u32, val: u32) {
    unsafe { ((dma_ch_base(ch) + offset) as *mut u32).write_volatile(val) };
}
fn dma_read(ch: u32, offset: u32) -> u32 {
    unsafe { ((dma_ch_base(ch) + offset) as *const u32).read_volatile() }
}

const READ_ADDR: u32 = 0x00;
const WRITE_ADDR: u32 = 0x04;
const TRANS_COUNT: u32 = 0x08;
const CTRL_TRIG: u32 = 0x0C;
const AL3_READ_ADDR_TRIG: u32 = 0x3C;

fn ctrl(en: bool, inc_read: bool, chain_to: u32, treq: u32) -> u32 {
    (en as u32)
        | (2 << 2)
        | ((inc_read as u32) << 4)
        | ((chain_to & 0xF) << 13)
        | ((treq & 0x3F) << 17)
        | (1 << 23)
}

fn install_data_sm(
    pio: &mut hal::pio::PIO<hal::pac::PIO0>,
    sm: hal::pio::UninitStateMachine<(hal::pac::PIO0, hal::pio::SM0)>,
) {
    let ss = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(ss);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut pixel_loop = a.label();

    a.out_with_side_set(pio::OutDestination::Y, 32, 0b00);
    a.bind(&mut wrap_target);
    a.mov_with_side_set(
        pio::MovDestination::X,
        pio::MovOperation::None,
        pio::MovSource::Y,
        0b00,
    );
    a.bind(&mut pixel_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 6, 0b00);
    a.out_with_side_set(pio::OutDestination::NULL, 2, 0b01);
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut pixel_loop, 0b00);
    a.wait_with_side_set(1, pio::WaitSource::IRQ, 0, false, 0b00);
    a.irq_with_side_set(false, false, 1, false, 0b10);
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, wrap_target);
    let installed = pio.install(&prog).unwrap();

    let (mut sm, _, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(hal::pio::Buffers::OnlyTx)
        .out_pins(0, 6)
        .side_set_pin_base(11)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(32)
        .clock_divisor_fixed_point(4, 0)
        .build(sm);

    sm.set_pindirs([
        (0, hal::pio::PinDir::Output),
        (1, hal::pio::PinDir::Output),
        (2, hal::pio::PinDir::Output),
        (3, hal::pio::PinDir::Output),
        (4, hal::pio::PinDir::Output),
        (5, hal::pio::PinDir::Output),
        (11, hal::pio::PinDir::Output),
        (12, hal::pio::PinDir::Output),
    ]);

    tx.write((WIDTH - 1) as u32);
    let _sm = sm.start();
    core::mem::forget(tx);
}

fn install_address_sm(
    pio: &mut hal::pio::PIO<hal::pac::PIO0>,
    sm: hal::pio::UninitStateMachine<(hal::pac::PIO0, hal::pio::SM1)>,
) {
    let ss = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::new_with_side_set(ss);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut next_row = a.label();
    let mut init = a.label();
    let mut off1 = a.label();
    let mut on = a.label();
    let mut off2 = a.label();

    a.jmp_with_side_set(pio::JmpCondition::Always, &mut init, 1);
    a.bind(&mut wrap_target);
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut next_row, 1);
    a.out_with_side_set(pio::OutDestination::NULL, 32, 1);
    a.bind(&mut init);
    a.out_with_side_set(pio::OutDestination::ISR, 32, 1);
    a.set_with_side_set(pio::SetDestination::X, 31, 1);
    a.bind(&mut next_row);
    a.irq_with_side_set(false, false, 0, false, 1);
    a.mov_with_side_set(
        pio::MovDestination::PINS,
        pio::MovOperation::Invert,
        pio::MovSource::X,
        1,
    );
    a.wait_with_side_set(1, pio::WaitSource::IRQ, 1, false, 1);
    a.mov_with_side_set(
        pio::MovDestination::Y,
        pio::MovOperation::None,
        pio::MovSource::ISR,
        1,
    );
    a.bind(&mut off1);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut off1, 1);
    a.mov_with_side_set(
        pio::MovDestination::Y,
        pio::MovOperation::None,
        pio::MovSource::OSR,
        1,
    );
    a.bind(&mut on);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut on, 0);
    a.mov_with_side_set(
        pio::MovDestination::Y,
        pio::MovOperation::None,
        pio::MovSource::ISR,
        1,
    );
    a.bind(&mut off2);
    a.jmp_with_side_set(pio::JmpCondition::YDecNonZero, &mut off2, 1);
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, wrap_target);
    let installed = pio.install(&prog).unwrap();

    let (mut sm, _, _) = hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(hal::pio::Buffers::OnlyTx)
        .out_pins(6, 5)
        .set_pins(6, 5)
        .side_set_pin_base(13)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(32)
        .clock_divisor_fixed_point(4, 0)
        .build(sm);

    sm.set_pindirs([
        (6, hal::pio::PinDir::Output),
        (7, hal::pio::PinDir::Output),
        (8, hal::pio::PinDir::Output),
        (9, hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output),
        (13, hal::pio::PinDir::Output),
    ]);
    let _sm = sm.start();
}

fn setup_dma_chains() {
    unsafe {
        dma_write(0, READ_ADDR, BUF_A.as_ptr() as u32);
        dma_write(0, WRITE_ADDR, PIO0_TXF0);
        dma_write(0, TRANS_COUNT, PIXEL_DATA_WORDS as u32);
        dma_write(0, CTRL_TRIG, ctrl(false, true, 1, 0));

        dma_write(1, READ_ADDR, core::ptr::addr_of!(ACTIVE_BUF_PTR) as u32);
        dma_write(1, WRITE_ADDR, dma_ch_base(0) + AL3_READ_ADDR_TRIG);
        dma_write(1, TRANS_COUNT, 1);
        dma_write(1, CTRL_TRIG, ctrl(true, false, 1, 0x3F));

        dma_write(2, READ_ADDR, core::ptr::addr_of!(TIMING_BUF) as u32);
        dma_write(2, WRITE_ADDR, PIO0_TXF1);
        dma_write(2, TRANS_COUNT, TIMING_BUF_WORDS as u32);
        dma_write(2, CTRL_TRIG, ctrl(false, true, 3, 1));

        dma_write(3, READ_ADDR, core::ptr::addr_of!(TIMING_BUF_PTR) as u32);
        dma_write(3, WRITE_ADDR, dma_ch_base(2) + AL3_READ_ADDR_TRIG);
        dma_write(3, TRANS_COUNT, 1);
        dma_write(3, CTRL_TRIG, ctrl(true, false, 3, 0x3F));
    }
}

fn start_dma() {
    dma_write(0, CTRL_TRIG, dma_read(0, CTRL_TRIG) | 1);
    dma_write(2, CTRL_TRIG, dma_read(2, CTRL_TRIG) | 1);
}

/// Initialise PIO + DMA scan engine. Call once at boot. After this
/// returns the panel scans autonomously; use [`set_pixels`] to update
/// what's shown.
///
/// `write_idx` rotates between BUF_A and BUF_B so we always pack into
/// the buffer DMA isn't currently reading.
pub struct Panel {
    write_idx: u8,
}

impl Panel {
    pub fn init(
        pio0: hal::pac::PIO0,
        resets: &mut hal::pac::RESETS,
        io_bank0: hal::pac::IO_BANK0,
        pads_bank0: hal::pac::PADS_BANK0,
        sio_gpio_bank0: hal::sio::SioGpioBank0,
        dma: hal::pac::DMA,
    ) -> Self {
        // DMA peripheral out of reset. We discard the split channels —
        // we drive DMA via raw register writes for the chained-restart
        // pattern the panel needs.
        use hal::dma::DMAExt;
        let _dma = dma.split(resets);

        // Assign GPIO 0..13 to PIO0.
        let pins = hal::gpio::Pins::new(io_bank0, pads_bank0, sio_gpio_bank0, resets);
        let _pins = (
            pins.gpio0.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio1.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio2.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio3.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio4.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio5.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio6.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio7.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio8.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio9.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio10.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio11.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio12.into_function::<hal::gpio::FunctionPio0>(),
            pins.gpio13.into_function::<hal::gpio::FunctionPio0>(),
        );

        unsafe {
            TIMING_BUF = generate_timing(25);
            TIMING_BUF_PTR = core::ptr::addr_of!(TIMING_BUF) as u32;
            ACTIVE_BUF_PTR = BUF_A.as_ptr() as u32;
        }

        let (mut pio, sm0, sm1, _, _) = pio0.split(resets);
        install_data_sm(&mut pio, sm0);
        install_address_sm(&mut pio, sm1);
        setup_dma_chains();
        start_dma();

        Self { write_idx: 1 }
    }

    /// Pack `pixels` into the inactive bitplane buffer and swap to it.
    /// The DMA chain reads the new pointer on its next cycle (~7 ms),
    /// so the visible frame updates well within human-perceptual time.
    pub fn set_pixels(&mut self, pixels: &[[Rgb; WIDTH]; HEIGHT]) {
        let buf = unsafe {
            if self.write_idx == 0 {
                &mut *core::ptr::addr_of_mut!(BUF_A)
            } else {
                &mut *core::ptr::addr_of_mut!(BUF_B)
            }
        };
        buf.pack(pixels);
        unsafe {
            ACTIVE_BUF_PTR = buf.as_ptr() as u32;
        }
        self.write_idx ^= 1;
    }
}
