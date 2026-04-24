//! # DP3364S S-PWM — Step 31: PIO zero-fill probe
//!
//! ## Purpose
//!
//! spwm_30 (2026-04-24) localized the echo mechanism to PINCTRL.OUT_COUNT:
//! setting OUT_COUNT=6 during data phase kills the echo; OUT_COUNT=11
//! allows it. The mechanism hypothesis is that RP2350 PIO's `out PINS, N`
//! with N < OUT_COUNT zero-fills pins OUT_BASE+N..OUT_BASE+OUT_COUNT-1.
//! This file tests that hypothesis directly, at the PIO level, with no
//! panel interaction.
//!
//! ## Experiment
//!
//! A minimal PIO program:
//!   1. Pull a test value from TX FIFO into OSR
//!   2. SET pins 6-10 to 0b11111 (ADDR pins driven HIGH)
//!   3. OUT PINS, 6 (write 6 bits of OSR to pins 0-5)
//!   4. IN PINS, 11 (read pins 0-10 back into ISR)
//!   5. Push ISR onto RX FIFO
//!   6. Repeat
//!
//! The CPU sends test values and reads back the observed pin state after
//! the OUT. Varying PINCTRL.OUT_COUNT at runtime (6 vs 11) reveals
//! whether the extra pins 6-10 are zero-filled or retained.
//!
//! ## Expected outcomes
//!
//! For test value 0x2A (0b101010) with pins 0-5 and pins 6-10 driven
//! HIGH by SET before the OUT:
//!   - OUT_COUNT=6  → pins 6-10 not in OUT group, retain SET (0b11111).
//!     IN reads (0b11111 << 6) | 0b101010 = 0x7EA.
//!   - OUT_COUNT=11 → if zero-fill: pins 6-10 get 0. IN reads 0x02A.
//!   - OUT_COUNT=11 → if retain: pins 6-10 stay 0b11111. IN reads 0x7EA.
//!
//! 0x7EA vs 0x02A at OUT_COUNT=11 decides whether zero-fill is the
//! mechanism or whether OUT_COUNT has some other effect.
//!
//! ```sh
//! cargo run --release --example spwm_31_zerofill_probe
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;
use hal::gpio::FunctionPio0;
use hal::pio::PIOExt;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Direct PINCTRL register access so we can change OUT_COUNT at runtime.
const PIO0_BASE: u32 = 0x5020_0000;
const SM0_PINCTRL: u32 = PIO0_BASE + 0x0DC;

fn set_out_count(count: u32) {
    unsafe {
        let current = (SM0_PINCTRL as *const u32).read_volatile();
        let mask = !(0x3F << 20);
        let new = (current & mask) | ((count & 0x3F) << 20);
        (SM0_PINCTRL as *mut u32).write_volatile(new);
    }
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS,
    );

    // Configure GPIO 0-10 for PIO0 (matches HUB75 RGB 0-5 + ADDR 6-10).
    let _p0 = pins.gpio0.into_function::<FunctionPio0>();
    let _p1 = pins.gpio1.into_function::<FunctionPio0>();
    let _p2 = pins.gpio2.into_function::<FunctionPio0>();
    let _p3 = pins.gpio3.into_function::<FunctionPio0>();
    let _p4 = pins.gpio4.into_function::<FunctionPio0>();
    let _p5 = pins.gpio5.into_function::<FunctionPio0>();
    let _p6 = pins.gpio6.into_function::<FunctionPio0>();
    let _p7 = pins.gpio7.into_function::<FunctionPio0>();
    let _p8 = pins.gpio8.into_function::<FunctionPio0>();
    let _p9 = pins.gpio9.into_function::<FunctionPio0>();
    let _p10 = pins.gpio10.into_function::<FunctionPio0>();

    // PIO program: pull → set ADDR high → out to RGB → in → push.
    // No side-set. Use a zero-count SideSet so delay field is 5 bits.
    let ss = pio::SideSet::new(false, 0, false);
    let mut a = pio::Assembler::new_with_side_set(ss);

    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    a.bind(&mut wrap_target);
    a.pull(false, true);                              // pull block
    a.set(pio::SetDestination::PINS, 0x1F);           // pins 6-10 ← 0b11111
    a.out(pio::OutDestination::PINS, 6);              // write OSR[5:0] → pins 0-5
    a.r#in(pio::InSource::PINS, 11);                  // read pins 0-10 → ISR
    a.push(false, true);                              // push block
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, wrap_target);

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio0.install(&prog).unwrap();

    let (mut sm, mut rx, mut tx) =
        hal::pio::PIOBuilder::from_installed_program(installed)
            .buffers(hal::pio::Buffers::RxTx)
            .out_pins(0, 6)          // OUT_BASE=0, OUT_COUNT=6 (we'll vary at runtime)
            .set_pins(6, 5)          // SET_BASE=6, SET_COUNT=5 (ADDR pins)
            .in_pin_base(0)          // IN_BASE=0 (read from pin 0 upward)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .autopull(false)
            .autopush(false)
            .pull_threshold(32)
            .push_threshold(32)
            .clock_divisor_fixed_point(1, 0)
            .build(sm0);

    sm.set_pindirs([
        (0,  hal::pio::PinDir::Output), (1,  hal::pio::PinDir::Output),
        (2,  hal::pio::PinDir::Output), (3,  hal::pio::PinDir::Output),
        (4,  hal::pio::PinDir::Output), (5,  hal::pio::PinDir::Output),
        (6,  hal::pio::PinDir::Output), (7,  hal::pio::PinDir::Output),
        (8,  hal::pio::PinDir::Output), (9,  hal::pio::PinDir::Output),
        (10, hal::pio::PinDir::Output),
    ]);
    let _sm = sm.start();

    defmt::info!("spwm_31 zero-fill probe starting");
    defmt::info!("PIO: pull → SET pins 6-10 = 0x1F → OUT PINS 6 → IN PINS 11 → push");

    let tests: [u32; 4] = [0x2A, 0x15, 0x3F, 0x00];
    let out_counts: [u32; 2] = [6, 11];

    for &oc in &out_counts {
        set_out_count(oc);
        let actual_pinctrl = unsafe { (SM0_PINCTRL as *const u32).read_volatile() };
        let actual_oc = (actual_pinctrl >> 20) & 0x3F;
        defmt::info!(
            "── OUT_COUNT={=u32} (PINCTRL=0x{=u32:08x}, field read-back={=u32}) ──",
            oc, actual_pinctrl, actual_oc,
        );

        for &v in &tests {
            while !tx.write(v) { cortex_m::asm::nop(); }
            let result = loop {
                if let Some(r) = rx.read() { break r; }
                cortex_m::asm::nop();
            };
            let pins_low_11 = result & 0x7FF;
            let pin_0_5 = pins_low_11 & 0x3F;
            let pin_6_10 = (pins_low_11 >> 6) & 0x1F;
            defmt::info!(
                "  tx=0x{=u32:02x}  rx raw=0x{=u32:08x}  pins[10:0]=0b{=u32:011b}  RGB=0x{=u32:02x} ADDR=0b{=u32:05b}",
                v, result, pins_low_11, pin_0_5, pin_6_10,
            );
        }
    }

    defmt::info!("spwm_31 probe done");
    loop { cortex_m::asm::wfi(); }
}

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 3] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"spwm_31 PIO zero-fill probe"),
];
