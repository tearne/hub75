//! Blink the onboard WS2812 LED on the Pimoroni Interstate 75 W (RP2350A).
//!
//! The RP2350 version has a WS2812 (NeoPixel) on GPIO 18, driven via PIO.

#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal as hal;
use hal::gpio::FunctionPio0;
use hal::pio::PIOExt;

/// Tell the Boot ROM about our application.
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External crystal frequency (12 MHz on the Interstate 75 W).
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO 18 for PIO0
    let ws2812_pin = pins.gpio18.into_function::<FunctionPio0>();
    let ws2812_pin_id = ws2812_pin.id().num;

    // Build the WS2812 PIO program (from ws2812-pio-rs)
    let side_set = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::new_with_side_set(side_set);

    const T1: u8 = 2; // start bit
    const T2: u8 = 5; // data bit
    const T3: u8 = 3; // stop bit
    const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;

    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut do_zero = a.label();

    a.bind(&mut wrap_target);
    a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
    a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
    a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
    a.bind(&mut do_zero);
    a.nop_with_delay_and_side_set(T2 - 1, 0);
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    // Install and configure PIO
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio0.install(&program).unwrap();

    // Clock divider: system_clock / (800kHz * cycles_per_bit)
    use hal::Clock;
    let clock_freq = clocks.system_clock.freq().to_Hz();
    let bit_freq = 800_000u32 * CYCLES_PER_BIT;
    let int = clock_freq / bit_freq;
    let rem = clock_freq - (int * bit_freq);
    let frac = (rem * 256) / bit_freq;

    let (mut sm, _, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(hal::pio::Buffers::OnlyTx)
        .side_set_pin_base(ws2812_pin_id)
        .out_shift_direction(hal::pio::ShiftDirection::Left)
        .autopull(true)
        .pull_threshold(24)
        .clock_divisor_fixed_point(int as u16, frac as u8)
        .build(sm0);

    sm.set_pindirs([(ws2812_pin_id, hal::pio::PinDir::Output)]);
    sm.start();

    // Helper: send one RGB pixel (GRB order, MSB first, in top 24 bits)
    let mut write_rgb = |r: u8, g: u8, b: u8| {
        let word = (u32::from(g) << 24) | (u32::from(r) << 16) | (u32::from(b) << 8);
        while !tx.write(word) {
            cortex_m::asm::nop();
        }
    };

    // Cycle R -> G -> B
    loop {
        defmt::info!("Red");
        write_rgb(255, 0, 0);
        timer.delay_ms(3000);

        defmt::info!("Green");
        write_rgb(0, 255, 0);
        timer.delay_ms(3000);

        defmt::info!("Blue");
        write_rgb(0, 0, 255);
        timer.delay_ms(3000);
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Interstate 75 W LED blink"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
