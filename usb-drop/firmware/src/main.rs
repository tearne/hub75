//! USB mass-storage HUB75 display firmware.
//!
//! Mounts as a 256 KB FAT12 drive over USB. Drop a JSON `*.H75` file
//! onto the drive in your OS file manager and the firmware parses it
//! and renders the result on the panel. After each host-side write
//! the firmware waits a short settle delay, re-scans the root
//! directory, and re-renders if it finds a new or modified frame.
//!
//! Built on `rp235x-hal` + `usb-device` + `usbd-storage` (the embassy
//! USB stack lacks MSC support today; see the change document).
//! Modelled on `usbd-storage`'s rp2040 example, adapted for RP2350
//! and RAM-backed storage.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};
use critical_section::Mutex;
use rp235x_hal as hal;
use rp235x_hal::pac;
use rp235x_hal::rom_data::sys_info_api::chip_info;
use rp235x_hal::usb::UsbBus;
use static_cell::StaticCell;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_storage::subclass::scsi::{Scsi, ScsiCommand};
use usbd_storage::subclass::Command;
use usbd_storage::transport::bbb::{BulkOnly, BulkOnlyError};
use usbd_storage::transport::TransportError;

mod fat;
mod frame;
mod panel;

use panel::{Panel, Rgb, HEIGHT, WIDTH};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: rp235x_hal::block::ImageDef = rp235x_hal::block::ImageDef::secure_exe();

const BLOCK_SIZE: u32 = 512;
const DISK_LEN: usize = 262_144;
const BLOCKS: u32 = (DISK_LEN as u32) / BLOCK_SIZE;
const USB_PACKET_SIZE: u16 = 64;
const MAX_LUN: u8 = 0;

const XTAL_FREQ_HZ: u32 = 12_000_000;

const SETTLE_DELAY_US: u64 = 50_000;

/// Maximum supported file size for a `*.H75` frame. 96 KB is enough
/// for the 64×64 JSON worst case and leaves room on the drive.
const FILE_BUF_LEN: usize = 96 * 1024;

/// Initial-content disk image, generated at build time by build.rs:
/// FAT12-formatted 256 KB volume containing `README.TXT`.
const INITIAL_DISK: &[u8; DISK_LEN] = include_bytes!(concat!(env!("OUT_DIR"), "/disk.img"));

/// RAM-backed block-device storage. Initialised from `INITIAL_DISK`
/// at boot. Host reads + writes go straight here.
static mut DISK: [u8; DISK_LEN] = [0u8; DISK_LEN];

/// Per-command state for SCSI Read/Write (transferred in chunks).
static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State {
    storage_offset: 0,
    sense_key: None,
    sense_key_code: None,
    sense_qualifier: None,
}));

#[derive(Default)]
struct State {
    storage_offset: usize,
    sense_key: Option<u8>,
    sense_key_code: Option<u8>,
    sense_qualifier: Option<u8>,
}

impl State {
    fn reset(&mut self) {
        self.storage_offset = 0;
        self.sense_key = None;
        self.sense_key_code = None;
        self.sense_qualifier = None;
    }
}

static mut USB_TRANSPORT_BUF: MaybeUninit<[u8; BLOCK_SIZE as usize]> = MaybeUninit::uninit();

/// Set by the SCSI Write handler when a complete write transaction
/// finishes. The main loop consumes it to start (or extend) the
/// settle-delay timer before re-scanning the directory.
static WRITE_COMPLETED: AtomicBool = AtomicBool::new(false);

/// Working buffer for the most recently scanned file. Sized for a
/// 64×64 JSON frame worst case.
static mut FILE_BUF: [u8; FILE_BUF_LEN] = [0u8; FILE_BUF_LEN];

/// Decoded RGB frame, fed to the panel.
const BLACK_ROW: [Rgb; WIDTH] = [Rgb::BLACK; WIDTH];
static mut FRAME: [[Rgb; WIDTH]; HEIGHT] = [BLACK_ROW; HEIGHT];

/// Identity of a `.H75` file. Includes the dir entry's write-time/date
/// because the host updates those on every `cp` even when content is
/// unchanged — without it, "drop the same file twice" is invisible to
/// the firmware (cluster + size stay the same).
#[derive(Copy, Clone, PartialEq, Eq)]
struct FileId {
    start_cluster: u16,
    size: u32,
    write_time_date: u32,
}

/// Snapshot of the `.H75` files observed in the most recent scan.
/// Refreshed at the end of every scan; any `.H75` not in this set on
/// the next scan is "new" and triggers a render.
const MAX_KNOWN: usize = 16;
static mut KNOWN: [Option<FileId>; MAX_KNOWN] = [None; MAX_KNOWN];


/// USB serial-number string the firmware advertises.
///
/// Emits 8 lowercase hex chars from the RP2350's `device_id`, giving each
/// board a unique stable identifier — the OS uses this in mass-storage
/// mount paths (e.g. `/dev/disk/by-id/usb-tearne_hub75-drop_<id>-0:0`).
fn panel_serial_number() -> &'static str {
    static BUF: StaticCell<[u8; 8]> = StaticCell::new();
    let buf = BUF.init([0u8; 8]);
    let id = chip_info().ok().flatten().map(|info| info.device_id).unwrap_or(0);
    for i in 0..8 {
        let nibble = ((id >> ((7 - i) * 4)) & 0xF) as u8;
        buf[i] = if nibble < 10 { b'0' + nibble } else { b'a' + (nibble - 10) };
    }
    core::str::from_utf8(buf).unwrap()
}

#[rp235x_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
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

    defmt::info!("usb-drop firmware: starting");

    // Seed RAM disk with the build-time-generated FAT12+README image.
    unsafe {
        let dst = core::ptr::addr_of_mut!(DISK) as *mut u8;
        core::ptr::copy_nonoverlapping(INITIAL_DISK.as_ptr(), dst, DISK_LEN);
    }

    let sio = hal::Sio::new(pac.SIO);

    let mut panel = Panel::init(
        pac.PIO0,
        &mut pac.RESETS,
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        pac.DMA,
    );

    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut scsi = Scsi::new(
        &usb_bus,
        USB_PACKET_SIZE,
        MAX_LUN,
        unsafe {
            #[allow(static_mut_refs)]
            USB_TRANSPORT_BUF.assume_init_mut()
        }
        .as_mut_slice(),
    )
    .unwrap();

    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::new(LangID::EN)
            .manufacturer("tearne")
            .product("hub75-drop")
            .serial_number(panel_serial_number())])
        .unwrap()
        .self_powered(false)
        .build();

    defmt::info!("usb-drop firmware: enumerating");

    let mut settle_deadline: Option<u64> = None;

    loop {
        if usb_device.poll(&mut [&mut scsi]) {
            if matches!(usb_device.state(), UsbDeviceState::Default) {
                critical_section::with(|cs| STATE.borrow_ref_mut(cs).reset());
            }

            let _ = scsi.poll(|command| {
                if let Err(err) = process_command(command) {
                    defmt::error!("scsi: {}", err);
                }
            });
        }

        if WRITE_COMPLETED.swap(false, Ordering::AcqRel) {
            let now = timer.get_counter().ticks();
            settle_deadline = Some(now + SETTLE_DELAY_US);
        }

        if let Some(deadline) = settle_deadline {
            let now = timer.get_counter().ticks();
            if now >= deadline {
                settle_deadline = None;
                scan_and_render(&mut panel);
            }
        }
    }
}

fn scan_and_render(panel: &mut Panel) {
    let disk: &[u8] = unsafe {
        #[allow(static_mut_refs)]
        &DISK[..]
    };

    let bpb = match fat::Bpb::parse(disk) {
        Some(b) => b,
        None => {
            defmt::warn!("fat: BPB parse failed");
            return;
        }
    };

    let mut current: [Option<FileId>; MAX_KNOWN] = [None; MAX_KNOWN];
    let mut current_count = 0usize;
    let mut rendered = false;

    for entry in fat::root_dir(disk, &bpb) {
        if !entry.is_h75() {
            continue;
        }

        let id = FileId {
            start_cluster: entry.start_cluster,
            size: entry.size,
            write_time_date: entry.write_time_date,
        };

        if current_count < MAX_KNOWN {
            current[current_count] = Some(id);
            current_count += 1;
        }

        if rendered {
            continue;
        }

        let known_match = unsafe {
            #[allow(static_mut_refs)]
            KNOWN.iter().any(|k| *k == Some(id))
        };
        if known_match {
            continue;
        }

        if (entry.size as usize) > FILE_BUF_LEN {
            defmt::warn!(
                "frame: file too large ({} bytes, max {})",
                entry.size,
                FILE_BUF_LEN
            );
            continue;
        }

        let buf: &mut [u8] = unsafe {
            #[allow(static_mut_refs)]
            &mut FILE_BUF[..]
        };
        let n = match fat::read_file(disk, &bpb, &entry, buf) {
            Some(n) => n,
            None => {
                defmt::warn!("fat: read_file failed (cluster chain)");
                continue;
            }
        };

        let frame: &mut [[Rgb; WIDTH]; HEIGHT] = unsafe {
            #[allow(static_mut_refs)]
            &mut FRAME
        };
        match frame::parse_into(&buf[..n], frame) {
            Ok(()) => {
                panel.set_pixels(frame);
                defmt::info!("frame: rendered {} bytes", n);
                rendered = true;
            }
            Err(e) => {
                defmt::warn!("frame: parse error: {}", e);
            }
        }
    }

    unsafe {
        #[allow(static_mut_refs)]
        {
            KNOWN = current;
        }
    }
}

fn process_command(
    mut command: Command<ScsiCommand, Scsi<BulkOnly<UsbBus, &mut [u8]>>>,
) -> Result<(), TransportError<BulkOnlyError>> {
    match command.kind {
        ScsiCommand::TestUnitReady { .. } => {
            command.pass();
        }
        ScsiCommand::Inquiry { .. } => {
            command.try_write_data_all(&[
                0x00, 0x80, 0x04, 0x02, 0x20, 0x00, 0x00, 0x00,
                b't', b'e', b'a', b'r', b'n', b'e', b' ', b' ',
                b'h', b'u', b'b', b'7', b'5', b'-', b'd', b'r', b'o', b'p', b' ', b' ', b' ', b' ',
                b' ', b' ',
                b'0', b'.', b'1', b'0',
            ])?;
            command.pass();
        }
        ScsiCommand::RequestSense { .. } => critical_section::with(|cs| {
            let mut state = STATE.borrow_ref_mut(cs);
            command.try_write_data_all(&[
                0x70, 0x00,
                state.sense_key.unwrap_or(0),
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                state.sense_key_code.unwrap_or(0),
                state.sense_qualifier.unwrap_or(0),
                0x00, 0x00, 0x00, 0x00,
            ])?;
            state.reset();
            command.pass();
            Ok(())
        })?,
        ScsiCommand::ReadCapacity10 { .. } => {
            let mut data = [0u8; 8];
            data[0..4].copy_from_slice(&u32::to_be_bytes(BLOCKS - 1));
            data[4..8].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::ReadCapacity16 { .. } => {
            let mut data = [0u8; 16];
            data[0..8].copy_from_slice(&u64::to_be_bytes((BLOCKS - 1) as u64));
            data[8..12].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::ReadFormatCapacities { .. } => {
            let mut data = [0u8; 12];
            data[0..4].copy_from_slice(&[0x00, 0x00, 0x00, 0x08]);
            data[4..8].copy_from_slice(&u32::to_be_bytes(BLOCKS));
            data[8] = 0x02;
            let block_be = u32::to_be_bytes(BLOCK_SIZE);
            data[9] = block_be[1];
            data[10] = block_be[2];
            data[11] = block_be[3];
            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::Read { lba, len } => critical_section::with(|cs| {
            let len = len as u32;
            let mut state = STATE.borrow_ref_mut(cs);

            if state.storage_offset != (len * BLOCK_SIZE) as usize {
                let start = (BLOCK_SIZE * lba) as usize + state.storage_offset;
                let end = (BLOCK_SIZE * lba) as usize + (BLOCK_SIZE * len) as usize;
                #[allow(static_mut_refs)]
                let count = command.write_data(unsafe { &DISK[start..end] })?;
                state.storage_offset += count;
            } else {
                command.pass();
                state.storage_offset = 0;
            }
            Ok(())
        })?,
        ScsiCommand::Write { lba, len } => critical_section::with(|cs| {
            let len = len as u32;
            let mut state = STATE.borrow_ref_mut(cs);

            if state.storage_offset != (len * BLOCK_SIZE) as usize {
                let start = (BLOCK_SIZE * lba) as usize + state.storage_offset;
                let end = (BLOCK_SIZE * lba) as usize + (BLOCK_SIZE * len) as usize;
                #[allow(static_mut_refs)]
                let count = command.read_data(unsafe { &mut DISK[start..end] })?;
                state.storage_offset += count;
                if state.storage_offset == (len * BLOCK_SIZE) as usize {
                    command.pass();
                    state.storage_offset = 0;
                    // The main loop will start (or extend) the settle
                    // delay before re-scanning.
                    WRITE_COMPLETED.store(true, Ordering::Release);
                }
            } else {
                command.pass();
                state.storage_offset = 0;
            }
            Ok(())
        })?,
        ScsiCommand::ModeSense6 { .. } => {
            command.try_write_data_all(&[0x03, 0x00, 0x00, 0x00])?;
            command.pass();
        }
        ScsiCommand::ModeSense10 { .. } => {
            command.try_write_data_all(&[0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])?;
            command.pass();
        }
        ref unknown => {
            defmt::error!("Unknown SCSI command: {}", unknown);
            critical_section::with(|cs| {
                let mut state = STATE.borrow_ref_mut(cs);
                state.sense_key.replace(0x05);
                state.sense_key_code.replace(0x20);
                state.sense_qualifier.replace(0x00);
                command.fail();
                Ok::<(), TransportError<BulkOnlyError>>(())
            })?;
        }
    }
    Ok(())
}
