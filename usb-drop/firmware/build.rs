use std::env;
use std::fs::File;
use std::io::{Cursor, Write};
use std::path::PathBuf;

// 256 KB is enough for the 64×64 JSON frame (~50–80 KB worst case)
// plus the README and FAT structures.
const DISK_BYTES: usize = 262_144;

const README_CONTENTS: &str = "\
HUB75 USB drop drive
====================\r\n\r\n\
Drop a JSON file (any name, extension .H75) onto this drive. The\r\n\
firmware will detect the new file, parse it, and render it on the\r\n\
panel.\r\n\r\n\
File format (version 1):\r\n\r\n\
{\r\n  \"version\": 1,\r\n  \"width\": 64,\r\n  \"height\": 64,\r\n  \"pixels\": [\r\n    [255, 0, 0], [0, 255, 0], [0, 0, 255], ...\r\n  ]\r\n}\r\n\r\n\
- width and height must match the panel (64x64).\r\n\
- pixels is a flat list of [R, G, B] triplets, row-major from top.\r\n\
- length must be width * height = 4096 entries.\r\n\
- values are 0..255 per channel; gamma is applied by the firmware.\r\n\r\n\
Multi-frame animations are not yet supported (planned for v2).\r\n";

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    let mut buf = vec![0u8; DISK_BYTES];
    {
        let cursor = Cursor::new(&mut buf[..]);
        fatfs::format_volume(
            cursor,
            fatfs::FormatVolumeOptions::new()
                .fat_type(fatfs::FatType::Fat12)
                .volume_label(*b"HUB75DROP  "),
        )
        .expect("format_volume");
    }
    {
        let cursor = Cursor::new(&mut buf[..]);
        let fs = fatfs::FileSystem::new(cursor, fatfs::FsOptions::new()).expect("mount");
        let root = fs.root_dir();
        let mut readme = root.create_file("README.TXT").expect("create README.TXT");
        readme
            .write_all(README_CONTENTS.as_bytes())
            .expect("write README.TXT");
        readme.flush().expect("flush");
    }

    File::create(out.join("disk.img"))
        .expect("create disk.img")
        .write_all(&buf)
        .expect("write disk.img");
    println!("cargo:rerun-if-changed=build.rs");
}
