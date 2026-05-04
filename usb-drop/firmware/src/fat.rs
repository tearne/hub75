//! Minimal FAT12 root-directory reader.
//!
//! Parses the BPB (boot sector), enumerates 32-byte root directory
//! entries, and reads file contents by walking the FAT12 cluster
//! chain. No FAT16/FAT32, no subdirectories, no LFN parsing — short
//! 8.3 names only. Just enough for the file-drop firmware to find a
//! `*.H75` file and read its bytes into a buffer.

#[derive(Copy, Clone)]
pub struct Bpb {
    pub bytes_per_sector: u16,
    pub sectors_per_cluster: u8,
    pub reserved_sectors: u16,
    pub num_fats: u8,
    pub root_entries: u16,
    pub sectors_per_fat: u16,
}

impl Bpb {
    pub fn parse(disk: &[u8]) -> Option<Self> {
        if disk.len() < 36 {
            return None;
        }
        let bps = u16::from_le_bytes([disk[0x0B], disk[0x0C]]);
        let spc = disk[0x0D];
        let rsv = u16::from_le_bytes([disk[0x0E], disk[0x0F]]);
        let nfats = disk[0x10];
        let rents = u16::from_le_bytes([disk[0x11], disk[0x12]]);
        let spf = u16::from_le_bytes([disk[0x16], disk[0x17]]);
        if bps == 0 || spc == 0 || nfats == 0 || spf == 0 {
            return None;
        }
        Some(Self {
            bytes_per_sector: bps,
            sectors_per_cluster: spc,
            reserved_sectors: rsv,
            num_fats: nfats,
            root_entries: rents,
            sectors_per_fat: spf,
        })
    }

    fn fat_offset(&self) -> usize {
        self.reserved_sectors as usize * self.bytes_per_sector as usize
    }

    fn root_dir_offset(&self) -> usize {
        self.fat_offset()
            + self.num_fats as usize
                * self.sectors_per_fat as usize
                * self.bytes_per_sector as usize
    }

    fn root_dir_bytes(&self) -> usize {
        self.root_entries as usize * 32
    }

    fn data_offset(&self) -> usize {
        self.root_dir_offset() + self.root_dir_bytes()
    }

    pub fn cluster_bytes(&self) -> usize {
        self.sectors_per_cluster as usize * self.bytes_per_sector as usize
    }

    pub fn cluster_data_offset(&self, cluster: u16) -> usize {
        self.data_offset() + (cluster as usize - 2) * self.cluster_bytes()
    }
}

#[derive(Copy, Clone)]
pub struct DirEntry {
    /// 8.3 short name, padded with spaces (e.g. b"FRAME   H75").
    pub name: [u8; 11],
    pub attr: u8,
    pub start_cluster: u16,
    pub size: u32,
    /// Combined write time (offsets 22-23) and write date (24-25)
    /// from the directory entry, packed as `time | (date << 16)`.
    /// Updated by the host on every overwrite, even if content is
    /// unchanged — gives the firmware a way to detect re-drops that
    /// keep the same cluster + size.
    pub write_time_date: u32,
}

impl DirEntry {
    /// True for `*.H75` (any base, three-char extension exactly "H75").
    pub fn is_h75(&self) -> bool {
        &self.name[8..11] == b"H75"
    }
}

pub struct RootDirIter<'a> {
    disk: &'a [u8],
    pos: usize,
    end: usize,
}

impl<'a> Iterator for RootDirIter<'a> {
    type Item = DirEntry;

    fn next(&mut self) -> Option<DirEntry> {
        while self.pos + 32 <= self.end {
            let e = &self.disk[self.pos..self.pos + 32];
            self.pos += 32;
            let first = e[0];
            if first == 0x00 {
                return None; // end of directory
            }
            if first == 0xE5 {
                continue; // deleted
            }
            let attr = e[11];
            if attr == 0x0F {
                continue; // LFN entry
            }
            if attr & 0x08 != 0 {
                continue; // volume label
            }
            if attr & 0x10 != 0 {
                continue; // subdirectory
            }
            let mut name = [0u8; 11];
            name.copy_from_slice(&e[0..11]);
            let start_cluster = u16::from_le_bytes([e[26], e[27]]);
            let size = u32::from_le_bytes([e[28], e[29], e[30], e[31]]);
            let wtime = u16::from_le_bytes([e[22], e[23]]) as u32;
            let wdate = u16::from_le_bytes([e[24], e[25]]) as u32;
            let write_time_date = wtime | (wdate << 16);
            return Some(DirEntry {
                name,
                attr,
                start_cluster,
                size,
                write_time_date,
            });
        }
        None
    }
}

pub fn root_dir<'a>(disk: &'a [u8], bpb: &Bpb) -> RootDirIter<'a> {
    let start = bpb.root_dir_offset();
    let end = start + bpb.root_dir_bytes();
    RootDirIter {
        disk,
        pos: start,
        end: end.min(disk.len()),
    }
}

/// Look up the FAT12 entry for cluster `n` (12-bit packed; entries
/// alternate between low and high nibble of the shared byte).
fn fat12_entry(disk: &[u8], bpb: &Bpb, cluster: u16) -> Option<u16> {
    let fat = bpb.fat_offset();
    let idx = cluster as usize;
    let byte_idx = fat + idx + idx / 2;
    if byte_idx + 1 >= disk.len() {
        return None;
    }
    let lo = disk[byte_idx] as u16;
    let hi = disk[byte_idx + 1] as u16;
    let raw = lo | (hi << 8);
    Some(if cluster & 1 == 0 {
        raw & 0x0FFF
    } else {
        raw >> 4
    })
}

/// Read the contents of `entry` into `out`, returning the number of
/// bytes written. Stops at `entry.size` or `out.len()`, whichever is
/// smaller. Returns `None` if the cluster chain is malformed.
pub fn read_file(disk: &[u8], bpb: &Bpb, entry: &DirEntry, out: &mut [u8]) -> Option<usize> {
    let mut written = 0usize;
    let mut remaining = (entry.size as usize).min(out.len());
    let cluster_bytes = bpb.cluster_bytes();
    let mut cluster = entry.start_cluster;

    while remaining > 0 {
        if cluster < 2 || cluster >= 0xFF8 {
            // Either bogus or end-of-chain before we read everything.
            return None;
        }
        let off = bpb.cluster_data_offset(cluster);
        if off + cluster_bytes > disk.len() {
            return None;
        }
        let take = remaining.min(cluster_bytes);
        out[written..written + take].copy_from_slice(&disk[off..off + take]);
        written += take;
        remaining -= take;

        if remaining == 0 {
            break;
        }
        cluster = fat12_entry(disk, bpb, cluster)?;
    }
    Some(written)
}
