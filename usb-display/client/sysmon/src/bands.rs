//! Banded rendering with inter-band pattern flow.
//!
//! The panel is split into 3 horizontal bands of unequal heights,
//! each at a different aggregation timescale. Data flows downward
//! from band to band: band 0 receives raw samples; deeper bands
//! aggregate factor-many evicted rows from the band above. At commit
//! time, the new band row's *dot pattern* is derived from the latest
//! non-blank pattern in the accumulator (the row that just aged out
//! of the band above), adjusted up or down to match the new
//! aggregated value's target dot count. Patterns therefore tend to
//! "flow through" band boundaries rather than reset to fresh
//! randomness, giving visual continuity at the seams.

pub const BAND_COUNT: usize = 3;
pub const BAND_HEIGHTS: [usize; BAND_COUNT] = [22, 21, 21];
pub const MAX_BAND_HEIGHT: usize = 22;

const AGGREGATION_FACTORS: [u32; BAND_COUNT] = [1, 12, 12];
const MAX_FACTOR: usize = 12;

#[derive(Clone, Copy, Default)]
pub struct BandRow {
    pub value: f32,
    pub pattern: u8,
    /// Whether this row's value sat at-or-above its band's
    /// pre-commit rolling mean. Fixed when the row was committed
    /// and carried unchanged as the row slides through the band.
    pub above_mean: bool,
}

pub struct Band {
    pub rows: [BandRow; MAX_BAND_HEIGHT],
    pub height: usize,
    accumulator: [BandRow; MAX_FACTOR],
    accumulator_count: u32,
    factor: u32,
}

impl Band {
    fn new(factor: u32, height: usize) -> Self {
        Self {
            rows: [BandRow::default(); MAX_BAND_HEIGHT],
            height,
            accumulator: [BandRow::default(); MAX_FACTOR],
            accumulator_count: 0,
            factor,
        }
    }

    /// Push an upstream `BandRow` into this band's accumulator. When
    /// the accumulator reaches `factor`, commit a new row at the top
    /// of this band: the value is the mean of the accumulated values;
    /// the pattern is derived from the latest non-blank accumulator
    /// pattern, adjusted to match the new value's target dot count.
    /// Returns the row evicted off the bottom (which flows to the
    /// next band's accumulator).
    fn push(&mut self, upstream: BandRow, width: usize, seed: u32) -> Option<BandRow> {
        shift_accumulator_down(&mut self.accumulator, self.accumulator_count as usize, upstream);
        self.accumulator_count += 1;
        if self.accumulator_count < self.factor { return None; }

        let count = self.factor as usize;
        let sum: f32 = self.accumulator[..count].iter().map(|r| r.value).sum();
        let mean = sum / self.factor as f32;
        let target_n = dot_count(mean, width);

        // Start from the latest non-blank pattern and adjust to
        // target_n. When adding/removing dots, the choice of which
        // column is biased by the *frequency* with which each column
        // appears across the accumulator's full history — adds prefer
        // columns lit by other accumulator entries; removes prefer
        // columns lit only by the basis itself.
        let basis = latest_non_blank_pattern(&self.accumulator[..count]);
        let pattern = flow_pattern(basis, &self.accumulator[..count], width, target_n, seed);

        let above_mean = mean >= prior_mean(&self.rows[..self.height]);

        self.accumulator_count = 0;

        let evicted = self.rows[self.height - 1];
        for i in (1..self.height).rev() {
            self.rows[i] = self.rows[i - 1];
        }
        self.rows[0] = BandRow { value: mean, pattern, above_mean };
        Some(evicted)
    }
}

pub struct MetricBands {
    pub bands: [Band; BAND_COUNT],
    pub width: usize,
    commit_count: u64,
}

impl MetricBands {
    pub fn new(width: usize) -> Self {
        let bands = std::array::from_fn(|i| Band::new(AGGREGATION_FACTORS[i], BAND_HEIGHTS[i]));
        Self { bands, width, commit_count: 0 }
    }

    pub fn push_sample(&mut self, value: f32) {
        self.commit_count = self.commit_count.wrapping_add(1);
        // Raw upstream row never hits the panel directly — it's just
        // an accumulator entry feeding band 0. `above_mean` is unused
        // until the row is committed downstream, where it's recomputed.
        let mut upstream = Some(BandRow { value, pattern: 0, above_mean: false });
        for (band_idx, band) in self.bands.iter_mut().enumerate() {
            let Some(row) = upstream else { break };
            let seed = mix32(self.commit_count as u32, band_idx as u32);
            upstream = band.push(row, self.width, seed);
        }
    }
}

// ── Pure helpers ───────────────────────────────────────────────────

fn shift_accumulator_down(
    acc: &mut [BandRow; MAX_FACTOR],
    current_count: usize,
    new_entry: BandRow,
) {
    let limit = current_count.min(MAX_FACTOR - 1);
    for i in (1..=limit).rev() {
        acc[i] = acc[i - 1];
    }
    acc[0] = new_entry;
}

fn latest_non_blank_pattern(entries: &[BandRow]) -> u8 {
    for entry in entries.iter() {
        if entry.pattern != 0 { return entry.pattern; }
    }
    0
}

fn prior_mean(rows: &[BandRow]) -> f32 {
    let sum: f32 = rows.iter().map(|r| r.value).sum();
    sum / rows.len() as f32
}

pub fn dot_count(value: f32, width: usize) -> usize {
    if value <= 0.0 { return 0; }
    ((value.clamp(0.0, 1.0) * width as f32).round() as usize).max(1)
}

/// Derive a target-N pattern from `basis`, biasing add/remove choices
/// by how often each column appears across the accumulator's entries.
/// - To add dots: prefer columns most frequently lit elsewhere in the
///   accumulator (the consensus), tie-broken by hash.
/// - To remove dots: prefer columns least frequently lit (i.e.
///   columns that only the basis carried), tie-broken by hash.
fn flow_pattern(
    basis: u8,
    accumulator: &[BandRow],
    width: usize,
    target_n: usize,
    seed: u32,
) -> u8 {
    if target_n == 0 { return 0; }
    let curr_n = basis.count_ones() as usize;
    if target_n == curr_n { return basis; }

    let mut freq = [0u32; 8];
    for entry in accumulator {
        for col in 0..width as u8 {
            if entry.pattern & (1 << col) != 0 {
                freq[col as usize] += 1;
            }
        }
    }

    if target_n > curr_n {
        let to_add = target_n - curr_n;
        let mut unlit: Vec<u8> = (0..width as u8).filter(|i| basis & (1 << i) == 0).collect();
        unlit.sort_unstable_by_key(|&i| {
            (std::cmp::Reverse(freq[i as usize]), mix32(seed, i as u32))
        });
        unlit.into_iter().take(to_add).fold(basis, |acc, i| acc | (1 << i))
    } else {
        let to_remove = curr_n - target_n;
        let mut lit: Vec<u8> = (0..width as u8).filter(|i| basis & (1 << i) != 0).collect();
        lit.sort_unstable_by_key(|&i| {
            (freq[i as usize], mix32(seed, i as u32))
        });
        lit.into_iter().take(to_remove).fold(basis, |acc, i| acc & !(1 << i))
    }
}

fn mix32(a: u32, b: u32) -> u32 {
    let mut x = a.wrapping_add(b.wrapping_mul(0x9e3779b9));
    x ^= x >> 16;
    x = x.wrapping_mul(0x85ebca6b);
    x ^= x >> 13;
    x.wrapping_mul(0xc2b2ae35) ^ (x >> 16)
}
