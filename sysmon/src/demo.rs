//! Synthetic metric source for button-B demo mode. Produces the nine
//! fractions sysmon would otherwise read from the live samplers, shaped
//! per-metric so the panel reads as "the host got busy" rather than as
//! a uniform-noise test pattern. See `map.md` "Modes".

use rand::{rngs::ThreadRng, Rng};

use crate::presentation::CORE_COUNT;

pub struct DemoSample {
    pub cpu: [f32; CORE_COUNT],
    pub ram: f32,
    pub disk_read: f32,
    pub disk_write: f32,
    pub net_down: f32,
    pub net_up: f32,
    pub peak_busy: f32,
}

pub struct DemoEngine {
    rng: ThreadRng,
    /// Slow [0, 1] walk gating per-core spike probability. When low,
    /// all cores tend to idle together, dropping peak_busy and slowing
    /// the adaptive frame rate; when high, cores spike often and the
    /// rate ramps up. Drives the visible speed-up / slow-down rhythm.
    intensity: Walk,
    cpu: [Walk; CORE_COUNT],
    ram: Walk,
    disk_read: Burst,
    disk_write: Burst,
    net_down: Burst,
    net_up: Burst,
}

/// Low-passed walk that mostly idles low and occasionally jumps to a
/// high target — gives CPU cores a bursty character rather than a
/// uniform mid-level. `tau` controls how quickly the value eases toward
/// `target`; lower tau = livelier metric.
struct Walk {
    value: f32,
    target: f32,
    tau: f32,
    idle_lo: f32,
    idle_hi: f32,
    spike_lo: f32,
    spike_hi: f32,
    spike_prob: f32,
    retarget_per_sec: f32,
}

impl Walk {
    fn step(&mut self, dt: f32, rng: &mut ThreadRng) -> f32 {
        if rng.gen::<f32>() < (dt * self.retarget_per_sec).min(1.0) {
            self.target = if rng.gen::<f32>() < self.spike_prob {
                rng.gen_range(self.spike_lo..=self.spike_hi)
            } else {
                rng.gen_range(self.idle_lo..=self.idle_hi)
            };
        }
        let alpha = 1.0 - (-dt / self.tau).exp();
        self.value += alpha * (self.target - self.value);
        self.value
    }
}

/// Bursty channel: near-zero baseline with Bernoulli-triggered bursts
/// that decay exponentially. Matches the visual character of disk and
/// network bytes/sec on a quiet host.
struct Burst {
    value: f32,
    burst_per_sec: f32,
    decay_tau: f32,
    peak_lo: f32,
    peak_hi: f32,
}

impl Burst {
    fn step(&mut self, dt: f32, rng: &mut ThreadRng) -> f32 {
        if rng.gen::<f32>() < (dt * self.burst_per_sec).min(1.0) {
            self.value = rng.gen_range(self.peak_lo..=self.peak_hi);
        } else {
            self.value *= (-dt / self.decay_tau).exp();
        }
        self.value
    }
}

impl DemoEngine {
    pub fn new() -> Self {
        let mut rng = rand::thread_rng();
        // Slow modulator: targets bounce between near-quiet (~0.15)
        // and full-busy (~0.95) every ~5–8 s, swinging the panel's
        // adaptive frame rate noticeably across the demo.
        let intensity = Walk {
            value: rng.gen_range(0.2..0.8),
            target: rng.gen_range(0.2..0.8),
            tau: 2.5,
            idle_lo: 0.05,
            idle_hi: 0.2,
            spike_lo: 0.85,
            spike_hi: 1.0,
            spike_prob: 0.5,
            retarget_per_sec: 0.15,
        };
        // Per-core taus chosen to give visibly different breathing
        // rates so cores don't move in lockstep.
        // Snappy taus + slow retargets: each core spends most of its
        // time sitting at either idle or spike, with only brief
        // transitions through the middle.
        let cpu_taus = [0.15f32, 0.25, 0.35, 0.5];
        let cpu_retarget = [0.25f32, 0.35, 0.2, 0.3];
        let cpu = std::array::from_fn(|i| Walk {
            value: rng.gen_range(0.0..0.1),
            target: rng.gen_range(0.0..0.1),
            tau: cpu_taus[i],
            idle_lo: 0.0,
            idle_hi: 0.1,
            spike_lo: 0.85,
            spike_hi: 1.0,
            spike_prob: 0.5,
            retarget_per_sec: cpu_retarget[i],
        });
        // RAM stays a slow drift, not bursty — that's its real character.
        let ram = Walk {
            value: rng.gen_range(0.4..0.6),
            target: rng.gen_range(0.4..0.7),
            tau: 6.0,
            idle_lo: 0.35,
            idle_hi: 0.85,
            spike_lo: 0.85,
            spike_hi: 0.95,
            spike_prob: 0.05,
            retarget_per_sec: 0.15,
        };
        // Bursts: rarer, sharper, decay faster — quiet baseline punctuated
        // by spikes rather than a constant mid-level shimmer.
        let disk_read = Burst { value: 0.0, burst_per_sec: 0.15, decay_tau: 0.5, peak_lo: 0.6, peak_hi: 1.0 };
        let disk_write = Burst { value: 0.0, burst_per_sec: 0.1, decay_tau: 0.8, peak_lo: 0.5, peak_hi: 1.0 };
        // Down-heavy bias: more frequent and slightly higher bursts on net_down.
        let net_down = Burst { value: 0.0, burst_per_sec: 0.2, decay_tau: 0.7, peak_lo: 0.6, peak_hi: 1.0 };
        let net_up = Burst { value: 0.0, burst_per_sec: 0.1, decay_tau: 0.6, peak_lo: 0.4, peak_hi: 0.8 };
        Self { rng, intensity, cpu, ram, disk_read, disk_write, net_down, net_up }
    }

    pub fn step(&mut self, dt_secs: f32) -> DemoSample {
        let dt = dt_secs.max(0.001);
        let intensity = self.intensity.step(dt, &mut self.rng).clamp(0.0, 1.0);
        let mut cpu = [0.0f32; CORE_COUNT];
        let mut peak_busy = 0.0f32;
        let base_spike = 0.5f32;
        for i in 0..CORE_COUNT {
            self.cpu[i].spike_prob = (base_spike * intensity).clamp(0.0, 1.0);
            cpu[i] = self.cpu[i].step(dt, &mut self.rng).clamp(0.0, 1.0);
            if cpu[i] > peak_busy { peak_busy = cpu[i]; }
        }
        DemoSample {
            cpu,
            ram: self.ram.step(dt, &mut self.rng).clamp(0.0, 1.0),
            disk_read: self.disk_read.step(dt, &mut self.rng).clamp(0.0, 1.0),
            disk_write: self.disk_write.step(dt, &mut self.rng).clamp(0.0, 1.0),
            net_down: self.net_down.step(dt, &mut self.rng).clamp(0.0, 1.0),
            net_up: self.net_up.step(dt, &mut self.rng).clamp(0.0, 1.0),
            peak_busy,
        }
    }
}
