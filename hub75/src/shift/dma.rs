//! Four-channel autonomous DMA chain for shift-register scanning.
//!
//! - **data**: bitplane buffer → SM0 (Data) TX FIFO. Chains to `data_reload`.
//! - **data_reload**: pointer (active buffer addr) → `data`'s read-addr trigger.
//! - **time**: timing buffer → SM1 (Address) TX FIFO. Chains to `time_reload`.
//! - **time_reload**: pointer (timing buf addr) → `time`'s read-addr trigger.
//!
//! The reload channel writes one word — the active buffer pointer —
//! into the `AL3_READ_ADDR_TRIG` alias of the data channel, which both
//! reloads its read pointer and re-triggers it. Application swaps
//! displayed buffers by updating the static the reload channel reads
//! from; the next chain cycle picks it up.
//!
//! No completion IRQ. The chains run forever once started.

use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TransCountMode, TreqSel};

use super::TIMING_BUF_WORDS;

/// Set up and start all four chain channels. Caller must have:
/// - written `*active_buf_ptr_addr` with the address of the initial
///   active bitplane buffer
/// - populated the timing buffer
/// - configured and enabled the SMs (so PIO can drain the FIFOs).
pub fn start_chains(
    cfg: ChainConfig,
) {
    configure_data(&cfg);
    configure_data_reload(&cfg);
    configure_time(&cfg);
    configure_time_reload(&cfg);

    // Trigger the data and time channels (the reload channels are
    // chained-to and will run after the first data/time block).
    trigger(cfg.data_ch);
    trigger(cfg.time_ch);
}

pub struct ChainConfig {
    pub data_ch: u8,
    pub data_reload_ch: u8,
    pub time_ch: u8,
    pub time_reload_ch: u8,
    /// Address of the (statically-stored) active-buffer pointer that
    /// `data_reload` reads from each cycle.
    pub active_buf_ptr_addr: u32,
    /// Address of the timing buffer (also used by `time_reload`).
    pub timing_buf_addr: u32,
    /// Total u32 words in one bitplane DMA buffer (`H * W` for the
    /// shift-register layout).
    pub data_words: u32,
    /// Address of PIO0 SM0 TX FIFO (data destination).
    pub txf_data_addr: u32,
    /// Address of PIO0 SM1 TX FIFO (timing destination).
    pub txf_time_addr: u32,
}

fn configure_data(cfg: &ChainConfig) {
    let ch = pac::DMA.ch(cfg.data_ch as usize);
    // Read addr will be loaded by data_reload before each block.
    ch.write_addr().write_value(cfg.txf_data_addr);
    ch.trans_count().write(|w| {
        w.set_count(cfg.data_words);
        w.set_mode(TransCountMode::NORMAL);
    });
    // Write CTRL via the AL1 alias so we don't trigger yet.
    pac::DMA.ch(cfg.data_ch as usize).al1_ctrl().write_value(
        ctrl_word_for_data(cfg.data_reload_ch),
    );
}

fn configure_data_reload(cfg: &ChainConfig) {
    let ch = pac::DMA.ch(cfg.data_reload_ch as usize);
    ch.read_addr().write_value(cfg.active_buf_ptr_addr);
    // Write into the data channel's al3_read_addr_trig so the write
    // both reloads the read pointer and retriggers data.
    let target = pac::DMA.ch(cfg.data_ch as usize).al3_read_addr_trig().as_ptr() as u32;
    ch.write_addr().write_value(target);
    ch.trans_count().write(|w| {
        w.set_count(1);
        w.set_mode(TransCountMode::NORMAL);
    });
    ch.ctrl_trig().write(|w| {
        w.set_en(true);
        w.set_data_size(DataSize::SIZE_WORD);
        w.set_incr_read(false);
        w.set_incr_write(false);
        // Chain to self disables further chaining; this channel is
        // armed by `data` chaining to it.
        w.set_chain_to(cfg.data_reload_ch);
        w.set_treq_sel(TreqSel::PERMANENT);
        w.set_irq_quiet(true);
    });
}

fn configure_time(cfg: &ChainConfig) {
    let ch = pac::DMA.ch(cfg.time_ch as usize);
    ch.write_addr().write_value(cfg.txf_time_addr);
    ch.trans_count().write(|w| {
        w.set_count(TIMING_BUF_WORDS as u32);
        w.set_mode(TransCountMode::NORMAL);
    });
    pac::DMA.ch(cfg.time_ch as usize).al1_ctrl().write_value(
        ctrl_word_for_time(cfg.time_reload_ch),
    );
}

fn configure_time_reload(cfg: &ChainConfig) {
    let ch = pac::DMA.ch(cfg.time_reload_ch as usize);
    // The pointer source is a static holding the timing buf address;
    // we point at that static. The address-of-the-address must outlive
    // this DMA, so the caller passes a `'static` u32 location.
    ch.read_addr().write_value(cfg.timing_buf_addr);
    let target = pac::DMA.ch(cfg.time_ch as usize).al3_read_addr_trig().as_ptr() as u32;
    ch.write_addr().write_value(target);
    ch.trans_count().write(|w| {
        w.set_count(1);
        w.set_mode(TransCountMode::NORMAL);
    });
    ch.ctrl_trig().write(|w| {
        w.set_en(true);
        w.set_data_size(DataSize::SIZE_WORD);
        w.set_incr_read(false);
        w.set_incr_write(false);
        w.set_chain_to(cfg.time_reload_ch);
        w.set_treq_sel(TreqSel::PERMANENT);
        w.set_irq_quiet(true);
    });
}

fn ctrl_word_for_data(reload_ch: u8) -> u32 {
    let mut w = pac::dma::regs::CtrlTrig(0);
    w.set_en(true);
    w.set_data_size(DataSize::SIZE_WORD);
    w.set_incr_read(true);
    w.set_incr_write(false);
    w.set_chain_to(reload_ch);
    w.set_treq_sel(TreqSel::from_bits(0)); // PIO0 SM0 TX
    w.set_irq_quiet(true);
    w.0
}

fn ctrl_word_for_time(reload_ch: u8) -> u32 {
    let mut w = pac::dma::regs::CtrlTrig(0);
    w.set_en(true);
    w.set_data_size(DataSize::SIZE_WORD);
    w.set_incr_read(true);
    w.set_incr_write(false);
    w.set_chain_to(reload_ch);
    w.set_treq_sel(TreqSel::from_bits(1)); // PIO0 SM1 TX
    w.set_irq_quiet(true);
    w.0
}

fn trigger(ch_no: u8) {
    // Touch ctrl_trig with the previously-written value to fire it.
    let regs = pac::DMA.ch(ch_no as usize);
    let cur = regs.al1_ctrl().read();
    regs.ctrl_trig().write_value(pac::dma::regs::CtrlTrig(cur));
}
