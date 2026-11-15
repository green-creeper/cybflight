use core::sync::atomic::{AtomicBool, Ordering};

use crate::hal::pac;

/// Whether [`init`] has started the IWDG. The param-store recovery path
/// can erase flash from `board_init`, *before* `init` runs; extending a
/// never-started watchdog would spin on the PVU flag with LSI possibly
/// off (only IWDG START forces LSI on), hanging boot forever.
static STARTED: AtomicBool = AtomicBool::new(false);

/// Initialize and start the Independent Watchdog (IWDG1).
///
/// Once started, IWDG cannot be stopped — it must be fed periodically via
/// [`feed()`] or the MCU will reset. This is intentional: if the firmware
/// hangs completely, the watchdog resets the system and ESCs disarm on their
/// own when DShot frames stop arriving.
///
/// Configuration: LSI ≈ 32 kHz, prescaler /32 → 1 kHz tick, reload = 500
/// → **~500 ms timeout**.
#[cfg(not(feature = "board_flywoof745"))]
const IWDG_REGS: pac::iwdg::Iwdg = pac::IWDG1;
#[cfg(feature = "board_flywoof745")]
const IWDG_REGS: pac::iwdg::Iwdg = pac::IWDG;

pub fn init() {
    let iwdg = IWDG_REGS;

    // Unlock PR/RLR registers
    iwdg.kr().write(|w| w.set_key(pac::iwdg::vals::Key::ENABLE));

    // Prescaler /32  → 32 kHz / 32 = 1 kHz
    iwdg.pr()
        .write(|w| w.set_pr(pac::iwdg::vals::Pr::DIVIDE_BY32));

    // Reload 500 → 500 ms timeout
    iwdg.rlr().write(|w| w.set_rl(500));

    // Start the watchdog — enables LSI and begins countdown (irreversible).
    // Initially counts with default values (prescaler /4, RL=0xFFF ≈ 512 ms).
    iwdg.kr().write(|w| w.set_key(pac::iwdg::vals::Key::START));

    // Wait for shadow register update (requires LSI running, which START enabled)
    while iwdg.sr().read().pvu() || iwdg.sr().read().rvu() {}

    // Feed to load the new reload value into the down-counter
    feed();

    STARTED.store(true, Ordering::Release);
    defmt::info!("IWDG started (~500 ms timeout)");
}

/// Feed (reload) the watchdog counter. Must be called before the timeout
/// expires or the MCU will reset.
#[inline]
pub fn feed() {
    IWDG_REGS
        .kr()
        .write(|w| w.set_key(pac::iwdg::vals::Key::RESET));
}

/// Temporarily extend the IWDG timeout for long-running blocking
/// operations (e.g. flash sector erase). Feeds the counter immediately so
/// the full extended period starts now.
///
/// Uses prescaler `/256` + reload `1500` (~8.1 s even at worst-case 47 kHz LSI),
/// safely covering the ~4.0 s max 256 KB single-bank STM32F745 sector erase.
///
/// The IWDG prescaler and reload registers can be modified while running by
/// first writing the unlock key (0x5555) to KR.
pub fn extend_timeout() {
    // No-op before `init`: there is no running watchdog to outlast the
    // erase, and the PVU/RVU spin below requires LSI (started by IWDG START).
    if !STARTED.load(Ordering::Acquire) {
        return;
    }
    let iwdg = IWDG_REGS;
    // Unlock PR/RLR
    iwdg.kr().write(|w| w.set_key(pac::iwdg::vals::Key::ENABLE));
    // Wait for any pending PR/RLR shadow register update before modifying them.
    while {
        let sr = iwdg.sr().read();
        sr.pvu() || sr.rvu()
    } {}
    iwdg.pr()
        .write(|w| w.set_pr(pac::iwdg::vals::Pr::DIVIDE_BY256));
    iwdg.rlr().write(|w| w.set_rl(1500));
    // Wait for both prescaler and reload updates to propagate before feeding.
    while {
        let sr = iwdg.sr().read();
        sr.pvu() || sr.rvu()
    } {}
    feed();
}

/// Restore the IWDG timeout to the normal ~500 ms and feed immediately.
pub fn restore_timeout() {
    if !STARTED.load(Ordering::Acquire) {
        return;
    }
    let iwdg = IWDG_REGS;
    // Unlock PR/RLR
    iwdg.kr().write(|w| w.set_key(pac::iwdg::vals::Key::ENABLE));
    while {
        let sr = iwdg.sr().read();
        sr.pvu() || sr.rvu()
    } {}
    // Prescaler /32 → 1 kHz nominal tick, reload 500 → ~500 ms
    iwdg.pr()
        .write(|w| w.set_pr(pac::iwdg::vals::Pr::DIVIDE_BY32));
    iwdg.rlr().write(|w| w.set_rl(500));
    while {
        let sr = iwdg.sr().read();
        sr.pvu() || sr.rvu()
    } {}
    feed();
}

/// Watchdog feeder task — runs at lowest priority on the thread executor.
///
/// Feeds the IWDG every 200 ms. If the thread executor stalls for >500 ms
/// (the IWDG timeout), the MCU resets.
#[embassy_executor::task]
pub async fn iwdg_feed_task() {
    loop {
        feed();
        embassy_time::Timer::after_millis(200).await;
    }
}
