//! Boot-time post-mortem recovery: read the BKPSRAM record left by
//! the prior boot, validate it, log a one-line summary, then publish
//! it for downstream consumers (shell, blackbox recorder).
//!
//! ## Sequence
//!
//! 1. `pre_init` already ran [`super::reset_cause::capture`], so
//!    `RAW_RESET_CAUSE` is populated.
//! 2. Caller (`main.rs`, after `bsp::init()`) invokes
//!    [`super::bkpsram::enable`] to clock the BKPSRAM controller.
//! 3. Caller invokes [`boot_recovery`] which:
//!    - Reads the BKPSRAM slot, validates magic + CRC.
//!    - If valid: copies the record into [`PRIOR_RECORD`] (a Signal
//!      consumed once by the blackbox recorder + shell), bumps
//!      `boot_count`, logs via defmt.
//!    - If invalid (cold boot, schema mismatch, torn write): treats
//!      it as the first boot.
//!    - Writes a fresh record into BKPSRAM with this boot's
//!      reset_cause + incremented boot_count, finalizes.
//!
//! After this returns, the BKPSRAM slot holds the *current* boot's
//! starting state. The post-mortem task and fault handlers may
//! mutate it freely from here.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use super::bkpsram::{self, PANIC_TEXT_MAX_BYTES};
use super::record::{self, FatalKind, PostmortemRecord};
use super::reset_cause;

/// Threshold (ms since boot) under which a fatal crash on the prior boot
/// triggers Early-Crash USB Safe Mode on the next boot so USB CDC stays
/// alive for `postmortem show` inspection instead of reset-looping.
const EARLY_CRASH_SAFE_MODE_MS: u32 = 10_000;

/// Latched copy of the prior boot's record. Filled by
/// [`boot_recovery`] when a valid record is found; otherwise stays
/// empty.
///
/// Single-shot semantics: the blackbox recorder takes it via
/// [`take_pending`] when the next flight session opens.
pub static PRIOR_RECORD: Signal<CriticalSectionRawMutex, PostmortemRecord> = Signal::new();

/// Non-consuming latched snapshot of the prior boot's record so
/// `postmortem show` in the USB shell can always inspect the prior crash
/// even after BKPSRAM has been re-initialized for the current boot.
static PRIOR_SNAPSHOT: Mutex<RefCell<Option<PostmortemRecord>>> = Mutex::new(RefCell::new(None));

/// Non-consuming latched copy of formatted `file:line:col: message` text
/// captured by the prior boot's `#[panic_handler]`.
static PRIOR_PANIC_TEXT: Mutex<RefCell<([u8; PANIC_TEXT_MAX_BYTES], usize)>> =
    Mutex::new(RefCell::new(([0u8; PANIC_TEXT_MAX_BYTES], 0)));

/// Set to `true` when [`boot_recovery`] decides this boot should run in
/// minimal USB Safe Mode due to an early crash on the previous boot.
static SAFE_MODE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// One-shot guard against re-entry. `boot_recovery` is idempotent
/// (a second call would not re-publish the prior record because the
/// BKPSRAM slot was already overwritten with this boot's state) but
/// the guard makes accidental double-calls cheap and observable.
static RECOVERED: AtomicBool = AtomicBool::new(false);

/// Boot-time entry point. Call exactly once from `main.rs` after
/// `bsp::init()` and [`super::bkpsram::enable`].
///
/// Returns `true` if the previous boot suffered an early fatal crash
/// (`Panic`, `HardFault`, or `IndependentWatchdog` within the first
/// [`EARLY_CRASH_SAFE_MODE_MS`] ms) and the caller should enter minimal
/// USB Safe Mode instead of re-triggering the crash loop.
///
/// **Hard rules:** does not allocate, does not await, does not panic
/// (a panic here would land in the same panic handler the post-mortem
/// subsystem provides — re-entry is guarded by the depth counter in
/// `fault.rs`, but we still avoid the situation).
pub fn boot_recovery() -> bool {
    if RECOVERED.swap(true, Ordering::AcqRel) {
        defmt::warn!("postmortem: boot_recovery called twice — ignoring");
        return SAFE_MODE_ACTIVE.load(Ordering::Acquire);
    }

    let cause = reset_cause::read();
    let cause_kind = reset_cause::classify(cause);

    let prior = bkpsram::with_record(|r| if record::is_valid(r) { Some(*r) } else { None });
    let mut panic_buf = [0u8; PANIC_TEXT_MAX_BYTES];
    let panic_len = bkpsram::take_panic_text(&mut panic_buf).unwrap_or(0);

    let mut enter_safe_mode = false;

    let next_boot_count = match prior.as_ref() {
        Some(p) => {
            let prior_kind = FatalKind::from_u8(p.fatal.kind);
            if matches!(prior_kind, FatalKind::None)
                && !matches!(cause_kind, reset_cause::ResetKind::IndependentWatchdog)
            {
                defmt::info!(
                    "postmortem: prior boot {} ended cleanly (reset_cause={:?}, uptime={=u32}ms)",
                    p.header.boot_count,
                    cause_kind,
                    p.header.uptime_ms,
                );
            } else {
                defmt::error!(
                    "postmortem: prior boot crash recovered (boot_count={=u32}, reset_cause={:?}, fatal={:?}, uptime={=u32}ms, pc=0x{=u32:08x}, lr=0x{=u32:08x})",
                    p.header.boot_count,
                    cause_kind,
                    prior_kind,
                    p.header.uptime_ms,
                    p.fatal.pc,
                    p.fatal.lr,
                );
                if panic_len > 0 {
                    if let Ok(s) = core::str::from_utf8(&panic_buf[..panic_len]) {
                        defmt::error!("postmortem: prior panic: {}", s);
                    }
                }
            }

            let is_crash = !matches!(prior_kind, FatalKind::Brownout)
                && (matches!(prior_kind, FatalKind::Panic | FatalKind::HardFault)
                    || matches!(cause_kind, reset_cause::ResetKind::IndependentWatchdog));
            if is_crash && p.header.uptime_ms < EARLY_CRASH_SAFE_MODE_MS {
                enter_safe_mode = true;
            }

            critical_section::with(|cs| {
                *PRIOR_SNAPSHOT.borrow_ref_mut(cs) = Some(*p);
                *PRIOR_PANIC_TEXT.borrow_ref_mut(cs) = (panic_buf, panic_len);
            });
            PRIOR_RECORD.signal(*p);
            p.header.boot_count.wrapping_add(1)
        }
        None => {
            if panic_len > 0 {
                critical_section::with(|cs| {
                    *PRIOR_PANIC_TEXT.borrow_ref_mut(cs) = (panic_buf, panic_len);
                });
            }
            defmt::info!(
                "postmortem: no prior record (cold boot or first run after firmware update); reset_cause={:?}",
                cause_kind
            );
            1
        }
    };

    SAFE_MODE_ACTIVE.store(enter_safe_mode, Ordering::Release);

    // Initialize this boot's record (clean FatalKind::None, so rebooting
    // out of Safe Mode will try normal boot again).
    let mut fresh = PostmortemRecord::ZERO;
    fresh.header.boot_count = next_boot_count;
    fresh.header.reset_cause = cause;
    fresh.header.fw_git_hash = git_hash_u32();
    fresh.header.uptime_ms = 0;
    bkpsram::store_finalized(&fresh);

    enter_safe_mode
}

/// Returns `true` if the firmware booted into Early-Crash USB Safe Mode.
pub fn in_safe_mode() -> bool {
    SAFE_MODE_ACTIVE.load(Ordering::Acquire)
}

/// Non-consuming read of the prior boot's record latched by [`boot_recovery`].
pub fn prior_boot_record() -> Option<PostmortemRecord> {
    critical_section::with(|cs| *PRIOR_SNAPSHOT.borrow_ref(cs))
}

/// Non-consuming copy of the prior boot's formatted panic string, if any.
pub fn prior_panic_text(out: &mut [u8; PANIC_TEXT_MAX_BYTES]) -> Option<usize> {
    critical_section::with(|cs| {
        let (ref buf, len) = *PRIOR_PANIC_TEXT.borrow_ref(cs);
        if len == 0 {
            None
        } else {
            out[..len].copy_from_slice(&buf[..len]);
            Some(len)
        }
    })
}

/// Clear the in-RAM latched prior-boot record and panic text (`postmortem clear`).
pub fn clear_prior_snapshot() {
    critical_section::with(|cs| {
        *PRIOR_SNAPSHOT.borrow_ref_mut(cs) = None;
        PRIOR_PANIC_TEXT.borrow_ref_mut(cs).1 = 0;
    });
}

/// Drain the prior-boot record. Returns `Some(...)` exactly once if a
/// prior crash was recovered; subsequent calls return `None`.
///
/// Used by the blackbox recorder at session-open to mirror the prior
/// boot's events into the new MCAP session as `/events` records.
pub fn take_pending() -> Option<PostmortemRecord> {
    PRIOR_RECORD.try_take()
}

/// Read the current boot's live BKPSRAM record without consuming anything.
pub fn current_boot_record() -> PostmortemRecord {
    bkpsram::with_record(|r| *r)
}

/// First 4 bytes of [`crate::GIT_HASH`] as a `u32`. If git_hash isn't
/// available (build without git), returns `0`.
fn git_hash_u32() -> u32 {
    let bytes = crate::GIT_HASH.as_bytes();
    if bytes.len() < 4 {
        return 0;
    }
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}
