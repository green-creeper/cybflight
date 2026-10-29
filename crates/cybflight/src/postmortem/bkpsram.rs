//! D3-domain backup SRAM access for the post-mortem subsystem.
//!
//! ## What lives here
//!
//! - `enable()`: turns on the BKPSRAM clock + opens the backup-domain
//!   write protection so writes to the slot succeed.
//! - `with_record()` / `with_record_mut()`: safe wrappers around the
//!   raw-pointer access to the `POSTMORTEM_SLOT` static. The static
//!   itself lives in the `.bkpsram` linker section (declared in
//!   `memory.x`); the section is `(NOLOAD)`, so no startup code zeroes
//!   it — that's the whole point: BKPSRAM contents survive across
//!   reboots so the post-mortem record from the *prior* boot is
//!   readable on the *next* boot.
//!
//! ## Concurrency / atomicity
//!
//! BKPSRAM is byte-addressable via a normal AHB4 access. There is no
//! erase cycle, no page boundary, no flash-style commit phase — a
//! `*mut u32` write is observable to the next reader as soon as the
//! AHB write posts. The post-mortem record's atomicity comes entirely
//! from the magic-last + CRC-over-body protocol implemented in
//! [`crate::postmortem::record`], not from any property of BKPSRAM
//! itself.
//!
//! ## D3 domain caveat
//!
//! BKPSRAM lives in the D3 domain. If the firmware ever puts D3 into
//! DSTOP / DSTANDBY for power saving, BKPSRAM access will hang the
//! AHB4 bus until the domain wakes. **This subsystem assumes D3 stays
//! in Run mode.** embassy-stm32's default `Config::default()` does not
//! enter D3 stop modes; if a future change adds explicit low-power
//! sleep, audit this module first.
//!
//! ## Hard rule for fault paths
//!
//! `with_record_mut` is callable from a `#[panic_handler]`, HardFault
//! exception, or PVD IRQ — it does no allocation, takes no locks, and
//! never awaits. The closure passed to it must respect the same
//! constraints.

use core::mem::{MaybeUninit, size_of};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::hal::pac;

use super::record::{PostmortemRecord, RECORD_SIZE};

const RECORD_WORDS: usize = RECORD_SIZE / size_of::<u32>();
const _: () = assert!(RECORD_SIZE % size_of::<u32>() == 0);

const PANIC_TEXT_MAGIC: u32 = 0x504E_4331; // "PNC1"
pub const PANIC_TEXT_MAX_BYTES: usize = 184;
const PANIC_TEXT_WORDS: usize = 2 + (PANIC_TEXT_MAX_BYTES / size_of::<u32>()); // 48 words = 192 B

/// The post-mortem record itself, pinned at a fixed address in the
/// BKPSRAM region declared in `memory.x`.
///
/// `MaybeUninit` is load-bearing: the linker section is `(NOLOAD)`, so
/// the runtime never initializes this static from program data. On
/// cold boot the contents are unspecified hardware state; on warm
/// boot they're whatever the prior session wrote. Either way, callers
/// must validate via [`super::record::is_valid`] (magic + CRC) before
/// trusting any field.
///
/// **ARMv7-M alignment note (STM32F745):** On STM32F745, BKPSRAM lives at
/// `0x4002_4000`, inside the `0x4000_0000..0x5FFF_FFFF` `Device` memory
/// region where Cortex-M7 faults (`UsageFault: UNALIGNED`) on any unaligned
/// access. Never hand a `&PostmortemRecord` or `&mut PostmortemRecord`
/// pointing directly into `.bkpsram` to general Rust code or `crc32fast`;
/// always copy to/from normal SRAM via 32-bit word `read_volatile` /
/// `write_volatile` helpers below.
#[unsafe(link_section = ".bkpsram")]
#[unsafe(no_mangle)]
static mut POSTMORTEM_SLOT: MaybeUninit<PostmortemRecord> = MaybeUninit::uninit();

/// Companion slot in `.bkpsram` storing formatted `file:line: message` bytes
/// from `#[panic_handler]`. Accessed strictly as aligned `u32` words.
#[unsafe(link_section = ".bkpsram")]
#[unsafe(no_mangle)]
static mut POSTMORTEM_PANIC_TEXT: MaybeUninit<[u32; PANIC_TEXT_WORDS]> = MaybeUninit::uninit();

/// Idempotency latch: `enable()` is safe to call more than once, but
/// the underlying RCC/PWR writes are best done once during init.
static ENABLED: AtomicBool = AtomicBool::new(false);

/// Bring BKPSRAM online. Call once during boot, before any
/// post-mortem reader/writer touches the static.
///
/// Steps:
/// 1. `RCC.AHB4ENR.BKPRAMEN = 1` — clocks the BKPSRAM controller so
///    AHB4 reads/writes complete instead of stalling.
/// 2. `PWR.CR1.DBP = 1` — disables backup-domain write protection.
///    Without this, writes to RTC backup registers + BKPSRAM are
///    silently dropped (the existing DFU magic at BKP0 set this
///    indirectly via embassy-stm32, but we make it explicit here).
/// 3. `PWR.CR2.BREN = 1` — enables the backup regulator. Without
///    this, BKPSRAM is **not retained across VBAT-only operation**;
///    cold-power-down survival depends on this bit. Setting it costs
///    a few µA of standby current — negligible on a flight controller.
///
/// Idempotent — repeated calls just observe `ENABLED == true` and
/// return.
pub fn enable() {
    if ENABLED.load(Ordering::Acquire) {
        return;
    }
    let rcc = pac::RCC;
    let pwr = pac::PWR;
    #[cfg(not(feature = "board_flywoof745"))]
    {
        rcc.ahb4enr().modify(|w| w.set_bkpsramen(true));
        let _ = rcc.ahb4enr().read();
        cortex_m::asm::dsb();
        pwr.cr1().modify(|w| w.set_dbp(true));
        pwr.cr2().modify(|w| w.set_bren(true));
        cortex_m::asm::dsb();
    }
    #[cfg(feature = "board_flywoof745")]
    {
        // STM32F7 Errata ES0290 §2.1.6: read back RCC clock-enable register
        // + barrier before accessing peripheral registers on slower APB1/AHB1.
        rcc.apb1enr().modify(|w| w.set_pwren(true));
        let _ = rcc.apb1enr().read();
        cortex_m::asm::dsb();

        pwr.cr1().modify(|w| w.set_dbp(true));
        for _ in 0..1024 {
            if pwr.cr1().read().dbp() {
                break;
            }
        }

        rcc.ahb1enr().modify(|w| w.set_bkpsramen(true));
        let _ = rcc.ahb1enr().read();
        cortex_m::asm::dsb();

        pwr.csr1().modify(|w| w.set_bre(true));
        cortex_m::asm::dsb();
    }
    // Mark `ENABLED` only AFTER clocks and `DBP` are live so an asynchronous
    // fault handler interrupting `enable()` mid-sequence still completes
    // hardware initialization rather than returning early on `ENABLED == true`.
    ENABLED.store(true, Ordering::Release);
}

#[inline(never)]
unsafe fn read_slot_words(dst: *mut u32) {
    let src = (&raw const POSTMORTEM_SLOT).cast::<u32>();
    let mut i = 0;
    while i < RECORD_WORDS {
        unsafe {
            dst.add(i).write(core::ptr::read_volatile(src.add(i)));
        }
        i += 1;
    }
}

#[inline(never)]
unsafe fn write_slot_words(src: *const u32) {
    let dst = (&raw mut POSTMORTEM_SLOT).cast::<u32>();
    unsafe {
        // Clear `magic` (word 0) first so any interrupted update reads invalid.
        core::ptr::write_volatile(dst, 0);
        cortex_m::asm::dsb();

        // Write payload body (words 2..RECORD_WORDS), then `crc32` (word 1).
        let mut i = 2;
        while i < RECORD_WORDS {
            core::ptr::write_volatile(dst.add(i), *src.add(i));
            i += 1;
        }
        core::ptr::write_volatile(dst.add(1), *src.add(1));
        cortex_m::asm::dsb();

        // Publish `magic` (word 0) last.
        core::ptr::write_volatile(dst, *src);
        cortex_m::asm::dsb();
    }
}

/// Read-only access to the post-mortem record. Caller's closure runs
/// against an aligned SRAM copy — safe on both Normal memory (STM32H7)
/// and Device-mapped BKPSRAM (STM32F7 `0x4002_4000`).
///
/// **Safety contract for the caller:** [`enable`] must have been
/// called already. Reading from BKPSRAM before clocking it produces a
/// bus fault.
pub fn with_record<R>(f: impl FnOnce(&PostmortemRecord) -> R) -> R {
    let mut local = MaybeUninit::<PostmortemRecord>::uninit();
    // SAFETY: `local` and `POSTMORTEM_SLOT` are both 8-byte aligned and
    // `RECORD_WORDS * 4 == size_of::<PostmortemRecord>()`.
    let r: &PostmortemRecord = unsafe {
        read_slot_words(local.as_mut_ptr().cast::<u32>());
        local.assume_init_ref()
    };
    f(r)
}

/// Mutable access to the post-mortem record. Caller's closure runs
/// against an aligned SRAM copy which is committed back via 32-bit
/// word-aligned volatile writes (magic-last).
///
/// **Hard rule:** the closure must not await, allocate, take a lock
/// that any post-mortem-emitting path also takes, or call back into
/// other post-mortem APIs. It is callable from a fault handler, but
/// only because it strictly avoids those things.
///
/// **Safety contract for the caller:** [`enable`] must have been
/// called already.
pub fn with_record_mut<R>(f: impl FnOnce(&mut PostmortemRecord) -> R) -> R {
    let mut local = MaybeUninit::<PostmortemRecord>::uninit();
    let res = unsafe {
        read_slot_words(local.as_mut_ptr().cast::<u32>());
        let r = local.assume_init_mut();
        let out = f(r);
        write_slot_words((r as *const PostmortemRecord).cast::<u32>());
        out
    };
    res
}

/// Convenience: write the entire record from a freshly-built value.
/// Finalizes CRC + magic in aligned SRAM, then commits to BKPSRAM via
/// 32-bit word-aligned volatile writes (magic-last) without needing to
/// read the prior slot contents first.
pub fn store_finalized(src: &PostmortemRecord) {
    let mut local = *src;
    super::record::finalize(&mut local);
    unsafe {
        write_slot_words((&local as *const PostmortemRecord).cast::<u32>());
    }
}

/// Save formatted panic text (`file:line: message`) into `.bkpsram` using
/// strictly 32-bit aligned volatile writes. Safe to call from `#[panic_handler]`.
pub fn store_panic_text(bytes: &[u8]) {
    enable();
    let len = bytes.len().min(PANIC_TEXT_MAX_BYTES);
    let dst = (&raw mut POSTMORTEM_PANIC_TEXT).cast::<u32>();
    unsafe {
        core::ptr::write_volatile(dst, 0);
        cortex_m::asm::dsb();
        core::ptr::write_volatile(dst.add(1), len as u32);
        let payload_words = PANIC_TEXT_WORDS - 2;
        let mut w = 0;
        while w < payload_words {
            let base = w * 4;
            let mut b = [0u8; 4];
            let mut k = 0;
            while k < 4 {
                if base + k < len {
                    b[k] = bytes[base + k];
                }
                k += 1;
            }
            core::ptr::write_volatile(dst.add(2 + w), u32::from_le_bytes(b));
            w += 1;
        }
        cortex_m::asm::dsb();
        core::ptr::write_volatile(dst, PANIC_TEXT_MAGIC);
        cortex_m::asm::dsb();
    }
}

/// Read and clear any panic text saved by the prior boot's `#[panic_handler]`.
pub fn take_panic_text(out: &mut [u8; PANIC_TEXT_MAX_BYTES]) -> Option<usize> {
    let ptr = (&raw mut POSTMORTEM_PANIC_TEXT).cast::<u32>();
    unsafe {
        let magic = core::ptr::read_volatile(ptr);
        if magic != PANIC_TEXT_MAGIC {
            return None;
        }
        let len = (core::ptr::read_volatile(ptr.add(1)) as usize).min(PANIC_TEXT_MAX_BYTES);
        let payload_words = (len + 3) / 4;
        let mut w = 0;
        while w < payload_words {
            let word = core::ptr::read_volatile(ptr.add(2 + w)).to_le_bytes();
            let base = w * 4;
            let mut k = 0;
            while k < 4 && base + k < len {
                out[base + k] = word[k];
                k += 1;
            }
            w += 1;
        }
        core::ptr::write_volatile(ptr, 0);
        cortex_m::asm::dsb();
        Some(len)
    }
}
