//! Fault handlers: custom `#[panic_handler]`, `HardFault` exception,
//! PVD brown-out IRQ. These three paths share a single contract:
//!
//! ## Hard rules (non-negotiable)
//!
//! - **No allocation.** No `String`, no `Vec`, no `Box`, no
//!   `format_args!` to a non-stack target.
//! - **No async.** No `await`, no `Signal::wait`, no embassy executor
//!   touchpoints. The thread executor is not running here.
//! - **No `Mutex<RefCell<T>>` or `critical_section::Mutex`.** Locks
//!   may already be held by the code that just panicked. The
//!   post-mortem record's atomicity comes from the magic-last + CRC
//!   protocol in [`super::record`], not from any lock.
//! - **No defmt logging during the actual commit.** A `defmt::error!`
//!   call panicking in the panic handler is a recursion source. We
//!   do log a single short message *before* the BKPSRAM commit so
//!   `probe-rs run` users see a clue, but the BKPSRAM commit itself
//!   is bracketed by `interrupt::disable` and is panic-free.
//! - **Re-entrancy guard.** A panic during the panic handler (e.g.
//!   a second fault while writing BKPSRAM) hits an `AtomicU8`
//!   depth counter; depth > 1 immediately `udf`s — no recursion,
//!   no infinite reset loop.

use core::fmt::Write as _;
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicU8, Ordering};

use cortex_m::peripheral::{DCB, SCB};

use super::bkpsram;
use super::record::{self, FatalKind, FatalSummary, PANIC_MSG_IDX_NONE};
use crate::hal::interrupt;

/// Re-entrancy guard. `0` = not in any fault handler, `1` = in one,
/// `>1` = recursive call (immediately abort).
static FAULT_DEPTH: AtomicU8 = AtomicU8::new(0);

#[inline(always)]
fn abort_recursive_fault() -> ! {
    if DCB::is_debugger_attached() {
        cortex_m::asm::udf();
    } else {
        SCB::sys_reset();
    }
}

struct PanicBuf<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl core::fmt::Write for PanicBuf<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let avail = self.buf.len().saturating_sub(self.len);
        let n = s.len().min(avail);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

/// `#[panic_handler]` registration. Replaces `panic_probe`'s default
/// when the `postmortem` feature is on.
///
/// Behavior:
/// 1. Disable interrupts so concurrent tasks can't observe a partly-
///    written record.
/// 2. Bump the re-entrancy counter; bail if we're already inside a handler.
/// 3. Format `file:line: message` into BKPSRAM via aligned `u32` writes
///    BEFORE defmt (so even a defmt failure cannot lose the panic message).
/// 4. Emit a short defmt warning so probe-rs / UART sees the panic before
///    the reset.
/// 5. Commit the fatal slot to BKPSRAM and hardware reset.
#[cfg(feature = "postmortem")]
#[panic_handler]
fn panic_handler(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    let depth = FAULT_DEPTH.fetch_add(1, Ordering::AcqRel);
    if depth >= 1 {
        abort_recursive_fault();
    }

    let lr_reg: u32;
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("mov {}, lr", out(reg) lr_reg, options(nomem, nostack, preserves_flags));
    }
    #[cfg(not(target_arch = "arm"))]
    {
        lr_reg = 0;
    }

    let (line, col) = match info.location() {
        Some(loc) => (loc.line(), loc.column()),
        None => (0, 0),
    };

    // Format compact `file:line:col: message` onto stack and commit to
    // `.bkpsram` via aligned u32 writes before touching defmt.
    {
        let mut raw = [0u8; bkpsram::PANIC_TEXT_MAX_BYTES];
        let mut w = PanicBuf {
            buf: &mut raw,
            len: 0,
        };
        if let Some(loc) = info.location() {
            let f = loc.file();
            let short = if f.len() > 64 {
                // Step forward to a UTF-8 char boundary in the last 64 bytes.
                let mut start = f.len() - 64;
                while start < f.len() && !f.is_char_boundary(start) {
                    start += 1;
                }
                &f[start..]
            } else {
                f
            };
            let _ = write!(w, "{}:{}:{}: ", short, loc.line(), loc.column());
        }
        let _ = write!(w, "{}", info.message());
        let written = w.len;
        bkpsram::store_panic_text(&raw[..written]);
    }

    let summary = FatalSummary {
        kind: FatalKind::Panic as u8,
        _pad0: [0; 3],
        pc: 0,
        lr: lr_reg,
        psr: 0,
        cfsr: 0,
        hfsr: 0,
        mmfar: line,
        bfar: col,
        panic_msg_idx: PANIC_MSG_IDX_NONE,
        _pad1: [0; 31],
    };
    commit_fatal(summary);

    // Best-effort defmt log after BKPSRAM is safely committed.
    defmt::error!("postmortem: panic (info={})", defmt::Display2Format(info));

    SCB::sys_reset();
}

/// Stub used by builds without the `postmortem` feature so the linker
/// can still find a panic handler (provided by `panic_probe`). When
/// the feature is off, `main.rs` keeps `use panic_probe as _;` and
/// this module's symbol isn't pulled in.
#[cfg(not(feature = "postmortem"))]
const _: () = ();

/// HardFault exception. Cortex-M-RT calls this with the stacked
/// exception frame; we extract PC/LR/PSR and the SCB fault status
/// registers.
///
/// Constraints same as panic handler — no async, no alloc, no
/// Mutex. Runs in handler mode with interrupts disabled by the
/// hardware (HardFault is escalated above the BASEPRI threshold).
#[cfg(feature = "postmortem")]
#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    let depth = FAULT_DEPTH.fetch_add(1, Ordering::AcqRel);
    if depth >= 1 {
        abort_recursive_fault();
    }

    // SAFETY: we own the SCB pointer for the duration of this
    // handler — no other code is running.
    let scb = unsafe { &*SCB::PTR };
    let cfsr = scb.cfsr.read();
    let hfsr = scb.hfsr.read();
    let mmfar = scb.mmfar.read();
    let bfar = scb.bfar.read();

    // Skip defmt here — a HardFault during a defmt::error! is the
    // textbook "panic in panic" recursion. Operators see "boot
    // recovered HardFault" on next boot via the post-mortem mirror.

    let summary = FatalSummary {
        kind: FatalKind::HardFault as u8,
        _pad0: [0; 3],
        pc: frame.pc(),
        lr: frame.lr(),
        psr: frame.xpsr(),
        cfsr,
        hfsr,
        mmfar,
        bfar,
        panic_msg_idx: PANIC_MSG_IDX_NONE,
        _pad1: [0; 31],
    };
    commit_fatal(summary);

    SCB::sys_reset();
}

/// Unhandled interrupt / exception handler. Without this, `cortex-m-rt`'s
/// default handler loops forever in handler mode, starving the thread-mode
/// watchdog feeder and showing up 500 ms later as an opaque `IWDG` reset.
#[cfg(feature = "postmortem")]
#[cortex_m_rt::exception]
unsafe fn DefaultHandler(irqn: i16) -> ! {
    cortex_m::interrupt::disable();
    let depth = FAULT_DEPTH.fetch_add(1, Ordering::AcqRel);
    if depth >= 1 {
        abort_recursive_fault();
    }

    let mut raw = [0u8; bkpsram::PANIC_TEXT_MAX_BYTES];
    let mut w = PanicBuf {
        buf: &mut raw,
        len: 0,
    };
    let _ = write!(w, "DefaultHandler: unhandled exception/IRQ irqn={}", irqn);
    let written = w.len;
    bkpsram::store_panic_text(&raw[..written]);

    let summary = FatalSummary {
        kind: FatalKind::Panic as u8,
        _pad0: [0; 3],
        pc: 0,
        lr: irqn as i32 as u32,
        psr: 0,
        cfsr: 0,
        hfsr: 0,
        mmfar: irqn as i32 as u32,
        bfar: 0,
        panic_msg_idx: PANIC_MSG_IDX_NONE,
        _pad1: [0; 31],
    };
    commit_fatal(summary);

    SCB::sys_reset();
}

/// PVD brown-out interrupt path. The PVD trips ~2.7 V (configurable
/// via PWR.CR1.PLS), well above the BOR threshold (~1.85 V), giving
/// us ~2–10 ms of headroom — depending on the board's bulk
/// capacitance — to commit the post-mortem record before BOR pulls
/// the rug.
///
/// Wired as the EXTI16 (PVD) interrupt handler. The user must:
/// 1. `pwr.cr1().modify(|w| w.set_pls(<threshold>))`
/// 2. `pwr.cr1().modify(|w| w.set_pvden(true))`
/// 3. `exti.imr1().modify(|w| w.set_line(16, true))`
/// 4. `exti.rtsr1().modify(|w| w.set_line(16, true))`  (rising edge =
///    voltage falling below threshold)
/// 5. NVIC unmask EXTI16.
///
/// Configuration lives in [`enable_pvd_brownout`]. The handler itself
/// is registered via `#[interrupt]`.
///
/// **Minimal-diff principle:** the steady-state ring is kept up to
/// date by the post-mortem task, so all this handler has to do is
/// record the fatal kind + finalize. Realistic budget: 2–3 µs vs. a
/// 2–10 ms PVD-to-BOR window — three orders of magnitude of margin.
#[cfg(all(feature = "postmortem", not(feature = "board_flywoof745")))]
#[interrupt]
fn PVD_AVD() {
    pvd_irq_body();
}

#[cfg(all(feature = "postmortem", feature = "board_flywoof745"))]
#[interrupt]
fn PVD() {
    pvd_irq_body();
}

#[cfg(feature = "postmortem")]
fn pvd_irq_body() {
    let depth = FAULT_DEPTH.fetch_add(1, Ordering::AcqRel);
    if depth >= 1 {
        abort_recursive_fault();
    }

    // Clear the EXTI pending bit — without this the IRQ would
    // re-fire on return. EXTI16 lives in bank 0 on STM32H7 (lines
    // 0..31).
    let exti = crate::hal::pac::EXTI;
    exti.pr(0).write(|w| w.set_line(16, true));

    // Confirm the comparator still sees VDD below threshold. A real
    // brownout stays asserted all the way down to BOR; a glitch (EMI
    // spike, marginal supply ringing) has already recovered by the
    // time we get here. Committing a fatal record and parking in
    // `wfi` on a glitch turns a survivable transient into a
    // guaranteed IWDG reset — treat it as spurious and return to the
    // running system instead.
    if !crate::hal::pac::PWR.csr1().read().pvdo() {
        FAULT_DEPTH.store(0, Ordering::Release);
        return;
    }

    let summary = FatalSummary {
        kind: FatalKind::Brownout as u8,
        _pad0: [0; 3],
        pc: 0,
        lr: 0,
        psr: 0,
        cfsr: 0,
        hfsr: 0,
        mmfar: 0,
        bfar: 0,
        panic_msg_idx: PANIC_MSG_IDX_NONE,
        _pad1: [0; 31],
    };
    commit_fatal(summary);

    // Park here until BOR or HW reset. WFI cuts power to the core
    // until any pending IRQ wakes us — the only thing that should
    // wake us is BOR, which doesn't take this path.
    loop {
        cortex_m::asm::wfi();
    }
}

/// Enable the PVD interrupt at the configured threshold. Call once
/// during init. The threshold-selection bits (PLS) on STM32H7
/// support several discrete levels; we pick the highest available
/// (~2.85 V on H7) to maximize the time-to-BOR window.
///
/// Idempotent: re-enabling is harmless. **Must be called after
/// clocks are configured** (PWR clock comes from APB1).
#[cfg(feature = "postmortem")]
pub fn enable_pvd_brownout() {
    use crate::hal::pac;
    let pwr = pac::PWR;
    let exti = pac::EXTI;

    #[cfg(not(feature = "board_flywoof745"))]
    {
        // Threshold = level 6 (~2.85 V on STM32H7).
        // NOT 0b111: on H7 that selects external PVD_IN (PB7 = I2C1 SDA).
        pwr.cr1().modify(|w| {
            w.set_pls(0b110);
            w.set_pvde(true);
        });
        exti.imr(0).modify(|w| w.set_line(16, true));
        exti.rtsr(0).modify(|w| w.set_line(16, true));
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::PVD_AVD);
        }
    }

    #[cfg(feature = "board_flywoof745")]
    {
        // Ensure PWR APB1 clock is gated on before touching PWR_CR1/CSR1.
        pac::RCC.apb1enr().modify(|w| w.set_pwren(true));
        let _ = pac::RCC.apb1enr().read();
        cortex_m::asm::dsb();
        // On STM32F745 (RM0385 §4.4.1), PLS=0b100 selects 2.6 V — comfortably
        // below normal 3.3 V USB LDO rail sag while leaving plenty of margin
        // above the ~2.1 V BOR reset threshold.
        pwr.cr1().modify(|w| {
            w.set_pls(0b100);
            w.set_pvde(true);
        });
        // Wait ~100 µs (> 50 µs max t_PVD comparator startup time) before
        // clearing any startup glitch on EXTI16 and checking PVDO.
        cortex_m::asm::delay(crate::bsp::SYSCLK_HZ / 10_000);
        exti.rtsr(0).modify(|w| w.set_line(16, true));
        exti.ftsr(0).modify(|w| w.set_line(16, false));
        exti.pr(0).write(|w| w.set_line(16, true));
        if !pwr.csr1().read().pvdo() {
            exti.imr(0).modify(|w| w.set_line(16, true));
            unsafe {
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::PVD);
            }
        } else {
            defmt::warn!("PVD: VDD already below 2.6V threshold at boot; skipping PVD IRQ arm");
        }
    }
}

// ── Internal commit helper ──────────────────────────────────────────────

/// Write `summary` into the BKPSRAM record's fatal slot, refresh the
/// header's CRC + magic. Single point of "actually finalize" so the
/// three handler paths all use the same memory ordering and
/// validation rules.
fn commit_fatal(summary: FatalSummary) {
    bkpsram::enable();
    bkpsram::with_record_mut(|r| {
        if !record::is_valid(r) {
            *r = record::PostmortemRecord::ZERO;
            r.header.boot_count = 1;
            r.header.reset_cause = crate::reset_cause::read();
        }
        r.fatal = summary;
        // Refresh uptime if monotonic time is available. We use
        // raw `embassy_time` access via a public fn; if anything
        // goes wrong (e.g. timer not initialized in early boot),
        // we leave the field at whatever the postmortem_task last
        // wrote.
        if let Some(now_ms) = monotonic_ms() {
            r.header.uptime_ms = now_ms;
        }
        record::finalize(r);
    });
}

/// Best-effort monotonic-time read. Returns `None` if `embassy_time`
/// isn't initialized yet (the post-mortem subsystem comes online
/// before the executor in `main`).
fn monotonic_ms() -> Option<u32> {
    // `Instant::now` is safe to call from any context post-init. Pre-
    // init it would still return a value (zero) — embassy's tick
    // counter is initialized by the bsp clock setup.
    Some(embassy_time::Instant::now().as_millis() as u32)
}
