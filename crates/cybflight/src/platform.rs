// ---------------------------------------------------------------------------
// Platform init helpers
// ---------------------------------------------------------------------------

/// Enable Cortex-M7 instruction cache for flash acceleration.
pub fn enable_icache() {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    cp.SCB.enable_icache();
}

// ---------------------------------------------------------------------------
// Platform reset helpers
// ---------------------------------------------------------------------------

/// Trigger an immediate software system reset via the Cortex-M SCB AIRCR.
pub fn sys_reboot() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

/// Unlock RTC backup register write protection (`PWREN` + `DBP`) so writes to
/// `RTC_BKP0R` succeed both in `pre_init()` (where `DBP` resets to `0`) and in
/// `enter_dfu()`.
#[inline]
fn unlock_backup_domain() {
    use crate::hal::pac;
    #[cfg(feature = "board_flywoof745")]
    {
        pac::RCC.apb1enr().modify(|w| w.set_pwren(true));
        let _ = pac::RCC.apb1enr().read();
        cortex_m::asm::dsb();
    }
    pac::PWR.cr1().modify(|w| w.set_dbp(true));
    for _ in 0..1024 {
        if pac::PWR.cr1().read().dbp() {
            break;
        }
    }
    cortex_m::asm::dsb();
}

#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    // ── Reset-cause capture ────────────────────────────────────────────
    //
    // Read RCC.RSR + PWR.CSR1 into a `.uninit` static **before** any
    // other code runs. The flags are sticky, so doing this in `main`
    // is fine — but `bsp::init()` may touch RCC for clock setup, and
    // a future change that writes RMVF anywhere upstream would
    // silently lose the prior boot's cause. Capturing here guarantees
    // we own the cause regardless.
    //
    // The capture itself is ~10 cycles and uses no peripherals beyond
    // raw RCC/PWR reads (always-on domain). Unconditional — NOT gated
    // on `postmortem`: a board that dies in the field on a build
    // without the postmortem subsystem must still be able to answer
    // "was that an IWDG reset or a power dip?" via the `resetcause`
    // shell verb on the next boot.
    crate::reset_cause::capture();

    const DFU_MAGIC: u32 = 0xDEAD_D00D;
    // RTC backup register 0: 0x5800_4050 on STM32H743, 0x4000_2850 on STM32F745
    #[cfg(not(feature = "board_flywoof745"))]
    let (bkpr0, dfu_base) = (0x5800_4050 as *mut u32, 0x1FF0_9800usize);
    #[cfg(feature = "board_flywoof745")]
    let (bkpr0, dfu_base) = (0x4000_2850 as *mut u32, 0x1FF0_0000usize);
    unsafe {
        if bkpr0.read_volatile() == DFU_MAGIC {
            // System reset clears `PWREN` (F745) and `PWR.CR1.DBP` (F745 & H743),
            // so unlock the backup domain before clearing `RTC_BKP0R` — otherwise
            // `write_volatile(0)` is ignored and the MCU loops back into DFU.
            unlock_backup_domain();
            bkpr0.write_volatile(0);
            cortex_m::asm::dsb();
            // Jump to system memory DFU bootloader
            let sp = (dfu_base as *const u32).read();
            let entry = (dfu_base as *const u32).add(1).read();
            cortex_m::asm::bootstrap(sp as *const u32, entry as *const u32);
        }
    }
}

/// Jump directly to the STM32 system-memory USB DFU bootloader.
pub fn enter_dfu() -> ! {
    const DFU_MAGIC: u32 = 0xDEAD_D00D;
    #[cfg(not(feature = "board_flywoof745"))]
    let bkpr0 = 0x5800_4050 as *mut u32;
    #[cfg(feature = "board_flywoof745")]
    let bkpr0 = 0x4000_2850 as *mut u32;
    unlock_backup_domain();
    unsafe { bkpr0.write_volatile(DFU_MAGIC) };
    cortex_m::asm::dsb();
    sys_reboot()
}
