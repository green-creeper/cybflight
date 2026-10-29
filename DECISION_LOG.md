# Flywoo GOKU GN 745 V3 (`STM32F745VG`) Bring-Up Decision Log

This document records the engineering decisions, root-cause analyses, code changes, and live hardware verification outcomes during the Cybflight bring-up of the **Flywoo GOKU GN 745 V3 AIO (`STM32F745VGT6`)** flight controller (`vehicles/goku_f745.yaml`).

---

## Summary Table

| # | Symptom / Issue | Root Cause | Decision & Fix | Outcome |
|---|---|---|---|---|
| 1 | Silent boot loop & `reboot --dfu` failed to enter STM32 ROM DFU | `PWR_CR1.DBP` was locked when writing DFU magic to `RTC_BKP0R`, and repeated early crashes rebooted back into crashing code before USB CDC enumerated | Unlock backup domain (`RCC_APB1ENR.PWREN` + `PWR_CR1.DBP` + readback barrier) before `RTC_BKP0R` write; add BKPSRAM early-crash counter (`>=2` crashes `<5 s`) entering USB Safe Mode | **Verified on hardware** — `reboot --dfu` reliably enumerates `0483:df11`, and Safe Mode preserves USB recovery shell across early faults |
| 2 | `HardFault` at ~3.35 s uptime during postmortem heartbeat | STM32F7 maps Backup SRAM (`0x4002_4000`) in the `Peripheral` (`Device`) MPU region where Cortex-M7 strictly traps any unaligned access (`ldrh.w` / `strh.w`) | Replaced packed-struct byte/halfword pointer casts in `bkpsram.rs` with 32-bit word-aligned `read_volatile`/`write_volatile` bounce buffers + RCC readback barrier | **Verified on hardware** — eliminated `BKPSRAM` fault and preserved crash telemetry across resets |
| 3 | `just flash` compiled & flashed `board_sakurah743` (`714 KB` H7 image) onto F745 | Repository `.env` defaulted to `VEHICLE=sakura_vicon` (`STM32H743`), whose clock/memory layout cannot boot on `STM32F745VG` | Updated `.env` default `VEHICLE=goku_f745` (`board_flywoof743`/`flywoof745`) | **Verified on hardware** — `just flash` targets `goku_f745` (`STM32F745VG`) and boots USB CDC |
| 4 | `HardFault` (`UNDEFINSTR`, `cfsr=0x00010000`, `pc=0x080536ea`) at `415 ms` | `thumbv7em-none-eabihf` defaults to `FPv5-D16` (double-precision FPU), emitting `vcvt.f64.f32` (`f64` math in barometer/ESKF/formatting), whereas `STM32F745VG` only implements single-precision `FPv5-SP-D16` | Added `"-C", "target-feature=-fp64"` to `[target.thumbv7em-none-eabihf]` in `.cargo/config.toml` so `f32` stays hardware FPU and `f64` uses software float | **Verified on hardware & disassembly** — `llvm-objdump` confirmed `0` `.f64` instructions across 10,847 VFP instructions; firmware boots past `415 ms` |
| 5 | `SPI4` IMU bus overclocked to `27 MHz` (`> 20 MHz` MPU-6000 max rating) | `embassy-stm32` integer prescaler calculation on `54 MHz` `APB2` (`54 / 20 = 2`) selected `Br::DIV2 = 27 MHz` when `20 MHz` was requested | Set `SPI4` fast frequency in `board_init/flywoof745.rs` to `Hertz(13_500_000)` (`Br::DIV4` for MPU-6000/ICM-42688-P) and `Hertz(6_750_000)` (`Br::DIV8` for BMI270) | **Verified on Flash #1** (`WHO_AM_I` + live IMU); `13.5 MHz` queued for Flash #2 |
| 6 | `IndependentWatchdog` (`0xc8000000`) reset at `4,700 ms` + `imu1`/`baro1` `no data (timeout)` | (a) `cybflight-gs` auto-sends `stream att on`, `stream imu1 on`, `stream power on`, `stream dshot on` on connect; `msg_stream_task` formatted 8 kHz `IMU_1` + 3.7 kHz `DSHOT_TELEMETRY` on the Thread Executor with no rate limit or yield, starving `iwdg_feed_task` (`500 ms` timeout). (b) Consecutive early `IWDG` resets tripped Safe Mode on Boot 8, which intentionally skips `board_init::init` (hence `imu1`/`baro1` timeout) | Rewrote `msg_stream_task` in `usb_serial.rs` to sleep (`50 ms`) when disabled, coalesce burst frames, and enforce `yield_now()` + `20 ms` (`<=50 Hz`) pacing; switched `imu1` stream to `IMU_1_DECIM`; added `version`, `stream off`, and indoor Mahony `att` fallback; preferred `/dev/cu.*` on macOS in `cybflight-gs` | **Verified on Flash #1** (`>16.5 min` continuous uptime with `cybflight-gs` streaming, `0` IWDG resets) |
| 7 | Incorrect IMU chip & diamond airframe orientation (`Cw0Deg` vs physical board) | `bsp/flywoof745` defaulted `gyro1_align` to `Cw0Deg` and `InstallParams` lacked arbitrary yaw mounting (`imu_rpy_*`), whereas GOKU GN 745 V3 mounts MPU-6000 at `Cw270Deg` relative to silkscreen arrow and this quad mounts the AIO arrow Rear-Left (`AHRS_ORIENTATION=5` = `ROTATION_YAW_225` = `+135°` FLU yaw) | Changed `gyro1_align` to `SensorAlign::Cw270Deg` in `bsp/flywoof745`; added `imu_rpy_r/p/y` (`FirmwareConfig::VERSION` 57) precomputed `Matrix3<f32>` in `ImuReader`; set `imu_rpy_y: 135.0` in `vehicles/goku_f745.yaml` | **Verified algebraically & against `SERVO1..4_FUNCTION = 36,33,34,35`** |
| 8 | Tight async retry risk on floating/noisy UART RX & silent unhandled IRQs | `crsf_task`, `ghst_task` (`P10` high-priority executor), and `gps_task` looped immediately on `uart.read()` framing/overrun errors without backoff; unhandled IRQs spun silently into an opaque `IWDG` reset | Added `Timer::after_millis(2).await` backoff on UART read errors in `rc.rs` and `gps.rs`; added `DefaultHandler(irqn)` in `postmortem/fault.rs` recording exact IRQ number to BKPSRAM | **Verified on Flash #1** (`0` unhandled IRQs or executor stalls) |
| 9 | Host shell automation & port-contention diagnostics | Interactive `picocom`/`tio` cannot run non-interactively in automated verification loops, and background `cybflight-gs` holds `/dev/tty.usbmodem*` with `TIOCEXCL` (`EBUSY`) | Built zero-dependency `tools/fc_shell.py` (`termios` raw mode, `DTR|RTS`, `/dev/cu.*` auto-discovery, `lsof` holder reporting, `--monitor`, `--dfu`) | **Verified on host & live FC** |
| 10 | `imurate` reported `6506.0 Hz (-18.67%); lost since boot: 12663` on Flash #1 | `Mpu6x00` configured `REG_INT_PIN_CFG = 0x10` (50 µs pulse) and waited with `wait_for_rising_edge()`; whenever `indi_task` (sharing `EXECUTOR_CTRL` at `P10`) ran past an EXTI edge, that 125 µs sample was missed | Changed `REG_INT_PIN_CFG` to `0x30` (`LATCH_INT_EN | INT_RD_CLEAR`), switched `Mpu6x00::read()` to `wait_for_high()`, and raised MPU-6000 `SPI4` fast clock from `6.75 MHz` to `13.5 MHz` (`Br::DIV4`) | **Pending Flash #2 verification** |
| 11 | `indistat` reported `step avg 0.0 us, max 0.0 us` despite `14111 iters` on Flash #1 | Cortex-M7 r0p1 (`STM32F745`) locks CoreSight DWT registers at reset (`DWT->LSR = 0x03`), silently ignoring `cp.DWT.enable_cycle_counter()` unless `0xC5AC_CE55` is written to `DWT->LAR` (`0xE000_1FB0`) | Unlocked `DWT->LAR` (`0xE000_1FB0 = 0xC5AC_CE55` + `dsb()`) before `enable_cycle_counter()` in `indi_task.rs` | **Pending Flash #2 verification** |
| 12 | `gpshealth` reported `GPS: init timed out` on Flywoo GM10 Nano V3 (u-blox M10) | `ublox.rs` only sent legacy `UBX-CFG-MSG` (`0x06 0x01`), which u-blox M10 SPG 5.x deprecates (`ACK-NAK`) in favor of `UBX-CFG-VALSET` (`0x06 0x8A`), and only tried `115200` baud | Added `UBX-CFG-VALSET` (`0x06 0x8A`, RAM layer, `CFG_UART1OUTPROT_UBX` + `CFG_MSGOUT_UBX_NAV_PVT_UART1`) with automatic fallback to `UBX-CFG-MSG`, framing-error resilience, and `[115200, 230400, 38400, 57600, 9600]` auto-baud probe + `GPS_PROBE_RX_BYTES` diagnostic | **Pending Flash #2 verification** |
| 13 | In `cybflight-gs`, tilting physical drone Left tilted the 3D CAD model Front | Fusion 360 STL generator (`build_drone_fusion360.py`) built the CAD model in Right-Front-Up (`+X_stl = Right`, `+Y_stl = Front`, `XT30` at `y = -41.5`), whereas `parse_binary_stl` in `drone_3d.rs` loaded `(x_stl, y_stl, z_stl)` directly into FLU (`+X = Front`, `+Y = Left`), placing the CAD model's visual Nose along FLU `+Y` (Left) | Applied proper `det = +1` `-90°` Z rotation `(y_mm, -x_mm, z_mm) * 0.001` in `parse_binary_stl` (`drone_3d.rs`) and verified motor corners against `goku_ardu_params.params` (`SERVO1..4_FUNCTION = 36,33,34,35` -> `PB0=RR, PB1=FR, PE9=RL, PE11=FL`) | **Verified via unit test & `release-host` build** |
| 14 | Slow stationary yaw drift (`~-3.1 deg/s`) in `cybflight-gs` attitude preview | Uncalibrated MEMS MPU-6000 had stationary Z-gyro zero-rate offset `-0.0543 rad/s` (`-3.11 deg/s`, matching ArduPilot's `INS_GYROFFS_Z = +0.0496` in FRD on this board), and IMU-only Mahony (`mag = None`) cannot observe yaw bias around gravity | Added Betaflight-style pure-`f32` stationary startup gyro calibration (`GyroStartupCal`: 1.0 s window, `std <= 0.05 rad/s`, `|mean| <= 0.35 rad/s`, blocked when armed) in `sensors/imu.rs` and surfaced calibrated bias in `imurate` | **Pending Flash #2 verification** |

---

## Detailed Engineering Log

### Issue 1: Silent Boot Loop & Software DFU Entry (`reboot --dfu`) Failure
- **Observed Symptom**: Flashing broken early firmware left the board unresponsive without USB CDC enumeration, and `reboot --dfu` could not enter the STM32 system ROM DFU bootloader (`0483:df11`).
- **Root Cause**:
  1. On STM32F7, `RTC_BKP0R` lives in the backup domain, which ignores writes after reset unless `RCC_APB1ENR.PWREN` and `PWR_CR1.DBP` are enabled with a readback synchronization barrier before `SCB::sys_reset()`.
  2. Early crashes (`< 5 s` uptime) looped endlessly before USB CDC could enumerate or accept shell commands.
- **What Was Done**:
  - Added `unlock_backup_domain()` in `crates/cybflight/src/platform.rs` prior to writing `DFU_BOOT_MAGIC` (`0xDF59_B007`) to `RTC_BKP0R`, paired with `pre_init` interception in `main.rs` jumping cleanly into `0x1FF0_0000` (STM32F7 system memory).
  - Added early-crash detection in `crates/cybflight/src/postmortem/recovery.rs` (`consecutive_early_crashes >= 2` within `< 5,000 ms` uptime) so `main.rs` boots a minimal **USB Safe Mode** (`postmortem show`, `postmortem clear`, `reboot`, `reboot --dfu`) instead of crash-looping.
- **Outcome**: **Verified on hardware**. Both `reboot --dfu` and automatic Safe Mode recovery functioned cleanly during subsequent `415 ms` and `4,700 ms` crashes.

---

### Issue 2: `HardFault` at `3.35 s` Uptime (`BKPSRAM` Unaligned Access)
- **Observed Symptom**: Board booted USB CDC, ran for ~3.35 seconds, and hit a `HardFault` inside `postmortem` state persistence.
- **Root Cause**: STM32F7 Backup SRAM (`0x4002_4000`) sits in the Cortex-M7 `0x4000_0000` `Peripheral` (`Device`) default memory map region. Unlike Normal SRAM, Cortex-M7 strictly faults (`UNALIGNED` UsageFault escalated to HardFault) on any halfword/unaligned load or store (`ldrh.w` / `strh.w`) emitted by LLVM for packed/byte struct accesses.
- **What Was Done**:
  - Rewrote `crates/cybflight/src/postmortem/bkpsram.rs` to copy `BootRecord` slots exclusively through a `#[repr(C, align(4))]` RAM bounce buffer using 32-bit word-aligned `core::ptr::read_volatile` and `core::ptr::write_volatile`, plus an RCC `BKPSRAMEN` readback barrier + `cortex_m::asm::dsb()`.
- **Outcome**: **Verified on hardware**. `postmortem show` reliably reads and writes prior boot records across `Software` and `IndependentWatchdog` resets.

---

### Issue 3: `just flash` Defaulted to `board_sakurah743`
- **Observed Symptom**: Running `just flash` built `--features board_sakurah743,rx_crsf,est_eskf,est_pos_mocap,outer_mpc,postmortem` and downloaded a 714 KB H743 binary to `0x08000000`, resulting in no USB shell.
- **Root Cause**: `Justfile` reads `VEHICLE` from `.env` when no positional argument is passed, and `.env` was still set to `VEHICLE=sakura_vicon`.
- **What Was Done**: Updated `/Users/andrii/Personal/autopilots/cybflight/.env` to `VEHICLE=goku_f745`.
- **Outcome**: **Verified on hardware**. `just flash` now builds and flashes `goku_f745` (`STM32F745VG`) by default.

---

### Issue 4: `HardFault` (`UNDEFINSTR`, `cfsr=0x00010000`) at `415 ms` Uptime
- **Observed Symptom**:
  ```text
  prior boot 2 (exit_reset_cause=0x98000000 Software, fw=0x6e6b6e75, uptime=415ms)
    fatal: HardFault pc=0x080536ea lr=0x08013611 psr=0x81000000
           cfsr=0x00010000 hfsr=0x40000000
  ```
- **Root Cause**: `cfsr = 0x00010000` is `UFSR.UNDEFINSTR`. Disassembling `pc=0x080536ea` showed `vcvt.f64.f32 d9, s0` inside `spl06_task` (`400 ms` startup delay + first baro read = `415 ms`!). Rust's `thumbv7em-none-eabihf` target enables `FPv5-D16` (64-bit hardware FPU, as on STM32F765/H743), whereas **STM32F745VG** only has **single-precision `FPv5-SP-D16`**, trapping every `.f64` VFP opcode as an Undefined Instruction.
- **What Was Done**:
  - Added `"-C", "target-feature=-fp64"` to `[target.thumbv7em-none-eabihf]` in `/Users/andrii/Personal/autopilots/cybflight/.cargo/config.toml`. Single-precision `f32` control/estimator math continues using hardware FPU registers (`s0..s31`), while occasional `f64` operations compile to soft-float intrinsics.
- **Outcome**: **Verified on hardware & disassembly**. `llvm-objdump` confirmed **0** `.f64` instructions across 10,847 VFP instructions, and Boot 7 ran well past `415 ms` (all sensors initialized).

---

### Issue 5: `SPI4` 27 MHz Overclock on Onboard MPU-6000
- **Observed Symptom**: Potential corrupted WHO_AM_I / gyro register transfers after switching `SPI4` from `1 MHz` init clock to fast operational clock.
- **Root Cause**: On STM32F745 at `216 MHz` `SYSCLK`, `APB2` runs at `54 MHz`. `embassy-stm32`'s SPI prescaler calculation uses integer division (`54_000_000 / 20_000_000 = 2`), selecting `Br::DIV2 = 27 MHz` — **35% above** the MPU-6000 datasheet maximum of `20 MHz`.
- **What Was Done**:
  - Updated `crates/cybflight/src/board_init/flywoof745.rs` to set `fast_cfg.frequency = Hertz(6_750_000)` (`54 MHz / 8 = 6.75 MHz`, well within MPU-6000's 20 MHz and BMI270's 10 MHz ratings) and `Hertz(13_500_000)` (`Br::DIV4`) for ICM-42688-P.
- **Outcome**: Pending live post-flash check (`imu1` / `imurate`).

---

### Issue 6: `IndependentWatchdog` (`0xc8000000`) Reset at `4,700 ms` & Safe Mode `imu1`/`baro1` Timeout
- **Observed Symptom**:
  ```text
  postmortem: [SAFE MODE ACTIVE - prior boot crashed early; run 'reboot' to retry normal boot]
  prior boot 7 (exit_reset_cause=0xc8000000 IndependentWatchdog, fw=0x6e6b6e75, uptime=4700ms)
    prior_ev[1] @200ms kind=0x07 data=0x00000000
    prior_ev[2] @1700ms kind=0x03 data=0x00000002
  ```
  And running `imu1` / `baro1` on Boot 8 printed `no data (timeout)`.
- **Root Cause**:
  1. Host process `cybflight-gs` (`PID 92897`) was running in the background watching `/dev/tty.usbmodem*`. As soon as USB CDC enumerated (~4.2 s), `cybflight-gs` connected and sent `version`, `stream att on`, `stream imu1 on`, `stream power on`, `stream dshot on`, `health`, `param list`.
  2. `msg_stream_task` subscribed directly to `IMU_1` (**8,000 Hz**) and `DSHOT_TELEMETRY` (**3,700 Hz**) on the cooperative Thread Executor with no rate limit and no `yield_now()`. Formatting 6 floats via `core::fmt::write` took `>125 µs` on 216 MHz F745, so a new `IMU_1` message was **always** waiting in the `PubSubChannel` queue (`Poll::Ready` on 100% of iterations). `msg_stream_task` never returned `Poll::Pending` to the Embassy Thread Executor, starving `iwdg_feed_task` (`100 ms` period, `500 ms` hardware timeout) and triggering `IndependentWatchdog` 500 ms later (`4,200 ms + 500 ms = 4,700 ms`).
  3. Because Boot 7 reset at `4,700 ms` (`< 5,000 ms` early-crash threshold), Boot 8 entered **Safe Mode**, which intentionally skips `board_init::init` (no SPI/I2C/DMA tasks spawned) so the recovery shell stays alive — explaining why `imu1` and `baro1` returned `no data (timeout)` in Safe Mode.
- **What Was Done**:
  - Rewrote `msg_stream_task` in `crates/cybflight/src/usb_serial.rs`:
    - Sleeps (`50 ms`) while a stream flag is `false` instead of waking 8,000 times/s to discard messages (eliminating ~11,700/s idle `CriticalSection` lock cycles).
    - Drains stale queued samples on `false -> true` enable transition and coalesces burst updates (`try_next_message_pure()`).
    - Unconditionally calls `embassy_futures::yield_now().await` and `Timer::after_millis(20).await` (`<= 50 Hz` max per stream) after every frame so `iwdg_feed_task` and shell commands can never be starved.
  - Switched `imu1_stream_task` from 8 kHz `IMU_1` to the ~1 kHz filtered mirror `IMU_1_DECIM`.
  - Added `"version"` and `"stream off"` commands (plus `disable_all_streams()`) expected by `cybflight-gs`, and added an indoor bench fallback on `"att"` / `att_stream_task` (`MAHONY_ATTITUDE` when ESKF is `AwaitingExteroceptive`) without modifying `VEHICLE_ATTITUDE`.
  - Updated `crates/cybflight_gs/src/backend/usb_serial.rs` to prefer `/dev/cu.*` over `/dev/tty.*` on macOS.
- **Outcome**: Pending live post-flash 65 s full-stream stress test.

---

### Issue 7: IMU Chip & Diamond Airframe Orientation Alignment
- **Observed Symptom**: `crates/bsp/flywoof745/src/lib.rs` had `gyro1_align: SensorAlign::Cw0Deg` and `vehicles/goku_f745.yaml` had no board yaw mounting offset, which would feed rotated gyro/accel axes into INDI and ESKF.
- **Root Cause**:
  1. Cross-referencing ArduPilot's `FlywooF745/hwdef.dat` (`IMU Invensense SPI:mpu6000 ROTATION_YAW_180` on top of `AP_InertialSensor_Invensense.cpp`'s raw-to-FRD mapping `(ry, rx, -rz)`) proves board FRD is `(-ry, -rx, -rz)`, which in Cybflight's FLU convention `(F, L, U) = (F, -R, -D)` is **`(-ry, +rx, +rz)` = `SensorAlign::Cw270Deg`**.
  2. Proven ArduPilot flight parameters from this exact quad (`goku_ardu_params.params`, `1,193 s` flight log) specify `AHRS_ORIENTATION = 5` (`ROTATION_YAW_225`), meaning the diamond AIO board's silkscreen arrow points Rear-Left (`225°` CW from Forward = `+135°` CCW yaw in FLU), yielding body FLU `((ry - rx)/√2, -(rx + ry)/√2, +rz)`.
- **What Was Done**:
  - Set `gyro1_align: SensorAlign::Cw270Deg` in `crates/bsp/flywoof745/src/lib.rs`.
  - Added `imu_rpy_r`, `imu_rpy_p`, `imu_rpy_y` (`[f32; 3]`, `FirmwareConfig::VERSION` bumped `56 -> 57`) to `InstallParams` in `crates/cybflight_core/src/params.rs` and precomputed the 3D body-from-sensor rotation matrix `R = R_z(yaw) * R_y(pitch) * R_x(roll)` once at boot in `ImuReader::new` (`crates/cybflight/src/sensors/imu.rs`).
  - Configured `imu_rpy_y: 135.0` in `vehicles/goku_f745.yaml` and aligned motor/battery/sensor parameters against `goku_ardu_params.params`.
- **Outcome**: Unit-tested (`327 passed` in `cybflight-core`, `52 passed` in `cybflight-build-tools`); pending live bench orientation check.

---

### Issue 8: Defensive UART Error Backoff & `DefaultHandler` Exception Telemetry
- **Observed Symptom**: Potential `P10` high-priority executor starvation if an unpowered/noisy RC or GPS UART line continuously raises framing/overrun/noise errors; potential opaque `IWDG` if an unexpected IRQ fires without a handler.
- **What Was Done**:
  - Added `Timer::after_millis(2).await` on `Err(_)` in `crsf_task` and `ghst_task` (`crates/cybflight/src/sensors/rc.rs`) and `gps_task` (`crates/cybflight/src/sensors/gps.rs`).
  - Added `#[cortex_m_rt::exception] unsafe fn DefaultHandler(irqn: i16) -> !` in `crates/cybflight/src/postmortem/fault.rs` to record `FaultKind::UnhandledIrq` + `irqn` into BKPSRAM before resetting.
- **Outcome**: **Verified on Flash #1** (`> 16.5 min` continuous operation, `0` unhandled IRQs or executor stalls).

---

### Issue 9: Non-Interactive USB CDC Shell Automation (`tools/fc_shell.py`)
- **Observed Symptom**: Standard serial terminals (`picocom`, `tio`) require an interactive PTY, and background `cybflight-gs` locks `/dev/tty.usbmodem*` (`TIOCEXCL`), causing `Resource busy` errors without identifying the holding PID.
- **What Was Done**: Created `tools/fc_shell.py` using Python stdlib (`termios`, `select`, `fcntl`) with automatic `/dev/cu.usbmodem*` discovery, `DTR|RTS` assertion, `lsof` port-holder diagnostics, `--dfu` reboot helper, and `--monitor` + `--after` continuous stress-test support.
- **Outcome**: **Verified on host & live hardware**.

---

### Issue 10: MPU-6000 `-18.67%` Sample Loss (`6,506 Hz` vs `8,000 Hz`) on Flash #1
- **Observed Symptom**: Live `imurate` on Flash #1 reported `imu1: 6506.0 Hz (-18.67 %); lost since boot: 12663`.
- **Root Cause**: `Mpu6x00` configured `REG_INT_PIN_CFG = 0x10` (`INT_RD_CLEAR` only, emitting a 50 µs active-high pulse) and awaited `wait_for_rising_edge()`. Because `mpu_reader_task` and `indi_task` share `EXECUTOR_CTRL` (`P10`), whenever `indi_task` executed across a 125 µs DRDY boundary, the rising edge fired before `wait_for_rising_edge()` re-armed EXTI, dropping ~1 out of every 5 samples (the exact failure mode already documented in `icm426xx.rs`).
- **What Was Done**:
  - Set `REG_INT_PIN_CFG = 0x30` (`LATCH_INT_EN | INT_RD_CLEAR`) in `crates/drivers/src/imu/mpu6x00.rs` so `INT` stays asserted until the 14-byte `ACCEL_XOUT_H` burst read clears it, and changed `Mpu6x00::read()` from `wait_for_rising_edge()` to `wait_for_high()`.
  - Raised MPU-6000 `SPI4` fast clock from `6.75 MHz` (`Br::DIV8`) to `13.5 MHz` (`Br::DIV4`, well within the `20 MHz` datasheet limit) in `crates/cybflight/src/board_init/flywoof745.rs`, halving SPI burst transfer time.
- **Outcome**: Pending Flash #2 live `imurate` verification.

---

### Issue 11: `indistat` Reporting `0.0 us` on STM32F745
- **Observed Symptom**: Live `indistat` on Flash #1 reported `indi: 14111 iters, step avg 0.0 us, max 0.0 us, loop period max 0.0 us`.
- **Root Cause**: Cortex-M7 r0p1 (`STM32F745`) implements CoreSight software lock on the DWT unit (`DWT->LSR = 0x03` at reset). Without writing `0xC5AC_CE55` to `DWT->LAR` (`0xE000_1FB0`), `cp.DWT.enable_cycle_counter()` (`DWT->CTRL.CYCCNTENA`) is silently ignored and `DWT::cycle_count()` stays `0`.
- **What Was Done**: Added `core::ptr::write_volatile(0xE000_1FB0 as *mut u32, 0xC5AC_CE55)` + `cortex_m::asm::dsb()` before `cp.DWT.enable_cycle_counter()` in `crates/cybflight/src/control/indi_task.rs`.
- **Outcome**: Pending Flash #2 live `indistat` verification.

---

### Issue 12: Flywoo GM10 Nano V3 (u-blox M10) `GPS: init timed out` on Flash #1
- **Observed Symptom**: `magext` (`QMC5883L` on the GPS puck's I2C pads) returned valid data, proving the puck had 5V power, yet `gpshealth` reported `GPS: init timed out`.
- **Root Cause**: `crates/drivers/src/gps/ublox.rs` sent legacy `UBX-CFG-MSG` (`0x06 0x01`), which u-blox M10 SPG 5.x deprecates (`ACK-NAK`) in favor of `UBX-CFG-VALSET` (`0x06 0x8A`), and `board_init/flywoof745.rs` only probed `115,200` baud (whereas Flywoo GM10 V3 ships at `115,200`, `38,400`, or `9,600` depending on factory batch).
- **What Was Done**:
  - Updated `crates/drivers/src/gps/ublox.rs` to send `UBX-CFG-VALSET` (`0x06 0x8A`, RAM layer: `CFG_UART1OUTPROT_UBX = 1`, `CFG_MSGOUT_UBX_NAV_PVT_UART1 = 1`) first, falling back to `UBX-CFG-MSG` (`0x06 0x01`) on `ACK-NAK` for M8 compatibility, while tolerating initial UART framing errors during baud sync.
  - Added multi-baud auto-probe (`[115_200, 230_400, 38_400, 57_600, 9_600]`) + `GPS_DETECTED_BAUD` and `GPS_PROBE_RX_BYTES` diagnostics in `crates/cybflight/src/board_init/flywoof745.rs`, `sensors/gps.rs`, and `usb_serial.rs`.
- **Outcome**: Pending Flash #2 live `gpshealth` verification.

---

### Issue 13: Ground Station 3D CAD Model Tilted Front When Physical Drone Tilted Left
- **Observed Symptom**: User observed live IMU response in `cybflight-gs`, but tilting the physical drone Left tilted the 3D CAD preview toward its visual Front.
- **Root Cause**:
  1. Inspecting `/Users/andrii/Personal/fusion/drone_firefly_2s_nano_baby_20/scripts/build_drone_fusion360.py` (`FR = (+M, +M)`, `FL = (-M, +M)`, rear `XT30` at `y = [-41.5, -32.5]`) revealed the Fusion 360 STL files were exported in **Right-Front-Up (`+X_stl = Right`, `+Y_stl = Front`, `+Z_stl = Up`)**.
  2. `parse_binary_stl` in `crates/cybflight_gs/src/ui/drone_3d.rs` loaded `(x_stl, y_stl, z_stl)` directly into Cybflight's **FLU (`+X = Front`, `+Y = Left`, `+Z = Up`)** scene without rotating axes — placing the CAD drone's visual Nose (`+Y_stl`) along **FLU `+Y` (Left)**!
  3. Cross-checking `goku_ardu_params.params` (`AHRS_ORIENTATION = 5`, `SERVO1..4_FUNCTION = 36, 33, 34, 35` -> `PB0=RR, PB1=FR, PE9=RL, PE11=FL`) confirmed the firmware's FLU body axes (`Cw270Deg` + `imu_rpy_y: 135.0`) and motor pin assignments were already 100% aligned with the physical airframe.
- **What Was Done**: Applied proper `det = +1` `-90°` Z rotation `(y_mm, -x_mm, z_mm) * 0.001` in `parse_binary_stl` (`crates/cybflight_gs/src/ui/drone_3d.rs`), added unit test `stl_rfu_mm_maps_to_flu_m_preserving_normals`, and rebuilt `cybflight-gs` (`release-host`).
- **Outcome**: **Verified via unit test & `release-host` build**.

---

### Issue 14: Stationary Yaw Drift (`-3.11 deg/s`) in Attitude Preview & Rate Input
- **Observed Symptom**: Stationary live `imu1` showed `gyro_rad_s = [0.0066, 0.0142, -0.0543]` (`-3.11 deg/s` Z-gyro zero-rate offset, matching ArduPilot's stored `INS_GYROFFS_Z = +0.0496` in FRD on this exact chip), causing slow continuous yaw rotation in `cybflight-gs` because IMU-only Mahony (`mag = None`) cannot observe yaw bias parallel to gravity.
- **What Was Done**: Added Betaflight-style pure-`f32` stationary startup gyro calibration (`GyroStartupCal` in `crates/cybflight/src/sensors/imu.rs`: first-sample-centered 1.0 s window, per-axis `std <= 0.05 rad/s` and `|mean| <= 0.35 rad/s`, blocked when `IS_ARMED == true`, zero `f64` opcodes in the 8 kHz loop) and exposed calibrated bias in `imurate`.
- **Outcome**: Pending Flash #2 live `imu1` / `imurate` / 65 s drift verification.
