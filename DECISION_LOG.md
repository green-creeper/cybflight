# Flywoo GOKU GN 745 V3 (`STM32F745VG`) Bring-Up Decision Log

This document records the engineering decisions, root-cause analyses, code changes, and live hardware verification outcomes during the Cybflight bring-up of the **Flywoo GOKU GN 745 V3 AIO (`STM32F745VGT6`)** flight controller (`vehicles/goku_f745.yaml`).

---

## Summary Table

| # | Symptom / Issue | Root Cause | Decision & Fix | Outcome |
|---|---|---|---|---|
| 1 | Silent boot loop & `reboot --dfu` failed to enter STM32 ROM DFU | `PWR_CR1.DBP` was locked when writing DFU magic to `RTC_BKP0R`, and repeated early crashes rebooted back into crashing code before USB CDC enumerated | Unlock backup domain (`RCC_APB1ENR.PWREN` + `PWR_CR1.DBP` + readback barrier) before `RTC_BKP0R` write; add BKPSRAM early-crash counter (`>=2` crashes `<5 s`) entering USB Safe Mode | **Verified on hardware (Flash #1–#3)** — `reboot --dfu` reliably enumerates `0483:df11` hands-free, and Safe Mode preserves USB recovery shell across early faults |
| 2 | `HardFault` at ~3.35 s uptime during postmortem heartbeat | STM32F7 maps Backup SRAM (`0x4002_4000`) in the `Peripheral` (`Device`) MPU region where Cortex-M7 strictly traps any unaligned access (`ldrh.w` / `strh.w`) | Replaced packed-struct byte/halfword pointer casts in `bkpsram.rs` with 32-bit word-aligned `read_volatile`/`write_volatile` bounce buffers + RCC readback barrier | **Verified on hardware (Flash #1–#3)** — eliminated `BKPSRAM` fault and preserved crash telemetry across resets |
| 3 | `just flash` compiled & flashed `board_sakurah743` (`714 KB` H7 image) onto F745 | Repository `.env` defaulted to `VEHICLE=sakura_vicon` (`STM32H743`), whose clock/memory layout cannot boot on `STM32F745VG` | Updated `.env` default `VEHICLE=goku_f745` (`board_flywoof745`) | **Verified on hardware (Flash #1–#3)** — `just flash` targets `goku_f745` (`STM32F745VG`) and boots USB CDC |
| 4 | `HardFault` (`UNDEFINSTR`, `cfsr=0x00010000`, `pc=0x080536ea`) at `415 ms` | `thumbv7em-none-eabihf` defaults to `FPv5-D16` (double-precision FPU), emitting `vcvt.f64.f32` (`f64` math in barometer/ESKF/formatting), whereas `STM32F745VG` only implements single-precision `FPv5-SP-D16` | Added `"-C", "target-feature=-fp64"` to `[target.thumbv7em-none-eabihf]` in `.cargo/config.toml` so `f32` stays hardware FPU and `f64` uses software float | **Verified on hardware & disassembly (Flash #1–#3)** — `llvm-objdump` confirmed `0` `.f64` VFP opcodes; firmware runs continuously |
| 5 | `SPI4` IMU bus overclocked to `27 MHz` (`> 20 MHz` MPU-6000 max rating) | `embassy-stm32` integer prescaler calculation on `54 MHz` `APB2` (`54 / 20 = 2`) selected `Br::DIV2 = 27 MHz` when `20 MHz` was requested | Set `SPI4` fast frequency in `board_init/flywoof745.rs` to `Hertz(13_500_000)` (`Br::DIV4` for MPU-6000/ICM-42688-P) and `Hertz(6_750_000)` (`Br::DIV8` for BMI270) | **Verified on hardware (Flash #2 & #3)** — `SPI4` @ `13.5 MHz` runs at `7,987.2 Hz` with clean `WHO_AM_I` & gyro/accel reads |
| 6 | `IndependentWatchdog` (`0xc8000000`) reset at `4,700 ms` + `imu1`/`baro1` `no data (timeout)` | (a) `cybflight-gs` auto-sends `stream att on`, `stream imu1 on`, `stream power on`, `stream dshot on` on connect; `msg_stream_task` formatted 8 kHz `IMU_1` + 3.7 kHz `DSHOT_TELEMETRY` on the Thread Executor with no rate limit or yield, starving `iwdg_feed_task` (`500 ms` timeout). (b) Consecutive early `IWDG` resets tripped Safe Mode on Boot 8, which intentionally skips `board_init::init` | Rewrote `msg_stream_task` in `usb_serial.rs` to sleep (`50 ms`) when disabled, coalesce burst frames, and enforce `yield_now()` + `20 ms` (`<=50 Hz`) pacing; switched `imu1` stream to `IMU_1_DECIM`; added `version`, `stream off`, and indoor Mahony `att` fallback; preferred `/dev/cu.*` on macOS in `cybflight-gs` | **Verified on hardware (Flash #1–#3)** — `>16.5 min` continuous uptime with `cybflight-gs` connected & streaming, `0` `IWDG` resets |
| 7 | Incorrect IMU chip & diamond airframe orientation (`Cw0Deg` vs physical board) | `bsp/flywoof745` defaulted `gyro1_align` to `Cw0Deg` and `InstallParams` lacked arbitrary yaw mounting (`imu_rpy_*`), whereas GOKU GN 745 V3 mounts MPU-6000 at `Cw270Deg` relative to silkscreen arrow and this quad mounts the AIO arrow Rear-Left (`AHRS_ORIENTATION=5` = `ROTATION_YAW_225` = `+135°` FLU yaw) | Changed `gyro1_align` to `SensorAlign::Cw270Deg` in `bsp/flywoof745`; added `imu_rpy_r/p/y` (`FirmwareConfig::VERSION` 57) precomputed `Matrix3<f32>` in `ImuReader`; set `imu_rpy_y: 135.0` in `vehicles/goku_f745.yaml` | **Verified live on hardware & against `goku_ardu_params.params` (`SERVO1..4 = 36,33,34,35`)** — user confirmed responsive live IMU tilt reaction in GS |
| 8 | Tight async retry risk on floating/noisy UART RX & silent unhandled IRQs | `crsf_task`, `ghst_task` (`P10` high-priority executor), and `gps_task` looped immediately on `uart.read()` framing/overrun errors without backoff; unhandled IRQs spun silently into an opaque `IWDG` reset | Added `Timer::after_millis(2).await` backoff on UART read errors in `rc.rs` and `gps.rs`; added `DefaultHandler(irqn)` in `postmortem/fault.rs` recording exact IRQ number to BKPSRAM | **Verified on hardware (Flash #1–#3)** — `0` unhandled IRQs or executor stalls |
| 9 | Host shell automation & port-contention diagnostics | Interactive `picocom`/`tio` cannot run non-interactively in automated verification loops, and background `cybflight-gs` holds `/dev/tty.usbmodem*` with `TIOCEXCL` (`EBUSY`) | Built zero-dependency `tools/fc_shell.py` (`termios` raw mode, `DTR|RTS`, `/dev/cu.*` auto-discovery, `lsof` holder reporting, `--monitor`, `--dfu`) | **Verified on host & live FC (Flash #1–#3)** |
| 10 | `imurate` reported `6506.0 Hz (-18.67%)` on Flash #1 and `7753.1 Hz (-3.09%)` on Flash #2 | (a) Flash #1: `Mpu6x00` used 50 µs pulse `REG_INT_PIN_CFG = 0x10` + `wait_for_rising_edge()`. (b) Flash #2: latching (`0x30` + `wait_for_high()`) cut loss to `2.6%`, and unlocked `indistat` revealed `indi_task` takes `126–210 µs` (`> 125 µs` IMU period) while sharing `EXECUTOR_CTRL` (`P10`) with `mpu_reader_task` | (a) Set `REG_INT_PIN_CFG = 0x30` (`LATCH_INT_EN \| INT_RD_CLEAR`) + `wait_for_high()` + `13.5 MHz` `SPI4` + `10 ms` timeout / `recover()`. (b) Isolated `mpu_reader_task` onto dedicated `EXECUTOR_IMU` (`CAN2_RX1`, `Priority::P8`, strictly between `P6` DShot and `P10` INDI) | **Verified on hardware (Flash #3)** — `imurate` improved `6,506 Hz (-18.67%)` → `7,753 Hz (-3.09%)` → **`7,987.2 Hz (-0.16%)`**, matching native MPU-6000 oscillator (`0.006%` loss) |
| 11 | `indistat` reported `step avg 0.0 us, max 0.0 us` despite `14111 iters` on Flash #1 | Cortex-M7 r0p1 (`STM32F745`) locks CoreSight DWT registers at reset (`DWT->LSR = 0x03`), silently ignoring `cp.DWT.enable_cycle_counter()` unless `0xC5AC_CE55` is written to `DWT->LAR` (`0xE000_1FB0`) | Unlocked `DWT->LAR` (`0xE000_1FB0 = 0xC5AC_CE55` + `dsb()`) before `enable_cycle_counter()` in `indi_task.rs` | **Verified on hardware (Flash #2 & #3)** — `indistat` reports live DWT cycle timing (`126.0 µs` pure step on Flash #2; `215.9 µs` incl. `P8` IMU preemption on Flash #3) |
| 12 | `gpshealth` reported `GPS: init timed out` on Flywoo GM10 Nano V3 (u-blox M10) | `ublox.rs` sent legacy `UBX-CFG-MSG` (`ACK-NAK` on M10), ignored RX Fletcher-8 checksums, left rate/dynmodel at defaults, and `flywoof745.rs` only probed `115200` baud with a `256 B` RX buffer (GM10 V3 runs at `230400` baud) | Added `UBX-CFG-VALSET` (`0x06 0x8A`: `OUTPROT_UBX=1`, `OUTPROT_NMEA=0`, `NAV_PVT_UART1=1`, `RATE_MEAS=100ms` **10 Hz**, `DYNMODEL=8` **Airborne <4g**), RX Fletcher-8 checksum verification, `1024 B` RX buffer, and `[115200, 230400, 38400, 57600, 9600]` auto-baud probe | **Verified on hardware (Flash #2 & #3)** — GM10 M10 initialized at `baud=230400`, `5.00 Hz` on Flash #2 → **`10.00 Hz` (`mean=99,963 µs`)** on Flash #3 |
| 13 | In `cybflight-gs`, tilting physical drone Left tilted the 3D CAD model Front | Fusion 360 STL generator (`build_drone_fusion360.py`) built the CAD model in Right-Front-Up (`+X_stl = Right`, `+Y_stl = Front`, `XT30` at `y = -41.5`), whereas `parse_binary_stl` in `drone_3d.rs` loaded `(x_stl, y_stl, z_stl)` directly into FLU (`+X = Front`, `+Y = Left`), placing the CAD model's visual Nose along FLU `+Y` (Left) | Applied proper `det = +1` `-90°` Z rotation `(y_mm, -x_mm, z_mm) * 0.001` in `parse_binary_stl` (`drone_3d.rs`) and verified motor corners against `goku_ardu_params.params` (`SERVO1..4_FUNCTION = 36,33,34,35` -> `PB0=RR, PB1=FR, PE9=RL, PE11=FL`) | **Verified via unit test (`drone_3d`) & `release-host` `cybflight-gs` build** |
| 14 | Slow stationary yaw drift (`~-3.1 deg/s`) in `cybflight-gs` attitude preview | Uncalibrated MEMS MPU-6000 had stationary Z-gyro zero-rate offset `-0.0543 rad/s` (`-3.11 deg/s`, matching ArduPilot's `INS_GYROFFS_Z = +0.0496` in FRD on this board), and IMU-only Mahony (`mag = None`) cannot observe yaw bias around gravity | Added Betaflight-style pure-`f32` stationary startup gyro calibration (`GyroStartupCal`: 1.0 s window, `std <= 0.05 rad/s`, `|mean| <= 0.35 rad/s`, blocked when armed) in `sensors/imu.rs` and surfaced calibrated bias in `imurate` | **Verified on hardware (Flash #2 & #3)** — calibrated `bias=[+0.0071, +0.0154, -0.0542] rad/s` in `1.0 s`; stationary `imu1` gyro `< 0.0035 rad/s` (`0.2 deg/s`), yaw drift eliminated |
| 15 | Reversed RC Yaw stick & RTK-only arming lockout in `outer_rate` + missing `GyroCalibrating` gate | Betaflight/ArduPilot audit (`FIRMWARE_COMPARISON.md`) caught: (a) `outer_rate` used un-inverted `centered_with(3, endpoints)` on yaw (`+ω_z` CCW on right stick); (b) `ArmStateMachine` & `health.rs` required `ESTIMATOR_READY` (RTK-Fixed `carr_soln == 2`) even in pure `outer_rate`; (c) fast arming before `1.0 s` bypassed `GyroStartupCal` | Changed `outer_rate` yaw to `centered_inverted_with(3, endpoints)` in `rc_interpreter.rs`; added `BlockReason::GyroCalibrating` (`!IMU1_GYRO_CALIBRATED`); gated `EstimatorNotReady`/`SensorHealthDegraded`/`EskfDegraded` on `not(feature = "outer_rate")` in `rc.rs` & `health.rs` | **Verified via unit tests, adversarial review & live `health` on Flash #3** |
| 16 | CRSFv3 `SpeedProposal` link-drop hazard & SPL06 18-byte I2C burst erratum | (a) `CrsfRunner` sent `ACK=true` on `SpeedProposal` without changing `USART2` baudrate; (b) `Spl06` read 18 `COEF` bytes in one burst (NACK erratum >9 B on affected SPL06/DPS310 batches) | Added `CrsfParser::reject_speed_proposal` (`ACK=false`) + `256 B` CRSF UART buffers; split `Spl06` `REG_COEF` read into two 9-byte reads (`0x10` + `0x19`) in `init()` and `recover()` | **Verified on hardware (Flash #3 Boot 7)** — `baro1` reads `101,376.33 Pa, 37.90 °C` cleanly |
| 17 | Warm-reset stuck `SDA` + mid-run `baro1`/`magext` `I2C1` wedge (`t = 4..15 s` on Flashes #3 & #4) | (a) Warm `SCB::sys_reset()` mid-byte left slave holding `SDA` low across reset; (b) `i2c_config.timeout = 20 ms` caused a >20 ms Thread-Executor round under `P6`/`P8`/`P10` preemption to time out `write_read` between `write_dma_internal(send_stop=false)` and `read_dma_internal`, wedging STM32F7 `I2C1` in `TC=1, AUTOEND=0, BUSY=1` holding `SCL` low | Added NXP UM10204 §3.1.16 open-drain 9-clock + `STOP` unstick at boot, raised `i2c_config.timeout = 50 ms` (`< 141 ms` worst-case `IWDG` margin), added `RM0385 §30.4.6` `reset_i2c1_if_wedged()` (`CR1.PE` `0 -> 1 µs -> 1`), `10 ms` yield before `recover()`, and `0..5` status poll cap in `Qmc5883l` | **Verified on hardware (Flash #4 & #5)** — warm-reset init (`101,376.42 Pa, 36.94 °C`) & continuous `I2C1` survival past `21.5 s` (`magext = [-27.67, -1.17, 42.27] uT`) |
| 18 | `baro1` reported constant `80,523.00 Pa, 101.50 °C` after `imurate` on Flash #5 | `measure_rate` starved `I2C1` for >50 ms during `imurate`, triggering `Spl06::recover()`, which wrote `REG_RESET = 0x09` with **0 ms delay** (SPL06 requires 40 ms OTP reload) and immediately wrote `MEAS_CFG = 0x07` while resetting, leaving SPL06 in standby (`raw_psr = 0, raw_tmp = 0` → `c00 = 80523 Pa`, `c0 * 0.5 = 101.5 °C`) | Removed un-delayed `REG_RESET` from `Spl06::recover()` so it calls `Self::configure()` directly; guarded `Spl06::read()` to return `Error::NotReady` if `MEAS_CTRL != 0x07` or `raw_psr == 0 && raw_tmp == 0` | **Pending Flash #7 live verification (before & after `imurate` + 65 s stress test)** |
| 19 | `imurate` hung `shell_loop` indefinitely on Flash #6 while keeping `IWDG` fed | Adding `yield_now().await` inside `measure_rate` while leaving `Timer::at(deadline)` inside `select(Timer::at(deadline), sub.next_message())` constructed a fresh `Timer { yielded_once: false }` every loop (`embassy-time` `Timer::poll` always returns `Poll::Pending` on its 1st poll!), while `P8` `IMU_1` made `sub.next_message()` `Poll::Ready` after every `0.5 ms` yield | Hoisted `let mut deadline_timer = Timer::at(deadline);` outside `loop` and added `if Instant::now() >= deadline { break; }` at top of `loop` in `usb_serial.rs` | **Pending Flash #7 live verification** |

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
- **Outcome**: **Verified on hardware (Flash #1–#3)**. Hands-free `python3 tools/fc_shell.py --dfu` reliably enumerates `0483:df11` on every flash cycle without touching the physical BOOT0 button.

---

### Issue 2: `HardFault` at `3.35 s` Uptime (`BKPSRAM` Unaligned Access)
- **Observed Symptom**: Board booted USB CDC, ran for ~3.35 seconds, and hit a `HardFault` inside `postmortem` state persistence.
- **Root Cause**: STM32F7 Backup SRAM (`0x4002_4000`) sits in the Cortex-M7 `0x4000_0000` `Peripheral` (`Device`) default memory map region. Unlike Normal SRAM, Cortex-M7 strictly faults (`UNALIGNED` UsageFault escalated to HardFault) on any halfword/unaligned load or store (`ldrh.w` / `strh.w`) emitted by LLVM for packed/byte struct accesses.
- **What Was Done**:
  - Rewrote `crates/cybflight/src/postmortem/bkpsram.rs` to copy `BootRecord` slots exclusively through a `#[repr(C, align(4))]` RAM bounce buffer using 32-bit word-aligned `core::ptr::read_volatile` and `core::ptr::write_volatile`, plus an RCC `BKPSRAMEN` readback barrier + `cortex_m::asm::dsb()`.
- **Outcome**: **Verified on hardware (Flash #1–#3)**. `postmortem show` reliably reads and writes prior boot records across `Software` and `IndependentWatchdog` resets with zero faults.

---

### Issue 3: `just flash` Defaulted to `board_sakurah743`
- **Observed Symptom**: Running `just flash` built `--features board_sakurah743,rx_crsf,est_eskf,est_pos_mocap,outer_mpc,postmortem` and downloaded a 714 KB H743 binary to `0x08000000`, resulting in no USB shell.
- **Root Cause**: `Justfile` reads `VEHICLE` from `.env` when no positional argument is passed, and `.env` was still set to `VEHICLE=sakura_vicon`.
- **What Was Done**: Updated `/Users/andrii/Personal/autopilots/cybflight/.env` to `VEHICLE=goku_f745`.
- **Outcome**: **Verified on hardware (Flash #1–#3)**. `just flash` builds and flashes `goku_f745` (`STM32F745VG`) cleanly.

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
- **Outcome**: **Verified on hardware & disassembly (Flash #1–#3)**. `llvm-objdump` confirmed **0** `.f64` instructions across all builds, and the MCU runs indefinitely without `UNDEFINSTR` faults.

---

### Issue 5: `SPI4` 27 MHz Overclock on Onboard MPU-6000
- **Observed Symptom**: Potential corrupted WHO_AM_I / gyro register transfers after switching `SPI4` from `1 MHz` init clock to fast operational clock.
- **Root Cause**: On STM32F745 at `216 MHz` `SYSCLK`, `APB2` runs at `54 MHz`. `embassy-stm32`'s SPI prescaler calculation uses integer division (`54_000_000 / 20_000_000 = 2`), selecting `Br::DIV2 = 27 MHz` — **35% above** the MPU-6000 datasheet maximum of `20 MHz`.
- **What Was Done**:
  - Updated `crates/cybflight/src/board_init/flywoof745.rs` to set `fast_cfg.frequency = Hertz(13_500_000)` (`54 MHz / 4 = 13.5 MHz`, safely inside MPU-6000's `20 MHz` and ICM-42688-P's `24 MHz` specs) and `Hertz(6_750_000)` (`Br::DIV8`) for BMI270 (`10 MHz` spec).
- **Outcome**: **Verified on hardware (Flash #2 & #3)**. `imu1` and `imurate` run cleanly at `7,987.2 Hz` with `40.06 °C` die temperature and zero corrupted frames.

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
  1. Host process `cybflight-gs` was running in the background watching `/dev/tty.usbmodem*`. As soon as USB CDC enumerated (~4.2 s), `cybflight-gs` connected and sent `version`, `stream att on`, `stream imu1 on`, `stream power on`, `stream dshot on`, `health`, `param list`.
  2. `msg_stream_task` subscribed directly to `IMU_1` (**8,000 Hz**) and `DSHOT_TELEMETRY` (**3,700 Hz**) on the cooperative Thread Executor with no rate limit and no `yield_now()`. Formatting 6 floats via `core::fmt::write` took `>125 µs` on 216 MHz F745, so a new `IMU_1` message was **always** waiting in the `PubSubChannel` queue (`Poll::Ready` on 100% of iterations). `msg_stream_task` never returned `Poll::Pending` to the Embassy Thread Executor, starving `iwdg_feed_task` (`100 ms` period, `500 ms` hardware timeout) and triggering `IndependentWatchdog` 500 ms later (`4,200 ms + 500 ms = 4,700 ms`).
  3. Because Boot 7 reset at `4,700 ms` (`< 5,000 ms` early-crash threshold), Boot 8 entered **Safe Mode**, which intentionally skips `board_init::init` (no SPI/I2C/DMA tasks spawned) so the recovery shell stays alive — explaining why `imu1` and `baro1` returned `no data (timeout)` in Safe Mode.
- **What Was Done**:
  - Rewrote `msg_stream_task` in `crates/cybflight/src/usb_serial.rs` to sleep (`50 ms`) when disabled, drain stale bursts on `false -> true`, coalesce updates, and enforce `yield_now().await` + `Timer::after_millis(20).await` (`<= 50 Hz` max per stream).
  - Switched `imu1_stream_task` from 8 kHz `IMU_1` to `IMU_1_DECIM`.
  - Added `"version"`, `"stream off"`, and indoor Mahony `"att"` fallback; updated `cybflight-gs` to prefer `/dev/cu.*` on macOS.
- **Outcome**: **Verified on hardware (Flash #1–#3)**. Survived `>16.5 min` continuous live `cybflight-gs` session with all 4 streams active (`0` `IWDG` resets).

---

### Issue 7: IMU Chip & Diamond Airframe Orientation Alignment
- **Observed Symptom**: `crates/bsp/flywoof745/src/lib.rs` had `gyro1_align: SensorAlign::Cw0Deg` and `vehicles/goku_f745.yaml` had no board yaw mounting offset, which would feed rotated gyro/accel axes into INDI and ESKF.
- **Root Cause**:
  1. Cross-referencing ArduPilot's `FlywooF745/hwdef.dat` (`IMU Invensense SPI:mpu6000 ROTATION_YAW_180`) and Betaflight's `FLYWOOF745NANO/config.h` (`GYRO_1_ALIGN CW270_DEG`) proves board FLU is **`SensorAlign::Cw270Deg`**.
  2. Proven ArduPilot flight parameters from this exact quad (`goku_ardu_params.params`, `1,193 s` flight log) specify `AHRS_ORIENTATION = 5` (`ROTATION_YAW_225`), meaning the diamond AIO board's silkscreen arrow points Rear-Left (`225°` CW from Forward = `+135°` CCW yaw in FLU).
- **What Was Done**:
  - Set `gyro1_align: SensorAlign::Cw270Deg` in `crates/bsp/flywoof745/src/lib.rs`.
  - Added `imu_rpy_r`, `imu_rpy_p`, `imu_rpy_y` (`[f32; 3]`, `FirmwareConfig::VERSION` bumped `56 -> 57`) to `InstallParams` in `crates/cybflight_core/src/params.rs` and precomputed `R = R_z(yaw) * R_y(pitch) * R_x(roll)` once at boot in `ImuReader::new` (`crates/cybflight/src/sensors/imu.rs`).
  - Configured `imu_rpy_y: 135.0` in `vehicles/goku_f745.yaml` and verified motor corners against `goku_ardu_params.params`.
- **Outcome**: **Verified via unit tests (`327/327` `cybflight-core`, `52/52` `vehicle-yaml`) & live bench motion**.

---

### Issue 8: Defensive UART Error Backoff & `DefaultHandler` Exception Telemetry
- **What Was Done**: Added `Timer::after_millis(2).await` on `Err(_)` in `crsf_task`, `ghst_task`, and `gps_task`, plus `DefaultHandler(irqn)` in `crates/cybflight/src/postmortem/fault.rs`.
- **Outcome**: **Verified on hardware (Flash #1–#3)** (`0` unhandled IRQs or UART error storms).

---

### Issue 9: Non-Interactive USB CDC Shell Automation (`tools/fc_shell.py`)
- **What Was Done**: Created `tools/fc_shell.py` using Python stdlib (`termios`, `select`, `fcntl`) with `/dev/cu.usbmodem*` auto-discovery, `DTR|RTS` assertion, `lsof` port-holder diagnostics, `--dfu`, and `--monitor` + `--after`.
- **Outcome**: **Verified on host & live hardware across all flash cycles**.

---

### Issue 10: Eliminating MPU-6000 Sample Loss (`6,506 Hz` → `7,753 Hz` → `7,987.2 Hz`)
- **Observed Symptom**:
  - Flash #1: `imurate` reported `6506.0 Hz (-18.67 %); lost since boot: 12663`.
  - Flash #2 (after latching `INT_PIN_CFG = 0x30` + `13.5 MHz` SPI4): `imurate` improved to `7753.1 Hz (-3.09 %)`, still losing ~212 samples/s (`2.6%`).
- **Root Cause**:
  1. Flash #1 used 50 µs active-high DRDY pulses (`0x10`) + `wait_for_rising_edge()`, missing edges whenever `EXECUTOR_CTRL` (`P10`) was busy.
  2. Unlocking `DWT->LAR` on Flash #2 revealed `indi_task` takes **`126.0 µs` avg / `209.7 µs` max** on 216 MHz STM32F745 (`> 125 µs` 8 kHz IMU period). Whenever `indi_task` ran >125 µs on the same `P10` executor as `mpu_reader_task`, two MPU-6000 DRDY samples arrived back-to-back before `mpu_reader_task` could run.
- **What Was Done**:
  1. Set `REG_INT_PIN_CFG = 0x30` (`LATCH_INT_EN | INT_RD_CLEAR`), switched `Mpu6x00::read()` to `wait_for_high()`, raised `SPI4` to `13.5 MHz`, and added a `10 ms` `with_timeout` + `Mpu6x00::recover()` (`REG_INT_STATUS` read).
  2. Added dedicated `static EXECUTOR_IMU: InterruptExecutor` on `CAN2_RX1` at **`Priority::P8`** (strictly between `P6` `EXECUTOR_HIGH` DShot and `P10` `EXECUTOR_CTRL` INDI/CRSF/Failsafe) in `crates/cybflight/src/board_init/flywoof745.rs`.
- **Outcome**: **Verified on hardware (Flash #3)** — `imurate` reached **`7,987.2 Hz (-0.16%)`** with only `2` samples lost over `4 s` (`0.006%`), matching the MPU-6000's native internal RC oscillator!

---

### Issue 11: `indistat` Reporting `0.0 us` on STM32F745
- **Observed Symptom**: Live `indistat` on Flash #1 reported `indi: 14111 iters, step avg 0.0 us, max 0.0 us, loop period max 0.0 us`.
- **Root Cause**: Cortex-M7 r0p1 (`STM32F745`) locks CoreSight DWT registers at reset (`DWT->LSR = 0x03`), ignoring `cp.DWT.enable_cycle_counter()` until `0xC5AC_CE55` is written to `DWT->LAR` (`0xE000_1FB0`).
- **What Was Done**: Unlocked `DWT->LAR` (`0xE000_1FB0 = 0xC5AC_CE55` + `dsb()`) before `cp.DWT.enable_cycle_counter()` in `crates/cybflight/src/control/indi_task.rs`.
- **Outcome**: **Verified on hardware (Flash #2 & #3)** — Flash #2 measured `step avg 126.0 us, max 209.7 us`; Flash #3 (with `P8` `EXECUTOR_IMU` preempting `P10` `indi_task`) measured `step avg 215.9 us, max 391.6 us` (well within the `500 us` 2 kHz INDI budget).

---

### Issue 12: Flywoo GM10 Nano V3 (u-blox M10) `GPS: init timed out` → `5 Hz` → `10 Hz`
- **Observed Symptom**: `gpshealth` reported `GPS: init timed out` on Flash #1.
- **Root Cause**: `ublox.rs` sent legacy `UBX-CFG-MSG` (`ACK-NAK` on u-blox M10), ignored RX Fletcher-8 checksums, did not configure `10 Hz` measurement rate or `Airborne <4g` dynamic model, and `flywoof745.rs` only probed `115,200` baud with a `256 B` RX ring buffer (whereas this GM10 Nano V3 operates at `230,400` baud).
- **What Was Done**:
  - Implemented `UBX-CFG-VALSET` (`0x06 0x8A`, RAM layer: `CFG_UART1OUTPROT_UBX = 1`, `CFG_UART1OUTPROT_NMEA = 0`, `CFG_MSGOUT_UBX_NAV_PVT_UART1 = 1`, `CFG_RATE_MEAS = 100 ms` [`10 Hz`], `CFG_RATE_NAV = 1`, `CFG_NAVSPG_DYNMODEL = 8` [`Airborne <4g`]) with `UBX-CFG-MSG` fallback, RX Fletcher-8 checksum validation, `1024 B` RX buffer, and `[115_200, 230_400, 38_400, 57_600, 9_600]` auto-baud probe.
- **Outcome**: **Verified on hardware (Flash #2 & #3)** — Flash #2 connected at `baud=230400` (`5.00 Hz`); Flash #3 confirmed **`mean=99963us (10.00 Hz)`** at `baud=230400`.

---

### Issue 13: Ground Station 3D CAD Model Tilted Front When Physical Drone Tilted Left
- **Observed Symptom**: User observed live IMU response in `cybflight-gs`, but tilting the physical drone Left tilted the 3D CAD preview toward its visual Front.
- **Root Cause**: Fusion 360 STL files (`build_drone_fusion360.py`: `FR = (+M, +M)`, `FL = (-M, +M)`, rear `XT30` at `y = -41.5`) were exported in **Right-Front-Up (`+X_stl = Right`, `+Y_stl = Front`)**, whereas `parse_binary_stl` (`crates/cybflight_gs/src/ui/drone_3d.rs`) loaded `(x_stl, y_stl, z_stl)` unrotated into **FLU (`+X = Front`, `+Y = Left`)**.
- **What Was Done**: Applied proper `det = +1` `-90°` Z rotation `(y_mm, -x_mm, z_mm) * 0.001` in `parse_binary_stl`, added unit test `stl_rfu_mm_maps_to_flu_m_preserving_normals`, and rebuilt `cybflight-gs` (`release-host`).
- **Outcome**: **Verified via unit test (`3/3` passed) & `release-host` `cybflight-gs` build**.

---

### Issue 14: Stationary Yaw Drift (`-3.11 deg/s`) in Attitude Preview & Rate Input
- **Observed Symptom**: Stationary live `imu1` on Flash #1 showed `gyro_rad_s = [0.0066, 0.0142, -0.0543]` (`-3.11 deg/s` Z-gyro zero-rate offset), causing slow continuous yaw rotation in `cybflight-gs`.
- **What Was Done**: Added Betaflight-style pure-`f32` stationary startup gyro calibration (`GyroStartupCal` in `crates/cybflight/src/sensors/imu.rs`: 1.0 s window, `std <= 0.05 rad/s`, `|mean| <= 0.35 rad/s`, blocked when armed, zero `f64` opcodes) and exposed calibrated bias in `imurate`.
- **Outcome**: **Verified on hardware (Flash #2 & #3)** — `imu1 gyro_cal: calibrated bias=[+0.0071, +0.0154, -0.0542] rad/s`; stationary `imu1` reads `gyro_rad_s=[0.0017, -0.0034, 0.0014]` and `att` quaternion stays steady.

---

### Issue 15: `outer_rate` Yaw Polarity & Arming Interlocks (`GyroCalibrating` + Non-RTK Rate Arming)
- **Observed Symptom / Audit Finding**: Cross-verification against Betaflight and ArduPilot (`FIRMWARE_COMPARISON.md`) uncovered three critical pre-flight issues:
  1. `outer_rate` (`crates/cybflight/src/control/rc_interpreter.rs`) used un-inverted `RateChannelCalibration::centered_with(3, endpoints)` on yaw channel 3, commanding `+ω_z` (CCW nose-left) on right stick instead of `-ω_z` (CW nose-right).
  2. `ArmStateMachine` (`sensors/rc.rs`) and `SystemHealth` (`health.rs`) gated `outer_rate` on `ESTIMATOR_READY`, which requires RTK-Fixed GPS (`carr_soln == 2`) — locking out bench/outdoor arming with a standard non-RTK M10 GPS (`GM10 Nano V3`).
  3. Arming within `<1.0 s` of power-up could abort `GyroStartupCal` before bias convergence.
- **What Was Done**:
  - Switched `outer_rate` yaw to `RateChannelCalibration::centered_inverted_with(3, endpoints)` (matching `RcMapper::aetr` and Position mode).
  - Added `BlockReason::GyroCalibrating` (`!IMU1_GYRO_CALIBRATED`) across all modes.
  - Gated `EstimatorNotReady`, `SensorHealthDegraded`, and `EskfDegraded` on `#[cfg(not(feature = "outer_rate"))]`.
- **Outcome**: **Verified via feature-matrix builds (`goku_f745` + `sakura_vicon`), unit tests, adversarial review, and live `health` on Flash #3**.

---

### Issue 16: CRSFv3 `SpeedProposal` Reject & SPL06 9-Byte Chunked `COEF` Read
- **Root Cause**:
  1. `CrsfRunner` replied `ACK = true` to CRSFv3 `SpeedProposal` (`0x70`) without reconfiguring `USART2` baudrate, which would drop ExpressLRS/Crossfire links on speed negotiation.
  2. `Spl06` read all 18 calibration bytes (`0x10..0x21`) in one I2C burst, hitting a known >9-byte burst NACK erratum on certain SPL06/DPS310 lots.
- **What Was Done**:
  - Added `CrsfParser::reject_speed_proposal` (`ACK = false`) in `crates/drivers/src/rc/crsf.rs` & `sensors/rc.rs` and increased CRSF UART TX/RX buffers to `256 B`.
  - Split `Spl06` `REG_COEF` read into two 9-byte reads (`0x10` + `0x19`) in both `init()` and `recover()` (`crates/drivers/src/baro/spl06.rs`).
- **Outcome**: **Verified on hardware (Flash #3 Boot 7)** — `baro1` reports `BaroSample(t=4568, pressure_pa=101376.33, temp_c=37.90)`.

---

### Issue 17: Warm-Reset Stuck `SDA` & Mid-Run `I2C1` Peripheral Wedge (`baro1` + `magext`)
- **Observed Symptom**:
  1. Immediately after `reboot --dfu` into Flash #3 (Boot 6), `baro1` and `magext` (`I2C1` `PB6` SCL / `PB7` SDA) timed out during init, then came online on Boot 7 (`101,376.33 Pa`, `[-27.67, -1.07, 41.87] uT`).
  2. On Flashes #3 & #4, `baro1` initialized cleanly (`101,376.42 Pa, 36.94 °C`), then both `baro1` and `magext` stopped updating at `t = 4..15 s` uptime.
- **Root Cause**:
  1. `QMC5883L` (`200 Hz`) and `SPL06` (`32 Hz`) stay powered across warm resets (`reboot`, `reboot --dfu`, `IWDG`); a reset mid-byte leaves a slave holding `SDA` low waiting for `SCL` clocks.
  2. `flywoof745.rs` set `i2c_config.timeout = 20 ms`. Under `P6` (DShot) + `P8` (8 kHz IMU) + `P10` (2 kHz INDI) preemption, a >20 ms Thread-Executor round caused `embassy-stm32` `i2c/v2.rs::write_read` to time out after `write_dma_internal(..., send_stop = false)` completed without `STOP`, leaving STM32F7 `I2C1` wedged in `TC = 1, AUTOEND = 0, ISR.BUSY = 1` holding `SCL` low permanently!
- **What Was Done**:
  - Added NXP UM10204 §3.1.16 compliant open-drain 9-clock `SCL` + `STOP` unstick sequence before `I2c::new` in `crates/cybflight/src/board_init/flywoof745.rs`.
  - Raised `i2c_config.timeout` to `50 ms` (`2 × 50 ms = 100 ms < 141 ms` worst-case `47 kHz` LSI `IWDG` margin), added `RM0385 §30.4.6` `reset_i2c1_if_wedged()` (`CR1.PE` `0 -> 1 µs -> 1`), added a `10 ms` yield before `recover()` in `sensors/baro.rs` and `sensors/mag.rs`, and bounded `Qmc5883l` status polling (`0..5`).
- **Outcome**: **Verified on hardware (Flash #4 & #5)** — warm-reset `baro1` init succeeded (`101,376.42 Pa, 36.94 °C`), and on Flash #5 `I2C1` remained continuously alive past `21.5 s` (`magext = [-27.67, -1.17, 42.27] uT`, `imurate = 7,997.2 Hz (-0.04%)`, `gpshealth = 10.00 Hz`).

---

### Issue 18: `Spl06::recover()` Un-Delayed `REG_RESET` Standby Bug (`80,523.00 Pa, 101.50 °C`)
- **Observed Symptom**: On Flash #5, after running `imurate` (`1.0 s`), `baro1` continued publishing at 32 Hz (`I2C1` recovered cleanly!), but reported constant `pressure_pa = 80523.00, temp_c = 101.50`.
- **Root Cause**: `measure_rate` starved `I2C1` for >50 ms during `imurate`, triggering `BaroReader` → `Spl06::recover()`. Because `ReadBaro::recover` has no `DelayNs` parameter, `Spl06::recover()` wrote `REG_RESET = 0x09` with **0 ms delay** (SPL06 requires 40 ms post-reset OTP reload) and immediately wrote `MEAS_CFG = 0x07` while the chip was still resetting — leaving SPL06 in standby (`MEAS_CTRL = 0`, `raw_psr = 0`, `raw_tmp = 0`, which evaluates to `c00 = 80523.00 Pa` and `c0 * 0.5 = 101.50 °C`).
- **What Was Done**: Removed un-delayed `REG_RESET` from `Spl06::recover()` (`crates/drivers/src/baro/spl06.rs`) so recovery calls `Self::configure(&mut self.bus)` directly (factory `self.coefs` are immutable and already validated in `init()`), and guarded `Spl06::read()` to return `Error::NotReady` whenever `MEAS_CTRL != 0x07` or `raw_psr == 0 && raw_tmp == 0`.
- **Outcome**: **Pending Flash #7 live verification (before & after `imurate` + 65 s multi-stream stress test)**.

---

### Issue 19: `measure_rate` (`imurate`) Infinite Loop via `embassy_time::Timer::at` `yielded_once`
- **Observed Symptom**: On Flash #6 (`a90953d`), adding `embassy_futures::yield_now().await` every 4 polls inside `measure_rate` (`crates/cybflight/src/usb_serial.rs`) caused `imurate` to never return on `shell_loop` while `iwdg_feed_task` kept feeding `IWDG` (no watchdog reset, USB stayed enumerated, shell stopped reading commands).
- **Root Cause**: In `embassy-time-0.5.0` (`timer.rs:106-116`), `Timer::at(expires_at)` initializes `yielded_once: false` and unconditionally returns `Poll::Pending` on its **first** poll even when `Instant::now() >= expires_at`! Because `yield_now().await` yielded ~0.5 ms while `P8` `EXECUTOR_IMU` published at 8 kHz (every `125 µs`), `sub.next_message()` was always `Poll::Ready` on the next iteration — winning `select(Timer::at(deadline), sub.next_message())`, dropping the `Timer`, and constructing a fresh `yielded_once: false` `Timer` on the next iteration forever.
- **What Was Done**: Hoisted `let mut deadline_timer = Timer::at(deadline);` outside `loop` (`select(&mut deadline_timer, sub.next_message())`) and added an explicit `if Instant::now() >= deadline { break; }` guard at the top of `loop` in `crates/cybflight/src/usb_serial.rs`.
- **Outcome**: **Pending Flash #7 live verification**.
