# Firmware Architecture & Hardware Comparison: Flywoo GOKU GN 745 AIO V3 (`STM32F745VG`)

**Compared Codebases (Workspace Snapshot):**
- **Cybflight (Rust / Embassy):**
  - BSP: `cybflight/crates/bsp/flywoof745/src/lib.rs`
  - Board Init: `cybflight/crates/cybflight/src/board_init/flywoof745.rs`
  - Vehicle Config: `cybflight/vehicles/goku_f745.yaml`
  - Linker & Toolchain: `cybflight/crates/cybflight/memory_f745.x`, `cybflight/.cargo/config.toml`
  - Core / Drivers / Tasks: `sensors/{imu,baro,mag,gps,rc,power}.rs`, `motors/{mod,dshot}.rs`, `control/{indi_task,rc_interpreter,failsafe}.rs`, `estimation/eskf_imu_gps.rs`, `postmortem/{bkpsram,fault,recovery}.rs`, `crates/drivers/src/**`
- **Betaflight (C / STM32 HAL + LL):**
  - Board Targets: `src/config/FLYWOOF745/config.h` (legacy V1.0 Hex/Octo) and `src/config/FLYWOOF745NANO/config.h` (V1.2 / V3 Nano & 45A AIO motor routing)
  - Platform & Drivers: `src/platform/STM32/target/STM32F745/target.h`, `startup/system_stm32f7xx.c`, `system_stm32f7xx.c`, `link/stm32_flash_f74x.ld`, `accgyro/accgyro_spi_mpu6000.c`, `sensors/gyro.c`, `barometer/barometer_dps310.c`, `compass/compass_qmc5883.c`, `io/gps.c`, `rx/crsf.c`
- **ArduPilot (C++ / ChibiOS RTOS):**
  - Board Target: `libraries/AP_HAL_ChibiOS/hwdef/FlywooF745/{hwdef.dat,defaults.parm,README.md}` + `STM32CubeConf/F745-8MHz/F745-8MHz.ioc`
  - Verified Vehicle Reference: `/Users/andrii/Personal/autopilots/goku_ardu_params.params` (ArduCopter V4.6.2 parameters flown on this exact diamond-mounted GOKU GN 745 V3 quadrotor)
  - Drivers: `AP_InertialSensor_{Invensense,Invensensev3,BMI270}.cpp`, `AP_InertialSensor.cpp`, `AP_Baro_SPL06.cpp`, `AP_Compass_QMC5883L.cpp`, `AP_GPS_UBLOX{,_CFGv2}.cpp`, `AP_RCProtocol_CRSF.cpp`

---

## 1. Executive Summary: Most Actionable Stability & Configuration Findings

### P0 — Critical Flight-Control & Arming Blockers

1. **RC Yaw Stick Reversed in `outer_rate` Mode (`control/rc_interpreter.rs:56`)**
   - Cybflight's body frame is **FLU** (`+X` Forward, `+Y` Left, `+Z` Up), where `+ω_z` is **CCW (nose-left)** and `SpinDir::Cw = +1` produces `+Z` CCW reaction torque.
   - Standard AETR transmitters (`RC4_REVERSED = 0` in `goku_ardu_params.params`) output `~2012 µs` when the pilot pushes Yaw stick **right** (requesting CW / nose-right yaw, i.e. `-ω_z` in FLU).
   - Both `RcMapper::aetr` (`cybflight_core/src/rc/rc_mapping.rs:271`) and Position mode (`control/rc_interpreter.rs:432`) invert channel 3 via `ChannelCalibration::centered_inverted_with(3, endpoints)`.
   - **Bug:** The `#[cfg(feature = "outer_rate")]` task in `control/rc_interpreter.rs:56` calls `RateChannelCalibration::centered_with(3, endpoints)` **without `_inverted_`**. Pushing Yaw stick right commands `+rate_ref.z`, spinning up CW motors M1 (`PB0`) & M4 (`PE11`) and yawing the aircraft **left**. (Roll `ch0` and Pitch `ch1` signs in `outer_rate` are verified correct against `goku_ardu_params.params` where `RC1_REVERSED = 0` and `RC2_REVERSED = 1`.)

2. **Permanent Arming Lockout (`EstimatorNotReady`) Without RTK-Fixed GPS (`sensors/rc.rs:320-324`, `estimation/eskf_imu_gps.rs:14-25`)**
   - `goku_f745.yaml` sets `outer_loop: rate`, `pos_source: gps`, `gps_model: ublox`, with no fixed `origin:`.
   - `vehicle-features.rs:92-113` unconditionally enables `est_eskf` and `est_pos_gps` on every build.
   - `ArmStateMachine::update_channels` (`sensors/rc.rs:320-324`) blocks arming with `BlockReason::EstimatorNotReady` whenever `#[cfg(feature = "est_eskf")]` is active and `!ESTIMATOR_READY` — **even though `outer_rate` never consumes `VEHICLE_ODOMETRY`**.
   - `estimation/eskf_imu_gps.rs` (`is_pvt_origin_anchor`) refuses to anchor the ENU frame or set `ESTIMATOR_READY = true` until `carr_soln == 2` (**RTK Fixed**). On a standard non-RTK u-blox M8/M10 (`Flywoo GM10 Nano V3`, where `carr_soln == 0` permanently) or indoors on the bench, **`goku_f745` can never arm**.

3. **Startup Gyro Bias Calibration Not Gated Before Arming (`sensors/rc.rs:275-331` vs `sensors/imu.rs:108-149, 406-425`)**
   - `ImuReader` now implements a single-precision first-sample-centered stationary startup gyro calibrator (`GyroStartupCal`, 8,000 samples = `1.0 s`, `std <= 0.05 rad/s` ≈ `2.86 deg/s`) and publishes `IMU1_GYRO_CALIBRATED`.
   - **Gap:** `ArmStateMachine` (`sensors/rc.rs`) **does not gate arming on `IMU1_GYRO_CALIBRATED`** (unlike Betaflight's `ARMING_DISABLED_CALIBRATING` and ArduPilot's `INS: Not calibrated` pre-arm check). If the quad is handled while plugging in the 2S LiPo and armed before 1.0 s of uninterrupted stillness, `GyroStartupCal::observe` resets on `armed == true` and **never calibrates for the entire flight**, leaving raw MEMS zero-rate offset active (measured at **`+2.84 deg/s` on Z** and **`+0.91 deg/s` on Y** on this exact board in `goku_ardu_params.params`: `INS_GYROFFS_Z = 0.04960 rad/s`).

### P1 — High-Impact Link & Sensor Reliability Gaps

4. **CRSFv3 `SpeedProposal` Accepted Without Switching FC UART Baudrate (`sensors/rc.rs:418-431`) + Executor Placement (`board_init/flywoof745.rs:260`)**
   - When an ELRS / TBS Crossfire receiver sends `CrsfEvent::SpeedProposal { port_id, baud }` (`0x70`), `CrsfRunner` replies with `accept_speed_proposal(port_id, baud)` (`ACK = true`), instructing the receiver to switch baudrate, **without reconfiguring `USART2` (`PD5/PD6`)** and with no link-loss baudrate fallback (unlike Betaflight `rx/crsf.c:501-505`). Any CRSFv3 speed negotiation will immediately drop the RC link.
   - Additionally, `board_init/flywoof745.rs:260` spawns `crsf_task` on `ctrl_spawner` (**`EXECUTOR_CTRL`, NVIC P10**) alongside the 8 kHz IMU reader and 2 kHz INDI loop (contradicting `main.rs:27, 176` which documents RC on the thread executor), with only 128-byte `BufferedUart` TX/RX buffers.

5. **u-blox GPS Checksum Ignored + Missing 10 Hz / Airborne-4g / NMEA-Disable Init (`drivers/src/gps/ublox.rs:41-45, 344-347`)**
   - `Ublox::read_payload` reads the 2-byte Fletcher-16 checksum and discards it (`let _ = ck;`). Single-bit framing or EMI corruption on `USART6` can pass unchecked into `parse_nav_pvt`.
   - `VALSET_NAV_PVT_UART1` only sets `CFG_UART1OUTPROT_UBX = 1` and `CFG_MSGOUT_UBX_NAV_PVT_UART1 = 1` (or legacy `CFG-MSG` on M8). Unlike Betaflight (`io/gps.c`) and ArduPilot (`AP_GPS_UBLOX`), Cybflight **does not configure**:
     - Measurement period (`CFG-RATE-MEAS = 100 ms` / `10 Hz` — unconfigured modules remain at factory default **1 Hz**),
     - Dynamic platform model (`CFG-NAVSPG-DYNMODEL = 8` `Airborne <4g` — factory default `Portable` clamps velocity/altitude filter dynamics during agile flight),
     - NMEA output suppression (`CFG_UART1OUTPROT_NMEA = 0` — leaving bursty NMEA text active on a 256-byte RX ring buffer).

6. **SPL06 Calibration Burst Erratum (`drivers/src/baro/spl06.rs`) & Unfused Baro/Mag**
   - `Spl06::init_common` reads all 18 calibration bytes (`0x10..0x21`) in a **single 18-byte I2C burst**. Both Betaflight (`barometer_dps310.c:178-190`) and ArduPilot (`AP_Baro_SPL06.cpp:187-196`) split `COEF` reads into **9-byte chunks (`READ_LENGTH = 9`)** because SPL06/DPS310 I2C state machines NACK or corrupt bursts >9 bytes on several silicon lots.
   - `BARO_1` (`SPL06`) and `MAG_EXT` (`QMC5883L`) are published to telemetry/shell channels only and are **not fused** into `eskf_imu_gps.rs` (outdoor yaw is unobservable in still hover without dual-antenna UM982).

### P2 — MCU Memory/Cache Performance & Hardware Revision Nuances

7. **AXIM Flash Wait-States (`ARTEN=0`, `PRFTEN=0`, `DCache` Disabled) & 512 KB Dual-Sector Param Store**
   - STM32F745 has dual Flash interfaces: **64-bit ITCM (`0x0020_0000`)** with the ART Accelerator, and **32-bit AXIM (`0x0800_0000`)** cached by Cortex-M7 `ICache`/`DCache`.
   - Betaflight links firmware at `0x0020_0000` (`ITCM_FLASH`), places hot loops in `16 KB ITCM_RAM (0x0000_0000)`, and enables **`ARTEN + PRFTEN + ICache + DCache`** (with MPU non-cacheable DMA regions).
   - Cybflight links at `0x0800_0000` (`AXIM_FLASH`), enables **`ICache` only** (`DCache` disabled), and `embassy-stm32` writes `FLASH.ACR = 7` (`ARTEN = 0`, `PRFTEN = 0`). Every `.rodata` constant lookup over AXIM incurs raw **7-wait-state** Flash stalls.
   - Cybflight reserves Flash **Sectors 6 & 7 (`2 × 256 KB = 512 KB`, 50% of the 1 MB MCU)** for `ekv` parameters; erasing a 256 KB single-bank sector stalls all Flash instruction fetches for **~1.5–2.5 s** ( exceeding the `500 ms` `IWDG` timeout if ever triggered post-init). Both Betaflight and ArduPilot store parameters in **Sector 1 (`0x0800_8000`, 32 KB, ~300 ms erase)**, leaving ~960 KB for firmware.

8. **Hardware Revision Differences: Beeper Polarity (`PD15`) & Current Scale (`PC2`)**
   - **Current sensor (`PC2`):** Cybflight (`current_scale = 351` → `28.5 A/V`) matches ArduPilot `hwdef.dat` (`HAL_BATT_CURR_SCALE 28.5`) and `goku_ardu_params.params` (`BATT_AMP_PERVLT = 28.5`). Note that Flywoo's V3 manual specifies scale `714` (`14.0 A/V`) on newer V3 45A shunts — verify against actual mAh drawn if current telemetry is used.
   - **Beeper (`PD15`):** Betaflight `FLYWOOF745` sets `BEEPER_INVERTED` (active-low), whereas ArduPilot (`PD15 OUTPUT LOW`, `NTF_BUZZ_ON_LVL = 1`) and Cybflight (`BEEPER_INVERTED = false`) drive `PD15` active-high.

---

## 2. Master Side-by-Side Comparison Matrix

| Subsystem / Dimension | Cybflight (`flywoof745` + `goku_f745.yaml`) | Betaflight (`FLYWOOF745` / `FLYWOOF745NANO`) | ArduPilot (`FlywooF745` + `goku_ardu_params.params`) |
| :--- | :--- | :--- | :--- |
| **RTOS / Execution Model** | Rust `no_std` + **Embassy** 3-tier async (`P6` DShot, `P10` IMU/INDI/Failsafe/CRSF, `Thread` ESKF/GPS/Baro/Mag/USB) | Bare-metal cooperative scheduler + hard real-time EXTI/DMA ISRs | **ChibiOS/RT** preemptive multi-threaded RTOS (priority-scheduled threads + DMA/IRQ handlers) |
| **Crystal (`HSE`) & `SYSCLK`** | `8 MHz` → `PLLM=4, PLLN=216, PLLP=2, PLLQ=9` → **`216 MHz`** (`OverDrive` ON, `VOS` Scale 1) | `8 MHz` → `PLLM=8, PLLN=432, PLLP=2, PLLQ=9` → **`216 MHz`** (`OverDrive` ON, `VOS` Scale 1) | `8 MHz` → `PLLM=8, PLLN=432, PLLP=2, PLLQ=9` → **`216 MHz`** (`OverDrive` ON, `VOS` Scale 1) |
| **Bus Clocks (`AHB / APB1 / APB2`)** | `216 MHz / 54 MHz / 54 MHz` (`APB2=DIV4` so `TIM1` & `TIM3` both get **`108 MHz`** kernel clock) | `216 MHz / 54 MHz / 108 MHz` (`TIMPRE=1` → all timers **`216 MHz`**) | `216 MHz / 54 MHz / 108 MHz` (`TIMPRE=0` → `APB1` timers `108 MHz`, `APB2` timers `216 MHz`) |
| **FPU Target** | `cortex-m7`, `-C target-feature=-fp64` (**`FPv5-SP-D16`** hardware `f32`, soft `f64`) | `-mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard` (`f32` DSP) | `-mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard` (`f32` control, soft `f64` EKF/geodetic) |
| **Flash Bus & Wait States** | **`AXIM_FLASH` (`0x0800_0000`)**, `7 WS`, **`ARTEN=0`, `PRFTEN=0`** | **`ITCM_FLASH` (`0x0020_0000`)**, `7 WS`, **`ARTEN=1`, `PRFTEN=1`** + `16 KB ITCM_RAM` | **`AXIM_FLASH` (`0x0800_0000`)**, `7 WS`, `ARTEN=1`, `PRFTEN=1` |
| **Caches & MPU** | **`ICache` ON**, **`DCache` OFF**, **MPU OFF** (`DTCM` stack-underflow BusFault trap at `0x2000_0000`) | **`ICache` ON**, **`DCache` ON**, **MPU ON** (non-cacheable DMA SRAM sections) | **`ICache` ON**, **`DCache` ON** (coherent bounce/aligned DMA cache ops in ChibiOS) |
| **Firmware / Param Flash Split** | `512 KB` code (`Sectors 0–5`) + **`512 KB` KV params (`Sectors 6–7`, 2×256 KB)**; no bootloader offset | `16 KB` config (`Sector 1`, 32 KB) + `992 KB` code (`Sector 0` ISR + `Sectors 2–7`) | `96 KB` reserved (`Sector 0` BL + **`Sector 1` 32 KB `HAL_STORAGE`** + `Sector 2`) + `928 KB` app |
| **BKPSRAM & Crash Recovery** | **4 KB BKPSRAM (`0x4002_4000`)** postmortem record + panic text + PVD brownout + 10 s boot-loop USB Safe Mode; `IWDG` (`500 ms`) | RTC `BKP0R` DFU magic + `BKP` crash/reboot flags; `IWDG` in flight builds | Watchdog fault struct saved in RTC backup regs (`save_fault_watchdog`) + `IWDG` |
| **Motors `M1..M4` Pinout** | `PB0` (`TIM3_CH3`), `PB1` (`TIM3_CH4`), `PE9` (`TIM1_CH1`), `PE11` (`TIM1_CH2`) — **V1.2/V3** | `FLYWOOF745`: `PB1, PC9, PB0, PE11` (V1.0)<br>`FLYWOOF745NANO`: **`PB0, PB1, PE9, PE11` (V1.2/V3)** | `PWM1=PB0, PWM2=PB1, PWM3=PE9, PWM4=PE11` (`SERVO1..4 = 36,33,34,35` QuadX) — **V1.2/V3** |
| **Motors `M5..M8` Pinout** | Not wired (4-motor quad BSP only) | `PC9` (`TIM8_CH4`), `PA3` (`TIM2/5_CH4`), `PB4` (`TIM3_CH1`), `PB5` (`TIM3_CH2`) | `PWM5=PC9` (`TIM8_CH4`), `PWM6=PA3` (`TIM5_CH4`), `PWM7=PB4` (`TIM3_CH1`), `PWM8=PB5` (`TIM3_CH2`) |
| **DShot & Bi-Dir RPM Telemetry** | **Bi-dir DShot600 on all 4 motors** (~3.7 kHz frame rate on F745) via per-channel OC/IC DMA (`DMA1_S7/S2`, `DMA2_S1/S2`) | **Bi-dir DShot300/600 on all motors** via GPIO `BSRR` timer-paced DMA bitbang or per-channel OC/IC DMA + Dynamic Notch | `MOT_PWM_TYPE=6` (DShot600); `BIDIR` enabled on `TIM3` (`PB0`) & `TIM1` (`PE11`) (`BLH_MASK=15`) |
| **Primary IMU (`SPI4`)** | Auto-probes **`MPU6000` / `MPU6500` / `ICM426xx` / `BMI270`** (`PE2/PE5/PE6`, `CS=PE4`, `DRDY=PE1`) | Compile-time `MPU6000` / `ICM42688P` / `BMI270` (`SPI4`, `CS=PE4`, `EXTI=PE1`) | Runtime probes `MPU6000`, `BMI270`, `ICM42688` on `SPI4` (`CS=PE4`, FIFO-polled, no `PE1` EXTI) |
| **IMU SPI4 Clock (Init / Run)** | `843.75 kHz` (`54M/64`) → **`13.5 MHz`** (`54M/4`, MPU/ICM) or **`6.75 MHz`** (`54M/8`, BMI270) | `421.9 kHz` (`108M/256`) → **`13.5 MHz`** (`108M/8` actual when requesting `20 MHz`) | `843.75 kHz` (`108M/128`) → **`3.375 MHz`** (`108M/32`, MPU6000) or **`6.75 MHz`** (`108M/16`, ICM/BMI) |
| **IMU DRDY & Hardware DLPF** | `INT_PIN_CFG=0x30` (latched, `INT_RD_CLEAR`), `wait_for_high()`, `DLPF_CFG=0` (`256 Hz` BW, `8 kHz` ODR) | `INT_PIN_CFG=0x10` (50 µs pulse, `ANYRD_2CLEAR`), rising-edge EXTI ISR, `DLPF_CFG=0` (`8 kHz`) | **Hardware FIFO** (`MPU_FIFO_BUFFER_LEN=8` @ 8 kHz fast-sampling, `DLPF_CFG=0`, temp sync check) |
| **IMU Chip + Diamond Yaw Align** | `Cw270Deg` (MPU/ICM) / `Cw180Deg` (BMI) + `imu_rpy_y: 135.0` (FLU) — **verified identical** | `CW270_DEG` (`config.h`) + `align_board_yaw = 2250` (`225.0°` CW in CLI for diamond mount) | `ROTATION_YAW_180` (MPU) / `PITCH_180` (BMI) / `PITCH_180_YAW_90` (ICM) + `AHRS_ORIENTATION = 5` (`YAW_225`) |
| **Startup Gyro Calibration** | `GyroStartupCal` (`1.0 s`, `std <= 0.05 rad/s`), **NOT gated in `ArmStateMachine`!** | `1.25 s` window, Welford `stddev <= 48` LSB gate, **blocks arming (`CALIBRATING`)** | Dual 50-sample convergence (`<0.04 deg/s` diff, accel motion gate) + NVRAM offsets, **blocks arming** |
| **Barometer (`I2C1` `PB6/PB7`)** | `SPL06` (`0x76`), `400 kHz`, internal pull-ups ON, `64x` P / `8x` T (~9.3 Hz), **18-B burst read**, unfused | `DPS310` / `BMP280` / `SPL06` (`0x76`), `400 kHz`, **9-B chunked coef read**, fused for alt/Vario | `BMP280` / `SPL06` / `DPS310` (`0x76`), `400 kHz`, pull-ups ON, **9-B chunked coef read**, EKF3 fused |
| **Magnetometer (`I2C1` `0x0D`)** | `QMC5883L`, `200 Hz` ODR / `8G` / `512x` OSR, `Cw270Deg`, `DRDY+DOR` check, unfused | `QMC5883L` (`0x0D`), `200 Hz` / `8G`, auto-detected external align | Auto-probed `QMC5883L` (`0x0D`), `100 Hz` / `8G`, `ROTATION_YAW_90` (= `Cw270Deg` FLU), EKF3 fused |
| **GPS (`USART6` `PC6/PC7`)** | Scans `[115200..9600]`, `CFG-VALSET`→`CFG-MSG` (`NAV-PVT`), **no checksum check, no 10 Hz / Airborne-4g / NMEA off** | Auto-baud, `MON-VER` probe, `CFG-VALSET` (M9/M10) or `CFG-PRT/RATE/NAV5/MSG` (M8), 10 Hz, Airborne-4g, Fletcher-16 | Auto-baud to `230400`, `MON-VER`, full `CFG-VALSET` (`CFGv2`) / legacy `CFG-*`, 10 Hz, Airborne-4g, Fletcher-16 |
| **RC (`USART2` `PD5/PD6`)** | CRSF @ `420 kbaud` (`BufferedUart` 128 B on P10); **buggy `SpeedProposal` ACK**, **reversed `outer_rate` yaw** | CRSF @ `420 kbaud` (DMA/IRQ); full CRSFv3 speed switch + error-count fallback to `420 kbaud` | `SERIAL2_PROTOCOL=23` (RCIN CRSF), full CRSFv2/v3 parser + telemetry (`RSSI_TYPE=3`) |
| **ADC (`VBAT=PC3`, `CURR=PC2`)** | `ADC3` (`IN13`, `IN12`), software-polled @ `100 Hz` (`480` cycles), `10.9×` V, `28.5 A/V` (`scale=351`) | `ADC1` circular DMA (`PC3`, `PC2`, `PC5` RSSI), `vbat_scale=110` (`11.0×`), `ibata_scale=170` (`58.8 A/V` V1 default) | `ADC1` ChibiOS DMA (`PC3=13`, `PC2=12`, `PC5=15`), `BATT_VOLT_MULT=10.9`, `BATT_AMP_PERVLT=28.5` |
| **Onboard SPI1 Flash & SPI2 OSD** | Pins claimed (`PA4/5/6/7`, `PB12/13/14/15`, CS high), **unused** (`HAS_BLACKBOX_STORAGE = false`) | `W25Q128` 16 MB Blackbox (`SPI1` DMA) + `AT7456E` analog OSD (`SPI2` DMA) | `W25Q128` DataFlash logging (`SPI1` @ `104 MHz` spec / `54 MHz` actual) + `AT7456E` OSD (`SPI2` @ `10 MHz`) |

---

## 3. Subsystem 1: MCU Clocks, PLL Tree, Flash Layout, FPU, Caches, MPU, BKPSRAM & Watchdog

### 3.1 PLL & Clock Tree (`STM32F745VGT6`, `HSE = 8 MHz`)

All three firmwares run the Cortex-M7 core at its maximum **`216 MHz`** with `PWR_CR1.VOS = Scale 1` and Over-Drive (`ODEN` + `ODSWEN`) enabled, and `FLASH_ACR.LATENCY = 7` wait states, but differ intentionally in **`APB2` prescaling** and **`TIMPRE`**:

| Clock Domain | Cybflight (`bsp/flywoof745/src/lib.rs:273-290`) | Betaflight (`system_stm32f7xx.c`) | ArduPilot (`F745-8MHz.ioc` / `mcuconf.h`) |
| :--- | :--- | :--- | :--- |
| **PLL Input / VCO** | `8 MHz / 4 × 216 = 432 MHz` (`2 MHz` phase detector) | `8 MHz / 8 × 432 = 432 MHz` (`1 MHz` phase detector) | `8 MHz / 8 × 432 = 432 MHz` (`1 MHz` phase detector) |
| **`SYSCLK` / `HCLK` (AHB)** | `432 / 2 = 216 MHz` (`DIV1`) | `432 / 2 = 216 MHz` (`DIV1`) | `432 / 2 = 216 MHz` (`DIV1`) |
| **`PLL48CLK` (USB OTG FS)** | `432 / 9 = 48.0 MHz` | `432 / 9 = 48.0 MHz` | `432 / 9 = 48.0 MHz` |
| **`PCLK1` (`APB1` Peripherals)** | `216 / 4 = 54 MHz` | `216 / 4 = 54 MHz` | `216 / 4 = 54 MHz` |
| **`APB1` Timer Kernel (`TIM3`, `TIM12`)** | **`108 MHz`** (`2 × PCLK1`, `TIMPRE=0`) | **`216 MHz`** (`TIMPRE=1` → `4 × PCLK1`) | **`108 MHz`** (`2 × PCLK1`, `TIMPRE=0`) |
| **`PCLK2` (`APB2` Peripherals: `SPI1/4`, `USART1/6`, `ADC`)** | **`54 MHz` (`DIV4`)** | **`108 MHz` (`DIV2`)** | **`108 MHz` (`DIV2`)** |
| **`APB2` Timer Kernel (`TIM1`, `TIM8`)** | **`108 MHz`** (`2 × PCLK2`, `TIMPRE=0`) | **`216 MHz`** (`2 × PCLK2`, `TIMPRE=1`) | **`216 MHz`** (`2 × PCLK2`, `TIMPRE=0`) |

- **Why Cybflight uses `APB2 = DIV4` (`54 MHz`):** `motors/mod.rs:43` computes a single compile-time `DSHOT600_PSC = (APB2_TIMER_HZ / 12_000_000) - 1 = 8` shared across `TIM3` (`APB1`) and `TIM1` (`APB2`). Setting `APB2 = DIV4` makes both `APB1_TIMER_HZ` and `APB2_TIMER_HZ` equal `108 MHz` (`9 × 12 MHz`), verified at boot against DWT cycle counts in `clocks::verify_clocks()` (`clocks.rs:85-137`).
- **Trade-off of `PCLK2 = 54 MHz`:** Halves maximum `SPI1` (W25Q128 flash) and `ADC3` peripheral bus clocks compared to Betaflight/ArduPilot (`27 MHz` max SPI vs `54 MHz`), though `SPI4` IMU reads at `13.5 MHz` (`54 MHz / 4`) are unaffected.

### 3.2 FPU (`FPv5-SP-D16`) Trap & Codegen
- **Hardware Reality:** Unlike STM32H743 (`FPv5-D16`, hardware `f32` + `f64`) and STM32F765, **`STM32F745VG` implements single-precision `FPv5-SP-D16` only**. Executing any 64-bit VFP instruction (`vldr.64`, `vmul.f64`, `vcvt.f64.f32`) immediately faults with **`UsageFault: UNDEFINSTR`** (or escalates to `HardFault`).
- **Cybflight Mitigation (`.cargo/config.toml:7`):** Compiles `thumbv7em-none-eabihf` with `["-C", "target-cpu=cortex-m7", "-C", "target-feature=-fp64"]`, keeping full single-cycle hardware VFP `f32` for the 8 kHz IMU biquads, 2 kHz INDI controller, and ESKF while lowering rare `f64` operations (`geodetic::llh_to_enu` at 10 Hz GPS rate and boot-time `SampleStamper` period calculation) to compiler-rt soft-float. Notably, `GyroStartupCal` (`sensors/imu.rs:68-80`) uses a first-sample-centered `f32` shift (`d = gyro - ref_sample`) specifically to avoid soft-float `f64` at 8 kHz.

### 3.3 Flash Bus (`ITCM` vs `AXIM`), Caches (`ICache`/`DCache`), and MPU
- **Betaflight:**
  - Links `.text` and `.rodata` at **`0x0020_0000` (`ITCM_FLASH`, 64-bit wide)** and relocates `FAST_CODE` ISRs/filters into **`16 KB ITCM_RAM` (`0x0000_0000`, 0-WS)**.
  - Enables **`FLASH_ACR_ARTEN` (ART Accelerator) + `FLASH_ACR_PRFTEN` (Prefetch) + `SCB_EnableICache()` + `SCB_EnableDCache()`**, and configures the Cortex-M7 **MPU** to mark DMA buffers (`DMA_RAM`) non-cacheable/shareable.
- **ArduPilot:**
  - Links at `0x0801_8000` (`AXIM_FLASH`), enables `ICache` + `DCache` + `ARTEN`/`PRFTEN`, and manages cache coherency in ChibiOS DMA/bounce buffers.
- **Cybflight (`platform.rs:6-9`, `memory_f745.x`):**
  - Links at **`0x0800_0000` (`AXIM_FLASH`)**, enables **`ICache` only**, leaves **`DCache` disabled** (making `SRAM1`/`SRAM2` at `0x2001_0000` naturally DMA-coherent without MPU regions), and places MSP + interrupt executor stacks in **`64 KB DTCM` (`0x2000_0000..0x2001_0000`)** so stack overflow below `0x2000_0000` triggers a deterministic `BusFault` instead of silently corrupting `.bss`.
  - **Performance Gap Found:** In `embassy-stm32-0.5.0/src/rcc/f247.rs`, `FLASH.acr().write(|w| w.set_latency(7))` overwrites `FLASH_ACR` with `0x0000_0007` (**`ARTEN = 0`, `PRFTEN = 0`**). Because Cybflight fetches via `AXIM` (`0x0800_0000`), `ICache` accelerates instruction fetches, **but every `.rodata` constant table read (filter coefficients, param metadata, format strings, lookup tables) bypasses `ICache` and hits uncached 7-wait-state AXIM Flash** because `DCache` is off! Enabling `PRFTEN` (or `DCache` with an MPU non-cacheable DMA Carveout in `SRAM2` `0x2004_C000`) will eliminate `.rodata` wait-state stalls.

### 3.4 Internal Flash Sector Layout & Parameter Erase Stall
`STM32F745VG` has 1 MB of **single-bank** Flash:
- `Sectors 0..3`: `4 × 32 KB` (`0x0800_0000..0x0802_0000`)
- `Sector 4`: `1 × 128 KB` (`0x0802_0000..0x0804_0000`)
- `Sectors 5..7`: `3 × 256 KB` (`0x0804_0000..0x0810_0000`)

| Firmware | Parameter Storage Sector(s) | Firmware Code Budget | Erase CPU Stall Impact |
| :--- | :--- | :--- | :--- |
| **Betaflight** | **Sector 1 (`0x0800_8000`, `32 KB`)** | `992 KB` (`Sector 0` vectors + `Sectors 2..7`) | ~250–400 ms (disarmed CLI save only) |
| **ArduPilot** | **Sector 1 (`0x0800_8000`, `32 KB`, `HAL_STORAGE_SIZE = 16 KB`)** | `928 KB` (`0x0801_8000..0x0810_0000`, after `Sector 0` bootloader) | ~250–400 ms (background storage thread; `in_expected_delay()` suppresses IMU/WDT faults) |
| **Cybflight** | **Sectors 6 & 7 (`0x0808_0000..0x0810_0000`, `2 × 256 KB = 512 KB`)** | **`512 KB`** (`Sectors 0..5`) | **~1.5–2.5 s** bus stall per 256 KB sector erase on single-bank F745! |

> **Warning on Cybflight Flash Erase + Watchdog:** Because STM32F745 is single-bank, erasing Sector 6 or 7 during `param save` or `param format` stalls AXIM instruction fetches for **1.5–2.5 seconds** — **longer than Cybflight's `500 ms` `IWDG` timeout** (`watchdog.rs`). Moving the `ekv` store to **Sectors 1 & 2 (`2 × 32 KB = 64 KB`)** via a split linker script would reclaim **416 KB** of firmware flash AND cut erase stall time by **8×** (~250 ms).

### 3.5 BKPSRAM, DFU Bootloader Entry & Fault Recovery
- **Cybflight (`platform.rs`, `postmortem/bkpsram.rs`):**
  - Correctly handles two subtle **STM32F745 hardware traps** that do not exist on STM32H743:
    1. **Device-memory unaligned fault at `0x4002_4000`:** On Cortex-M7, `0x4000_0000..0x5FFF_FFFF` has architectural `Device` memory attributes where **any unaligned load/store faults (`UsageFault: UNALIGNED`)**, even on byte slices. `postmortem/bkpsram.rs:147-290` strictly accesses `POSTMORTEM_SLOT` and `POSTMORTEM_PANIC_TEXT` via 32-bit aligned `read_volatile`/`write_volatile` word loops into an aligned SRAM staging buffer, respecting STM32F7 Errata ES0290 §2.1.6 RCC readback + `DSB`.
    2. **RTC Backup Register DFU Lock-Loop Prevention (`platform.rs:24-39, 67-78`):** System reset clears `RCC.APB1ENR.PWREN` and `PWR.CR1.DBP` on F745; `pre_init()` explicitly re-enables `PWREN` + `DBP` before zeroing `RTC_BKP0R` (`0x4000_2850`) and jumping to STM32 system DFU ROM (`0x1FF0_0000`).
  - Also implements early-boot crash detection (`postmortem/recovery.rs`): if a boot panics/faults within the first 10 s, the next reboot enters **USB Safe Mode** (spawning only USB CDC + status LED + `IWDG` feed) so the operator can run `postmortem show` or `dfu` without a crash-reboot loop.

---

## 4. Subsystem 2: Complete Pinout, Buses, Timers, DMA Streams & Bidirectional DShot

### 4.1 Complete Pin-by-Pin Hardware Mapping (`STM32F745VG` LQFP100)

| Pin(s) | Hardware Function on GOKU GN 745 V3 | Cybflight (`bsp/flywoof745`) | Betaflight (`FLYWOOF745` / `NANO`) | ArduPilot (`FlywooF745/hwdef.dat`) |
| :--- | :--- | :--- | :--- | :--- |
| `PA4 / PA5 / PA6 / PA7` | **SPI1:** `W25Q128` 16 MB NOR Flash (`CS / SCK / MISO / MOSI`) | Claimed (`flash_cs = High`), **not driven** | `FLASH_CS=PA4`, `USE_FLASH_M25P16` Blackbox | `SPIDEV dataflash SPI1` (`HAL_LOGGING_DATAFLASH_ENABLED=1`) |
| `PB12 / PB13 / PB14 / PB15` | **SPI2:** `AT7456E` Analog OSD (`CS / SCK / MISO / MOSI`) | Claimed (`osd_cs = High`), **not driven** | `MAX7456_SPI_CS=PB12`, `SPI2` OSD | `SPIDEV osd SPI2 MODE0 10*MHZ` (`OSD_ENABLED=1`) |
| `PE4 / PE2 / PE5 / PE6` | **SPI4:** Primary IMU (`CS / SCK / MISO / MOSI`) | `SPI4` (`DMA2_S4` TX, `DMA2_S0` RX), `CS=PE4` | `SPI4`, `GYRO_1_CS=PE4` | `SPIDEV mpu6000/icm42688/bmi270 SPI4`, `CS=PE4` |
| `PE1` | **IMU Data-Ready (`EXTI1`)** | `ExtiInput` (`Pull::None`, `EXTI1` IRQ) | `GYRO_1_EXTI_PIN = PE1` | Not configured (uses 8 kHz SPI FIFO polling) |
| `PB6 / PB7` | **I2C1 (`SCL / SDA`):** Onboard Baro (`0x76`) + Ext Mag (`0x0D`) | `I2C1` @ `400 kHz`, pull-ups ON (`DMA1_S6/S0`) | `I2C_DEVICE_1` (`PB6/PB7`) @ `400 kHz` | `I2C1` (`PB6/PB7 PULLUP`) @ `400 kHz` |
| `PB10 / PB11` | **USART3 (`TX3 / RX3`)** *(alt `I2C2` on V1)* | `USART3` `BufferedUart` (alt GPS port) | `UART3` (`PB10/PB11`) & `I2C2` (`PB10/PB11`) | `USART3` (`SERIAL3`, DMA TX/RX; `SERIAL3_PROTOCOL=2` MAVLink2) |
| `PA9 / PA10` | **USART1 (`TX1 / RX1`)** | `PORT_ESP_BRIDGE` (`460800` baud, `DMA2_S7/S5`) | `UART1` | `USART1` (`SERIAL1`, TX DMA / RX NODMA; MAVLink2 @ `57600`) |
| `PD5 / PD6` | **USART2 (`TX2 / RX2`)** | `PORT_SERIAL_RX` (`CRSF` @ `420000`, `Pull::Up`) | `UART2` (`SERIALRX_UART` default) | `USART2` (`SERIAL2`, TX DMA / RX NODMA; `PROTOCOL=23` RCIN) |
| `PA0 / PA1` | **UART4 (`TX4 / RX4`)** | `UART4` `BufferedUart` (alt SerialRx) | `UART4` | `UART4` (`SERIAL4`, TX DMA / RX NODMA; DJI FPV OSD `PROTOCOL=33` in `.params`) |
| `PC12 / PD2` | **UART5 (`TX5 / RX5`)** | `UART5` `BufferedUart` (alt SerialRx) | `UART5` | `UART5` (`SERIAL5`, NODMA) |
| `PC6 / PC7` | **USART6 (`TX6 / RX6`)** | `PORT_GPS` (`BufferedUart`, `115200` scan) | `UART6` | `USART6` (`SERIAL6`, TX DMA / RX NODMA; `PROTOCOL=5` GPS @ `115200`) |
| `PE8 / PE7` | **UART7 (`TX7 / RX7`)** | `PE8` = blocking `defmt` logger @ `921600` (`PE7` unused) | `UART7` (`PE8/PE7`, ESC telemetry RX7 on some builds) | `UART7` (`SERIAL7`, NODMA; `PROTOCOL=16` ESC Telem in `.params`) |
| `PA11 / PA12` | **USB OTG FS (`DM / DP`)** | USB CDC shell + telemetry (`OTG_FS` IRQ) | USB VCP + MSC + DFU | `OTG1` (`SERIAL0` MAVLink) |
| `PB0` | **Motor 1 (Rear-Right, CW)** | `TIM3_CH3` AF2 (`DMA1_S7` Ch5) | `M3` in `FLYWOOF745` (V1.0); **`M1` in `FLYWOOF745NANO` (V1.2/V3)** | `PWM(1)` `TIM3_CH3` `BIDIR` (`SERVO1_FUNCTION=36` Motor4/RR-CW!) |
| `PB1` | **Motor 2 (Front-Right, CCW)** | `TIM3_CH4` AF2 (`DMA1_S2` Ch5) | `M1` in `FLYWOOF745` (V1.0); **`M2` in `FLYWOOF745NANO` (V1.2/V3)** | `PWM(2)` `TIM3_CH4` (`SERVO2_FUNCTION=33` Motor1/FR-CCW!) |
| `PE9` | **Motor 3 (Rear-Left, CCW)** | `TIM1_CH1` AF1 (`DMA2_S1` Ch6) | `M5` in `FLYWOOF745` (V1.0); **`M3` in `FLYWOOF745NANO` (V1.2/V3)** | `PWM(3)` `TIM1_CH1` (`SERVO3_FUNCTION=34` Motor2/RL-CCW!) |
| `PE11` | **Motor 4 (Front-Left, CW)** | `TIM1_CH2` AF1 (`DMA2_S2` Ch6) | `M4` (`TIM1_CH2`) in both V1.0 & V1.2/V3 | `PWM(4)` `TIM1_CH2` `BIDIR` (`SERVO4_FUNCTION=35` Motor3/FL-CW!) |
| `PC9 / PA3 / PB4 / PB5` | **Motors 5..8** *(Hex/Octo pads)* | Unassigned | `TIM8_CH4 / TIM2_CH4 / TIM3_CH1 / TIM3_CH2` | `PWM(5..8)`: `TIM8_CH4 / TIM5_CH4 / TIM3_CH1 / TIM3_CH2` |
| `PD12` | **WS2812 LED Strip** | Claimed (`TIM4_CH1`), not spawned in `board_init` | `TIM4_CH1` (`DMA1_S0`) | `PWM(9)` `TIM4_CH1` NeoPixel (`NTF_LED_TYPES=257`) |
| `PA2` | **Status LED0** | `Output` (`active_low = false`) | `LED0_PIN = PA2` | `PA2 LED0 OUTPUT LOW` |
| `PD15` | **Piezo Beeper** | `Output`, `BEEPER_INVERTED = false` | `BEEPER_PIN = PD15`, **`BEEPER_INVERTED`** | `PD15 BUZZER OUTPUT LOW` (`NTF_BUZZ_ON_LVL=1`) |
| `PC3 / PC2 / PC5` | **ADC (`VBAT / CURR / RSSI`)** | `ADC3` (`IN13`, `IN12`; `PC5` claimed unused) | `ADC1` (`PC3`, `PC2`, `PC5`) | `ADC1` (`PC3=13`, `PC2=12`, `PC5=15`) |
| `PC14` | **PINIO1 (9V VTX BEC Switch)** | **Not configured** (floats at reset state) | `PINIO1_PIN = PC14` (`PINIO1_CONFIG = 129` inverted) | User GPIO (`RELAY` / `PINIO`) if configured |

### 4.2 Motor Order & Spin-Direction Verification Against `goku_ardu_params.params`
A common pitfall on Flywoo F745 boards is confusing V1.0 (`FLYWOOF745`: `PB1, PC9, PB0, PE11`) with V1.2/V3 (`FLYWOOF745NANO` / 45A AIO V3: `PB0, PB1, PE9, PE11`). `goku_ardu_params.params` proves the physical wiring on this exact vehicle:
- ArduCopter QuadX motor functions are: `33 = Motor1 (Front-Right CCW)`, `34 = Motor2 (Rear-Left CCW)`, `35 = Motor3 (Front-Left CW)`, `36 = Motor4 (Rear-Right CW)`.
- In `goku_ardu_params.params`:
  - `SERVO1_FUNCTION = 36` → **`PB0` = Rear-Right (CW)** = Betaflight/Cybflight **`m0` (`[-0.0315, -0.0315]`, `cw`)**
  - `SERVO2_FUNCTION = 33` → **`PB1` = Front-Right (CCW)** = Betaflight/Cybflight **`m1` (`[+0.0315, -0.0315]`, `ccw`)**
  - `SERVO3_FUNCTION = 34` → **`PE9` = Rear-Left (CCW)** = Betaflight/Cybflight **`m2` (`[-0.0315, +0.0315]`, `ccw`)**
  - `SERVO4_FUNCTION = 35` → **`PE11` = Front-Left (CW)** = Betaflight/Cybflight **`m3` (`[+0.0315, +0.0315]`, `cw`)**
- **Result:** Cybflight's motor pinout, positions, and spin directions in `bsp/flywoof745/src/lib.rs` and `vehicles/goku_f745.yaml:29-33` **match the physical V3 airframe 100%**.

### 4.3 Cybflight DMA1 / DMA2 Stream Audit (Zero Collisions)

| DMA Controller & Stream | Channel (`dma_request`) | Peripheral Signal | Usage in Cybflight `flywoof745` |
| :--- | :--- | :--- | :--- |
| `DMA1_Stream0` (`DMA1_CH0`) | Ch 1 | `I2C1_RX` | Shared `I2C1` RX (`SPL06` + `QMC5883L`) |
| `DMA1_Stream2` (`DMA1_CH2`) | Ch 5 | `TIM3_CH4` | **Motor 2 (`PB1`)** Bi-dir DShot600 OC TX + GCR IC RX |
| `DMA1_Stream6` (`DMA1_CH6`) | Ch 1 | `I2C1_TX` | Shared `I2C1` TX (`SPL06` + `QMC5883L`) |
| `DMA1_Stream7` (`DMA1_CH7`) | Ch 5 | `TIM3_CH3` | **Motor 1 (`PB0`)** Bi-dir DShot600 OC TX + GCR IC RX |
| `DMA2_Stream0` (`DMA2_CH0`) | Ch 4 | `SPI4_RX` | **Primary IMU (`SPI4` MISO `PE5`)** 8 kHz burst RX |
| `DMA2_Stream1` (`DMA2_CH1`) | Ch 6 | `TIM1_CH1` | **Motor 3 (`PE9`)** Bi-dir DShot600 OC TX + GCR IC RX |
| `DMA2_Stream2` (`DMA2_CH2`) | Ch 6 | `TIM1_CH2` | **Motor 4 (`PE11`)** Bi-dir DShot600 OC TX + GCR IC RX |
| `DMA2_Stream4` (`DMA2_CH4`) | Ch 5 | `SPI4_TX` | **Primary IMU (`SPI4` MOSI `PE6`)** 8 kHz burst TX |
| `DMA2_Stream5` (`DMA2_CH5`) | Ch 4 | `USART1_RX` | ESP / Offboard bridge RX (`PA10`) |
| `DMA2_Stream7` (`DMA2_CH7`) | Ch 4 | `USART1_TX` | ESP / Offboard bridge TX (`PA9`) |

*Note:* If `TIM4_CH1` (`PD12` WS2812 LED strip) is ever enabled later, its STM32F7 DMA mapping (`DMA1_Stream0` Ch 2) **collides with `I2C1_RX` (`DMA1_Stream0` Ch 1)** (`I2C1_RX` could move to `DMA1_Stream5` Ch 1 if needed). Today `PD12` is unused, so all 10 active DMA streams are completely conflict-free.

### 4.4 Bidirectional DShot600 Implementation Comparison
- **Cybflight (`motors/dshot.rs`):** Runs on `EXECUTOR_HIGH` (`CAN2_TX` NVIC **P6**). Uses **per-channel timer Output Compare (`PWM_MODE1`, inverted polarity `CCxP=1`) + Input Capture (`CCMR_IC = 0x2121`, both edges `CCER_IC = 0xBBBB`)** across `TIM3` and `TIM1`, holding the pins HIGH (`MODER::OUTPUT`, `BSRR`) during OC↔IC register transitions (`PIN_SETTLE_NS = 420 ns`) just like Betaflight's HAL DShot driver. Because `embassy-time` runs at `32,768 Hz` (`30.5 µs`/tick), `dshot.rs:524-525` paces F745 frames with `Timer::after_micros(160)` (**~3.7 kHz DShot frame rate**, well above the 2 kHz INDI update rate `indi_ctrl_div = 4`) so P6 does not starve lower-priority tasks.
- **Betaflight:** Defaults to **DShot Bitbang** (`TIM1`/`TIM8`-paced DMA writing directly to `GPIOx->BSRR` and sampling `GPIOx->IDR` at 2.5× oversampling) or per-channel HAL OC/IC DMA, feeding per-motor eRPM into the **Dynamic RPM Notch Filter** (`gyro_rpm_notch_harmonics`). Note that in `goku_f745.yaml`, Cybflight captures DShot eRPM telemetry (`DSHOT_TELEMETRY`), while `indi_rpm_notch_en` defaults to `0` unless explicitly enabled.

---

## 5. Subsystem 3: Primary IMU, Orientation Proofs & Startup Gyro Calibration

### 5.1 SPI4 Clock Prescalers, `INT_PIN_CFG` & Hardware Filtering (`MPU-6000` / `ICM-42688-P` / `BMI270`)

| Parameter | Cybflight (`mpu6x00.rs` / `icm426xx.rs` / `bmi270.rs`) | Betaflight (`accgyro_spi_mpu6000.c`) | ArduPilot (`AP_InertialSensor_Invensense.cpp`) |
| :--- | :--- | :--- | :--- |
| **SPI Mode** | `MODE_3` (`CPOL=1, CPHA=1`) | `MODE_3` | `MODE3` |
| **Init / Config SCK** | `54 MHz / 64 =` **`843.75 kHz`** (`<= 1 MHz` spec) | `108 MHz / 256 =` **`421.88 kHz`** (`0.5 MHz` req) | `108 MHz / 128 =` **`843.75 kHz`** (`1 MHz` req) |
| **Runtime Read SCK** | **MPU6000 / ICM426xx:** `54 MHz / 4 =` **`13.5 MHz`**<br>**BMI270:** `54 MHz / 8 =` **`6.75 MHz`** | **MPU6000:** `108 MHz / 8 =` **`13.5 MHz`** (`20 MHz` req)<br>**BMI270:** `108 MHz / 16 =` **`6.75 MHz`** (`10 MHz` req) | **MPU6000:** `108 MHz / 32 =` **`3.375 MHz`** (`4 MHz` req)<br>**ICM / BMI:** `108 MHz / 16 =` **`6.75 MHz`** (`8 MHz` req) |
| **Read Mechanism** | 15-byte DMA burst (`0x3B \| 0x80` + 14 B) on `PE1` level-high (`wait_for_high()`) | 14-byte DMA burst triggered directly by `PE1` rising-edge EXTI ISR | Periodic ChibiOS thread draining **hardware FIFO** (`MPUREG_FIFO_R_W`, up to 8 samples/burst) + temp sync check + `check_next_register()` |
| **`INT_PIN_CFG` (`0x37`)** | **`0x30`** (`LATCH_INT_EN \| INT_RD_CLEAR`) | **`0x10`** (`INT_ANYRD_2CLEAR`, 50 µs pulse) | **`0x30`** (`LATCH_INT_EN \| INT_RD_CLEAR`) |
| **Gyro / Accel FSR** | `±2000 dps` (`0x18`) / `±16 g` (`0x18`) | `±2000 dps` / `±16 g` | `±2000 dps` / `±16 g` (+ Rev C4/C5 half-scale `PRODUCT_ID` check) |
| **Hardware DLPF / AAF** | **MPU6000:** `CONFIG=0x00` (`256 Hz` gyro BW, `8 kHz` ODR; `260 Hz` / `1 kHz` accel)<br>**ICM42688P:** UI Filter Order 2 (`BW = ODR/4`), **hardware AAF left at chip default (`~1961 Hz` 3dB)** | **MPU6000:** `CONFIG=0x00` (`256 Hz` / `8 kHz`)<br>**ICM42688P:** **Configures Bank 1/2 hardware AAF (`258 Hz` gyro / `258 Hz` accel)**, UI filters bypassed! | **MPU6000:** `DLPF_CFG=0` (`256 Hz` / `8 kHz` fast-sampling)<br>**ICM42688P:** Hardware AAF (`~536 Hz` or configurable) + 20-bit hires FIFO |
| **Timestamping** | PLL-like `SampleStamper` (`Q16` ticks) reconstructs jitter-free 8 kHz grid & counts `IMU1_LOST_SAMPLES` | Hard EXTI ISR timestamp (`cmpTimeUs`) | FIFO sample-rate interpolation + PLL clock-drift tracker |

- **Why Cybflight's `INT_PIN_CFG = 0x30` + `wait_for_high()` works well:** Because `mpu_reader_task` shares `EXECUTOR_CTRL` (P10) cooperatively with `indi_task` (~40–60 µs compute), a 50 µs unlatched pulse (`0x10`) can expire while INDI is executing if `wait_for_rising_edge()` is not yet armed, dropping ~19% of samples. Latching `INT` (`0x20`) + `INT_RD_CLEAR` (`0x10`) + level-triggered `wait_for_high()` (`mpu6x00.rs:167, 282`), paired with `SampleStamper` (`sensors/imu.rs:392-397`), turns cooperative scheduling jitter into zero-loss delayed reads with reconstructed on-grid timestamps.
- **Minor `ImuReader` Recovery Note (`sensors/imu.rs:384`):** `Mpu6x00::read()` awaits `self.drdy.wait_for_high()` **without a timeout** (whereas `Icm426xx` wraps DRDY in a timeout). If an EMI spike ever resets the MPU-6000 (`PWR_MGMT_1` defaults back to sleep `0x40`, stopping DRDY), `self.imu.read().await` will pend forever instead of entering the `Err(e) => self.imu.recover()` branch at line 461. Adding a `with_timeout(Duration::from_millis(5), ...)` around `self.imu.read()` closes that corner case.

### 5.2 Complete Mathematical Proof of Chip Alignment & 45° Diamond Airframe Mounting

On this vehicle (`goku_ardu_params.params`), the AIO board is mounted **diamond-style** (`AHRS_ORIENTATION = 5` = `ROTATION_YAW_225` in ArduPilot). Let $(r_x, r_y, r_z)$ be raw sensor register counts:

1. **ArduPilot (`FlywooF745/hwdef.dat` + `goku_ardu_params.params`):**
   - `AP_InertialSensor_Invensense.cpp:686-688, 714-716` maps raw MPU-6000 registers to sensor FRD:
     $$v_0 = (r_y,\; r_x,\; -r_z)$$
   - `hwdef.dat:143` applies `ROTATION_YAW_180` (`vector3.cpp:51-53`), giving PCB FRD:
     $$v_{\text{pcb,FRD}} = (-r_y,\; -r_x,\; -r_z)$$
     Converting FRD $(x, y, z) \mapsto \text{FLU } (x, -y, -z)$ yields PCB FLU:
     $$v_{\text{pcb,FLU}} = (-r_y,\; +r_x,\; +r_z) = \text{SensorAlign::Cw270Deg}(r_x, r_y, r_z)$$
     *(Identical to Betaflight `FLYWOOF745/config.h`: `#define GYRO_1_ALIGN CW270_DEG`! Similarly, `BMI270` `ROTATION_PITCH_180` → `(-rx, +ry, -rz)` FRD → `(-rx, -ry, +rz)` FLU = `Cw180Deg`, and `ICM42688` `ROTATION_PITCH_180_YAW_90` → `(-ry, -rx, -rz)` FRD → `(-ry, +rx, +rz)` FLU = `Cw270Deg` — **all three IMU variants in `board_init/flywoof745.rs` match ArduPilot exactly**.)*
   - Next, `AHRS_ORIENTATION = 5` (`ROTATION_YAW_225`, `vector3.cpp:54-58`) rotates $v_{\text{pcb,FRD}}$ into Body FRD:
     $$v_{\text{body,FRD}} = \left(\frac{\sqrt{2}}{2}(r_y - r_x),\;\; \frac{\sqrt{2}}{2}(r_x + r_y),\;\; -r_z\right)$$
     Converting Body FRD to **Body FLU** $(x, -y, -z)$:
     $$v_{\text{body,FLU}}^{\text{ArduPilot}} = \left(\frac{\sqrt{2}}{2}(r_y - r_x),\;\; -\frac{\sqrt{2}}{2}(r_x + r_y),\;\; +r_z\right)$$

2. **Cybflight (`bsp/flywoof745/src/lib.rs:407` + `goku_f745.yaml:39` + `sensors/imu.rs:275-285, 399-404`):**
   - `apply_alignment(Cw270Deg, r)` gives $u = (-r_y, r_x, r_z)$.
   - `imu_rpy_y: 135.0` builds $R_z(+135^\circ)$ with $\cos 135^\circ = -\frac{\sqrt{2}}{2}, \sin 135^\circ = +\frac{\sqrt{2}}{2}$:
     $$R_z(+135^\circ)\,u = \begin{bmatrix} -\frac{\sqrt{2}}{2} & -\frac{\sqrt{2}}{2} & 0 \\ +\frac{\sqrt{2}}{2} & -\frac{\sqrt{2}}{2} & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} -r_y \\ r_x \\ r_z \end{bmatrix} = \begin{bmatrix} \frac{\sqrt{2}}{2}(r_y - r_x) \\ -\frac{\sqrt{2}}{2}(r_x + r_y) \\ +r_z \end{bmatrix}$$
   - **Conclusion:** Cybflight's `Cw270Deg` + `imu_rpy_y: 135.0` is **algebraically identical** in all three axes to the flown ArduPilot configuration.

### 5.3 Startup Gyro Bias Calibration Comparison

| Feature | Betaflight (`sensors/gyro.c:218-272`) | ArduPilot (`AP_InertialSensor.cpp:1709-1857`) | Cybflight (`sensors/imu.rs:62-150, 406-425`) |
| :--- | :--- | :--- | :--- |
| **Window Duration** | `1.25 s` (`10,000` samples @ 8 kHz) | Dual `50`-sample windows (`~1.0–2.5 s`) | `1.0 s` (`8,000` samples @ 8 kHz) |
| **Motion Rejection** | Welford per-axis `stddev <= 48` LSB (`~2.93 deg/s`); restarts window + beeps if moved | Consecutive window diff `< 0.04 deg/s` (`0.0007 rad/s`) + accel diff `< 0.2 m/s²` | Shifted `f32` variance `std <= 0.05 rad/s` (`~2.86 deg/s`) + sanity ceiling `\|bias\| <= 0.35 rad/s` (`20 deg/s`) |
| **Arming Interlock** | **YES** (`ARMING_DISABLED_CALIBRATING`) | **YES** (`!gyros_calibrated()` blocks pre-arm) | **NO (`ArmStateMachine` ignores `IMU1_GYRO_CALIBRATED`!)** |

- **Why the missing arming interlock matters on this hardware:** In `goku_ardu_params.params`, this board's MPU-6000 zero-rate bias was calibrated to `INS_GYROFFS = [-0.0029, +0.0159, +0.0496] rad/s` (**`2.84 deg/s` yaw offset**). If a pilot plugs in the battery on uneven ground and flips ARM within `< 1.0 s` of setting the quad down, `GyroStartupCal::observe` sees `armed == true`, resets its window, and never completes calibration (`bias` remains `[0, 0, 0]`). Adding `if !crate::sensors::imu::IMU1_GYRO_CALIBRATED.load(Ordering::Acquire)` as an arming gate in `sensors/rc.rs` makes calibration deterministic.

---

## 6. Subsystem 4: Barometer (`SPL06` `0x76`) & External Magnetometer (`QMC5883L` `0x0D`) on `I2C1`

### 6.1 Shared `I2C1` Bus (`PB6` SCL / `PB7` SDA) Configuration & Error Recovery
- **Bus Speed & Pull-ups:** All three firmwares run `I2C1` at **`400 kHz` Fast Mode** and enable STM32 internal GPIO pull-ups (`PB6/PB7 PULLUP` in `hwdef.dat`; `sda_pullup = true, scl_pullup = true` in `board_init/flywoof745.rs:401-403`) to supplement external pull-ups when a long GPS/compass cable is attached.
- **Bus Sharing Discipline:** `board_init/flywoof745.rs:395-508` probes and initializes both `QMC5883L` (`0x0D`) and `SPL06` (`0x76`) sequentially under `with_timeout(...)` **before** spawning `qmc5883l_mag_task` and `spl06_i2c_baro_task` on the thread executor (`NoopRawMutex` is sound because both tasks stay on the single-core thread executor).
- **Bus Recovery Gap:** `embassy-stm32`'s `I2c` enforces a `20 ms` transaction timeout (`flywoof745.rs:404`), and `MagReader`/`BaroReader` back off on read errors (`100 ms`), **but Cybflight does not perform 9-clock SCL bit-bang bus unsticking** if an I2C slave holds `SDA` low across a warm MCU reset (whereas ChibiOS/ArduPilot and Betaflight toggle `SCL` 9 pulses during I2C init/recovery).

### 6.2 Barometer (`SPL06-001` at `0x76`)
- **Hardware Variants Across Board Batches:** Notice that Flywoo shipped three pin-compatible barometers at `0x76` across GOKU F745 revisions (`BMP280` on Nano, `DPS310` in Betaflight `FLYWOOF745`, and **`SPL06`** on V3 — confirmed on this exact unit by `goku_ardu_params.params` `BARO1_DEVID = 329002`, which decodes in ArduPilot as `DEVTYPE_BARO_SPL06 = 0x05` on `I2C0:0x76`). Cybflight probes `SPL06` only (`CHIP_ID = 0x10`).
- **I2C 9-Byte Burst Limit Erratum (`drivers/src/baro/spl06.rs`):**
  - Both Betaflight (`barometer_dps310.c:178-190`) and ArduPilot (`AP_Baro_SPL06.cpp:187-196`: `#define READ_LENGTH 9`) explicitly split the 18-byte `COEF` (`0x10..0x21`) read into **two 9-byte reads** because SPL06/DPS310 I2C controllers fail or NACK bursts longer than 9 bytes on several production batches.
  - Cybflight's `Spl06::init_common` currently reads all 18 bytes in a single `write_read(addr, &[0x10], &mut coef[0..18])` transaction. Chunking this into two 9-byte reads (`0x10..0x18` and `0x19..0x21`) is a 4-line reliability hardening.
- **ODR & Oversampling:** Cybflight configures `PM_CFG = 0x36` (`8 Hz` background, `64×` pressure oversampling ≈ `104.4 ms`), `TMP_CFG = 0xB3` (external MEMS sensor, `8 Hz`, `8×` oversampling ≈ `14.8 ms`), `CFG_REG = 0x04` (`P_SHIFT=1, T_SHIFT=0`). Note that `104.4 ms + 14.8 ms = 119.2 ms > 125 ms` (95% duty cycle) is right at the ceiling of `8 Hz` continuous mode; Betaflight/ArduPilot run `16×` or `32×` oversampling at `32 Hz` (`~27–53 ms` conversion) for lower phase lag.

### 6.3 External Magnetometer (`QMC5883L` at `0x0D`) Orientation Proof
- Verified in `goku_ardu_params.params`: `COMPASS_DEV_ID = 590115` (`DEVTYPE_QMC5883L = 0x09` on `I2C0:0x0D`, external = `1`), `COMPASS_ORIENT = 0` (`ROTATION_NONE`).
- **Algebraic Orientation Proof:**
  - In `AP_Compass_QMC5883L.cpp:259-263`, ArduPilot maps raw QMC5883L registers `(rx, ry, rz)` to `field = (-rx, +ry, -rz)`, then applies hardcoded `ROTATION_YAW_90` (`AP_Compass_QMC5883L.cpp:176-179`), giving `(-ry, -rx, -rz)` in FRD.
  - Converting FRD `(x, y, z)` to FLU `(x, -y, -z)` yields **`(-ry, +rx, +rz)` = `SensorAlign::Cw270Deg`**.
  - This matches `bsp/flywoof745/src/lib.rs:410` (`mag_align: SensorAlign::Cw270Deg`), **provided the external GPS/compass module is mounted pointing along the airframe nose (`COMPASS_ORIENT = 0`), NOT rotated 45° with the diamond AIO PCB** (`MagReader` applies `mag_align` only and does *not* multiply by `imu_rpy_deg`, which is the right behavior for a nose-aligned GPS-mast compass).

---

## 7. Subsystem 5: GPS (`USART6` `PC6/PC7`)

### 7.1 Baudrate Negotiation & Protocol Initialization

| Feature | Cybflight (`flywoof745.rs` + `ublox.rs`) | Betaflight (`io/gps.c` + `gps_ublox.c`) | ArduPilot (`AP_GPS_UBLOX.cpp` + `CFGv2.cpp`) |
| :--- | :--- | :--- | :--- |
| **UART & Buffers** | `USART6` (`PC6/PC7`, `Pull::Up`), `BufferedUart` `256 B` RX / `256 B` TX | `UART6`, ISR ring buffer | `USART6` (`SERIAL6_BAUD=115`), DMA TX + IRQ RX ring (`>= 1 KB`) |
| **Baudrate Negotiation** | Passive RX/TX scan across `[115200, 230400, 38400, 57600, 9600]` (`550 ms` per baud); **does not switch receiver baudrate** (stays at whatever baud module booted on!) | Sends baud-switch command (`CFG-PRT` / `CFG-VALSET`) at `[115200, 57600, 38400, 19200, 9600]` → switches both module & FC to target (`57600`/`115200`) | Cycles `[9600, 19200, 38400, 57600, 115200, 230400]`, sends `CFG-PRT` / `CFG-VALSET` init blob switching module to `230400` / configured baud |
| **M8 vs M9/M10 Config** | Sends `UBX-CFG-VALSET` (`0x06 0x8A`, RAM layer: `UART1OUTPROT_UBX=1`, `MSGOUT_NAV_PVT_UART1=1`); on `ACK-NAK` falls back to `UBX-CFG-MSG` (`0x06 0x01`); also accepts live `NAV-PVT` | Queries `UBX-MON-VER`; uses `CFG-VALSET` for u-blox M9/M10 (`PROTVER >= 27`) or legacy `CFG-PRT`/`CFG-RATE`/`CFG-NAV5`/`CFG-MSG` for M8 | Queries `UBX-MON-VER`; uses key-value `AP_GPS_UBLOX_CFGv2` (`CFG-VALSET`/`VALGET`) on M9/F9/M10 or legacy `CFG-*` on M8 (`GPS_DRV_OPTIONS=0`) |
| **Update Rate (`MEAS`)** | **Unconfigured!** (Module stays at factory default **1 Hz** unless pre-flashed in u-center) | Configures `5 Hz` or `10 Hz` (`CFG-RATE-MEAS = 100 ms`) | Configures **`10 Hz`** (`GPS_RATE_MS = 100`) |
| **Dynamic Model** | **Unconfigured!** (Module stays at factory default `Portable = 0`) | Configures **`Airborne <4g` (`8`)** (or `Airborne <1g`) | Configures **`Airborne <4g` (`GPS_GNSS_MODE` / `nav_engine = 8`)** |
| **NMEA Suppression** | **Left ON!** (NMEA GGA/RMC/GSV bursts share 256 B RX buffer with UBX) | **Disabled** (`UART1OUTPROT_NMEA = 0`) | **Disabled** (`UART1OUTPROT_NMEA = 0`) |
| **Fletcher-16 Checksum** | **NOT VALIDATED** (`let _ = ck;` at `ublox.rs:346`) | **Validated** on every UBX frame | **Validated** byte-by-byte in state machine |

### 7.2 Actionable `VALSET` Fix for u-blox M9/M10 (`Flywoo GM10 Nano V3`)
Adding 4 key-value pairs to `VALSET_NAV_PVT_UART1` in `crates/drivers/src/gps/ublox.rs:41` (and matching `CFG-RATE` + `CFG-NAV5` in the M8 fallback) immediately brings Cybflight to parity with Betaflight & ArduPilot without requiring pre-configuration in u-center:
- `CFG_UART1OUTPROT_NMEA` (`0x1074_0002`, `L` 1B) = `0` (suppress NMEA buffer pressure)
- `CFG_RATE_MEAS` (`0x3021_0001`, `U2` 2B) = `100` (`10 Hz` navigation rate)
- `CFG_RATE_NAV` (`0x3021_0002`, `U2` 2B) = `1` (`1` measurement per nav solution)
- `CFG_NAVSPG_DYNMODEL` (`0x2011_0021`, `E1` 1B) = `8` (`Airborne <4g`)
- Plus if the baud scanner locks onto a module factory-defaulted at `9600` or `38400` baud, `10 Hz` `NAV-PVT` (`1000 B/s`) will saturate `9600` baud (`960 B/s` max) — so shifting the module to `115200` (`CFG_UART1_BAUDRATE` `0x4052_0001 = 115200` / `CFG-PRT`) before enabling `10 Hz` is essential for out-of-the-box `9600`-baud GPS modules.

---

## 8. Subsystem 6: RC Receiver (`CRSF` on `USART2` `PD5/PD6`) & Battery/Current ADC (`PC3`/`PC2`)

### 8.1 RC Receiver (`CRSF` / ExpressLRS on `USART2`)
- **Hardware & Electrical:** `PD5` (`USART2_TX`) and `PD6` (`USART2_RX`) require no hardware signal inverter (CRSF is 3.3V uninverted UART). Cybflight enables internal `Pull::Up` on `PD6` (`flywoof745.rs:243`) so an unplugged RX pad idles cleanly HIGH instead of firing framing-error IRQs.
- **Channel Order & Stick Signs (`goku_ardu_params.params` vs Cybflight):**
  - `goku_ardu_params.params` confirms standard **AETR** (`RCMAP_ROLL=1, RCMAP_PITCH=2, RCMAP_THROTTLE=3, RCMAP_YAW=4`, CRSF range `989..2012 µs`, mid `1500 µs`, `RC2_REVERSED = 1` per standard OpenTX/EdgeTX AETR where Pitch stick forward = `~2011 µs`).
  - **Axis Sign Audit in Cybflight `outer_rate` (`control/rc_interpreter.rs:53-56`):**
    - `Roll (ch0)`: Stick Right (`2012 µs`) → `+roll` → `+rate_ref.x` (`+X` Forward = CW roll right). **Correct.**
    - `Pitch (ch1)`: Stick Forward (`2011 µs`) → `+pitch` → `+rate_ref.y` (`+Y` Left = nose-down pitch forward). **Correct.**
    - `Throttle (ch2)`: Stick Up (`2012 µs`) → `+thrust`. **Correct.**
    - **`Yaw (ch3)`:** Stick Right (`2012 µs`) → `+yaw` → **`+rate_ref.z` (`+Z` Up = CCW nose-left yaw). REVERSED!** Must use `RateChannelCalibration::centered_inverted_with(3, endpoints)` to match `RcMapper::aetr` and Position mode.
- **CRSFv3 Speed Negotiation Bug (`sensors/rc.rs:418-431`):**
  - Currently calls `self.crsf.accept_speed_proposal(port_id, baud)` (`0x71` response with `status = true`) without changing `USART2` baudrate. Until runtime UART baudrate switching + link-timeout fallback to `420_000` is wired up, **`CrsfRunner` MUST reject `SpeedProposal` (`send_speed_response(port_id, false)` or ignore it)** so ExpressLRS / Crossfire stays at `420,000` baud!

### 8.2 Battery Voltage (`PC3`) & Current (`PC2`) ADC Scaling

| Parameter | Cybflight (`bsp/flywoof745/src/lib.rs:50-56`, `power.rs`) | Betaflight (`FLYWOOF745` / `FLYWOOF745NANO`) | ArduPilot (`FlywooF745/hwdef.dat` + `goku_ardu_params.params`) |
| :--- | :--- | :--- | :--- |
| **ADC Peripheral & Mode** | **`ADC3`** (`PC3=IN13`, `PC2=IN12`), software single-conversion @ `100 Hz`, `SMP = 480` cycles (`35.6 µs`), 1st-order IIR (`α=0.1`) | **`ADC1`** (`PC3`, `PC2`, `PC5`), continuous circular DMA + oversampling/LPF | **`ADC1`** (`PC3=13`, `PC2=12`, `PC5=15`), ChibiOS continuous DMA group conversion |
| **Voltage Multiplier (`PC3`)** | `voltage_scale = 109`, `divider = 10` → **`10.90 × V_adc`** (`VREF = 3.30 V`) | Default `vbat_scale = 110` (`11.0 × V_adc`) | `HAL_BATT_VOLT_SCALE = 10.9`, flown **`BATT_VOLT_MULT = 10.9`** (`2S` `BATT_LOW_VOLT = 6.8 V`, `CRT = 6.5 V`) |
| **Current Scale (`PC2`)** | `current_scale = 351` (`0.351 mV/A` → **`28.50 A/V`**) | V1 default `ibata_scale = 170` (`58.8 A/V`); Flywoo V3 manual specifies **`714` (`14.0 A/V`)** | `HAL_BATT_CURR_SCALE = 28.5`, flown **`BATT_AMP_PERVLT = 28.5`** |

- **Conversion Math Equivalence:**
  - Cybflight: `voltage_v = (adc / 4095) * 3.3 * (109 / 10) = (adc / 4095) * 3.3 * 10.9` — **exact match** to ArduPilot `BATT_VOLT_MULT = 10.9`.
  - Cybflight: `current_a = ((adc / 4095) * 3300 mV) / (351 * 0.1 mV/A) = V_adc * (10000 / 351) = V_adc * 28.49 A/V` — **exact match** to ArduPilot `BATT_AMP_PERVLT = 28.5`.
- **Battery Failsafe Integration Note:** `goku_f745.yaml:71-74` configures `batt_nominal_v: 7.6`, `batt_min_v: 6.0`, `batt_max_v: 10.0` (used by INDI's thrust-voltage compensation when `PC3` reads within `[6.0 V, 10.0 V]`, falling back cleanly to `7.6 V` on USB-only bench power).

---

## 9. Stability, Fault-Handling & Prioritized Recommendations for Cybflight

### Ranked Remediation Checklist

| Priority | Subsystem & File Reference | Issue | Recommended Fix | Status |
| :--- | :--- | :--- | :--- | :--- |
| **P0-1** | `crates/cybflight/src/control/rc_interpreter.rs:58` | **Reversed RC Yaw in `outer_rate`:** Pushing yaw stick right commanded `+ω_z` (CCW / nose-left). | Change `RateChannelCalibration::centered_with(3, endpoints)` to `RateChannelCalibration::centered_inverted_with(3, endpoints)`. | **Fixed & Verified (`324e411`)** |
| **P0-2** | `crates/cybflight/src/sensors/rc.rs` & `health.rs` | **Permanent `EstimatorNotReady` Arming Lockout:** `outer_rate` waited for `ESTIMATOR_READY`, which requires RTK-Fixed GPS (`carr_soln == 2`). | Gate `EstimatorNotReady`, `SensorHealthDegraded`, and `EskfDegraded` with `#[cfg(not(feature = "outer_rate"))]` so manual rate mode can arm indoors or with non-RTK GPS. | **Fixed & Verified (`324e411`)** |
| **P0-3** | `crates/cybflight/src/sensors/rc.rs` & `health.rs` | **Gyro Bias Calibration Bypassed on Fast Arm:** Arming before `1.0 s` of stillness aborted `GyroStartupCal` permanently for the flight. | Add `BlockReason::GyroCalibrating` gate checking `!crate::sensors::imu::IMU1_GYRO_CALIBRATED.load(Ordering::Acquire)` before arming. | **Fixed & Verified (`324e411`)** |
| **P1-1** | `crates/drivers/src/rc/crsf.rs` & `board_init/flywoof745.rs` | **CRSFv3 `SpeedProposal` Link Drop:** Sent `ACK=true` without switching `USART2` baudrate; 128 B UART buffers. | Reject `SpeedProposal` (`reject_speed_proposal` `ACK=false`) until dynamic UART baud switching + fallback is implemented; bump CRSF `TX_BUF`/`RX_BUF` to `256 B`. | **Fixed & Verified (`324e411`)** |
| **P1-2** | `crates/drivers/src/gps/ublox.rs` & `board_init/flywoof745.rs` | **u-blox Checksum Unchecked + 1 Hz / 3.33 Hz Cold-Boot M10 Multi-Band Cap:** `fletcher8` ignored on RX; `RATE_MEAS`, `DYNMODEL`, `OUTPROT_NMEA=0`, and M10 single-band `VALSET_M10_SIGNALS` (`BDS B1C` + `GLO off` + `550 ms` GNSS reset wait) not sent. | Verify `(ck_a, ck_b)` on RX (`Error::BadChecksum`); send `VALSET_M10_SIGNALS` + `550 ms` reset wait on M10 followed by `VALSET_NAV_PVT_UART1` (`100 ms` `10 Hz`, `Airborne <4g`, `NMEA = off`). | **Fixed & Verified Live (`67b57a5`: `10.00 Hz` / `99,957 µs` @ `230400` baud on cold boot, warm reboot, & 65 s stress)** |
| **P1-3** | `crates/drivers/src/baro/spl06.rs` & `board_init/flywoof745.rs` | **SPL06 18-Byte I2C Burst Erratum, Un-Delayed `recover()` Reset & `I2C1` Wedge:** Single 18-byte `COEF` read + `REG_RESET` without 40 ms delay in `recover()` + `20 ms` `I2C1` timeout wedging `TC=1, AUTOEND=0`. | Split `0x10..0x21` `COEF` read into two 9-byte reads; remove un-delayed `REG_RESET` from `Spl06::recover()` & guard standby `0x000000` reads; add UM10204 9-clock + `STOP` unstick, `50 ms` timeout & `RM0385 §30.4.6` `reset_i2c1_if_wedged()`. | **Fixed & Verified Live (`67b57a5`: `101,371 Pa, 37.7 °C` across `imurate`, warm `reboot`, & 65 s stress)** |
| **P1-4** | `crates/cybflight/src/sensors/imu.rs` & `board_init/flywoof745.rs` | **`Mpu6x00::read()` Unbounded Wait & `P10` Preemption Loss:** Stuck DRDY could hang forever; sharing `P10` with `indi_task` (`126–210 µs`) lost `2.6%` of 8 kHz samples. | Isolate IMU onto `P8` `EXECUTOR_IMU` (`CAN2_RX1`); wrap `self.imu.read()` in `10 ms` timeout calling `Mpu6x00::recover()`. | **Fixed & Verified Live (`67b57a5`: `8,000.3 Hz`, `+0.00%`)** |
| **P1-5** | `crates/drivers/src/mag/qmc5883l.rs` & `board_init/flywoof745.rs` | **`QMC5883L` `ROL_PNT` `189 °C` Temperature Bug & Warm-Reset Probe Miss:** Writing `REG_CONTROL2 = 0x40` wrapped `0x06 -> 0x00` before `TEMP` (`0x07..0x08`) and single-shot `probe()` missed across warm MCU resets. | Set `REG_CONTROL2 = 0x00`, read `0x00..=0x05` (`6 B`) + `0x07..=0x08` (`2 B`), prime `0x00` with `5x` retries in `probe()` + `3x` outer retries in `flywoof745.rs`. | **Fixed & Verified Live (`67b57a5`: `[-27.7, 10.7, 42.3] uT, 27.6 °C` across normal boot, warm `reboot`, & 65 s stress)** |
| **P2-1** | `crates/cybflight/src/platform.rs:6` & `memory_f745.x` | **Uncached 7-WS `.rodata` Reads & 512 KB Param Sector Stall:** `FLASH_ACR` has `PRFTEN=0, ARTEN=0`; `Sectors 6–7` (`2×256 KB`) stall CPU for ~2 s on erase. | Optional future optimization: set `PRFTEN=1` or relocate `ekv` param flash to `Sectors 1–2` (`2×32 KB`) to reclaim 416 KB of code flash. | **Open (Future P2 Optimization)** |
| **P2-2** | `crates/bsp/flywoof745/src/lib.rs` | **`PC14` (9V BEC PINIO) Uninitialized:** On GOKU GN 745 V3, `PC14` controls the switched 9V BEC pad (`PINIO1_CONFIG=129` in Betaflight). | Optional: drive `PC14` explicitly if powering a VTX/companion device from the switched 9V rail. | **Open (Hardware Optional)** |
