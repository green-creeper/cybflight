# Cybflight Hardware & Tooling Lessons (What Worked vs. What Didn't)

Real lessons isolated during `goku_f745` (`STM32F745VG` / Flywoo Firefly 2S Nano Baby 20 V3), `cybflight` firmware auditing, and `cybflight-gs` bring-up.

---

## 1. Liftoff Failure: Collective Thrust Scaling & Wrong Airframe Config
- **What didn't work**:
  1. `rate_stick_to_thrust_n` in `crates/cybflight/src/control/rc_interpreter.rs` capped full throttle stick (`1.0`) at `2 * hover_thrust_n`. With an underestimated `mass_kg: 0.0618`, full stick capped collective thrust around `~1.21 N` (`~124 gf`), leaving more than half of the four motors' `2.425 N` capacity unreachable so the quad could not lift off at 100% throttle.
  2. Default/5"-like tuning values on `goku_f745.yaml` (`inertia_kg_m2` ~7x too high, `motor_poles: 14`, `m0..3_omega_max: 9600.0`, `batt_max_cells: 6`, `imu_accel_lpf_hz: 80.0`) distorted WLS allocation, eRPM scaling, cell detection, and accelerometer filtering.
- **What worked**:
  - Updated `rate_stick_to_thrust_n` to a piecewise linear curve: `[0.0..=0.5] -> [0.0..=hover_thrust_n]` and `(0.5..=1.0] -> (hover_thrust_n..=max_collective_thrust_n]`.
  - Updated `vehicles/goku_f745.yaml` to true 2S 2" hardware parameters (`mass_kg: 0.090`, measured inertia `[0.000045, ..., 0.000048, ..., 0.000072]`, 4× `max_thrust_n: 0.6063`, `motor_poles: 12`, `m0..3_omega_max: 5800.0`, `batt_nominal_v: 7.6`, `batt_max_cells: 2`, `imu_accel_lpf_hz: 20.0`, `imu_gyro_lpf_hz: 175.0`).
  - Mirrored the exact piecewise curve in `cybflight_gs/src/ui/rc_tab.rs` (locked with unit test `rate_stick_to_thrust_n_matches_piecewise_firmware_curve`) so bench throttle preview matches flight hardware.

---

## 2. Normal-Mode Boot IWDG Reset: 2 kHz CPU Saturation on STM32F745
- **What didn't work**:
  - Enabling `rpm_notch_en: 1` at 2 kHz `indi_task` on 216 MHz STM32F745 without D-Cache on `SRAM1`. Recomposing `6` gyro + `2` accel notches per 500 µs tick (24 trig updates + 48 uncached SRAM1 DF1 filter steps) alongside `indi.step` (`~202 µs` avg), `P8` 8 kHz IMU, and `P6` DShot exceeded 100% CPU immediately after `mark_boot_stage(7)`. Thread-mode `main()`, `postmortem_task`, and `iwdg_feed_task` never got a time slice -> 500 ms `IndependentWatchdog` fired at `uptime=0ms` -> next boot dropped into `[SAFE MODE ACTIVE]`.
  - Boot-stage visibility was blind (`uptime_ms = 0` couldn't tell whether the hang occurred in `board_init`, `indi_task`, or `iwdg_feed_task`).
- **What worked**:
  - Set `rpm_notch_en: 0` in `vehicles/goku_f745.yaml` (`indistat` confirmed ~202 µs avg step time with ~60% headroom inside the 500 µs period).
  - Added `cutoff_hz` change-threshold caching (`>= 0.25` Hz) + `reset()` in `crates/cybflight_core/src/indi/rpm_notch.rs` (tested by `repeated_idle_then_active_reset_and_reengage`) to eliminate redundant trig recomputations when motors are idle or steady.
  - Added `mark_boot_stage(1..9)` in `postmortem/mod.rs`, `main.rs`, `flywoof745.rs`, and `postmortem/task.rs` to pinpoint exact boot progress in BKPSRAM.

---

## 3. STM32 Hardware Watchdog (`IWDG`) Across Warm Reboots, DFU & Slow Init
- **What didn't work**:
  - Treating `IWDG` as temporary during a boot phase or assuming soft reset stops `IWDG`. On STM32, **once `IWDG` is started, it keeps counting across `SCB::sys_reset()` (`reboot` / `reboot --dfu`)**. Any unguarded slow path on boot (blocking GPS autobaud probing, flash erases, `enter_dfu()` handoff, or `pre_init()` zeroing BSS before feeding `IWDG`) causes spurious watchdog resets on warm reboots.
  - Calling `extend_timeout()` / `restore_timeout()` without nesting awareness or polling `IWDG.SR` (`PVU || RVU`) risks corrupting prescaler/reload updates or shrinking the timeout prematurely.
- **What worked**:
  - Feed/extend `IWDG` inside `platform.rs` (`pre_init()`, `sys_reboot()`, `enter_dfu()`) and track nested extensions via `EXTEND_DEPTH` + `PVU || RVU` register waits in `watchdog.rs`.
  - Moved blocking GPS autobaud probing off `board_init` into background `gps_autobaud_task` on `spawner`.

---

## 4. STM32F745 Hardware Quirks vs. STM32H743
- **Memory layout (`memory_f745.x`)**: Do **not** alias ITCM/DTCM carelessly into general `RAM` — keep `RAM` strictly within the 240 KB `SRAM1` window (`0x20010000`, `LENGTH = 240K`), `DTCM` (`64K`) separate, and crash scratch (`FAULT_SCRATCH`) in `BKPSRAM` (`4K`).
- **Single-byte UART `RDR` (no FIFO on F7)**: Leaving `USART1..6` or `I2C1` at lower NVIC priority than 8 kHz IMU (`P8`) or 2 kHz INDI (`P10`) drops bytes (`ORE`) at 420k–460.8k baud. Raising `USART1..6` + `I2C1_EV`/`I2C1_ER` to `Priority::P5` (just below `DMA` `P4`) eliminated CRC errors (`crc_err=0`).
- **Dual-timer DShot (`TIM3` + `TIM1`) & AM32 on F745**:
  - `TIM1` is an advanced-control timer — outputs stay dead unless `TIM1` `BDTR` sets **`MOE + AOE`**.
  - Bidirectional DShot requires single-owner `ACTUATOR_MOTORS.try_take()`, atomic GPIO direction turnaround, `Timer::after_micros(160)` timing on F745, and `detect_ticks_per_bit` for AM32 ESCs.
- **Shared `I2C1` (`SPL06` + `QMC5883L`)**: Clock-stretched/wedged slaves across warm resets require `reset_i2c1_if_wedged()` and probing all shared-bus devices before spawning tasks.

---

## 5. ELRS MAVLink2 Passive Link vs. Active Pilot Control
- **What didn't work**:
  - Marking `RC_LINK_ACTIVE = true` as soon as the UART scanner locked onto MAVLink `HEARTBEAT`/`RADIO_STATUS` packets (`link_frames`), even when the handset TX was turned off (`rc_frames=0`). That masked failsafe / disarmed pre-arm checks.
  - Retiring Ghost cleanly (`01c29a8`) removed ~755 net lines of dead protocol paths and simplified `multi_rc_task`.
- **What worked**:
  - `multi_rc_task` locks the UART port/baud/protocol on valid receiver `link_frames` (`crc_err=0`) so `rcdiag` reports `state=LOCKED`, while `RC_LINK_ACTIVE` remains `false` (Failsafe held) until actual RC channel frames (`rc_frames`) arrive from a powered-on TX.

---

## 6. Build, DFU & USB Serial Tooling Gotchas
- **Concurrent builds**: Running two `cargo`/`just` commands in parallel fights over `target/` and overwrites `target/thumbv7em-none-eabihf/release/cybflight.bin`. Always build/flash sequentially.
- **Flash parameter shadowing**: Editing `tuning:` in `vehicles/*.yaml` has no effect on keys already stored in KV flash Sectors 6+7 until cleared with `param reset <key|all>` + **`param save --prune`** (plain `param save` re-writes the old baked value rather than pruning).
- **macOS `dfu-util` `:leave` hang**: Raw `dfu-util -s 0x08000000:leave` frequently stalls in `Transitioning to dfuMANIFEST state` even after the chip has already jumped to firmware. `just flash <vehicle>` handles this automatically; use it instead of manual `dfu-util`.
- **`embassy-usb` CDC wake-up**: Opening `/dev/tty.usbmodem*` or omitting `DTR` stalls `CdcAcmClass::wait_connection()` — use `tools/fc_shell.py` (`--dfu`, `--wait`, `--monitor`, `--after`).

---

## 7. Ground Station (`cybflight-gs`) Lessons
- Fixed pixel widths caused overlapping/clipped controls in Receiver & Modes across compact laptop and fullscreen monitors -> replaced with responsive layout bounds and shared `ui/widgets.rs` helpers.
- Stale serial handle / stale telemetry after DFU or unplug -> explicitly close port/flush before spawning `dfu-util`, auto-reconnect, and reset telemetry state on disconnect.
- Align UI math with firmware math -> match piecewise collective thrust (`rate_stick_to_thrust_n`) and Schmitt-trigger hysteresis previews directly to firmware behavior, and label `"FWD (+X)"` on the 3D quad model to prevent orientation confusion.

---

## 8. `goku_f745` Onboard `W25Q128` (`SPI1`) Blackbox: 512 KiB Flash Ceiling & 216 MHz CPU Budget
- **What didn't work**:
  1. Enabling `HAS_BLACKBOX_STORAGE = true` with `crc32fast`'s `16 KiB` LUT and verbose JSON schemas overflowed `STM32F745`'s `512 KiB` code window (`memory_f745.x`, reserving `Sectors 6+7` KV params). Switching `cybflight` to `opt-level = "z"` shrank Flash below `470 KiB`, **but** disabled loop unrolling in 8 kHz/2 kHz tasks (`indi_task`, IMU, DShot), saturating CPU at boot stage `7` and triggering deterministic `IWDG` -> `[SAFE MODE]`.
  2. Polling `W25Q128` busy wait with `Timer::after_micros` on `embassy-time`'s `tick-hz-32_768` under ~92% `P6/P8/P10` ISR load stretched each sleep to `~1–1.25 ms`, capping NOR throughput at `32–58 KiB/s`.
  3. Subscribing F745 blackbox `/imu1` to the shared `1 kHz` `IMU_1_DECIM` exceeded the ~7% Thread-Mode CPU ceiling (~740 msgs/s max vs ~1,330 msgs/s), and `CAP = 4` (`40 ms`) on `MAHONY_ATTITUDE` dropped 1 sample during schema emission.
- **What worked**:
  - Kept `cybflight` at `opt-level = "s"` (setting `"s"` on `embassy-stm32`/`embassy-usb`), replaced `crc32fast` with a `64 B` nibble CRC-32 LUT (`crates/cybflight_core/src/crc32.rs`), minified JSON schemas (`-12 KiB`), and factored synchronous topic encoders + `NorBlackboxStore` (`~521 KiB` Flash, `>128 KiB` SRAM1 headroom).
  - Implemented non-blocking `W25q128::poll_ready()`, a `4 KiB` (`16-page`) `.bss` ring with cooperative `yield_now()` per kicked page, and `Op+Len` / `Body` bulk copies (`nor_store.rs`).
  - Added `BLACKBOX_IMU_1` (`sensors/mod.rs`), thinned at the publisher by `blackbox_rate_div` (`4` -> `250 Hz` on `goku_f745` under `blackbox_tier: 2` `mid`), widened `MAHONY_ATTITUDE` & `RC_INPUT` to `CAP = 8`, and added `is_capturing()` so IMU logs through the 3 s post-disarm tail with **0 drops / 0 enc-ovf**.
