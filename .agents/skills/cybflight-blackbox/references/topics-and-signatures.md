# Cybflight Blackbox Topics, Event Codes & Failure Signatures

Source definitions:
- Registry & record sets: [topics/mod.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/blackbox/topics/mod.rs), [record_set.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/blackbox/record_set.rs)
- Recorder & Metadata: [recorder.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/blackbox/recorder.rs), [mcap.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/blackbox/mcap.rs)
- NOR flash backend (`flywoof745`): [nor_store.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/blackbox/nor_store.rs)

---

## 1. MCAP Channel Catalogue & Tiers

Channel IDs are globally fixed across all tiers (`Topics::ALL`):

| ID | Topic | Schema | Tiers | Nominal Rate / Notes |
|---:|---|---|---|---|
| 1 | `/imu1` | `Imu.v2` | `mid`, `large` | `8000 / blackbox_rate_div` (`250 Hz` on `goku_f745` div=4) |
| 2 | `/attitude` | `VehicleAttitude` | `mid`, `large` | `~100 Hz` Mahony IMU-only `[w, i, j, k]` FLU |
| 3 | `/rc` | `RcInput` | `small`, `mid`, `large` | Link rate (`50–250 Hz` ELRS; `CH3=Thr`, `CH1=R`, `CH2=P`, `CH4=Y`) |
| 4 | `/events` | `Event` | all | Bracket + fault/edge transitions (unmaskable) |
| 5 | `/odometry` | `VehicleOdometry.v2` | `large`, `sysid` | ESKF fused pose + twist |
| 6 | `/mpc` | `OcpSolverOutput` | `large` (`outer_mpc`) | Outer-loop NMPC command, convergence, `solve_time_us` |
| 7 | `/motors` | `ActuatorMotors` | `mid`, `large`, `sysid` | `100 Hz` (`>=500 Hz` in `sysid`), normalized `[0..1]` `[m0..m3]` |
| 8 | `/motor_state` | `MotorStateTelemetry` | `mid`, `large`, `sysid` | Filtered `omega`, `omega_dot`, raw bidir-DShot `raw` (`null` on miss) |
| 9 | `/tracking_error` | `TrackingError` | `large` | Controller error (`source`: `0=cascade`, `1=mpc`, `2=indi`) |
| 10 | `/health` | `SystemHealth.v2` | `mid`, `large`, `sysid` | `20 Hz` mirror + `indi_step_max_us`, `indi_period_max_us`, RC link stats |
| 11 | `/gps_health` | `GpsHealth` | `mid`, `large` | On change + `1 Hz` heartbeat (`NotConfigured` on rate/mocap) |
| 12 | `/estimator_state` | `EstimatorBias` | `large` (`est_eskf`) | `~10 Hz` gyro/accel bias estimates |
| 13 | `/imu1_raw` | `ImuRaw.v2` | `sysid` | Pre-biquad-LP IMU thinned by `blackbox_rate_div` |
| 14 | `/control_setpoint` | `AttitudeControlSetpoint` | `large` | `50–100 Hz` outer-loop `collective_thrust_n`, `body_rate_rad_s` |
| 15 | `/power` | `PowerTelemetry` | `sysid` | `10 Hz` filtered + unfiltered VBAT, current, mAh, cell count |
| 16 | `/mpc_cost` | `MpcCostAdapt` | `mid`, `large` (`outer_mpc`) | Learned cost residual `z` + effective weights + `gain` |

---

## 2. `/events` & `/health` Bitfields

### `/events` Kind Codes ([events.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/blackbox/topics/events.rs))
- `0x01 ARM` — Session open on `IS_ARMED` (after 40 ms debounce) or `RECORDER_HOLD`.
- `0x02 DISARM` — `data`: `0=Commanded`, `1=ControllerTimeout`, `2=RcLoss`.
- `0x03 FAILSAFE` / `0x04 FAILSAFE_CLEAR` — `data`: `1=ControllerTimeout`, `2=RcLoss`.
- `0x05 ESTIMATOR_DOWN` / `0x06 ESTIMATOR_UP`.
- `0x07 RC_LOSS` / `0x08 RC_RECOVERED`.
- `0x09 MISSION_PLANNING` / `0x0A MISSION_EXECUTING` / `0x0B MISSION_IDLE` (`data`: prior state).
- `0x0C INNER_SILENT` — `data`: `1=voltage_stale`, `2=WLS_NaN`.
- `0x0D POWER_STALE` (`data`: boot episode index) / `0x0E POWER_OK` (`data`: duration ms).
- `0x0F RECORDER_OVERRUN` — `data`: cumulative dropped messages this session (`<= 1 Hz` after first edge).
- `0x10 LOG_END` — Clean end-of-capture marker before `DataEnd`/`Footer`.
- `0x20 PANIC`, `0x21 HARDFAULT`, `0x22 BROWNOUT`, `0x23 IWDG_RESET`, `0x24 BOOT_POSTMORTEM`.

### `/health` Bitfields ([estimation/mod.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/estimation/mod.rs#L124-L149))
- `eskf_faults`: `1=NAN_RESET`, `2=POS_STALE`, `4=VEL_STALE`, `8=ATT_STALE`, `16=COV_TRACE_BLOWUP`, `32=GUARD_JUMP_CASCADE`, `64=GUARD_REJECT_CASCADE`.
- `attitude_health`: `1=ACCEL_OK`, `2=GYRO_OK`, `4=NO_RECENT_NAN`.

---

## 3. Forensic Failure Signatures → Firmware Root Causes

| Log Signature | Root Mechanism | Where to Inspect |
|---|---|---|
| `CH3 >= 1155 µs` while `mean(motors) < 0.10` & upright `az ~ +9.81 m/s²`, then sudden `100%` jump past `1500 µs` | **INDI premature ground exit**: `touching_ground` becomes `false` once `spf_sp_z >= indi_ground_thrust_sp` (`3 m/s²`) while quad still rests on the `1g` floor reaction force (`dv.z = spf_sp_z - 9.81 < 0`). | [controller.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight_core/src/indi/controller.rs#L618-L645), [goku_f745.yaml](file:///Users/andrii/Personal/autopilots/cybflight/vehicles/goku_f745.yaml#L216-L221) |
| Armed `indi_step_max_us > 500 µs`, disarmed `< 500 µs`, `indi_period_max_us > 10,000 µs`, `rc_link_age_ms >= 100 ms` (`LQ=100%`), armed `RECORDER_OVERRUN` | **Armed 2 kHz CPU saturation**: High-priority tasks overrun `500 µs` budget while armed (e.g., 4× `RpmEstimator` EKFs running even when `indi_omega_kf=0`), starving Thread Mode (`rc_interpreter_task` > 100 ms -> `indi_task` silent -> `dshot_task` `10 ms` idle drop & `blackbox_task` overrun). | [indi_task.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/control/indi_task.rs#L1200-L1261), [dshot.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/motors/dshot.rs#L165-L173) |
| Sequential `m0 -> m1 -> m2 -> m3` eRPM valid % drop (`m3` ~0% spinning while armed) + `|omega_dot| > 5,000 rad/s²` | **Bidir-DShot GCR IC DMA preemption + ZOH steps**: `dshot_task` releases GPIO lines and starts IC timers before sequential DMA setup at `P8`, letting `P6` `imu1_task` preempt `m1..m3`; `rpm_all_stale` stays `false`, feeding noisy ZOH `omega_dot_fs`. | [dshot.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/motors/dshot.rs#L283-L385), [indi_task.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/control/indi_task.rs#L1294-L1313) |
| Idle/low-throttle CW `(m0,m3)` vs CCW `(m1,m2)` ping-pong `0% ↔ 30%` every ~20 ms | **Uncalibrated `G2` yaw reaction feedback**: `do_indi_f * g2 * omega_dot_fs` turns noisy eRPM jumps into full diagonal yaw reversals. | [controller.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight_core/src/indi/controller.rs#L636-L645), [goku_f745.yaml](file:///Users/andrii/Personal/autopilots/cybflight/vehicles/goku_f745.yaml#L68-L71) |
| Rapid flip (`|Roll| > 85°` or `|Pitch| > 80°`) right at liftoff with high gyro rates | Check CW/CCW motor order & prop direction, FLU board orientation (`gyro1_align`), or motor-saturation destabilization. | [flywoof745/src/lib.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/bsp/flywoof745/src/lib.rs#L129-L150), [goku_f745.yaml](file:///Users/andrii/Personal/autopilots/cybflight/vehicles/goku_f745.yaml) |
| `DISARM (ControllerTimeout)` preceded by `INNER_SILENT (WLS_NaN)` or `POWER_STALE` | Planned watchdog disarm triggered intentionally when WLS produced sustained NaN or battery telemetry dropped past 2 s in Table thrust mode. | [indi_task.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/control/indi_task.rs#L1564-L1700), [failsafe.rs](file:///Users/andrii/Personal/autopilots/cybflight/crates/cybflight/src/control/failsafe.rs#L29-L42) |
