---
type: workflow
title: Hardware Bring-up & Pre-flight Protocol
description: Step-by-step procedures, safety guidelines, and verification workflows for initial physical quadrotor bring-up, sensor probing, DShot motor testing, and pre-flight ESKF alignment.
tags: [hardware, bringup, safety, dshot, sensors, eskf, groundstation, calibration]
verified:
  - by: openwiki/0.7.2
    at: 2026-10-30T14:16:13.882Z
sources:
  - id: openwiki-source-07f86515d1eb9c334e9b0b9d
    resource: repo://crates/cybflight_gs/src/ui/setup_tab.rs
  - id: openwiki-source-421509a684b4a9886a56ae42
    resource: repo://crates/cybflight/src/board_init/flywoof745.rs
  - id: openwiki-source-7f428a787c91f1ad9a34d6e2
    resource: repo://crates/cybflight/src/estimation/eskf_imu_gps.rs
  - id: openwiki-source-a7c02f463def0d4c51e56b7f
    resource: repo://crates/cybflight/src/motors/dshot.rs
  - id: openwiki-source-81f5af43a09aea236742f075
    resource: repo://crates/cybflight/src/sensors/power.rs
  - id: openwiki-source-dad3aff7b82563507fc8a56b
    resource: repo://crates/cybflight/src/usb_serial.rs
  - id: openwiki-source-5cfdd9c4d4136c45880a6785
    resource: repo://docs/safety_protocol.md
generated: { by: "openwiki/0.7.2", at: "2026-10-30T14:16:13.882Z" }
---

# Hardware Bring-up & Pre-flight Protocol

This document provides step-by-step bring-up procedures, safety protocols, and verification workflows for newly assembled or updated quadrotor hardware. Following this sequence ensures that physical components—power monitoring, sensor buses, actuators, radio control, and state estimators—are systematically verified on the bench prior to flight.

---

## Safety Protocol & Bench Guidelines

Physical quadrotor bring-up involves high-power electronic speed controllers (ESCs) and brushless motors capable of high rotational speeds. Testing hardware while connected to power source batteries requires strict adherence to safety protocol.

### Fundamental Safety Directives

1. **PROPELLERS MUST BE REMOVED**: Never install propellers during initial bench bring-up, motor order/direction checks, CLI parameter configuration, or firmware flashes. Propellers may only be installed immediately before controlled flight testing in a designated flight area.
2. **Current-Limited Power Supply**: When applying initial battery power (LiPo/LiHV), use a current-limited bench power supply or a inline current-limiting device ("smoke stopper") to prevent short-circuit damage.
3. **Silence Is the Universal Failure Signal**: The cybflight architecture enforces a strict silence-propagation principle. When sensor inputs or controller calculations become invalid, non-finite, or stale, tasks stop publishing outputs rather than forcing hard resets or direct disarms.
4. **Staleness Gate Cascade**: Downstream consumers detect upstream silence via strict recency timeouts:
   - **Outer Loop Odometry Gate**: 50 ms (MPC) / 100 ms (Geometric Cascade).
   - **INDI Rate Command Gate**: 100 ms timeout before stopping actuator command publication.
   - **DShot Motor Command Gate**: 10 ms timeout before dropping motor output to idle throttle (`DSHOT_IDLE_THROTTLE`).
   - **Failsafe Watchdog**: 500 ms timeout of `LAST_CONTROLLER_PUBLISH` before triggering automated disarm.
   - **RC Link Timeout**: 150 ms before entering failsafe guard period (1500 ms).

<!-- openwiki: mermaid parse failed and this diagram was converted to a text fence so it does not break rendering. Fix the diagram source and restore the mermaid fence. Parser error: Heuristic: an unescaped angle bracket inside a label breaks rendering; rephrase the label. -->
```text
flowchart TD
    A["1. Bench Setup & Power<br/>(Props OFF, ADC Calibration)"] --> B["2. Sensor Bus Probing<br/>(SPI IMU, Shared I2C Bus)"]
    B --> C["3. RC Channel Mapping<br/>(CRSF/GHST, Stick Endpoints)"]
    C --> D["4. DShot Motor Testing<br/>(Bidirectional DShot, eRPM)"]
    D --> E["5. Estimator Alignment<br/>(Mahony, RTK-Fixed ESKF)"]
    E --> F["6. Pre-flight Verification<br/>(health CLI, cybflight-gs 3D)"]
```
*Step-by-step sequence for hardware bring-up and pre-flight verification.*

---

## Step 1: Power & ADC Calibration

The autopilot measures pack voltage ($V_{BAT}$) and total current draw ($I_{CURR}$) via dedicated microcontroller ADC channels. On boards like the Flywoo Goku GN 745 AIO (`STM32F745VG`), ADC3 monitors VBAT on pin `PC3` (Channel 13) and Current on pin `PC2` (Channel 12).

### Calibration Principles

The `PowerMonitor` task runs at 100 Hz, sampling raw 12-bit ADC values and applying single-pole PT1 low-pass filtering (`batt.lpf_hz`) to smooth voltage ripple caused by motor switching:

1. **Voltage Conversion**:
   $$\text{Voltage (cV)} = \text{round}\left( \frac{\text{raw\_count} \times \text{voltage\_scale} \times 330}{4095 \times \text{voltage\_divider} \times \text{voltage\_multiplier}} \right)$$
2. **Current Conversion**:
   $$\text{Current (cA)} = \frac{\text{raw\_count} \times 3300}{4096} \times \frac{10000}{\text{current\_scale}} + \text{current\_offset}$$
3. **Auto Cell Detection**:
   On boot, the monitor discards initial settling readings (`batt.settle_ticks`, ~100 ms) before evaluating total pack voltage against `batt.no_battery_v` and `batt.cell_detect_v` to determine the S-count ($1\text{S} \dots 6\text{S}$).

### Verification Procedure

1. Connect the board via USB serial terminal or `cybflight-gs`.
2. Connect a calibrated multimeter across the battery pads/LiPo main lead.
3. Query live power metrics in the shell:
   ```bash
   > power
   ```
4. Verify that reported cell count (`cell_count`) and pack voltage match the multimeter reading.
5. If voltage deviates, adjust parameters in-memory and save:
   ```bash
   > param set voltage_scale <calibrated_val>
   > param save
   ```

---

## Step 2: Sensor Bus Probing & Verification

Hardware initialization routines probe SPI and I2C buses to verify sensor communication before spawning async reader tasks.

### SPI IMU Probing & High-Speed Switching

On the primary SPI bus (e.g., SPI4 on FlywooF745):
- **Initialization Rate**: SPI starts at 1 MHz ($54\text{ MHz} / 64 = 843.75\text{ kHz}$) to meet the MPU-6000 register write specification ($\le 1\text{ MHz}$) during `WHO_AM_I` probing.
- **Auto-Detection**: The driver automatically identifies connected IMU chips (MPU6000/MPU6500, ICM42688P/ICM426xx, or BMI270).
- **High-Speed Switch**: Upon successful initialization, the SPI clock scales up to 13.5 MHz (MPU/ICM) or 6.75 MHz (BMI270) for 8 kHz DMA data transfers.
- **Dedicated Interrupt Executor**: IMU DRDY reads execute on a dedicated P8 interrupt executor (`EXECUTOR_IMU` / `CAN2_RX1`), positioned strictly between P6 DShot and P10 Control (`EXECUTOR_CTRL`) to prevent 126–210 µs INDI matrix solves from stalling 125 µs IMU reads.

### Shared I2C Bus Unsticking & Probing

Shared I2C buses (e.g., I2C1 with SPL06 barometer at `0x76` and QMC5883L compass at `0x0D`) feature hardware unsticking routines to recover from warm MCU resets (`reboot`, `reboot --dfu`, or IWDG reset):
1. **Bus Recovery**: GPIO SCL/SDA lines are configured in open-drain mode with pull-ups. SCL toggles 9 clock pulses with SDA high to release any slave holding SDA low, followed by a clean I2C STOP condition.
2. **Ordered Probing**: Per the shared bus specification, **all** devices on a shared I2C bus are probed and initialized before spawning any asynchronous reader tasks.
3. **Wedged Recovery**: `reset_i2c1_if_wedged()` clears stuck peripheral states (`TC=1, AUTOEND=0, BUSY=1`) by cycling `CR1.PE`.

### Verification Procedure

Execute snapshot commands in the serial CLI:
```bash
> imu1
> imurate
> baro1
> magext
```
Verify that `imurate` reports sample rates matching the expected build configuration (e.g., 8000 Hz or 1000 Hz). In `cybflight-gs` Configurator, select the **Setup Tab** and physically tilt the flight controller board by hand to confirm 3D model orientation and Forward-Left-Up (FLU) axis alignment:
- $+X$: Forward (Nose Down = Positive Pitch)
- $+Y$: Left (Left Wing Down = Positive Roll)
- $+Z$: Up (Counter-Clockwise Rotation from top = Positive Yaw)

---

## Step 3: RC Channel Mapping & Receiver Setup

The serial receiver task supports CRSF (ELRS/TBS Crossfire at 420,000 baud) and GHST protocols.

### Stick Calibration & Axis Assignment

The RC interpreter converts raw servo pulse widths ($\mu\text{s}$) into normalized control inputs:
- **Axis Mapping Order**: `Axis::Throttle` (0), `Axis::Roll` (1), `Axis::Pitch` (2), `Axis::Yaw` (3).
- **Endpoint Limits**: Configured via `StickEndpoints` (`rc_min_us`, `rc_mid_us`, `rc_max_us`), defaulting to standard OpenTX/EdgeTX travel ($988\ \mu\text{s}$, $1500\ \mu\text{s}$, $2012\ \mu\text{s}$).
- **Safety Fallback**: `StickEndpoints::or_default()` rejects degenerate or misordered endpoints (e.g., $mid < min$) to prevent stick axis inversion.

### Verification Procedure

1. Power on the RC transmitter and ensure receiver binding.
2. Run the RC snapshot command in the CLI:
   ```bash
   > rc
   > rcstats
   ```
3. Move transmitter sticks through full travel:
   - Roll Right: Mapped value increases towards $+1.0$.
   - Pitch Forward: Mapped value increases towards $+1.0$.
   - Yaw Right: Mapped value increases towards $+1.0$.
   - Throttle Up: Mapped value increases from $0.0$ to $1.0$.
4. Check arming switch allocation and confirm `rcstats` reports high link quality (LQ) and frame rates.

---

## Step 4: DShot Motor Direction & eRPM Testing

Actuator output uses dual-timer Bidirectional DShot600 (e.g., TIM3 and TIM1 on FlywooF745). Bidirectional DShot transmits motor speed commands while capturing GCR pulses on the inverted timer channel to extract live motor eRPM telemetry.

### QuadX Motor Mapping Conventions

Standard QuadX motor numbering and rotation direction (top view):
- **Motor 1 (m0)**: Rear-Right (Clockwise, CW)
- **Motor 2 (m1)**: Front-Right (Counter-Clockwise, CCW)
- **Motor 3 (m2)**: Rear-Left (Counter-Clockwise, CCW)
- **Motor 4 (m3)**: Front-Left (Clockwise, CW)

### Bench Motor Test Procedure (Props OFF)

1. Ensure propellers are completely removed.
2. Apply LiPo power to power the ESCs.
3. Test individual motor spin and direction using the CLI `motor` command:
   ```bash
   > motor 1 10
   > motor 2 10
   > motor 3 10
   > motor 4 10
   ```
4. Confirm motor direction visually or by lightly touching the motor bell:
   - M1 (Rear-Right): CW
   - M2 (Front-Right): CCW
   - M3 (Rear-Left): CCW
   - M4 (Front-Left): CW
5. Query live DShot telemetry and eRPM:
   ```bash
   > dshot
   ```
   Verify that valid non-zero eRPM values are returned for spinning motors with zero packet error rate.
6. To stop motor testing, command $0\%$ throttle:
   ```bash
   > motor 1 0
   ```

---

## Step 5: Attitude Filter & ESKF Alignment

The autopilot runs dual state estimators: an always-on Mahony complementary filter and an optional GNSS/INS or Mocap/INS Error-State Kalman Filter (ESKF).

### Mahony Filter Warm-up

The Mahony filter subscribes to the decimated IMU stream (`IMU_1_DECIM`, ~1 kHz) and updates an IMU-only $SO(3)$ orientation estimate. It requires ~1000 updates (~1 second) to converge gravity-aided roll/pitch.

### ESKF Frame Alignment & ENU Anchoring

The ESKF estimator task (`EskfGpsGuard` / `EskfMocapGuard`) handles position and velocity fusion:
- **Origin Selection**: Configured via vehicle YAML `origin:` section (fixed geodetic coordinates) or runtime anchor (first **RTK-fixed** GNSS sample with `carr_soln == 2`).
- **Alignment Phase**: The ESKF waits for an RTK-fixed position fix before initializing state prediction at 1 kHz and publishing valid `/odometry` messages.
- **Heading Fusion**: Dual-antenna GNSS systems (e.g., Unicore UM982 via `gps_unicore` feature) provide absolute yaw alignment; single-antenna systems rely on IMU tilt and accel-coupled motion.

### Verification Procedure

1. Monitor estimator status in the CLI:
   ```bash
   > eskf
   > gpsrtk
   > att
   ```
2. Verify that `eskf` transitions to `EstimatorPhase::Ready` and covariance values remain bounded.
3. Confirm that roll and pitch return to near zero ($0.0^\circ \pm 1.0^\circ$) when the airframe rests level on the bench.

---

## Step 6: Configurator Verification & Pre-flight Checklist

The `cybflight-gs` Ground Station Configurator provides a unified UI for hardware verification before first flight.

### Ground Station Setup Tab Features

- **Interactive 3D Viewport**: Renders the quadrotor 3D model with FLU body axes, CAD STL mesh overlays, and dynamic 3D thrust pillars.
- **Telemetry Display**: Displays live Roll/Pitch/Yaw angles, quaternion components $[x, y, z, w]$, accelerometer readings $(\text{m/s}^2)$, gyroscope rates $(\text{deg/s})$, pack voltage, cell count, and IMU temperature.
- **Live Motor Output Progress Bars**: Displays individual motor throttle percentages, calculated thrust outputs ($\text{gf}$ and $\text{N}$), and telemetry eRPM values.
- **Attitude Stream Re-activation**: Offers a one-click button ("Re-enable Attitude/IMU Streams") to send `stream att on`, `stream imu1 on`, `stream power on`, and `stream dshot on` commands over USB CDC serial.

```
+-----------------------------------------------------------------------------------+
| Live Motor Outputs & Mixer Response (QuadX)                                      |
| m0 (M1 Rear-Right CW)  [====================          ] 45.0% | 220.5 gf | 8500 eRPM|
| m1 (M2 Front-Right CCW)[====================          ] 45.0% | 220.5 gf | 8510 eRPM|
| m2 (M3 Rear-Left CCW)  [====================          ] 45.0% | 220.5 gf | 8490 eRPM|
| m3 (M4 Front-Left CW)  [====================          ] 45.0% | 220.5 gf | 8505 eRPM|
+-----------------------------------------------------------------------------------+
```
*Live motor telemetry panel in cybflight-gs Setup Tab.*

### Final Pre-flight Checklist

Before installing propellers and moving to flight testing, verify every item in the checklist:

| Verification Item | Command / UI Tool | Acceptance Criteria |
|---|---|---|
| **Board & Build Identity** | `version` CLI | Firmware version, git commit, and board target match expectation |
| **Power Calibration** | `power` CLI / Setup Tab | Reported pack voltage matches multimeter within $\pm 0.05\text{ V}$ |
| **IMU & Axes** | `imu1`, 3D Viewport | 8 kHz IMU active; physical tilt matches 3D model roll/pitch |
| **Shared I2C Sensors** | `baro1`, `magext` CLI | Barometer and compass detected and returning non-zero data |
| **RC Control Link** | `rc`, `rcstats` CLI | Channels respond cleanly; LQ $> 90\%$; endpoints within limits |
| **Motor Order & Rotation** | `motor 1..4` CLI | Props OFF; M1..M4 spin in correct QuadX CW/CCW directions |
| **DShot Telemetry** | `dshot` CLI / Setup Tab | Live eRPM reported per motor with 0% CRC error rate |
| **Estimator Ready** | `eskf`, `health` CLI | ESKF phase is `Ready`; `health` command reports zero arming blocks |
| **Failsafe Behavior** | RC Power-off Test | Transmitter power-off triggers RC loss guard and disarm |

---

## Related Pages

- [BSP & Hardware Initialization](../architecture/bsp-and-hardware-init.md)
- [Safety Architecture & Post-Mortem](../concepts/safety-and-postmortem.md)
- [Ground Station & cybflight-gs](../integrations/ground-station-and-gs.md)
- [Quickstart Guide](../quickstart.md)
