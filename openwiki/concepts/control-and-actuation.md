---
type: concept
title: Flight Control & Motor Actuation Systems
description: Multi-rate inner-loop flight control stack, including SE(3) geometric cascade control, Incremental Nonlinear Dynamic Inversion (INDI) rate control, Savitzky-Golay rate-dot estimation, RPM notch filtering, active-set WLS motor allocation, and bi-directional DShot actuation.
tags: [control, indi, dshot, attitude-control, motor-mixing, notch-filter]
verified:
  - by: openwiki/0.7.2
    at: 2026-10-30T14:16:13.882Z
sources:
  - id: openwiki-source-cdae040dbb735f0630239835
    resource: repo://crates/cybflight_core/src/attitude_control/geometric_controller.rs
  - id: openwiki-source-a136f318523f076425d36317
    resource: repo://crates/cybflight_core/src/indi/controller.rs
  - id: openwiki-source-dea498633c81b5d90d0ac34c
    resource: repo://crates/cybflight_core/src/indi/linearization.rs
  - id: openwiki-source-9c59c713b2256ac820f13fcf
    resource: repo://crates/cybflight_core/src/indi/rate_dot_estimator.rs
  - id: openwiki-source-3abcf6a07db9ecf4f6f303d3
    resource: repo://crates/cybflight_core/src/indi/rpm_notch.rs
  - id: openwiki-source-a7b5033554a56fc18df0a911
    resource: repo://crates/cybflight_core/src/mixer.rs
  - id: openwiki-source-674ee94ad2f4d31a3bad99a0
    resource: repo://crates/cybflight/src/control/indi_task.rs
  - id: openwiki-source-a7c02f463def0d4c51e56b7f
    resource: repo://crates/cybflight/src/motors/dshot.rs
  - id: openwiki-source-c7594cc3094729d0aed983a3
    resource: repo://crates/drivers/src/dshot/gcr.rs
  - id: openwiki-source-e9b0c5081cb9ce7cf4e5a8e6
    resource: repo://crates/drivers/src/dshot/telemetry.rs
generated: { by: "openwiki/0.7.2", at: "2026-10-30T14:16:13.882Z" }
---

The flight control and motor actuation architecture in CybFlight provides closed-loop control across multi-rate task loops, bridging trajectory references to low-level motor hardware. The system combines an outer-loop cascade/MPC position and attitude controller (~100 Hz), an inner-loop Incremental Nonlinear Dynamic Inversion (INDI) rate controller running at the IMU sample rate (1 kHz to 2 kHz depending on `indi_ctrl_div`), and a real-time bi-directional DShot motor driver task (~5 kHz to 8 kHz) with Grey Coded Response (GCR) RPM telemetry.

---

## Architectural Multi-Rate Control Strategy

CybFlight organizes flight control into decoupled control rates to handle different dynamic bandwidths:

1. **Outer Loop Control (~100 Hz)**: Executed by `cascade_task` (or MPC / RL policy tasks), generating body-rate references ($\boldsymbol{\omega}_{ref}$) and collective thrust setpoints ($T_{sp}$) published on the `RATE_COMMAND` channel.
2. **Inner Loop Control (`indi_task`, 1–2 kHz)**: Subscribes to raw IMU sensor data (`IMU_1`), `RATE_COMMAND`, and motor RPM telemetry (`DSHOT_TELEMETRY`). Computes body angular acceleration derivatives, evaluates motor dynamics, solves a Weighted Least Squares (WLS) allocation problem, and outputs normalized motor throttle commands (`ACTUATOR_MOTORS`).
3. **Actuation & Telemetry (`dshot_task`, ~5–8 kHz)**: Converts normalized throttle setpoints to 16-bit DShot600 frames sent via DMA timer PWM, reads 21-bit GCR telemetry replies during line turnaround, decodes eRPM or Extended DShot Telemetry (EDT), and publishes decoded telemetry to `DSHOT_TELEMETRY`.

```mermaid
flowchart TD
    subgraph OuterLoop["Outer Loop (~100 Hz)"]
        TR["Trajectory / RC Reference"] --> PC["Geometric Position Controller / MPC"]
        PC -->|"collective_thrust_n, body_rate_rad_s"| RC["RATE_COMMAND Signal"]
    end

    subgraph InnerLoop["Inner Loop Loop (indi_task, 1–2 kHz)"]
        RC --> IT["INDI Task Step"]
        IMU["IMU_1 (Gyro & Accel)"] --> RN["RPM Notch Filter Bank"]
        RN -->|"gyro_corrected"| RD["Rate-Dot Estimator (SG + Biquad)"]
        RD -->|"rate_dot_fs"| IT
        
        TELEM["DSHOT_TELEMETRY"] --> SLEW["Slew Outlier Filter"]
        SLEW --> EKF["FOPDT EKF RPM Estimator"]
        EKF -->|"omega_hat"| TRACK["RpmTracker / LPF"]
        TRACK -->|"omega_fs, omega_dot_fs"| IT

        IT -->|"v_ind = Δv - G2·Ω_dot"| WLS["Active-Set WLS Allocator"]
        WLS -->|"Δu"| LIN["Thrust Linearization (Quad / SqrtSq / Table)"]
        LIN -->|"output.motor_commands"| AM["ACTUATOR_MOTORS Signal"]
    end

    subgraph DirectOverride["Direct Offboard Mode"]
        DMC["DIRECT_MOTOR_COMMAND"] -->|"override"| AM
    end

    subgraph Actuation["Actuation & Driver (dshot_task, ~5–8 kHz)"]
        AM --> DT["DShot DMA Driver"]
        DT -->|"DShot600 Frame"| ESC["ESC Hardware / Motors"]
        ESC -->|"GCR Line Reply"| GCR["GCR Packet Decode"]
        GCR -->|"eRPM / EDT"| TELEM
    end

    caption End-to-end control loops, notch filtering, WLS allocation, and DShot actuation pipeline.
```

---

## SE(3) Geometric Cascade Position & Attitude Control

The geometric control module in `crates/cybflight_core/src/attitude_control/geometric_controller.rs` implements differential-flatness position control coupled with $SO(3)$ or tilt-prioritized attitude tracking based on Lee et al. and Faessler et al.

### Position Control & Differential Flatness

Given position reference $\mathbf{p}_{ref}$, velocity $\mathbf{v}_{ref}$, acceleration $\mathbf{a}_{ref}$, jerk $\mathbf{j}_{ref}$, and snap $\mathbf{s}_{ref}$, desired spatial acceleration $\mathbf{a}_{des}$ is derived using saturated PD errors:

$$\mathbf{a}_{des} = K_p \cdot \text{sat}(\mathbf{p}_{ref} - \mathbf{p}) + K_d \cdot \text{sat}(\mathbf{v}_{ref} - \mathbf{v}) + \mathbf{a}_{ref} + g \hat{\mathbf{z}}$$

Mass-normalized collective thrust along body axis $z_B$ is computed by projecting desired acceleration onto the estimated body vertical axis $\mathbf{z}_B = R \hat{\mathbf{e}}_3$:

$$c = \max(\mathbf{a}_{des} \cdot \mathbf{z}_B, \, c_{min})$$

The desired rotation matrix $R_{des} = [\mathbf{x}_B, \mathbf{y}_B, \mathbf{z}_B]$ is constructed by normalizing $\mathbf{z}_B = \mathbf{a}_{des} / \|\mathbf{a}_{des}\|$, taking $\mathbf{x}_C = [\cos \psi, \sin \psi, 0]^T$ from target heading $\psi$, and defining orthogonal axes $\mathbf{y}_B = \frac{\mathbf{z}_B \times \mathbf{x}_C}{\|\mathbf{z}_B \times \mathbf{x}_C\|}$ and $\mathbf{x}_B = \mathbf{y}_B \times \mathbf{z}_B$.

Reference body rates $\boldsymbol{\omega}_{ff}$ are computed via differential flatness inversion:

$$\omega_x = -\frac{1}{c} \mathbf{y}_B \cdot \mathbf{j}_{ref}, \quad \omega_y = \frac{1}{c} \mathbf{x}_B \cdot \mathbf{j}_{ref}$$

### Attitude Tracking & Error Laws

`evaluate_attitude_error` supports three attitude error formulation modes:

1. **`GeometricSO3`**: Skew-symmetric matrix difference error $\mathbf{e}_R = \frac{1}{2} (R_{des}^T R - R^T R_{des})^\vee$.
2. **`QuaternionBased`**: Quaternion vector error $-2 \cdot \text{copysign}(\mathbf{q}_e.vec, q_{e,w})$ where $\mathbf{q}_e = \mathbf{q}^{-1} \otimes \mathbf{q}_{des}$.
3. **`TiltPrioritizing`**: Separates tilt error from yaw rotation, preventing yaw deviations from corrupting thrust vector alignment during large orientation steps.

Feedback rates $\boldsymbol{\omega}_{fb} = -K_{rp, yaw} \cdot \mathbf{e}_R$ are added to feedforward body rates $\boldsymbol{\omega}_{ff}$ to produce total commanded body rates. Optional exact feedback linearization cancels natural gyroscopic coupling ($\boldsymbol{\omega} \times J \boldsymbol{\omega}$) and transport cross-terms $J(\boldsymbol{\omega} \times \boldsymbol{\omega}_{ref})$.

---

## Incremental Nonlinear Dynamic Inversion (INDI) Rate Control

The inner-loop controller (`crates/cybflight_core/src/indi/controller.rs`) tracks body-rate setpoints by controlling angular accelerations $\dot{\boldsymbol{\omega}}$ directly. Unlike classical PID controllers that depend on integral error terms, INDI relies on high-rate measurement of angular acceleration feedback ($\dot{\boldsymbol{\omega}}_{fs}$) and estimated actuator states ($u_{state}$).

### Incremental Law Derivation

1. **Angular Acceleration Setpoint ($\boldsymbol{\alpha}_{sp}$)**:
   For rate tracking, desired angular acceleration is computed from rate error:
   $$\boldsymbol{\alpha}_{sp} = K_{rate} \odot (\boldsymbol{\omega}_{ref} - \boldsymbol{\omega}) + \dot{\boldsymbol{\omega}}_{ref}$$
   In full-state torque mode (`outer_mpc_full`), $\boldsymbol{\alpha}_{sp}$ is derived directly from desired body torque $\boldsymbol{\tau}_d$ and gyro feedback $\boldsymbol{\omega}$:
   $$\boldsymbol{\alpha}_{sp} = I^{-1} \left( \boldsymbol{\tau}_d - \boldsymbol{\omega} \times (I \boldsymbol{\omega}) \right)$$

2. **Virtual Control Increment ($\Delta \mathbf{v}$)**:
   The core INDI assumption posits that disturbances change slowly relative to the control frequency. The required virtual control change $\Delta \mathbf{v}$ compensates for the difference between target acceleration $\boldsymbol{\alpha}_{sp}$ and filtered derivative feedback $\dot{\boldsymbol{\omega}}_{fs}$:
   $$\Delta \mathbf{v} = \begin{bmatrix} \Delta f_z \\ \boldsymbol{\alpha}_{sp} - \dot{\boldsymbol{\omega}}_{fs} \end{bmatrix}$$

3. **Rotor Dynamics & Gyroscopic Torque Compensation ($G_2$ Matrix)**:
   Rapid rotor speed changes produce counter-torques due to propeller rotational inertia. INDI accounts for motor dynamics via matrix $G_2$:
   $$\mathbf{v}_{ind} = \Delta \mathbf{v} - G_2 \dot{\boldsymbol{\Omega}}_{fs}$$
   where $\dot{\boldsymbol{\Omega}}_{fs}$ is the filtered time derivative of per-motor rotational speeds.

4. **Weighted Least Squares (WLS) Allocation**:
   The controller maps $\mathbf{v}_{ind}$ to motor command increments $\Delta \mathbf{u}$ using an active-set WLS solver (`flight_solver::cls`):
   $$\min_{\Delta \mathbf{u}} \| W_v (G_1 \Delta \mathbf{u} - \mathbf{v}_{ind}) \|^2 + \| W_u (\mathbf{u}_{state,fs} + \Delta \mathbf{u} - \mathbf{u}_0) \|^2$$
   subject to actuator limits $u_{min, i} \le u_{state,fs, i} + \Delta u_i \le u_{max, i}$.

5. **State Update & Command Output**:
   The new control state $\mathbf{u}_{state} = \mathbf{u}_{state,fs} + \Delta \mathbf{u}$ is updated and passed through thrust linearization to produce raw motor throttle commands $\mathbf{u} \in [0, 1]^4$.

### Ground Contact & Pre-Launch Modes

- **Ground Contact Detection**: When gyro norm is below `ground_gyro_rad_s`, specific force magnitude matches gravity ($\approx 9.81 \text{ m/s}^2$), and vertical thrust command is low, INDI disables incremental feedback (`do_indi = false`). This turns INDI into a pure proportional rate controller on the ground, preventing integrator windup and ground bounce.
- **Pre-Launch Idle Bypass**: When armed but not launched (`armed && !launched`), `indi_task` bypasses the INDI and EKF pipeline entirely, outputting uniform idle throttle (`idle_normalized`) to all motors. On the `!launched -> launched` transition edge, `reset_indi_state!` re-seeds all filters, slew gates, and EKF states.

---

## Rate-Dot Estimation & Group-Delay Synchronization

Computing clean angular acceleration $\dot{\boldsymbol{\omega}}$ from high-frequency gyro data requires noise reduction without introducing group-delay mismatches.

### Savitzky-Golay Derivative Filtering

Rather than using backward-differencing (which severely amplifies high-frequency IMU noise), `RateDotEstimator` (`crates/cybflight_core/src/indi/rate_dot_estimator.rs`) applies a streaming Savitzky-Golay (SG) FIR filter:
- Performs local least-squares polynomial fitting (order $n=2$, window size $N$ derived from loop rate via `rate_dot_sg_window`) over past gyro samples.
- Evaluates the analytical polynomial derivative at the latest sample, providing a low-noise estimate of $\dot{\boldsymbol{\omega}}$.
- Followed by a single biquad low-pass filter to smooth residual high-frequency artifacts.

### Sync Filter Group-Delay Matching

A core invariant of INDI stability is **group-delay alignment**: feedback signals $\dot{\boldsymbol{\omega}}_{fs}$, actuator state $\mathbf{u}_{state,fs}$, and specific force $spf_{fs}$ must be evaluated at the exact same point in time.

All sync biquad filters (`spf_filter`, `u_state_filter`, `omega_filter`, `rate_dot_estimator.post`) derive their cutoff frequency from `sync_filter_hz` (Nyquist-clamped to $0.48 \cdot f_{loop}$). This ensures equal delay across sensor feedback, actuator history, and rate derivatives.

---

## RPM-Tracking Notch Filters

Vibration injected by rotating motors can cause positive feedback loops through INDI's high-bandwidth rate loop. `RpmNotchBank` (`crates/cybflight_core/src/indi/rpm_notch.rs`) places dynamic, motor-frequency-tracked biquad notch filters directly on gyro and accelerometer signals.

```
Motor Telemetry (eRPM) ──> PT1 LPF (@150 Hz) ──> Round-Robin Notch Refresh ──> DirectForm1 Biquad Cascade ──> Notched IMU
```

1. **DirectForm1 Architecture**: DirectForm1 biquads store past input/output signal samples in delay lines rather than intermediate internal states. When notch center frequencies move dynamically, DF1 structure prevents state transients and output spikes.
2. **Harmonic Cascades**: Supports $NU$ motors with $NH$ harmonics per motor ($NU=4$, $NH=3$ on gyro; $NH=1$ on accel).
3. **PT1 Notch Frequency Tracker**: Motor speeds from DShot telemetry pass through a 1st-order PT1 LPF (cutoff `rpm_notch_lpf_hz`, default 150 Hz) dedicated to frequency tracking. Operating independently from the 15 Hz sync biquad, it tracks $1P$ motor frequencies without transient phase lag during throttle steps.
4. **Round-Robin Batched Coefficient Refresh**: Recomputing all $NU \times NH \times 3$ notch filter coefficients every tick is computationally expensive. `RpmNotchBank` refreshes a subset (`notches_per_call`) of notch positions on each loop step, updating the full filter bank within a ~1 ms time window.
5. **External Crossfade**: When motor frequency drops below `min_hz` or approaches Nyquist ($0.48 \cdot f_{loop}$), the notch crossfades smoothly to passthrough ($y = w \cdot \text{biquad}(x) + (1-w) \cdot x$).

---

## Motor Effectiveness, Active-Set WLS, & Thrust Linearization

Motor mixing and thrust generation transform non-dimensional throttle commands into physical forces and moments.

### Physical Effectiveness Matrix ($G_1$)

`MotorEffectiveness` (`crates/cybflight_core/src/mixer.rs`) constructs the $4 \times N$ matrix $G_1$ mapping throttle vector $\mathbf{u} \in [0,1]^N$ to virtual forces/torques $\mathbf{v} = [T_z, \tau_x, \tau_y, \tau_z]^T$:

$$G_1 = \begin{bmatrix}
T_{max,0} & T_{max,1} & T_{max,2} & T_{max,3} \\
p_{y,0} T_{max,0} & p_{y,1} T_{max,1} & p_{y,2} T_{max,2} & p_{y,3} T_{max,3} \\
-p_{x,0} T_{max,0} & -p_{x,1} T_{max,1} & -p_{x,2} T_{max,2} & -p_{x,3} T_{max,3} \\
s_0 c_0 T_{max,0} & s_1 c_1 T_{max,1} & s_2 c_2 T_{max,2} & s_3 c_3 T_{max,3}
\end{bmatrix}$$

- Forward-Left-Up (FLU) body frame: $+x$ forward, $+y$ left, $+z$ up.
- Spin direction sign $s_i \in \{+1, -1\}$: CW prop (looking down) $\rightarrow s_i = +1$ (positive reaction yaw torque in $+z$); CCW prop $\rightarrow s_i = -1$.
- Torque coefficient $c_i = \text{torque\_coeff\_m}$ (ratio of reaction torque to thrust force).

### Thrust-to-Command Linearization Models

To map normalized thrust demand $d \in [0, 1]$ to DShot throttle command $u \in [0, 1]$, `ThrustLinearization` (`crates/cybflight_core/src/indi/linearization.rs`) provides three models:

| Thrust Model | Equation / Source | Characteristics |
| :--- | :--- | :--- |
| **`Quadratic`** | $u = k \cdot d^2 + (1-k) \cdot d$ | Standard Indiflight model; parameter $k$ fits steady-state thrust curves. |
| **`SqrtSquared`** | $u = \left( k \cdot d + (1-k) \sqrt{d} \right)^2$ | Models linear motor speed mixing where $T \propto \omega^2$; fits bench data closely. |
| **`Table`** | 2D Bilinear Lookup: $(T_{per-rotor}, V_{batt}) \rightarrow u$ | Evaluates bench CSV grid data across thrust $N$ and battery voltage $V$. Automatically compensates for battery sag. |

When in `Table` mode, `indi_task` subscribes to `POWER_STATUS` (100 Hz). Plausible voltage readings ($V_{min} \le V \le V_{max}$) update the lookup grid. If power telemetry stalls past `VOLTAGE_STALE_TIMEOUT` (500 ms), the held voltage is flagged as stale; if it remains stale past `VOLTAGE_FAILSAFE_TIMEOUT` (2000 ms) while armed, the task trips a failsafe.

---

## Direct Motor Override Mode & Safety Watchdogs

`indi_task` includes safety gates and offboard override modes to protect hardware and respond to upstream system failures.

### Direct Motor Override Mode

When `OFFBOARD_ACTIVE` atomic is set, offboard controllers can publish directly to `DIRECT_MOTOR_COMMAND`.
- If a fresh `DIRECT_MOTOR_COMMAND` arrives and no overriding `RATE_COMMAND` is present, `indi_task` bypasses the INDI calculation and passes `direct_cmds` directly to `ACTUATOR_MOTORS`.
- Receiving any new `RATE_COMMAND` from the inner/outer loop immediately clears and supersedes the direct motor command override.

### Silence-as-Failure Protocol & Failsafes

Rather than publishing zero thrust or corrupted values during severe faults, `indi_task` **goes silent** (stops signaling `ACTUATOR_MOTORS`). The DShot task watchdog monitors `LAST_CONTROLLER_PUBLISH`; if no command is received within 10 ms (`MOTOR_CMD_STALE`), DShot drops output to idle throttle. If silence persists beyond `fs_ctrl_timeout_s` (e.g. 500 ms), the failsafe controller disarms the vehicle.

Silence is triggered by:
1. **Command Staleness**: No fresh `RATE_COMMAND` or `DIRECT_MOTOR_COMMAND` received within 100 ms (`CMD_STALE_TIMEOUT`).
2. **Non-Finite Controller Outputs**: Any NaN or $\pm\infty$ value produced by WLS allocation or filter steps.
3. **Sustained WLS NaN**: If WLS solver returns NaN for `nan_limit` consecutive ticks (~2.5 ms at 8 kHz), `INNER_SILENT_CAUSE` is set to `SILENT_CAUSE_WLS_NAN` and output stops.
4. **Table Voltage Failsafe**: In `Table` mode while armed, if battery voltage stays stale for $>2000 \text{ ms}$, `INNER_SILENT_CAUSE` is set to `SILENT_CAUSE_VOLTAGE_STALE` and output stops.

---

## DShot Bi-Directional Protocol & Telemetry Integration

The motor driver (`crates/cybflight/src/motors/dshot.rs` & `crates/drivers/src/dshot/`) implements DShot600 digital motor signaling with bi-directional GCR telemetry decoding.

### Protocol Timing & DMA Execution

1. **DShot Output**: Transmits 16-bit DShot command words at 600 kbit/s (1.67 µs per bit) using timer DMA. PWM duty cycles encode zero (`DSHOT600_BIT_0`) and one (`DSHOT600_BIT_1`) bits.
2. **Line Turnaround & GCR Capture**: Following frame transmission (~27 µs), the GPIO pin switches to Input Capture (IC) mode to record edge transitions during the ESC's reply window (~80 µs).
3. **Dynamic Ticks-Per-Bit Detection**: `detect_ticks_per_bit` (`gcr.rs`) calculates the minimum capture gap across edge timings. This automatically adapts bit timing to ESC clock drifts (e.g., AM32 vs BLHeli_32).
4. **GCR Nibble Decode & Checksum**:
   Decodes 21 transition edges into 4 $\times$ 5-bit GCR nibbles using lookup table `GCR_DECODE`. Verifies that the 4-bit XOR nibble checksum equals `0xF`.

### Telemetry Decoding & Health Buckets

The 12-bit raw telemetry frame is decoded via `telemetry::interpret`:
- **eRPM Decode**: For raw value $R$, period is $P = \text{mantissa} \ll \text{exponent}$. Speed in eRPM $\times 100$ is computed as:
  $$\text{eRPM} = \frac{20{,}000{,}000}{P}$$
  Raw value `0x0FFF` signals motor stopped (`TelemetryValue::Stopped`).
- **Extended DShot Telemetry (EDT)**: Decodes auxiliary telemetry when enabled (temperature, voltage, current, state events).
- **Health Tracking**:
  `DshotMotorHealth` tracks frame stats:
  - `passed`: Valid eRPM decoded and passed range + slew gates.
  - `no_fresh`: Frame decoded with period 0 (ESC reported no new electrical commutation since last frame; expected low-RPM traffic).
  - `no_reply`: GCR decode failure or line turnaround timeout.
  - `range_reject` / `slew_reject` / `nis_reject`: Samples exceeding physical limits or EKF NIS gates.

---

## EKF RPM Estimation & Slew Filtering

Telemetry raw samples pass through outlier filtering and an Extended Kalman Filter (EKF) to produce smooth motor speed feedback ($\hat{\omega}$).

### Slew Outlier Filter

Before feeding the EKF or raw telemetry path, incoming eRPM values pass through a per-motor `SlewFilter<f32>`:
- Max allowable inter-sample delta is bounded by physical acceleration limits:
  $$\Delta \omega_{max} = \frac{\omega_{max}}{\tau_{min}} \cdot \Delta t_{worst}$$
  where $\tau_{min} = 0.005 \text{ s}$ and $\Delta t_{worst} = 1.0 \text{ ms}$.
- Samples exceeding $\Delta \omega_{max}$ trigger a zero-order hold (ZOH) freeze on state.

### FOPDT Motor EKF

`RpmEstimator` (`crates/cybflight/src/estimation/rpm_estimator.rs`) runs a First-Order Plus Dead Time (FOPDT) EKF for each motor:
- **State Model**: Tracks motor mechanical speed $\omega$ and motor gain $c_m$:
  $$\frac{d\omega}{dt} = \frac{c_m \cdot f_{curve}(u(t - \tau_d)) - \omega}{\tau_m}$$
  where $\tau_d$ represents transport/communication delay and $\tau_m$ is the motor spool-up time constant.
- **Normalized Throttle History**: A ring buffer stores historical throttle commands $u(t)$ over $\tau_d$.
- **Innovation Gating**: incoming eRPM measurements are gated against Normalized Innovation Squared (NIS) bounds (`nis_gate`). Rejected measurements update error counters without corrupting state covariance.

---

## Verification & Test Evidence

The control and actuation subsystem is covered by host unit tests, C-reference regression suites, and bench comparison tests:

1. **INDI Golden C-Reference Test (`crates/cybflight_core/tests/indi_golden.rs`)**:
   Compares the Rust INDI controller, biquad filters, WLS solver, and $G_1$ allocation against reference C fixtures (`golden.csv`) generated from `indiflight`. Asserts exact numerical equivalence across filter states, virtual controls, and motor outputs.
2. **Thrust Model Bench Comparison (`crates/cybflight_core/tests/thrust_model_compare.rs`)**:
   Evaluates `Quadratic`, `SqrtSquared`, and 2D `Table` thrust models against A2RL 6S bench dataset (`a2rl_0114.csv`). Verifies that analytical $k$-fit approximations remain within target error bounds ($\approx 0.27 \text{ N}$ RMS for Quadratic, $\approx 0.23 \text{ N}$ RMS for SqrtSquared) relative to ground-truth bench tables.
3. **Control Convergence Tests (`crates/cybflight_core/tests/control_convergence.rs`)**:
   Validates tracking convergence and closed-loop stability of the geometric position and attitude controller under simulated initial state offsets.

---

## Related References

- [Channel Dataflow Architecture](../architecture/channel-dataflow.md) — Inter-task messaging, publish/subscribe channels, and execution rates.
- [Neural & Offboard Control](neural-and-offboard-control.md) — Neural network policy execution and direct offboard control integration.
- [NMPC & Trajectory Planning](nmpc-and-trajectory.md) — High-level trajectory generation, MINCO trajectory sampling, and non-linear MPC outer loop.
