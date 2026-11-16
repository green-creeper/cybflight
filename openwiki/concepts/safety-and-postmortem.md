---
type: concept
title: Safety, Failsafes & Postmortem Diagnostics
description: Comprehensive safety architecture in Cybflight including arming gates, failsafe modes, hardware watchdog supervision, and BKPSRAM postmortem crash diagnostics.
tags: [safety, failsafe, arming, watchdog, postmortem, bkpsram, fault-handling]
verified:
  - by: openwiki/0.7.2
    at: 2026-10-30T14:16:13.882Z
sources:
  - id: openwiki-source-06e47b1a6b02445ffc736038
    resource: repo://crates/cybflight/src/control/failsafe.rs
  - id: openwiki-source-38f657a50c6f0b7f5d8760f6
    resource: repo://crates/cybflight/src/health.rs
  - id: openwiki-source-97772717fcba28a929712e2d
    resource: repo://crates/cybflight/src/postmortem/bkpsram.rs
  - id: openwiki-source-de5d9fd7a471a1fc9892be45
    resource: repo://crates/cybflight/src/postmortem/fault.rs
  - id: openwiki-source-e19c26180c97696dcc366c2d
    resource: repo://crates/cybflight/src/postmortem/record.rs
  - id: openwiki-source-01e870ed3dc00f90925a72b7
    resource: repo://crates/cybflight/src/postmortem/recovery.rs
  - id: openwiki-source-c80122f65f45026746082d9d
    resource: repo://crates/cybflight/src/postmortem/task.rs
  - id: openwiki-source-bc1732e4c4dc88f9080205a6
    resource: repo://crates/cybflight/src/reset_cause.rs
  - id: openwiki-source-8e5b9bfc09f752e58bfd84a6
    resource: repo://crates/cybflight/src/sensors/rc.rs
  - id: openwiki-source-17bd21c0657cb91b659b54ff
    resource: repo://crates/cybflight/src/watchdog.rs
generated: { by: "openwiki/0.7.2", at: "2026-10-30T14:16:13.882Z" }
---

# Safety, Failsafes & Postmortem Diagnostics

Cybflight implements a multi-layered safety architecture designed to maintain flight safety during operation and preserve critical diagnostic state across fatal system failures. The safety subsystem spans arming interlocks, a two-stage radio control (RC) loss failsafe, inner-loop controller liveness monitoring, hardware watchdog hardware supervision, and non-volatile crash diagnostics backed by STM32 Backup SRAM (BKPSRAM).

---

## Architecture Overview

Safety enforcement is partitioned into proactive gate checks during boot and arming, continuous runtime monitoring during flight, and reactive fault logging during fatal crashes:

```
+-------------------------------------------------------------------------------+
|                             ARMING STATE MACHINE                              |
|   RC Link Active | Min Throttle | Gyro Calibrated | ESKF Ready | Hold Time    |
+-------------------------------------------------------------------------------+
                                       |
                                       v
+-------------------------------------------------------------------------------+
|                            RUNTIME FAILSAFE TASK                              |
|   Stage 1: RC Loss Guard (1.5 s) ---------> Stage 2: Immediate Disarm         |
|   Controller Timeout (>500 ms) -----------> Hard Disarm & Failsafe Active     |
+-------------------------------------------------------------------------------+
                                       |
                                       v
+-------------------------------------------------------------------------------+
|                         HARDWARE WATCHDOG (IWDG)                              |
|   Thread Executor Feeder (200 ms) --------> 500 ms Timeout MCU Hardware Reset |
+-------------------------------------------------------------------------------+
                                       |
                                       v
+-------------------------------------------------------------------------------+
|                          POSTMORTEM FAULT LOGGING                             |
|   Early Reset Capture (pre_init) ---------> BKPSRAM Fault Record & Event Ring |
|   Panic Handler / HardFault Ex ----------> Boot-Time USB Safe Mode Recovery   |
+-------------------------------------------------------------------------------+
```

The system interacts directly with the system state machine described in [Architecture Overview](../architecture/overview.md), the navigation pipeline in [State Estimation ESKF](state-estimation-eskf.md), and physical verification procedures in [Hardware Bringup](../workflows/hardware-bringup.md).

---

## Arming State Machine & Interlock Gates

System arming is managed by the RC sensor subsystem in `crates/cybflight/src/sensors/rc.rs`. Arming commands are received over radio receiver channels (e.g., CRSF or GHST protocols). Arming state updates are broadcast using the global signal `ARM_STATE` (`cybflight_msgs::ArmDisarm`), which is consumed by the DShot motor driver task in `crates/cybflight/src/motors/dshot.rs`.

### Arming Safety Gates

Before an arm transition can occur, all arming gates must pass sequentially. Disarming is always immediate and un-gated whenever the arm switch transitions off.

1. **Failsafe Inactive (`FAILSAFE_ACTIVE`)**:
   Arming is blocked if `FAILSAFE_ACTIVE` is `true`. Mirrors Betaflight `ARMING_DISABLED_FAILSAFE`.
2. **Throttle Minimum (`throttle_mincheck_us`)**:
   The throttle stick position must be below the configured threshold (default `1050 µs`) to prevent runaway throttles upon arming (`ARMING_DISABLED_THROTTLE`).
3. **RC Link Health & Quality (`link_active`, `min_link_quality`, `link_stats_max_age_ms`)**:
   The RC link must be active, with frame age under threshold (default `500 ms`) and link quality equal to or exceeding `min_link_quality` (default `50%`) (`ARMING_DISABLED_RX_FAILSAFE`).
4. **Gyro Calibration Complete (`IMU1_GYRO_CALIBRATED`)**:
   Stationary startup gyroscope calibration for the primary IMU must have completed (`ARMING_DISABLED_CALIBRATING`).
5. **Estimator Readiness (`ESTIMATOR_READY`)**:
   On builds configured with ESKF position estimation (`est_pos_gps` or `est_pos_mocap`), the state estimator must have converged and reported ready.
6. **Switch Hold Debounce (`arm_switch_hold_ms`)**:
   The arm switch must remain continuously in the arm position for a minimum hold duration (default `100 ms`) to prevent accidental arming from transient switch bounces.

If any gate fails when arming is attempted, the rejection reason is recorded in the static cell `LATEST_BLOCK_REASON` along with a timestamp. This allows the `health` shell command to explain why an arming attempt was rejected even after console output has scrolled past.

---

## Failsafe System & Transition Dynamics

The failsafe task (`crates/cybflight/src/control/failsafe.rs`) monitors two independent runtime failure conditions: radio control link dropouts and controller inner-loop heartbeats.

### RC Link Loss Failsafe (Two-Stage Model)

The RC failsafe implements a two-stage guard model:

* **Stage 1 (Guard Period)**: Triggered when no valid RC frame arrives within `fs_rxloss_trigger_s` (default `150 ms`). The task transitions to `Phase::GuardPeriod` and counts down the guard window (`fs_guard_period_s`, default `1.5 s`). If valid RC frames resume before the guard window expires, the system returns immediately to `Phase::Idle` without arming disruption.
* **Stage 2 (Commit & Disarm)**: If the guard period expires without RC recovery, the task calls `enter_failsafe(FailsafeReason::RcLoss)`. This immediately disarms the vehicle via `ARM_STATE`, flags `FAILSAFE_ACTIVE = true`, clears any active flight trajectory or mission planner state, and transitions to `Phase::Landed`.
* **Failsafe Recovery**: While in `Phase::Landed`, re-arming remains blocked by `FAILSAFE_ACTIVE`. Clearing the failsafe requires receiving continuous valid RC frames for `fs_recovery_period_s` (default `500 ms`). Once recovered, `FAILSAFE_ACTIVE` is set to `false` and `FAILSAFE_REASON` is reset to `None`.

### Controller Watchdog (Inner-Loop Liveness)

Independent of RC frame reception, the failsafe task verifies that the inner loop (e.g., INDI rate/attitude controller) publishes motor commands at its expected frequency.

* The controller stamps `LAST_CONTROLLER_PUBLISH` whenever motor commands are emitted.
* If the duration since `LAST_CONTROLLER_PUBLISH` exceeds `fs_ctrl_timeout_s` (default `500 ms`), `enter_failsafe(FailsafeReason::ControllerTimeout)` is invoked immediately.
* This catches critical internal controller failures, including ESKF divergence downstream effects, floating-point `NaN` propagation, stale sensor odometry, or frozen setpoints.

### State Transitions Diagram

<!-- openwiki: mermaid parse failed and this diagram was converted to a text fence so it does not break rendering. Fix the diagram source and restore the mermaid fence. Parser error: Heuristic: an unescaped angle bracket inside a label breaks rendering; rephrase the label. -->
```text
stateDiagram-v2
    [*] --> Disarmed : System Boot

    state Disarmed {
        [*] --> CheckingGates
        CheckingGates --> Blocked : Gate Failure (Throttle/RC/Gyro/ESKF/Failsafe)
        Blocked --> CheckingGates : Switch Re-evaluation
        CheckingGates --> ReadyToArm : All Gates Passed
    }

    ReadyToArm --> Armed : Arm Switch Flipped (Debounce Held)
    Armed --> Disarmed : Arm Switch Disarmed (Immediate)

    state Armed {
        [*] --> NormalFlight

        state NormalFlight {
            [*] --> LinkOk
            LinkOk --> RcLossGuard : First RC Timeout (150ms)
        }

        RcLossGuard --> LinkOk : RC Restored within Guard (1.5s)
        RcLossGuard --> FailsafeLanded : Guard Period Expired (RcLoss)
        NormalFlight --> FailsafeLanded : Controller Timeout (>500ms)
    }

    state FailsafeLanded {
        [*] --> DisarmingAndAbort
        DisarmingAndAbort --> WaitingForRecovery : FAILSAFE_ACTIVE = true
        WaitingForRecovery --> FailsafeCleared : Continuous Valid RC (500ms)
    }

    FailsafeCleared --> Disarmed : Re-arming Unblocked
```
*System arming state machine, failsafe guard transitions, and recovery flow.*

### Memory Order and Blackbox Synchronization

When entering failsafe, `enter_failsafe` writes `FAILSAFE_REASON` with `Ordering::Release` *before* storing `true` in `FAILSAFE_ACTIVE` with `Ordering::Release`. Any reader observing `FAILSAFE_ACTIVE == true` via `Ordering::Acquire` is guaranteed to observe the correct reason code.

The atomic `RC_LINK_HEALTHY` tracks frame liveness independently of `FAILSAFE_ACTIVE`. Transient dropouts that recover during Stage 1 toggle `RC_LINK_HEALTHY`, allowing the blackbox event logger to emit `KIND_RC_LOSS` and `KIND_RC_RECOVERED` events for link quality diagnostic analysis.

---

## Hardware Watchdog & Health Tracking

### Independent Watchdog (IWDG)

System-level hardware execution is supervised by the STM32 Independent Watchdog (`crates/cybflight/src/watchdog.rs`).

* **Hardware Configuration**: Driven by the internal low-speed oscillator (LSI ~32 kHz) with a prescaler of `/32` (1 kHz tick rate) and a reload register value of `500`, yielding a hardware timeout of **~500 ms**.
* **Supervisor Task**: The `iwdg_feed_task` runs on the lowest-priority thread executor, feeding the IWDG every `200 ms`. If the thread executor stalls or hangs for longer than 500 ms, the IWDG triggers an involuntary microcontroller reset. DShot pulses stop, and external Electronic Speed Controllers (ESCs) automatically disarm.
* **Timeout Extension**: Long blocking operations such as internal flash sector erases temporarily invoke `extend_timeout()`, changing the prescaler to `/256` (~4 s timeout). Upon completion, `restore_timeout()` returns the prescaler to `/32` (~500 ms) and feeds the watchdog immediately.

### System Health Snapshot (`SystemHealth`)

The `health` module (`crates/cybflight/src/health.rs`) aggregates all health and gate signals into a unified point-in-time struct: `SystemHealth`.

`SystemHealth` collects:
* Arming state (`armed`) and failsafe status (`failsafe_active`).
* Sensor calibration flags (`gyro_calibrated`).
* Live RC receiver link quality and throttle stick position.
* State estimator health status (`estimator_ready`, `estimator_degraded`, `estimator_faults`, `estimator_severe`, and phase summaries `AwaitingExteroceptive`, `Converging`, `Running`, `NotConfigured`).
* Attitude cross-check health bitfields and timestamp tracking for position/velocity updates.
* Latched arm rejection reason (`LATEST_BLOCK_REASON`).

This aggregated state is queried directly by the CLI `health` command.

---

## Postmortem Crash Diagnostics

Cybflight includes a non-volatile postmortem diagnostic subsystem that captures crash context across panic events, hardware faults, brownouts, and watchdog resets.

### Reset Cause Capture (`reset_cause.rs`)

Reset reason flags must be captured early during system boot before software or HAL initialization clears peripheral registers:

1. **Execution Point**: `reset_cause::capture()` runs inside `platform::pre_init()` prior to `bsp::init()`, clock configuration, or `.bss` zeroing.
2. **Register Sampling**: Reads MCU reset flags from `RCC.RSR` (or `RCC.CSR` depending on board target) and power brownout status from `PWR.CSR1.PVDO`.
3. **Storage**: Packs flags into `RAW_RESET_CAUSE` situated in the `.uninit` linker section. Because `.uninit` is omitted from initialization routines, startup runtime zeroing does not overwrite the stashed flags.
4. **Hardware Clear**: Writes `RMVF = true` to clear register flags, ensuring subsequent reboots observe a clean latch.

The captured cause is classified into `ResetKind` variants: `IndependentWatchdog`, `Brownout`, `Software`, `Pin`, `PowerOn`, `LowPower`, or `Multiple`.

### Non-Volatile Backup SRAM (BKPSRAM) Storage

The postmortem record is stored in STM32 D3-domain Backup SRAM (`crates/cybflight/src/postmortem/bkpsram.rs` and `record.rs`).

* **Memory Section**: Allocated in the `.bkpsram` linker section marked `(NOLOAD)`.
* **Retention Hardware**: `bkpsram::enable()` enables the BKPSRAM clock (`RCC.AHB4ENR.BKPRAMEN`), disables backup domain write protection (`PWR.CR1.DBP`), and enables the backup regulator (`PWR.CR2.BREN`) so data persists even when main supply power is removed (VBAT powered).
* **Record Struct Layout**: The `PostmortemRecord` struct is 2 KiB (`RECORD_SIZE = 2048` B) aligned to 8 bytes:
  * **Header (32 B)**: Magic marker (`MAGIC = 0xC0DE_0001`), CRC32 checksum over body, boot counter, reset cause bitfield, git commit hash, uptime milliseconds.
  * **Fatal Slot (64 B)**: Stores `FatalSummary` containing fault kind (`Panic`, `HardFault`, `IwdgReset`, `Brownout`), instruction pointer (`pc`), link register (`lr`), program status register (`psr`), and Fault Status Registers (`cfsr`, `hfsr`, `mmfar`, `bfar`).
  * **Event Ring (256 B)**: 16-entry ring (`EventEntry`) capturing timestamped system state transitions (`KIND_FAILSAFE`, `KIND_ESTIMATOR_UP/DOWN`, `KIND_RC_LOSS`, mission states).
  * **Snapshots (~256 B)**: Last attitude quaternion, position/velocity vectors, yaw angle, and raw RC channel PWM values.

### Atomic Commits & Word Access

BKPSRAM is located in MCU Device memory (`0x4002_4000` on STM32F7). Unaligned byte or half-word accesses to Device memory trigger CPU `UsageFault: UNALIGNED`.

To prevent memory corruption or torn reads:
* All reads and writes to `POSTMORTEM_SLOT` use 32-bit aligned volatile word transfers (`read_volatile` / `write_volatile`).
* **Magic-Last Protocol**: When updating the slot, word 0 (`magic`) is written to `0` first. The body payload and CRC32 are written next, followed by a Data Synchronization Barrier (`dsb`). `MAGIC` is written last to finalize the record. On read, `is_valid()` verifies both `MAGIC` and the CRC32 checksum before accepting record contents.

### Panic & Exception Fault Handlers (`fault.rs`)

When a fatal runtime error occurs, custom fault handlers intercept execution:

```
+-------------------------------------------------------------------------------+
|                       PANIC HANDLER / HARDFAULT ENTRY                         |
+-------------------------------------------------------------------------------+
                                       |
                                       v
                     [ Disable Interrupts (cpsid i) ]
                                       |
                                       v
                     [ Check FAULT_DEPTH Re-entrancy ]
                       /                           \
               Depth >= 1                        Depth == 0
                 /                                   \
   [ Abort & Reset / UDF ]              [ Format Panic Text to BKPSRAM ]
                                                     |
                                                     v
                                        [ Populate FatalSummary ]
                                                     |
                                                     v
                                        [ Commit BKPSRAM Record ]
                                                     |
                                                     v
                                        [ Trigger SCB System Reset ]
```

1. **Re-Entrancy Protection**: Intercepts `#[panic_handler]` and `HardFault` exception vectors. Increments an atomic `FAULT_DEPTH` counter. If `FAULT_DEPTH >= 1` (a fault occurred inside the fault handler itself), execution immediately bails out via `cortex_m::asm::udf()` or `SCB::sys_reset()` to prevent infinite crash loops.
2. **Panic Formatting**: The panic handler formats `file:line:col: message` into `POSTMORTEM_PANIC_TEXT` using stack buffers without dynamic allocation or locks.
3. **Register Extraction**: HardFault handlers extract stacked CPU context (`PC`, `LR`, `xPSR`) and SCB status registers (`CFSR`, `HFSR`, `MMFAR`, `BFAR`).
4. **System Reset**: Writes `FatalSummary` to BKPSRAM and invokes `SCB::sys_reset()`.

### Steady-State Background Task (`task.rs`)

The `postmortem_task` runs at a `100 ms` cadence during normal flight. It edge-detects changes in health atomics (`FAILSAFE_ACTIVE`, `ESTIMATOR_READY`, `RC_LINK_HEALTHY`), appends event records to the BKPSRAM event ring, updates attitude and position snapshots, and periodically flushes uptime milliseconds to BKPSRAM.

### Boot Recovery & Early-Crash USB Safe Mode (`recovery.rs`)

During startup, `postmortem::boot_recovery()` evaluates the BKPSRAM record from the prior boot session:

1. **Validation**: Checks `is_valid()` on the previous record.
2. **Snapshot Preservation**: Copies the prior record into `PRIOR_RECORD` for blackbox logging and `PRIOR_SNAPSHOT` for CLI inspection (`postmortem show`).
3. **Early-Crash Safe Mode Detection**: If the prior session suffered a fatal crash (`Panic`, `HardFault`, or `IndependentWatchdog`) within `10,000 ms` (`10 s`) of booting, `boot_recovery()` sets `SAFE_MODE_ACTIVE = true`.
4. **Safe Mode Execution**: When `in_safe_mode()` is active, `main.rs` bypasses full flight loop initialization and enters a minimal USB Safe Mode shell. This keeps USB CDC serial communication operational, allowing engineers to inspect crash dumps via `postmortem show` without triggering an infinite crash/reset loop.

---

## Technical Specifications & Parameters

### Safety & Failsafe Parameters

Failsafe parameters are defined in the `safety` parameter group (`crates/cybflight/src/params.rs`):

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `fs_rxloss_trigger_s` | `f32` | `0.150` (150 ms) | Elapsed duration without RC frames before Stage 1 failsafe triggers. |
| `fs_guard_period_s` | `f32` | `1.500` (1.5 s) | Guard duration before Stage 2 failsafe disarms the vehicle. |
| `fs_recovery_period_s` | `f32` | `0.500` (500 ms) | Required duration of valid RC frames to clear failsafe state. |
| `fs_ctrl_timeout_s` | `f32` | `0.500` (500 ms) | Inner-loop controller heartbeat timeout before emergency disarm. |

### Shell Diagnostic Commands

The firmware provides dedicated console diagnostic utilities for verifying safety state:

```bash
# Query live system safety, arming gate status, link quality, and estimator readiness
health

# Display the postmortem record and panic log from the prior boot session
postmortem show

# Clear latched postmortem snapshot state
postmortem clear
```

---

## Related Documentation

* [Architecture Overview](../architecture/overview.md) — High-level firmware task architecture, concurrency model, and system execution layout.
* [State Estimation ESKF](state-estimation-eskf.md) — Navigation filter pipeline, estimator readiness flags, and failure detection.
* [Hardware Bringup](../workflows/hardware-bringup.md) — Step-by-step physical bringup, motor testing, and safety checks.
