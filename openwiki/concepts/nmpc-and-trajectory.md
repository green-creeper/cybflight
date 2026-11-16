---
type: concept
title: NMPC & Trajectory Generation
description: Architectural overview of Cybflight outer-loop position control, NMPC/SQP/TinyMPC solvers, MINCO trajectory generation, learned cost adaptation, and mission execution.
tags: [nmpc, mpc, trajectory-planning, minco, tinympc, cost-adapt, cybflight]
verified:
  - by: openwiki/0.7.2
    at: 2026-10-30T14:16:13.882Z
sources:
  - id: openwiki-source-c95a59219210d72d608840cd
    resource: repo://crates/cybflight_core/src/mpc/cost_adapt.rs
  - id: openwiki-source-f59a33ee4f367ddd25ed8f8d
    resource: repo://crates/cybflight_core/src/mpc/sqp_solver.rs
  - id: openwiki-source-6e8572c0325680485c0b71d0
    resource: repo://crates/cybflight_core/src/mpc/tinympc.rs
  - id: openwiki-source-08e349c992bdfbb99166772e
    resource: repo://crates/cybflight_core/src/trajectory_planning/minco_snap.rs
  - id: openwiki-source-0b55213211ea45c026e77ae0
    resource: repo://crates/cybflight_core/src/trajectory_planning/planner.rs
  - id: openwiki-source-b707ae24c8e324e057099068
    resource: repo://crates/cybflight/src/control/mission_planner.rs
generated: { by: "openwiki/0.7.2", at: "2026-10-30T14:16:13.882Z" }
---

# NMPC & Trajectory Generation

The Cybflight position control framework couples high-grade trajectory optimization with high-frequency Model Predictive Control (MPC) tracking. Trajectories are parameterized using **MINCO** (Minimum Control Effort) piecewise polynomials and solved either offline via closed-form banded linear system solves or online using on-device trust-region BFGS optimization. Outer-loop position tracking is driven by a real-time **Sequential Quadratic Programming (SQP) NMPC** solver—or alternatively **TinyMPC**, an ADMM-based linear MPC solver—operating at 50 Hz. To adapt to dynamic conditions and unmodeled aerodynamic disturbances, an online **learned cost adaptation policy** modulates the NMPC stage and terminal cost weights in real time.

---

## Architecture Overview

The system bridges high-level mission directives with low-level attitude and rate control loops described in [Control & Actuation](./control-and-actuation.md). The mission architecture operates across three distinct operational states: `Idle`, `Planning`, and `Executing`.

```mermaid
flowchart TD
    subgraph Mission Execution
        RC["RC Trigger / Mission Request"] --> MP["Mission Planner Task"]
        MP -->|Online BFGS / Offline Snap| MINCO["MINCO Trajectory Solver"]
        MINCO -->|Publish under Slot Lock| SLOT["MISSION_TRAJECTORY_SLOT"]
    end

    subgraph Outer Loop Execution (50 Hz)
        SLOT -->|Sample at tau0| SAMPLER["TimeSampler / SamplerInputs"]
        SAMPLER --> FLAT["Differential Flatness Mapping"]
        FLAT -->|x_ref, u_ref over N=20| NMPC["Tracking NMPC Solver (SQP / TinyMPC)"]
        OBS["Situation Observation (59-dim)"] --> POLICY["Learned Cost Policy (cost_adapt)"]
        POLICY -->|Modulate Weights w| NMPC
        STATE["ESKF State (x0)"] --> NMPC
        NMPC -->|Optimal u0: Thrust & Rates| RATE["Inner Loop Rate Controller"]
    end
```
*Figure 1: Data flow mapping trajectory planning to NMPC setpoint tracking and setpoint generation.*

---

## Mission Execution & Outer-Loop Setpoint Pipeline

### Operational State Machine

Mission management uses three core states defined in `MissionState`:
1. **`Idle`**: Hovering on stick inputs or holding active position setpoint. `rc_interpreter` refreshes `ACTIVE_POSITION_SETPOINT` every frame.
2. **`Planning`**: Triggered by a rising edge on the mission AUX channel (`PLAN_REQUEST`). Pilot stick inputs are gated, and the planner computes or evaluates the mission trajectory.
3. **`Executing`**: The published trajectory is actively sampled by the 50 Hz outer loop ticker. At mission completion ($\tau_0 \ge T_{\text{total}}$), the state transitions smoothly back to `Idle` while retaining the trajectory's tail position and yaw setpoint.

### Start Pose Handoff Semantics

To guarantee smooth transition from hover to mission tracking without position or velocity steps, the mission planner snapshots `ACTIVE_POSITION_SETPOINT` via `try_snapshot_setpoint()` (`crates/cybflight/src/control/mission_planner.rs#L280-L299`). The request is validated against strict safety criteria:
- **Finiteness**: Position and yaw values must be finite.
- **Liveness / Freshness**: Setpoint timestamp must not be future-dated and cannot be older than `SETPOINT_STALE_TIMEOUT` (50 ms). Stale setpoints indicate an inactive controller chain or unready estimator.
- **Environment Compatibility**: The mission profile's environment (`indoor` vs. `outdoor`) must match the build's position estimator source (`est_pos_mocap` vs. `est_pos_gps`).

By using `ACTIVE_POSITION_SETPOINT` as the trajectory's head pose, the first sample of the trajectory ($\tau = 0$) matches the last `Idle` setpoint identically.

### Online vs. Offline Planning Schemas

The firmware supports two mutually exclusive planning schemas selected at compile time via cargo features:

| Feature / Schema | Solver Mechanism | Execution Model | Resource / Safety Profile |
| :--- | :--- | :--- | :--- |
| **Offline** (`plan_offline`, default) | Reads precomputed waypoints & timestamps from YAML profiles (`offline_mission`). Solves $8N \times 8N$ MINCO Snap banded LU system. | Synchronous single-digit millisecond solve. No cooperative yielding required. | BSS-resident solver static (`OFFLINE_MINCO`). Sized to `OFFLINE_MAX_PIECES`. Fast & deterministic. |
| **Online** (`plan_online`) | On-device trust-region BFGS optimizer (`bfgs_trust`). Reparameterized times & stereographic ball waypoints. | Asynchronous cooperative execution in bursts (`BFGS_ITERS_PER_YIELD = 1`) with `INTER_BURST_DELAY` (1 ms). | Workspace in `BfgsWorkspace` static cell (~55 KiB). Constrained by `SOLVE_BUDGET` (5000 ms) and emergency disarm interlock (`IS_ARMED`). |

Under `plan_online`, yielding every outer iteration guarantees that peer async Embassy tasks (outer MPC loop, ESKF, sensor drivers, watchdog feed) run without CPU starvation (`crates/cybflight/src/control/mission_planner.rs#L420-L468`).

### Critical Section Slot Synchronization & Failsafe Handling

Trajectory publication uses `MISSION_TRAJECTORY_SLOT` guarded by a critical section raw mutex (`crates/cybflight/src/control/mission_planner.rs#L955-L985`):
- Both the slot write (`Some(MissionTrajectory)`) and the transition `MISSION_STATE -> Executing` occur within a single `MISSION_TRAJECTORY_SLOT.lock` call.
- Re-checks of `MISSION_ABORT_REQUESTED` and `MISSION_STATE == Planning` take place inside the lock before publication.
- On pilot abort or disarm failsafe, `failsafe.rs` or `outer_loop.rs` takes the same slot lock, clears the slot to `None`, resets state to `Idle`, and captures the trajectory sample at the current timestamp $\tau$ as the new hover setpoint.

---

## Trajectory Planning & MINCO Formulation

### MINCO Polynomial Representation

 trajectories are parameterized using **MINCO** (Minimum Control Effort) space, representing an $N$-segment 3D polynomial trajectory with time allocation $T = [T_1, \dots, T_N] \in \mathbb{R}^N_{>0}$ and intermediate junction points $q = [q_1, \dots, q_{N-1}] \in \mathbb{R}^{3 \times (N-1)}$.

Cybflight provides three distinct MINCO solver variants (`crates/cybflight_core/src/trajectory_planning/`):
1. **MINCO Snap (`MincoSnapN`, $s=4$, Degree 7)**:
   - Sized for $8N \times 8N$ linear systems with half-bandwidth 8 (`minco_snap.rs#L53-L126`).
   - Boundary conditions: Position, Velocity, Acceleration, Jerk (PVAJ) at head and tail.
   - Junction conditions: Enforces position match at intermediate waypoints and continuity up to derivative 6 (velocity, acceleration, jerk, snap, crackle, pop).
   - Minimizes $\int \|\text{snap}(t)\|^2 dt$. Used for smooth offline schedule generation.
2. **MINCO Jerk (`MincoJerkN`, $s=3$, Degree 5)**:
   - Sized for $6N \times 6N$ linear systems with half-bandwidth 6 (`minco_jerk.rs`).
   - Boundary conditions: PVA. Minimizes $\int \|\text{jerk}(t)\|^2 dt$. Used by the online BFGS planner.
3. **MINCO Acc (`MincoAccN`, $s=2$, Degree 3)**:
   - Sized for $4N \times 4N$ linear systems (`minco_acc.rs`).
   - Boundary conditions: PV. Minimizes $\int \|\text{accel}(t)\|^2 dt$. Used for 1D yaw spline solves in schedule-driven missions.

### Unconstrained Reparameterization & Stereographic Chart

To enable unconstrained optimization using BFGS, the decision variables are reparameterized (`crates/cybflight_core/src/trajectory_planning/planner.rs#L280-L360`):
- **Segment Times**: Mapped via smooth exponential map $T_i(x_i) = \text{forward\_t}(x_i)$.
- **Waypoint Positions**: Constrained to a ball of radius $r$ (default `DEFAULT_WAYPOINT_RADIUS` = 0.01 m, clamped to $\ge 10^{-4}$ m) around nominal target $\hat{P}_i$ via stereographic projection:
  $$P_i = \hat{P}_i + \frac{2 r D_i}{\|D_i\|^2 + 1}$$
  This map bounds $\|P_i - \hat{P}_i\| \le r$ for all $D_i \in \mathbb{R}^3$, eliminating bound constraints while providing well-conditioned gradients.

### Penalty Functions & Objective Function

The online planner minimizes a composite cost function evaluated by `CostEvaluator` (`crates/cybflight_core/src/trajectory_planning/cost_eval.rs`):
$$J = w_T \sum_{i=1}^N T_i + w_E \int_0^{T_{\text{tot}}} \|\text{jerk}(t)\|^2 dt + \sum \int_0^{T_{\text{tot}}} P_{\text{constraint}}(t) dt$$

Constraints are enforced via smooth $C^2$ penalty functions integrated along trajectory segments:
- **Velocity Limit**: $P_v(v) = \max(0, \|v\|^2 - v_{\text{max}}^2)^3$
- **Collective Thrust Limits**: $P_T(a) = \max(0, T_{\text{min}} - T(a))^3 + \max(0, T(a) - T_{\text{max}})^3$
- **Tilt Angle Limit**: $P_{\theta}(a) = \max(0, \cos \theta_{\text{max}} - \cos \theta(a))^3$
- **Body Rate Limits**: $P_{\omega}(a, j) = \max(0, \|\omega(a, j)\|^2 - \omega_{\text{max}}^2)^3$

### Yaw Guidance Modes

The yaw setpoint trajectory during mission execution is determined by `MissionYawMode` (`crates/cybflight/src/control/mod.rs`):
1. **`Constant(yaw)`**: Holds entry yaw angle fixed throughout the mission.
2. **`Lookahead { dt_s, max_rate_rad_s }`**: Dynamically aligns heading with the reference velocity vector $v_{\text{ref}}(t + \Delta t)$ evaluated $\Delta t$ seconds ahead, rate-limited by `max_rate_rad_s`.
3. **`Schedule(yaw_traj)`**: Evaluates a dedicated 1D `PiecewisePolynomial` generated from offline heading waypoints using $S^1$-unwrapping (`unwrap_nearest`) and `MincoAccN` spline solving.

---

## Tracking NMPC & Numerical Solvers

### Nonlinear Model & Differential Flatness

The NMPC tracks the sampled reference trajectory $[p_{\text{ref}}, q_{\text{ref}}, v_{\text{ref}}]_{0..N}$ over a prediction horizon $N=20$ (with step $dt = 0.05\text{ s} \implies 1\text{ s}$ horizon). 

System states and controls are defined by `QuadModel` (`crates/cybflight_core/src/mpc/quad_model.rs`):
- **State** ($x \in \mathbb{R}^{10}$): $[p_x, p_y, p_z, q_x, q_y, q_z, q_w, v_x, v_y, v_z]$ (position, unit quaternion, velocity).
- **Control** ($u \in \mathbb{R}^4$): $[T_{\text{coll}}, \omega_x, \omega_y, \omega_z]$ (collective thrust in Newtons, body angular rates in rad/s).

Differential flatness maps reference acceleration $a_{\text{ref}}$ and jerk $j_{\text{ref}}$ to collective thrust magnitude and attitude quaternion $q_{\text{ref}}$, establishing the nominal feedforward reference $u_{\text{ref}}$.

### Multiple-Shooting SQP Solver

Tracking optimization uses real-time Sequential Quadratic Programming (`SqpSolver`, `crates/cybflight_core/src/mpc/sqp_solver.rs`). Each 50 Hz control tick runs **one SQP iteration** warm-started from the previous tick's solution ($u_{\text{bar}}$):

1. **Linearization & Quadraticization**:
   Linearizes quadrotor dynamics $x_{k+1} = f(x_k, u_k)$ and quadraticizes stage cost $l_k(x_k, u_k)$ and terminal cost $l_N(x_N)$ around current iterate trajectories $(\bar{x}, \bar{u})$.
2. **Structured Riccati Backward Sweep**:
   Computes feedback gains $K_k \in \mathbb{R}^{4 \times 10}$ and feedforward steps $k_k \in \mathbb{R}^4$ by sweeping backwards from stage $N$ to $0$.
   - **Sparse-B Exploitation**: Exploits the property that input Jacobian $\frac{\partial f}{\partial u}$ has nonzeros only in rows `BNZ_START` to `BNZ_START + BNZ_LEN` (rows 3..9 for `QuadModel`).
   - **Unrolled $4 \times 4$ Cholesky Inversion**: Inverts $H_{uu} \in \mathbb{R}^{4 \times 4}$ using `cholesky_inv_4x4` (`sqp_solver.rs#L55-L157`), featuring diagonal regularization ($10^{-4}, 10^{-3}, 10^{-2}$) and scaled identity fallbacks.
   - **Sparse-A Matrix Products**: Exploits zero structure in $\frac{\partial f}{\partial x}$ (`JAC_X_NZ_COL_START`) to skip identity multiplication loops.
3. **Forward Pass & Step Acceptance**:
   Updates state trajectory via nonlinear model propagation $x_{k+1} = f(x_k, u_k)$ and updates control iterate $u_k \leftarrow \text{clamp}(u_k + \Delta u_k)$.

### Divergence Detection & Sanity Interlocks

To prevent numerical corruption (e.g., from NaN localized states or corrupt reference inputs), the backward sweep validates finiteness (`sqp_solver.rs#L214-L420`):
- If terminal cost $P_N$, matrix $H_{uu}$, feedback gain $K_k$, or cost-to-go $P_k$ contains a non-finite value, `backward_sweep` bails immediately and returns `diverged: true`.
- The forward step is rejected, leaving $u_{\text{bar}}$ at the last valid iterate (the warm start) without propagating NaNs to motor outputs.

### TinyMPC: ADMM Linear MPC

As an alternative to SQP NMPC, `TinyMpc` (`crates/cybflight_core/src/mpc/tinympc.rs`) provides a linear MPC solver based on ADMM:
- **Model**: Linearized hover dynamics (`hover_linear_model`, 9 states $[p, \theta, v]$, 4 inputs $[\delta c, \omega]$) discretized with exact zero-order-hold matrix exponentials (`tinympc.rs#L417-L451`).
- **Offline Cache (`TinyCache`)**: Precomputes discrete Riccati solution $K_\infty, P_\infty$ and ADMM gain matrices $C_1 = (\tilde{R} + B^T P_\infty B)^{-1}$, $C_2 = (A - B K_\infty)^T$ on regularized matrices $\tilde{Q} = Q + \rho I, \tilde{R} = R + \rho I$.
- **Online Execution**: Alternates fast matrix-vector backward/forward passes with box constraint slack/dual projections ($z = \text{clip}(u + y)$, $v = \text{clip}(x + g)$).

---

## Learned Cost Adaptation

To adapt tracking performance dynamically during high-speed maneuvers or unmodeled wind, Cybflight implements situation-conditioned learned cost adaptation (`crates/cybflight_core/src/mpc/cost_adapt.rs`).

### Observation Encoding (59 Dimensions)

At each control tick, `situation_obs()` assembles a 59-dimensional observation vector $o \in \mathbb{R}^{59}$ (`cost_adapt.rs#L280-L361`). All error and velocity terms are transformed into the reference **path frame** $\{ \hat{t} = v_{\text{ref}}/\|v_{\text{ref}}\|, \hat{n}, \hat{b} \}$ at node 0, rendering the observation **yaw-invariant**:

| Indices | Description | Scaling / Clamping |
| :--- | :--- | :--- |
| `0..3` | Position error $[e_t, e_n, e_b]$ in path frame | Clamped to $\pm 3\text{ m}$ (training crash bound) |
| `3..6` | Velocity error $[\Delta v_t, \Delta v_n, \Delta v_b]$ in path frame | Scaled by `V_ERR_SCALE` = 15.0 m/s, clamped to $\pm 3$ |
| `6..9` | Actual body $z$-axis vector in path frame | Dimensionless $[-1, 1]$ |
| `9..12` | Reference body $z$-axis vector (node 0) in path frame | Dimensionless $[-1, 1]$ |
| `12..15` | Attitude error rotation vector $\text{Log}(q_{\text{ref}}^{-1} q)$ | Clamped to $\pm 3\text{ rad}$ |
| `15..18` | Measured body angular rates $[\omega_x, \omega_y, \omega_z]$ | Scaled by `RATE_SCALE` = 10.0 rad/s |
| `18..21` | Velocity vector in path frame | Scaled by `V_SCALE` = 60.0 m/s |
| `21` | Reference speed $\|v_{\text{ref}}\|$ | Scaled by `V_SCALE` = 60.0 m/s |
| `22..57` | Horizon maneuver preview at nodes $\{0, 5, 10, 15, 20\}$ | 7 features/node ($\cos \theta_{\text{ref}}$, body $z$, rate frac, thrust/rate margins) |
| `57..59` | Commanded thrust fraction & max rate fraction | Normalised $[0, 1.5]$ |

### Weight Modulation Policy (19 Output Dimensions)

A small MLP policy network (`CostPolicy`, `cost_adapt.rs#L367-L393`) evaluates $o$ and outputs a 19-dimensional residual vector $z \in [-1, 1]^{19}$:
- **Stage Weights** ($z_{0..7}$): $[ \text{contour}, \text{lag}, v_x, v_y, v_z, \text{att}_x, \text{att}_y, \text{att}_z ]$
- **Terminal Weights** ($z_{8..15}$): $[ \text{contour}, \text{lag}, v_x, v_y, v_z, \text{att}_x, \text{att}_y, \text{att}_z ]$
- **Rate Input Weights** ($z_{16..18}$): $[ w_{\omega_x}, w_{\omega_y}, w_{\omega_z} ]$

State weights $w_{\text{nom}}$ are modulated using piecewise log maps (`CostNominal::apply`, `cost_adapt.rs#L166-L218`):
$$w = \begin{cases} w_{\text{nom}} \cdot \text{UP}^z & z \ge 0 \\ w_{\text{nom}} \cdot \text{DOWN}^{-z} & z < 0 \end{cases}$$

### Safety Boundaries & Version Interlock

To prevent policy instability, per-channel fences bound weight variations:
- **Contour Weight**: Range $[100, 2000]$ ($\text{UP}=4.0, \text{DOWN}=0.2$).
- **Lag Weight**: Range $[50, 500]$ ($\text{UP}=2.5, \text{DOWN}=0.25$).
- **Yaw Attitude Weight**: Range $[50, 200]$ ($\text{UP}=1.0, \text{DOWN}=0.25$) — yaw weight can only be *softened*.
- **Body Rate Weights**: $\text{UP}=4.0, \text{RATE\_FLOOR\_FRAC}=1.0$ — rate input weights can only *increase* from nominal to damp oscillation.
- **Thrust Input Weight**: Unmodified (remains at nominal tune).

**Version Interlock**: The parameter map exports `COST_MAP_VERSION` = 20 (`cost_adapt.rs#L74`). The firmware asserts this version during policy loading, rejecting incompatible neural network checkpoints at compile/bake time.

---

## Operations & Configuration Reference

### Key Configuration Parameters

Planner and controller parameters are governed by `PlannerParams` (`crates/cybflight_core/src/params.rs`):

| Parameter | Type / Unit | Default | Operational Impact |
| :--- | :--- | :--- | :--- |
| `duration_min_s` | `f32` [s] | 0.5 s | Floor for solved trajectory duration; rejects collapsed solves. |
| `duration_max_s` | `f32` [s] | 300.0 s | Ceiling for solved trajectory duration; rejects uncompressed solves. |
| `max_vel_m_s` | `f32` [m/s] | 12.0 m/s | Kinematic velocity penalty knee in online MINCO planning. |
| `max_thrust_n` | `f32` [N] | 34.0 N | Upper collective thrust penalty bound. |
| `min_thrust_n` | `f32` [N] | 2.0 N | Lower collective thrust penalty bound (prevents total thrust loss). |
| `max_tilt_deg` | `f32` [deg] | 45.0 deg | Maximum tilt angle constraint penalty knee. |

### Diagnostic & Status Telemetry

Mission execution emits `msgs::MissionStatus` breadcrumbs on `MISSION_STATUS` at 10 Hz during planning and execution:
- `state`: Current `MissionState` (`Idle`, `Planning`, `Executing`).
- `tau_s`: Active trajectory elapsed time $\tau_0$.
- `total_duration_s`: Total duration $T_{\text{tot}}$ of the active trajectory.
- `solve`: `SolveDiagnostics` detailing solve time (ms), iterations, final cost, peak trajectory velocity, and reject reason flags (`SOLVE_REJECT_NONE`, `SOLVE_REJECT_BUDGET_EXCEEDED`, `SOLVE_REJECT_DISARMED`, `SOLVE_REJECT_UNDER_COMPRESSED`, etc.).

---

## Related Documentation

- [Control & Actuation](./control-and-actuation.md) — Details the lower-level rate controller, INDI, and motor actuation stack.
- [Offboard & RL Deployment](../workflows/offboard-and-rl-deployment.md) — Describes offboard trajectory streaming and policy deployment pipelines.
