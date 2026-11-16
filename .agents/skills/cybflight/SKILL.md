---
name: cybflight
description: >-
  Complete engineering guide and runbook for the Cybflight Rust flight stack
  (cybflight firmware, cybflight-core, cybflight-drivers, cybflight-gs,
  cybflight-sim, cybflight-rl, vehicle-yaml, and BSP crates). Use whenever
  programming, building, DFU flashing, connecting via USB CDC CLI (fc_shell.py),
  debugging boot/IWDG/RC/CPU/Safe Mode issues, tuning vehicle YAML or runtime
  parameters, running SITL/Rerun sims, launching cybflight-gs, or adding boards
  and parameters.
---

# Cybflight Engineering & Operations Skill

Repository root: `/Users/andrii/Personal/autopilots/cybflight`  
Wire protocol crate: `/Users/andrii/Personal/autopilots/cybflight-msgs`

Deep manuals in this skill (load on demand):
- [CLI & Hardware Debugging Guide](./references/cli-and-debugging.md) — `tools/fc_shell.py`, every USB CDC shell command, `rcdiag`/`postmortem`/`health`/`indistat`, boot stages `1..9`, Safe Mode recovery, and diagnostic playbooks.
- [Pitfalls & Lessons (What Worked vs. What Didn't)](./references/pitfalls-and-lessons.md) — STM32F745 liftoff fix, 2 kHz CPU/IWDG traps, F7 vs. H7 hardware quirks, ELRS MAVLink2 passive lock, DFU/Cargo traps, and `cybflight-gs` fixes.
- **Blackbox Forensics Skill**: Activate `cybflight-blackbox` ([`../cybflight-blackbox/SKILL.md`](../cybflight-blackbox/SKILL.md)) for pulling `.mcap` logs, automated anomaly triage (`scripts/blackbox_forensics.py`), and safely erasing FC flash logs.

---

## 1. Golden Workspace Rules

1. **Always run `just` commands from `/Users/andrii/Personal/autopilots/cybflight`** and run **all Cargo/`just` builds sequentially** — they share `target/` locks and overwrite a single `target/thumbv7em-none-eabihf/release/cybflight.bin`.
2. **Never add BSP crates or `cybflight-gs` to root `Cargo.toml` `default-members`** — BSPs unify `stm32-metapac` chip features and break cross-MCU builds.
3. **Single USB serial owner** — Close `cybflight-gs` (or any serial terminal) before running `tools/fc_shell.py`, `just param-sync`, or `just blackbox-pull` on `/dev/cu.usbmodem*`.
4. **Always assert `DTR | RTS` on USB CDC** — Use `tools/fc_shell.py` rather than bare `cat`/`echo` on `/dev/tty.usbmodem*` so `embassy-usb` wakes and macOS does not block on DCD.
5. **Supported RC protocols**: **CRSF** (`420_000` baud) and **ELRS MAVLink2** (`460_800` baud) via runtime UART auto-scan (`multi_rc_task`). ImmersionRC Ghost (`ghst`) is permanently removed.

---

## 2. Crate Topology & STM32F745 Priority Layout

### Crate Map
| Crate | Target | Responsibility |
|---|---|---|
| `crates/cybflight` | `thumbv7em-none-eabihf` | Firmware: `board_init/`, sensor tasks, DShot, USB CDC shell, blackbox MCAP (`FatFs`/SDMMC & `NorBlackboxStore` SPI-NOR), offboard UART bridge, INDI/outer-loop tasks, `postmortem` |
| `crates/cybflight_core` | Host & `no_std` | Pure math/control (no `static` state): INDI + WLS allocator, SQP-RTI NMPC, TinyMPC, geometric cascade, ESKF, Mahony, `RpmNotchBank`, MINCO, `FirmwareConfig` params |
| `crates/bsp/types` | `no_std` | Shared hardware types (`SensorAlign`, `MotorMeta`, `SerialPortId`) |
| `crates/bsp/{sakurah743,foxeerh743,micoair743v2,flywoof745}` | `no_std` | Board pinouts, clocks, DMA channels, capability flags (only depends on `embassy-stm32`, `bsp-types`, `cybflight-drivers`) |
| `crates/drivers` | Host & `no_std` | Async drivers (`Icm426xx`, `Mpu6x00`, `Bmi270`, `Bmi088`, `Dps310`, `Spl06`, `Icp20100`, `Qmc5883l`, `Ist8310`, `Ublox`, `Unicore`, `Crsf`, `MavlinkRcParser`, `W25q128`) |
| `crates/cybflight_gs` | Host (`egui`) | Cross-platform Ground Station & Configurator (3D Attitude, Receiver & Modes, Parameters, DFU Flasher, CLI Shell) |
| `crates/vehicle_yaml` | Host | Compile-time `vehicles/*.yaml` & `missions/*.yaml` parser/validator + `vehicle-features` CLI |
| `crates/cybflight_sim` | Host | 6-DOF closed-loop SITL simulator (`sim-run`), Rerun `viz`, golden regression (`sim-check`) |
| `crates/cybflight_rl` | Host (`cdylib`) | PyO3 + Rayon `VecEnv` (`RecoveryEnv`) for RL / adaptive-MPC training |

### STM32F745 (`flywoof745` / `goku_f745`) NVIC & Executor Hierarchy
- **`PVD` (`Priority::P0`)**: Brownout early-warning pre-mortem save.
- **`DMA` (`Priority::P4`)**: SPI/UART/DShot DMA completion interrupts.
- **`USART1..6`, `I2C1_EV`, `I2C1_ER` (`Priority::P5`)**: Must sit above `CAN2_TX`/`CAN2_RX1`/`CAN2_RX0` on STM32F7 (single-byte UART `RDR`, no FIFO — lower priority causes `ORE` byte drops at 420k–460.8k baud).
- **`CAN2_TX` (`Priority::P6`)**: `EXECUTOR_HIGH` (DShot600 motor output / bi-directional eRPM).
- **`EXTI1` (`Priority::P7`)**: IMU data-ready EXTI.
- **`CAN2_RX1` (`Priority::P8`)**: `EXECUTOR_IMU` (8 kHz SPI IMU acquisition).
- **`CAN2_RX0` (`Priority::P10`)**: `EXECUTOR_CTRL` (2 kHz `indi_task`).
- **`OTG_FS` (`Priority::P12`)**: USB OTG FS interrupt.
- **Thread Executor (lowest)**: `iwdg_feed_task`, `postmortem_task`, `usb_serial`, `multi_rc_task`, `gps_autobaud_task`, I2C baro/mag, ESKF/Mahony, `power_task`, `blackbox_task`.

---

## 3. Building, Testing, Ground Station & Flashing

### Build & Host Test Commands
```bash
# Inspect vehicles & resolved Cargo features
just vehicles
just print-features goku_f745

# Build firmware for a specific vehicle + inspect Flash/RAM/stack headroom
just build goku_f745
just build-mpc goku_f745       # temporary outer_loop=mpc override
just build-cascade goku_f745   # temporary outer_loop=cascade override
just size goku_f745

# Host verification suites (run sequentially; check for zero rustc warnings)
just check-all                 # compile-check all supported firmware feature combinations
just test-build-tools          # vehicle-yaml & vehicle-features
just test-drivers              # cybflight-drivers
just test                      # cybflight-core unit & convergence tests
just sim-check                 # golden controller/plant regression snapshot
just test-all                  # test + test-build-tools + test-drivers + sim-check

# Ground Station GUI & host checks
HOST=$(rustc -vV | sed -n 's/^host: //p')
cargo clippy -p cybflight-gs --target "$HOST" --no-deps --tests -- -D warnings
cargo test -p cybflight-gs --target "$HOST"
just gs goku_f745
```

### Flashing Workflow (`dfu-util`, VID:PID `0483:df11`)
Never run `dfu-util` directly against a stale binary — `just flash <vehicle>` always rebuilds `<vehicle>` first and handles macOS libusb's `:leave` hang in `dfuMANIFEST`.

1. **Enter DFU mode** (choose one):
   - Software reboot from USB shell: `python3 tools/fc_shell.py --dfu`
   - Hardware recovery (if USB shell doesn't enumerate): hold the physical **BOOT** button while plugging in USB.
   - Confirm DFU presence: `dfu-util -l | grep 0483:df11`
2. **Build, flash, and boot**:
   ```bash
   just flash goku_f745
   ```
   Retries up to 3x on `SET_ADDRESS` glitches and waits up to 10 s for `/dev/tty.usbmodem*` or `/dev/serial/by-id/*cybflight*` to re-enumerate.
3. **Byte-verify suspect flash writes**:
   ```bash
   just flash-verify goku_f745
   # Leaves board in DFU on success — follow with `just flash goku_f745` or power-cycle.
   ```

---

## 4. Quick CLI Verification After Flashing

Run with the FC plugged into USB (after closing `cybflight-gs`):
```bash
python3 tools/fc_shell.py --wait 5 --banner \
  "version" "resetcause" "postmortem show" "param diff" "health" "rcdiag" "indistat"
```
Healthy post-flash baseline on `goku_f745` (USB power, TX off):
- `version`: shows expected git hash, build timestamp, `[FLYWOOF745] vehicle=goku_f745` (no `[SAFE MODE ...]` tag).
- `postmortem show`: shows advancing `uptime=...ms` (normal runtime, not stuck in boot stages `1..9`).
- `param diff`: `(no overrides - matches baked defaults)` unless intentional runtime overrides exist.
- `health`: reports expected gates (`BLOCKED — first failing gate: Failsafe` when TX is off; `mahony : ready=yes`).
- `rcdiag`: `state=LOCKED, crc_err=0` once ELRS RX frames arrive; `rc_frames=0` when handset TX is off.
- `indistat` (run twice, ~0.3–1.0 s apart): iteration count advances at ~2 kHz with `step avg` well under the `500 µs` period.

For full CLI usage and troubleshooting playbooks, read [`./references/cli-and-debugging.md`](./references/cli-and-debugging.md).

---

## 5. Configuration, Control Modes & Simulation

### Vehicle YAML (`vehicles/<name>.yaml`) & Parameter Precedence
Precedence (lowest → highest): **Rust Schema Defaults (`crates/cybflight_core/src/params.rs`)** → **Vehicle YAML Bake (`build.rs`)** → **KV Flash Overrides (Sectors 6+7)**.
- Changing `vehicles/<name>.yaml` **will not** override existing keys saved in FC flash until you run in USB CLI:
  ```text
  param reset <key|all>
  param save --prune
  ```
- Syncing bench-tuned parameters back to YAML: `just param-sync <vehicle>` (`tools/param_sync.py`).

### Control Modes & Onboard Blackbox (`goku_f745` `W25Q128`)
- **`rate`** (`goku_f745`): Sticks command Roll/Pitch/Yaw rates + piecewise thrust (`0..0.5` → `0..hover_thrust_n`, `0.5..1.0` → `hover_thrust_n..max_collective_thrust_n`) into 2 kHz INDI. **Can arm indoors without GPS/mocap lock.** Raising `RC7` (`rc_mission_channel`) engages `OFFBOARD_ACTIVE` for companion UART control (`cybflight-msgs` COBS frames on `PORT_ESP_BRIDGE`, `460_800` baud).
- **`mpc`** / **`mpc_full`** / **`cascade`**: Require `ESKF_CONVERGED` (GPS/mocap lock) to arm (`mpc_full` drives INDI angular-acceleration `α`).
- **Onboard Blackbox (`goku_f745`)**: Logs `.mcap` (CBOR) to onboard 16 MiB `W25Q128JV` on `SPI1` (`PA4..PA7`, `HAS_BLACKBOX_STORAGE = true`, zero in-flight erases). Default config is `blackbox_tier: 2` (`mid`), `blackbox_rate_div: 4` (`250 Hz` `/imu1` + `100 Hz` `/attitude`, `/motors`, `/motor_state` + `20 Hz` `/health` + `RC`). Pull with `just blackbox-pull` and summarize with `python3 analysis/read_mcap.py logs/flight_0001.mcap --summary` (zero-dep stdlib fallback built in; also supports `uv run --with mcap --with cbor2`).

### SITL & Rerun (`cybflight-sim`) & `cybflight_rl`
```bash
just sim-run
SCENARIO=mission_square CONTROLLER=mpc-indi NOISE=mems GPS=sbas just sim-run
VIZ=1 SCENARIO=mission_square CONTROLLER=mpc-indi just sim-run
just sim-compare
just sim-nn
just sim-acmpc
```
For Python RL training (`crates/cybflight_rl`): run `maturin develop --release` inside `crates/cybflight_rl`, then instantiate `cybflight_rl.VecEnv(n=64, vehicle_yaml="vehicles/goku_f745.yaml", seed=0)`.

---

## 6. Extending Cybflight

### Adding a New Flight Controller Board
1. Create `crates/bsp/<board>/` exporting `hal`, `Board`, `init()`, capability consts, `ExtiIrqs`, and `UsbIrqs`. Ensure `APB1_TIMER_HZ` and `APB2_TIMER_HZ` are exact integer multiples of `12_000_000` Hz (`DSHOT600_TIMER_HZ`).
2. Add the BSP crate to root `Cargo.toml` `members` ONLY (never `default-members`).
3. Wire feature gates in `crates/cybflight/{Cargo.toml,src/lib.rs,src/board_init/<board>.rs}`:
   - Dual-timer DShot (`TIM3` + `TIM1`): set `TIM1` `BDTR.MOE + AOE`.
   - STM32F7 linker/RAM: select `memory_f745.x` (`RAM` = `240K` in `SRAM1` at `0x20010000`, `FAULT_SCRATCH` in `BKPSRAM`), raise `USART1..6` and `I2C1` IRQs to `Priority::P5`, and keep `rpm_notch_en: 0` unless CPU budget is verified with `indistat`.
   - Shared I2C buses: call bus recovery (e.g., `reset_i2c1_if_wedged`) and initialize all devices on the bus before spawning reader tasks.
4. Register the board in `crates/vehicle_yaml/src/lib.rs`, `crates/vehicle_yaml/src/bin/vehicle-features.rs`, and `Justfile` (`check-all`).

### Adding a New Parameter
1. Add the field with `#[param(key = "...", unit = "...", min = ..., max = ...)]` in `crates/cybflight_core/src/params.rs`.
2. Bump `FirmwareConfig::VERSION` and update `scaffold()` + tests in `params.rs`.
3. Run `just params-doc` to regenerate `docs/parameters.md`.
