---
type: workflow
title: Building & Flashing Firmware
description: Complete guide to compiling, optimizing, verifying static memory sizing, resolving features, and DFU flashing Cybflight firmware onto target flight controllers.
tags: [firmware, build, flashing, dfu, optimization, feature-resolution, static-memory]
verified:
  - by: openwiki/0.7.2
    at: 2026-10-30T14:16:13.882Z
sources:
  - id: openwiki-source-4905fab56ecf9fa5e1ebbf3f
    resource: repo://.cargo/config.toml
  - id: openwiki-source-651d1fb6c9e49916a916ab51
    resource: repo://Cargo.toml
  - id: openwiki-source-466a0061f76c44079beb5e18
    resource: repo://crates/cybflight/build.rs
  - id: openwiki-source-5f260bc2c4379465f3e68d11
    resource: repo://crates/vehicle_yaml/src/bin/vehicle-features.rs
  - id: openwiki-source-14e6a4e1ddc06973b783ff20
    resource: repo://Justfile
  - id: openwiki-source-5b4e6f1fd76c065dd5235300
    resource: repo://tools/build.sh
  - id: openwiki-source-7d0bddff1822dc15c69ce615
    resource: repo://tools/size_report.py
  - id: openwiki-source-08d95a8fd0e8a4cf8589caa8
    resource: repo://tools/vehicle_features.sh
generated: { by: "openwiki/0.7.2", at: "2026-10-30T14:16:13.882Z" }
---

# Building & Flashing Firmware

Cybflight uses a deterministic build and packaging workflow to target ARM Cortex-M7 microcontrollers (such as STM32H743 and STM32F745). The compilation pipeline combines workspace Cargo configurations, vehicle declarations (`vehicles/*.yaml`), offline mission profiles (`missions/*.yaml`), motor thrust tables (`data/thrust_tables/*.csv`), and neural cost policies (`data/cost_policies/*.bin`) into a standalone binary (`cybflight.bin`).

This workflow ensures hardware safety through build-time validation, opt-level compiler tuning, static RAM headroom auditing, and DFU flashing procedures with post-flash USB CDC shell boot confirmation.

---

## Architectural Workflow & Feature Pipeline

The build process converts high-level vehicle YAML definitions into compiled binary images through feature resolution, asset baking, compile-time validation, and packaging:

<!-- openwiki: mermaid parse failed and this diagram was converted to a text fence so it does not break rendering. Fix the diagram source and restore the mermaid fence. Parser error: Heuristic: an unescaped angle bracket inside a label breaks rendering; rephrase the label. -->
```text
flowchart TD
    A["Vehicle Specification\n(vehicles/*.yaml)"] --> B["Feature Resolution\n(tools/vehicle_features.sh)"]
    ENV["Dev Env Overrides\n(BOARD, OUTER_LOOP, etc.)"] --> B
    B --> C["Cargo Build Invocation\n(cargo run --release --features ...)"]
    C --> D["Build Script Validation\n(crates/cybflight/build.rs)"]
    D --> E["Bake Static Assets\n(missions, thrust tables, cost policies)"]
    D --> F["Feature Guard Validation\n(Build Section vs Active Cargo Features)"]
    E --> G["Compile Firmware ELF\n(thumbv7em-none-eabihf)"]
    F --> G
    G --> H["Runner Packaging Script\n(tools/build.sh)"]
    H --> I["llvm-objcopy ELF -> BIN\n(cybflight.bin)"]
```
*Firmware feature resolution, build-time asset baking, consistency validation, and binary generation pipeline.*

---

## Target Architecture & Compiler Profiles

### Target Triple & Hardware Floating-Point Constraints

The default target triple for firmware builds is `thumbv7em-none-eabihf` (Cortex-M7 with hardware single-precision VFP).

Target compiler flags are configured in `.cargo/config.toml`:

```toml
[target.thumbv7em-none-eabihf]
rustflags = ["-C", "target-cpu=cortex-m7", "-C", "target-feature=-fp64"]
runner = "bash tools/build.sh"
```

> **Hardware Instruction Fault Guard (`-fp64`)**: Microcontrollers such as the STM32F745VG feature single-precision FPv5 hardware acceleration (`FPv5-SP-D16`), but lack hardware double-precision units. Any double-precision instruction (`vcvt.f64.f32`, `vmul.f64`) triggers an immediate `UsageFault: UNDEFINSTR`. Disabling `fp64` in `rustflags` forces the Rust compiler to emit single-precision instructions across all math pipelines (8 kHz IMU loops, ESKF estimation, and MPC control).

### Cargo Release Profile & Opt-Level Tuning

Compilation performance and code size are tuned in `Cargo.toml` using package-level optimization overrides under `[profile.release]`:

```toml
[profile.release]
debug = 2
lto = true
codegen-units = 1
panic = "abort"
overflow-checks = false

[profile.release.package.cybflight]
opt-level = "s"
[profile.release.package.cybflight-drivers]
opt-level = 2
[profile.release.package.embassy-stm32]
opt-level = 2
[profile.release.package.embedded-fatfs]
opt-level = "s"
```

* **`cybflight` (Firmware Shell)**: Standardized at `opt-level = "s"` to optimize code size while preserving loop unrolling (unlike `"z"`), allowing the top-level binary to fit within 512 KiB flash allocations on STM32F745 targets.
* **`cybflight-core`, `nalgebra`, `air-filters`**: Default to `opt-level = 3` to deliver high performance in real-time control, matrix operations, and filtering loops.
* **`cybflight-drivers`, `embassy-stm32`**: Tuned to `opt-level = 2` to balance peripheral driver efficiency with binary footprint.
* **`profile.release-host`**: A host-side profile that inherits from `release` but disables LTO (`lto = false`) and relaxes `codegen-units = 16` to accelerate incremental rebuilds of host simulation tools (`cybflight-sim`) without paying the cross-crate LTO penalty required by embedded targets.

---

## Vehicle Feature Resolution Pipeline

Build-time feature selections originate from vehicle definition files located in `vehicles/*.yaml` (e.g., `vehicles/goku_f745.yaml`, `vehicles/sakura_vicon.yaml`).

### The Build Section Scheme

Each vehicle YAML defines its hardware board, receiver protocol, control architecture, sensor sources, and rates in its `build:` block:

```yaml
build:
  board: flywoof745        # board_flywoof745 | board_sakurah743 | board_foxeerh743 | board_micoair743v2
  rc_protocol: crsf        # rx_crsf | rx_ghst
  outer_loop: rate         # outer_rate | outer_geometric | outer_mpc | outer_mpc_full
  pos_source: gps          # est_pos_gps | est_pos_mocap
  gps_model: ublox         # ublox (default) | gps_unicore
  gps_dual_antenna: no     # gps_dual_antenna
  imu_rate: 8khz           # 8khz (default) | imu_1khz
  indi: yes                # yes (default) | indi_off
```

### Feature Resolution Script (`tools/vehicle_features.sh`)

When running `just build [VEHICLE]`, `Justfile` invokes `tools/vehicle_features.sh`, which executes the `vehicle-features` host binary (`crates/vehicle_yaml/src/bin/vehicle-features.rs`). The resolution logic converts YAML keys into Cargo feature flags:

| YAML Key | YAML Value | Resolved Cargo Feature |
| :--- | :--- | :--- |
| `board` | `flywoof745` | `board_flywoof745` |
| `rc_protocol` | `crsf` | `rx_crsf` |
| `outer_loop` | `mpc` / `cascade` / `rate` | `outer_mpc` / `outer_geometric` / `outer_rate` |
| `pos_source` | `gps` / `mocap` | `est_pos_gps` / `est_pos_mocap` |
| `gps_model` | `unicore` (when `pos_source: gps`) | `gps_unicore` |
| `gps_dual_antenna` | `yes` (when `pos_source: gps`) | `gps_dual_antenna` |
| `imu_rate` | `1khz` | `imu_1khz` (8 kHz emits no extra feature flag) |
| `indi` | `no` | `indi_off` (default `yes` emits no extra feature flag) |
| `plan_online` | `yes` | `plan_online` |

### Dev Environment Overrides & Feature Provenance

Developer environment variables (`BOARD`, `RC_PROTOCOL`, `OUTER_LOOP`, `POS_SOURCE`, `GPS_MODEL`, `IMU_RATE`, `INDI`, `PLAN_ONLINE`) can override vehicle YAML facts during bench testing.

To inspect the resolved feature set and variable provenance for a vehicle without building:

```bash
just print-features goku_f745
```

Output highlights resolved features and source provenance:

```text
VEHICLE=goku_f745
vehicle-features: board            = "flywoof745" (goku_f745.yaml)
vehicle-features: rc_protocol      = "crsf" (goku_f745.yaml)
vehicle-features: outer_loop       = "rate" (goku_f745.yaml)
vehicle-features: pos_source       = "gps" (goku_f745.yaml)
...
FEATURES=board_flywoof745,rx_crsf,outer_rate,est_pos_gps
```

To list all available vehicle YAML target names:

```bash
just vehicles
```

---

## Build-Time Baking & Safety Validation (`build.rs`)

The firmware build script (`crates/cybflight/build.rs`) serves as the primary build guard, asset generator, and parameter baker.

### Linker Script & Memory Layout Overrides

The build script copies board-specific linker memory configurations to `OUT_DIR`:

* **STM32F745 (`board_flywoof745`)**: Copies `memory_f745.x` to override memory boundaries for 512 KiB flash layouts.
* **STM32H743 (`board_sakurah743`, etc.)**: Copies `memory.x`, which shrinks the `FLASH` region from 2048 KiB to 1792 KiB, reserving Bank-2 sectors 6 and 7 (the final 256 KiB) for the non-volatile parameter key-value store.

### Asset Baking

1. **Mission Baking (`bake_missions`)**: Parses all `missions/*.yaml` files, validates piece bounds against `cybflight_core::trajectory_planning::MAX_PIECES`, auto-calculates waypoint timestamps, and generates `$OUT_DIR/missions.rs`.
2. **Thrust Table Baking (`bake_thrust_tables`)**: Parses CSV files in `data/thrust_tables/*.csv`. It enforces a 50x50 grid dimension, checks monotonicity along the thrust axis, converts collective test-rig thrust to per-rotor thrust by dividing bounds by `N_MOTORS = 4`, and emits `$OUT_DIR/thrust_tables.rs`.
3. **Learned Cost Policy Baking (`emit_cost_policy`)**: Reads binary neural policy files in `data/cost_policies/<stem>.bin` specified by `mpc_cost_policy` in vehicle YAML. It verifies header magic `0x50434643` (`CFCP`), map revision (`COST_MAP_VERSION`), and network shape dimensions before emitting weights directly into initialized `.rodata` static arrays.

### Parameter Integrity & Hardware-Coupled Key Guard

`build.rs` enforces the **"no default mass"** rule from the [Parameter System](../concepts/parameter-system.md) architecture:

* **Required Physical Fields**: `airframe` mass, inertia matrix, and motor geometry must be fully declared in YAML; missing physical identity fields cause a hard build error.
* **Hardware-Coupled Tuning Guard**: Hardware-dependent keys (`m0_tau`..`m3_tau`, `m0_omega_max`..`m3_omega_max`, `batt_nominal_v`, `indi_ctrl_div`, `eskf_max_pos_jump_m`, `fence_*`, `rc_*`) must be explicitly pinned in the YAML `tuning:` block. Unpinned hardware keys trigger `cargo:warning` messages during compilation.

### Feature↔YAML Consistency Guard (`check_build_section`)

To eliminate split-brain issues where a developer selects one vehicle in `.env` but builds with incompatible Cargo feature flags, `build.rs` cross-checks active Cargo features (`CARGO_FEATURE_*`) against the vehicle's `build:` declaration.

If active features conflict with the YAML declaration (e.g., building `board_sakurah743` when `goku_f745.yaml` declares `board: flywoof745`), `build.rs` aborts with a panic:

```text
vehicle goku_f745: build.board implies "board_flywoof745" but active features are
[missing: false, conflicting: ["board_sakurah743"]] — feature list and vehicle YAML disagree
```

> **CI Matrix Override**: The workspace build-check recipe (`just check-all`) sets `VEHICLE_GUARD=off` to intentionally compile feature matrices across boards without matching vehicle YAML files.

---

## Step-by-Step Firmware Building

### Basic Vehicle Commands

To build the default vehicle specified by `VEHICLE` in `.env` (defaults to `sakura_vicon`):

```bash
just build
```

To build a specific vehicle binary (e.g., `goku_f745` or `sakura_vicon`):

```bash
just build goku_f745
just build sakura_vicon
```

Behind the scenes, `just build` executes:

```bash
cargo run --release --no-default-features \
  --features $(bash tools/vehicle_features.sh goku_f745),postmortem
```

The Cargo runner script (`tools/build.sh`) converts the resulting ELF into a raw binary image at `target/thumbv7em-none-eabihf/release/cybflight.bin` using `llvm-objcopy`.

### Building Controller Variants

You can override the outer loop selection on the command line without modifying vehicle YAML files:

* **MPC Outer Loop**:
  ```bash
  just build-mpc goku_f745
  ```
* **Geometric Cascade Outer Loop**:
  ```bash
  just build-cascade goku_f745
  ```

### Workspace Smoke Testing (`check-all`)

To verify that all supported board, receiver, estimator, and outer-loop feature combinations compile cleanly without flashing hardware:

```bash
just check-all
```

---

## Static Sizing & Stack Headroom Verification

Embedded STM32 targets locate `.data`, `.bss`, `.uninit`, and the main execution stack within a single shared 512 KiB AXI SRAM region. Static RAM expansion reduces the remaining room available for stack growth.

### Generating Size Reports (`just size`)

Run `just size` to compile the firmware and display static memory usage, stack headroom, and top symbol allocations:

```bash
just size goku_f745
```

Example size report output:

```text
target/thumbv7em-none-eabihf/release/cybflight
  flash image         342,112 B   (  334.1 KiB)
  .vector_table         1,024 B   (    1.0 KiB)
  .data                 2,412 B   (    2.4 KiB)
  .bss                118,544 B   (  115.8 KiB)
  static RAM          120,956 B   (  118.1 KiB)  = 23.1 % of 512 KiB
  headroom            403,236 B   (  393.8 KiB)  end of statics 0x2401D83C -> stack top 0x24080000

  largest .bss (b) / .data (d) symbols:
      65,536 B  b  cybflight::blackbox::sdmmc_block::SDMMC_BUFFER
      16,384 B  b  cybflight::telemetry::MAVLINK_TX_BUF
       8,192 B  b  cybflight_core::mpc::SOLVER_SCRATCH
```

### The 8 kHz `.bss` Boot-Loop Invariant

In high-rate flight controllers running 8 kHz IMU loops, large zero-initialized `.bss` structures or oversized stack frames can lead to runtime stack overflow into static RAM, triggering CPU hard faults and watchdog reboot loops.

> **Operational Invariant**: Always execute `just size` after introducing large static buffers, task futures, or data structures. Verify that `headroom` remains adequate for worst-case stack growth before flight testing.

---

## DFU Flashing Procedures

Flashing is handled via `dfu-util` over USB DFU mode (`USB ID 0483:df11`).

### Hardware Preparation & DFU Boot Mode

1. Connect the flight controller to the host via USB.
2. If the board is already running Cybflight or Betaflight, trigger DFU mode via serial CLI or press and hold the physical **BOOT0** button while plugging in the USB cable.
3. Verify that the system sees the device in DFU mode:
   ```bash
   dfu-util -l
   ```
   The output must display vendor/product ID `0483:df11`.

### Flashing Firmware (`just flash`)

To compile and flash a target vehicle in one step:

```bash
just flash goku_f745
```

### DFU Recipe Mechanics

The `just flash` recipe executes a multi-step retry and boot verification sequence:

1. **DFU Device Check**: Verifies `0483:df11` is active on the USB bus.
2. **Download & Jump (`:leave`)**: Executes `dfu-util -d "0483:df11" -a 0 -s 0x08000000:leave -D <BIN>`. The `:leave` command instructs the STM32 system bootloader to jump to flash address `0x08000000` upon completion.
3. **Transfer Failure Retry**: If the USB transfer interrupts (e.g., macOS `dfuMANIFEST` timeout or USB reset), `just flash` automatically retries the download up to 3 times.
4. **CDC Shell Boot Verification**: After issuing the leave request, `just flash` polls `/dev/serial/by-id/*cybflight*` and `/dev/tty.usbmodem*` for up to 10 seconds to confirm that the board booted and enumerated its USB CDC shell.

```text
flashed and booted: /dev/serial/by-id/usb-Cybflight_Cybflight_FC_387834313030-if00
```

### Verified Flashing (`just flash-verify`)

When validating hardware or troubleshooting suspicious flash behavior, use `just flash-verify` to write the binary, read back the written byte range from flash memory, and perform a byte-for-byte comparison:

```bash
just flash-verify goku_f745
```

Output confirming verified flash image:

```text
VERIFY OK: 342112 bytes on flash match target/thumbv7em-none-eabihf/release/cybflight.bin
  (board is still in DFU — 'just flash' or a power-cycle boots it)
```

> **Note**: `just flash-verify` leaves the board in DFU mode after verification. Power-cycle the board or run `just flash` to jump into the main application.

### Partial Flash Failure & Recovery

If a DFU download fails midway, the flash memory contains an incomplete firmware image. Because an incomplete image cannot initialize USB hardware, the flight controller will not enumerate as a CDC serial shell or automatically re-enter DFU mode.

**Recovery Procedure**:
1. Unplug the USB cable from the flight controller.
2. Press and hold the physical **BOOT0** button on the flight controller board.
3. Re-plug the USB cable while holding **BOOT0** to force the microcontroller's ROM bootloader into DFU mode (`0483:df11`).
4. Re-run `just flash <VEHICLE>`.

---

## Related Documentation

* [BSP & Hardware Initialization](../architecture/bsp-and-hardware-init.md) — Board support packages, pin mappings, and memory boundaries.
* [Parameter System](../concepts/parameter-system.md) — Parameter registry, YAML schema loading, and non-volatile KV storage.
* [Quickstart](../quickstart.md) — Workspace setup, toolchain requirements, and initial bench testing steps.
