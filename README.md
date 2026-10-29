<p align="center">
  <img src="docs/assets/cybird-logo.png" alt="Cybird logo" width="160" height="160">
</p>

<h1 align="center">Cybflight</h1>

<p align="center">Open-source flight control, written in Rust.</p>

<p align="center">
  <a href="#getting-started">Get started</a> ·
  <a href="#documentation">Documentation</a> ·
  <a href="#ecosystem">Ecosystem</a> ·
  <a href="CONTRIBUTING.md">Contribute</a>
</p>

Cybflight is a modular autopilot for multirotors, part of the [Cybird project](https://github.com/cybird-robotics). It runs on STM32H743 and STM32F745 flight controllers, with an ESP32 WiFi / high-speed UART bridge for telemetry, offboard control, positioning data, and RTK corrections.

> **Fork Notice:** This repository ([`green-creeper/cybflight`](https://github.com/green-creeper/cybflight)) is a fork of [`cybird-robotics/cybflight`](https://github.com/cybird-robotics/cybflight) extending the stack with STM32F745 hardware support (Flywoo GOKU GN 745 AIO), the FLYWOO Firefly 2S Nano Baby 20 airframe (`goku_f745`), an `egui` desktop Configurator & Ground Station (`cybflight-gs`), and onboard neural-network / offboard direct motor control.
>
> **⚠️ Status — Work in Progress (WIP):** All fork additions (specifically the `flywoof745` STM32F745 port, `goku_f745` airframe profile, offboard/RL motor override paths, and `cybflight-gs` configurator) are currently **work in progress and have only been validated in simulation / host tests — they have NOT been flight-tested on physical hardware yet.** Bench-test with propellers removed before attempting any hardware bring-up.

- **Control and estimation** — INDI, model predictive control (NMPC / TinyMPC), geometric SE(3) & cascade control, onboard MLP race policy execution, and a 15-state error-state Kalman filter (ESKF).
- **Indoor and outdoor positioning** — Vicon motion capture and u-blox / Unicore RTK GNSS.
- **Configuration and diagnostics** — YAML airframes, a desktop `egui` Configurator (`cybflight-gs`), a USB parameter shell, live telemetry, and onboard blackbox logging.

## What Was Added in This Fork

1. **STM32F745 Hardware Port — Flywoo GOKU GN 745 45A AIO V3 (`crates/bsp/flywoof745`, `board_flywoof745`)**
   - Full Board Support Package (`bsp-flywoof745`) and board bring-up (`crates/cybflight/src/board_init/flywoof745.rs`) for `STM32F745VG` at 216 MHz (`memory_f745.x`: 512 KiB application flash + 512 KiB Sectors 6 & 7 flash KV store).
   - **Sensors & Peripherals**: SPI4 primary IMU auto-probing (`MPU6000`, `ICM42688P`, `BMI270` at 8 kHz with `PE1` EXTI1), I2C1 `SPL06` barometer (`0x76`) + `QMC5883L` magnetometer (`0x0D`), and ADC3 battery voltage/current monitoring (`PC3` / `PC2`).
   - **Generalized Dual-Timer Bidirectional DShot600**: Abstracted BSP DMA channels (`DshotDmaM1..M4`) supporting split-timer motor outputs (`PB0`/`PB1` on `TIM3_CH3/4` + `PE9`/`PE11` on `TIM1_CH1/2`) with GCR eRPM telemetry capture.
   - **Single-Bank Flash KV Store & Size Optimization**: Adapted `params.rs` for STM32F745 256 KiB sectors (`0x0808_0000` / `0x080C_0000`) with IWDG timeout extension during sector erase, and tuned per-crate release profiles (`opt-level = "z"`/`"s"` on shell/drivers/HAL while keeping `cybflight-core` at `opt-level = 3`) so the firmware fits cleanly within 512 KiB.
   - **Runtime UART Role Mapping**: Configurable `SerialPortId` routing for CRSF/ELRS (`USART2`), GPS (`USART6`), and high-rate DMA Offboard / ESP bridge (`USART1`).

2. **FLYWOO Firefly 2S Nano Baby 20 Vehicle Profile (`vehicles/goku_f745.yaml`)**
   - Complete 61.8 g 2S micro-quadrotor configuration (FLYWOO RB1003 14800KV motors, Gemfan 2015-2 props, 80 mm GPS mast extrinsics, quadratic thrust model, INDI, ESKF, MPC, and CRSF switch mapping).

3. **Offboard Direct Motor Control & Onboard RL Policy Task (`rl_policy_task.rs`, `indi_task.rs`, `esp_bridge.rs`)**
   - Added `DIRECT_MOTOR_COMMAND` bypass in `indi_task` with a 100 ms staleness watchdog and `OFFBOARD_ACTIVE` mode switching on `rc_mission_channel` (RC7).
   - Added `crates/cybflight/src/control/rl_policy_task.rs` to step a compiled-in neural-network `RacePolicy` (`cybflight_core::nn`) onboard from live odometry, IMU, and DShot rotor speed feedback.
   - Extended `esp_bridge.rs` for high-speed UART DMA offboard motor and trajectory setpoint streaming.

4. **Cybflight Configurator & Ground Station GUI (`crates/cybflight_gs`)**
   - Added a Betaflight-style native desktop configurator built with `egui` / `eframe` (`just gs` or `cargo run -p cybflight-gs --profile release-host -- --vehicle goku_f745`).
   - Supports both live USB VCP serial connection and an offline Mock Drone simulator (`--mock`), featuring interactive tabs for Setup & 3D Attitude, UART Ports, Airframe Configuration, PID/INDI/MPC Tuning, Receiver, Modes, Motor Testing, Sensors/ESKF, Blackbox, and CLI Shell.

## Getting started

Install [Rust](https://rustup.rs/) with the platform prerequisites listed by its installer. The repository pins the Rust toolchain and embedded target; build commands use Bash and `just`.

```sh
git clone https://github.com/green-creeper/cybflight.git
cd cybflight
cargo install just --locked

# Build STM32F745 firmware for Flywoo GOKU GN 745 AIO (FLYWOO Firefly 2S Nano Baby 20)
just build goku_f745

# Or build STM32H743 firmware for SAKURA
just build sakura_vicon

# Launch the Cybflight Configurator & Ground Station GUI
just gs goku_f745
```

Use `just vehicles` to list configurations. Choose `sakura_um982` or `sakura_ublox_f9` for RTK GNSS. Builds produce an ELF and binary in `target/thumbv7em-none-eabihf/release/`.

See [hardware setup](docs/hardware-support.md) for wiring, receiver configuration, and bring-up. Adapt the airframe settings, tuning, and mission coordinates to your vehicle before flying.

## Documentation

- [Architecture](docs/architecture.md) — boards, drivers, tasks, and control.
- [Hardware setup](docs/hardware-support.md) — SAKURA, ESP32 boards, and positioning.
- [Parameters](docs/parameters.md) — configuration and tuning.
- [Blackbox logging](docs/blackbox.md) — recording and analyzing flight data.
- [Coordinated releases](docs/releasing.md) — compatible firmware, bridge, and ground-station versions.

## Ecosystem

| Project | Purpose |
|---|---|
| [cybesp-bridge](https://github.com/cybird-robotics/cybesp-bridge) | WiFi bridge for ESP32-C6 boards, including Waveshare and Seeed XIAO |
| [cybgcs](https://github.com/cybird-robotics/cybgcs) | Ground station for Vicon, RTK corrections, telemetry, and visualization |
| [cybflight-msgs](https://github.com/cybird-robotics/cybflight-msgs) | Shared wire protocol |

The [um982](https://github.com/cybird-robotics/um982) driver and [vicon-sdk](https://github.com/cybird-robotics/vicon-sdk) bindings are available on crates.io.

## Contributing

Bug fixes, hardware support, controllers, documentation, and tools are welcome. Read [CONTRIBUTING.md](CONTRIBUTING.md) to get started.

```sh
just test
just test-drivers
```

## License

[Apache-2.0](LICENSE). Copyright 2026 The Cybird project contributors. See [NOTICE](NOTICE).

## Citation

If you use Cybflight in your research, please cite:

```bibtex
@inproceedings{lin2026cybflight,
  author    = {Yifan Lin and Chao Qin and H. S. Helson Go
               and Hugh H.-T. Liu},
  title     = {{Cybflight}: An Embedded {Rust} Autopilot
               for Aerial Robotics Research},
  booktitle = {{IEEE IROS} 2026 Workshop
               ``Why Rust for Robotics: A Perspective From Industry''},
  intype    = {presented at the},
  year      = {2026},
  note      = {{arXiv}:2609.31232},
  url       = {https://arxiv.org/abs/2609.31232}
}
```
