# Cybflight USB CDC CLI & Hardware Debugging Reference

## 1. Host Tools for Communicating with the FC

Always close `cybflight-gs` or any serial terminal first — only one host process can hold the USB CDC port cleanly.

### `tools/fc_shell.py` (Zero-Dependency Recommended CLI)
Uses Python stdlib (`termios` + `fcntl` + `select` — no `pyserial` needed). On open, it prefers `/dev/cu.usbmodem*` on macOS (callout device never blocks on DCD) and **asserts `DTR | RTS`** so `embassy-usb` `CdcAcmClass::wait_connection()` wakes immediately. If the port returns `EBUSY`, it automatically runs `lsof` to show which process is holding it.

```bash
# Show firmware banner + run one-shot status checks
python3 tools/fc_shell.py --banner "version" "resetcause" "postmortem show" "health"

# Wait up to 10s for USB re-enumeration after flashing/rebooting, then check sensors
python3 tools/fc_shell.py --wait 10 "imu1" "imurate" "baro1" "power"

# Check RC detection & 2 kHz INDI CPU timing
python3 tools/fc_shell.py "rcdiag" "indistat"

# Stream live telemetry for 15s, then cleanly stop all streams before exit
python3 tools/fc_shell.py "stream att on" "stream imu1 on" --monitor 15 --after "stream off"

# Reboot straight into STM32 ROM DFU bootloader (0483:df11) and verify via dfu-util
python3 tools/fc_shell.py --dfu

# Target a specific port or raise per-command timeout
python3 tools/fc_shell.py --port /dev/cu.usbmodem0011 --timeout 4.0 "param list indi"
```

> **Why plain `cat` / `echo` / unconfigured serial fails**: `embassy-usb` CDC waits for `DTR` before entering `shell_loop()`, and macOS `/dev/tty.usbmodem*` blocks waiting for Carrier Detect unless DCD/CLOCAL is handled. Always use `tools/fc_shell.py` or a DTR-asserting terminal on `/dev/cu.usbmodem*`.

### Companion Python CLI Helpers
```bash
# Pull runtime param overrides (`param diff --yaml`) and merge into vehicles/<vehicle>.yaml
just param-sync goku_f745
python3 tools/param_sync.py --vehicle goku_f745 --dry-run

# List and download MCAP blackbox logs (verifies CRC-32; stores in ./logs/)
just blackbox-pull --list
just blackbox-pull
just blackbox-pull --all --delete
```

---

## 2. USB CDC Shell Command Reference

### System, Health, CPU Timing & Crash Inspection
| Command | What it prints / does |
|---|---|
| `help` | Prints full command & tab-completion help (`<Tab>` completes commands, topics, params, missions) |
| `version` | Firmware version, git hash, build UTC timestamp, `[BOARD]`, `vehicle=<name>`, and `[SAFE MODE ...]` if active |
| `resetcause` | Raw `RCC.RSR`/`PWR.CSR1` bits captured in `pre_init`: `IWDG`, `PVD-brownout`, `SFT`, `PIN`, `POR`, `BOR`, `LPWR` |
| `postmortem show` | Prior-boot exit reset cause, fatal `HardFault`/`Panic` registers (`pc`, `lr`, `psr`, `cfsr`, `hfsr`, `mmfar`, `bfar`) & panic text, prior events, plus current boot's `boot_count` and `uptime` (stages `1..9` early at boot, then ms) |
| `postmortem clear` | Wipes the BKPSRAM postmortem slot & prior-boot snapshot |
| `health` | Aggregated pre-arm health gates, `first failing gate`, `failsafe_active`, `rc link`, `mahony`, estimator readiness |
| `indistat` | Reports INDI iterations, `step avg` (µs), `max` (µs), and `loop period max` (µs) **since the previous `indistat` call** |
| `imurate` | Measures true IMU ODR over a 1000 ms window against MCU clock vs. build ODR, plus lost samples & gyro calibration bias |
| `reboot` / `reboot --dfu` | Software reset (normal) or write magic into `RTC_BKP0R` and enter STM32 ROM DFU bootloader (refused if armed) |

### Receiver (RC) Diagnostics
| Command | What it prints / does |
|---|---|
| `rcdiag` | Prints `RcDiag(state=..., port=..., proto=..., baud=..., rc_frames=..., link_frames=..., crc_err=..., u2=..., u1=..., u5=..., u4=..., u3=..., vbat_mv=...)` + latest channel pulse widths |
| `rcdiag rescan` | Unlocks RC port, zeroes RX counters, and re-runs multi-UART CRSF/MAVLink auto-scan (disarmed only) |
| `rc` / `rcstats` | One-shot parsed RC channels (CH1..CH16) or link quality / RSSI / SNR / `RC_LINK_ACTIVE` |

### Sensors, Estimators & Live Telemetry Streams
- **One-shot snapshots**: `imu1`, `imu2`, `att`, `eskf`, `baro1`, `baro2`, `magext`, `magint`, `gps`, `gpshealth`, `gpsrtk`, `vicon`, `power`, `dshot`, `ocp`, `attcontrol`, `timesync`, `fleetstate`.
- **Streams (throttled to `<= 50 Hz` per topic so USB output cannot starve `iwdg_feed_task`)**:
  - `stream <topic> on` / `stream <topic> off` / `stream off`
  - Topics: `imu1`, `imu2`, `att`, `ocp`, `rc`, `rcstats`, `dshot`, `power`, `gps`, `gpsrtk`, `magext`, `magint`, `baro1`, `baro2`, `attcontrol`, `vicon`, `timesync`, `eskf`.

### Parameters, Actuators, Missions & Blackbox
- **Parameters (`param`)** — `set`, `reset`, `save`, `defaults` are refused while armed; `blackbox_*` edits are also refused while recording:
  - `param list [group]` / `param <group> list` (`build`, `airframe.body`, `indi`, `mpc`, `rc`, `eskf`, etc.; `.` / `-` / `_` interchangeable)
  - `param get <key>` / `param set <key> <val>` (live RAM edit, notes `(reboot required)` when latched at boot)
  - `param diff` / `param diff --yaml` (overrides vs. baked YAML defaults)
  - `param reset <key|all>` then `param save --prune` (**mandatory `--prune`** if you want YAML edits to take effect instead of keeping flash-pinned values)
  - `param save` (append overrides to flash Sectors 6+7) / `param defaults` (in-memory reset)
- **Missions (`mission`, when `outer_mpc` compiled in)**: `mission list`, `mission get`, `mission set <name>` or `<env> <variant> <speed>` (disarmed only).
- **Bench Actuators (PROPS OFF, disarmed only)**: `motor <1-4> <0-100>`, `led on|off`.
- **Blackbox**: `blackbox status`, `blackbox set <none|small|mid|large|sysid>`, `blackbox record on|off`, `blackbox ls`, `blackbox get <f> [<off> [<len>]]`, `blackbox rm <f>`, `blackbox clean`.

---

## 3. Step-by-Step Debugging Playbooks

### Playbook A: Early-Crash Safe Mode or Unexpected `IWDG` Reboot
If a crash (`Panic`, `HardFault`, or `IndependentWatchdog`, excluding `Brownout`) occurs before `EARLY_CRASH_SAFE_MODE_MS` (`10_000` ms), the next boot enters **Early-Crash USB Safe Mode** (`[SAFE MODE ACTIVE]` in banner/`postmortem show`) — keeping USB CDC alive with heavy control/sensor tasks disabled.

1. Connect and inspect the prior crash:
   ```bash
   python3 tools/fc_shell.py --banner "resetcause" "postmortem show"
   ```
2. Check **how far the crashed boot progressed** via `uptime` on the prior record:
   - `uptime = 0 ms`: Crashed/stalled before `mark_boot_stage(1)` or never updated BKPSRAM.
   - `uptime = 1..9 ms`: **Boot-stage marker** encoded by `mark_boot_stage(stage)` before steady-state `postmortem_task` ticks overwrite it with real uptime:
     - `1`: Entered `main()` before `board_init::init`
     - `2`: USB shell spawned
     - `3`: SPI IMUs probed and spawned
     - `4`: RC (`multi_rc_task`) & GPS autobaud tasks spawned
     - `5`: Shared `I2C1` baro (`SPL06`) + mag (`QMC5883L`) probed and spawned
     - `6`: Dual-timer DShot + `power_task` spawned (`board_init::init` complete)
     - `7`: `EXECUTOR_CTRL` / `indi_task` spawned (**if `IWDG` resets right after stage `7`, the high-priority 2 kHz `indi_task` + `P8` IMU + `P6` DShot saturated CPU and starved thread-mode `iwdg_feed_task`!**)
     - `8`: All tasks spawned (`main()` finished)
     - `9`: `postmortem_task` reached its first tick (`uptime_ms >= 100` once ticking steadily)
3. If `fatal: Panic` or `HardFault`, decode `pc` / `lr` against the ELF (`target/thumbv7em-none-eabihf/release/cybflight`) using `llvm-objdump` or `arm-none-eabi-addr2line`.
4. After fixing/flashing (or to retry normal boot from Safe Mode), run `reboot` or power-cycle.

### Playbook B: Drone Will Not Arm (`health`)
Run `python3 tools/fc_shell.py "health"`:
- **`Failsafe`**: No active RC frames (`rc_frames`) within staleness window — verify transmitter is powered on and bound, then check `rcdiag`.
- **`ThrottleHigh`**: Throttle channel is above `rc_thr_mincheck_us` (`1050` µs default). Lower throttle trim/stick.
- **`GyroNotCalibrated`**: Hold quad still for ~1 s (`imurate` prints `gyro_cal` status).
- **`EstimatorNotReady`**: Built with `outer_loop: mpc|mpc_full|cascade` indoors without GPS/mocap lock. Use `outer_loop: rate` (`vehicles/goku_f745.yaml`) for manual acro/bench arming.

### Playbook C: RC Receiver Connected, Sticks Do Nothing (`rcdiag`)
Run `python3 tools/fc_shell.py "rcdiag" "rcstats"`:
- **`u2=0, u1=0, u5=0, u4=0, u3=0` (zero RX bytes on every UART)**:
  - Check TX/RX wiring crossover, YAML/pad assignment, or whether the receiver's 5V rail only powers on with LiPo connected (`vbat_mv=20` indicates USB-only power without LiPo).
- **`state=LOCKED, proto=MAVLink, link_frames>0, crc_err=0, rc_frames=0`**:
  - Expected **passive lock**: the ELRS receiver is powered and sending MAVLink `HEARTBEAT`/`RADIO_STATUS` packets on UART, but your **handset TX is OFF or unbound** (firmware keeps `RC_LINK_ACTIVE=false` and failsafe ON until real stick `RC_CHANNELS` / CRSF RC frames arrive).
- **`crc_err` climbing rapidly**:
  - Check baud/protocol or UART IRQ starvation (verify `USART1..6` sit at `Priority::P5` above `CAN2_TX`/`CAN2_RX1`/`CAN2_RX0` on F745).

### Playbook D: Verify 2 kHz CPU Headroom (`indistat` + `imurate`)
```bash
python3 tools/fc_shell.py "indistat"
sleep 1
python3 tools/fc_shell.py "indistat" "imurate"
```
On 216 MHz STM32F745 (`indi_ctrl_div: 4` -> 2 kHz INDI, `500 µs` period):
- `step avg` should be well under `500 µs` (observed baseline with `rpm_notch_en: 0` is `~202 µs` avg, `~340–402 µs` max).
- `imurate` should show `~8000 Hz` and `lost since boot: 0`.
- If `step avg` approaches `450–500 µs` or climbs toward watchdog timeout, disable expensive per-sample DSP (e.g., `rpm_notch_en: 0`) before flying.

### Playbook E: Blackbox Recording & Post-Flight Analysis (`goku_f745` `W25Q128`)
Arming starts `flight_NNNN.mcap` automatically (`IS_ARMED` edge with 40 ms debounce + 3 s post-disarm tail grace); you can also trigger bench logging without spinning motors:
```bash
# Check NOR storage readiness (expects `JEDEC ef:40:18`, `16320 KiB ready` when clean)
python3 tools/fc_shell.py "blackbox status" "blackbox ls"

# Pull all flight logs to ./logs/ (verifies CRC-32; zero third-party deps)
just blackbox-pull

# Summarize topics, rates, drops & events (stdlib MCAP/CBOR reader built in)
python3 analysis/read_mcap.py logs/flight_0001.mcap --summary
python3 analysis/read_mcap.py logs/flight_0001.mcap --topic /imu1 --limit 10

# Wipe all flight logs on flash while disarmed (pre-erases ready blocks in background)
python3 tools/fc_shell.py --timeout 90 "blackbox clean"
```
- **Clean state**: Pre-erases run only while disarmed via background `idle_maintenance()` (`64 KiB` blocks). Never unplug immediately during the 3 s post-disarm close window (`ClosePage` seals at `LOG_END`).
