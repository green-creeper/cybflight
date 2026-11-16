---
name: cybflight-blackbox
description: >-
  Workflow and forensic analysis runbook for Cybflight MCAP blackbox flight
  logs on STM32 flight controllers (W25Q128 SPI NOR flash on flywoof745/goku_f745
  and SDMMC/FAT on H7 boards). Use whenever listing, downloading/pulling,
  inspecting, analyzing or troubleshooting flight logs (.mcap), diagnosing
  liftoff/control/CPU/DShot eRPM/estimator/failsafe issues from blackbox data,
  or safely erasing logs from the flight controller.
---

# Cybflight Blackbox Operations & Forensic Analysis

Repository root: `/Users/andrii/Personal/autopilots/cybflight`  
Primary helper script: [`./scripts/blackbox_forensics.py`](./scripts/blackbox_forensics.py) (zero external dependencies; stdlib POSIX USB CDC + streamed MCAP/CBOR parser + automated anomaly detectors)  
Schema & failure-signature catalog: [`./references/topics-and-signatures.md`](./references/topics-and-signatures.md)

---

## 1. Pre-Flight USB & Safety Rules

1. **Single USB serial owner**: Close `cybflight-gs` or any serial monitor before running USB CDC commands. `blackbox_forensics.py` prefers `/dev/cu.usbmodem*`, asserts `DTR | RTS` for `embassy-usb`, silences streams (`stream off`), and uses `lsof` to report blockers on `EBUSY`.
2. **Never unplug immediately after disarming**: `blackbox_task` waits through a **3 s post-disarm grace window** (`SESSION_ACTIVE`) before closing `DataEnd` / `Footer` / trailing `Magic`. Always let the close complete before cutting battery/USB power.
3. **Storage backend differences (`goku_f745` NOR vs H7 SDMMC)**:
   - `flywoof745` (`goku_f745`): `16 MiB` `W25Q128JV` on `SPI1` (`NorBlackboxStore`, `16,320 KiB` usable). Pre-erases `64 KiB` blocks only in background while disarmed; never erases in flight.
   - `blackbox rm` (on the last remaining file) and `blackbox clean` invoke `reclaim_all_disarmed()` synchronously across dirty `64 KiB` blocks — allow **60–95 s timeouts** (built into `blackbox_forensics.py`).
4. **Refused while armed or recording**: `ls`, `get`, `rm`, `clean`, and `blackbox set` are rejected by firmware during active arming or `blackbox record on` holds.

---

## 2. Standard Post-Flight Workflow

### Step A — Check FC Status & Log Inventory
```bash
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py status
# Or quick listing via Makefile/Justfile:
just blackbox-pull --list
```
Verify:
- `blackbox status`: confirms backend (`nor flash: JEDEC ef:40:18`, `used/ready KiB`, `record_set`, `rate_div`, `last session` drops/enc-ovf).
- `param diff`: records any runtime overrides active during that flight.
- `postmortem show` & `resetcause`: checks for brownouts, IWDG resets, panics, or HardFaults.

### Step B — Download & CRC-32 Verify Logs
```bash
# Pull all missing *.mcap logs into /Users/andrii/Personal/autopilots/cybflight/logs/
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py pull

# Force re-pull specific file(s):
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py pull flight_0001.mcap --force
```
*(Or via `just blackbox-pull`. Every downloaded file is verified byte-for-byte against the FC's `CRC <hex8>` trailer before atomic `os.replace`.)*

### Step C — Run Forensic Analysis & Inspect Timeline
```bash
# Analyze all logs in logs/ with automated detectors + 0.5s chronological timeline
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py analyze --timeline

# One-shot pull + analyze + JSON output for machine consumption:
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py analyze --pull --json
```
Always follow automated findings with targeted verification:
1. **Event bracket & structure**: Verify `[ARM ... DISARM (commanded|ControllerTimeout|RcLoss) ... LOG_END]` and check `"sealed": true`. Any `RECORDER_OVERRUN` or `seq_gaps` indicates Thread-Mode starvation or storage backpressure while armed.
2. **Armed vs disarmed split (`/health`)**: Compare `indi_step_max_us` against loop period (`500 us` at 2 kHz, `1000 us` at 1 kHz), `indi_period_max_us` against `MOTOR_CMD_STALE` (`10,000 us`), and `rc_link_age_ms` against `CMD_STALE_TIMEOUT` (`100 ms`).
3. **Raw topic dump / plots (when needed)**:
   ```bash
   python3 analysis/read_mcap.py logs/flight_0001.mcap --summary
   python3 analysis/read_mcap.py logs/flight_0001.mcap --topic /events
   python3 analysis/read_mcap.py logs/flight_0001.mcap --topic /motors --topic /rc
   ```
   Read [`./references/topics-and-signatures.md`](./references/topics-and-signatures.md) to match specific telemetry patterns to exact firmware files and lines.
4. **Live config correlation**: Compare `Metadata` (`board`, `record_set`, `rate_div`, `imu_odr_hz`, `imu_*_lpf_hz`) and `vehicles/<vehicle>.yaml` + live `param diff` to avoid misattributing runtime overrides.

### Step D — Safely Erase Logs from the FC
**Never erase logs on the FC before verifying local CRC-checked backups in `logs/`, unless the operator explicitly requests unbacked deletion.**

```bash
# Recommended: pull+CRC-verify missing backups, dry-run first, then erase all & reclaim NOR
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py erase --all --pull-first --dry-run
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py erase --all --pull-first

# Erase a single backed-up log (deleting the last log triggers NOR reclaim automatically):
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py erase flight_0001.mcap --pull-first

# Explicit force erase when intentionally discarding corrupted/unwanted test files:
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py erase --all --force-unbacked-up
```
Confirm after erasure:
- `blackbox status` shows `0 file(s)` and `ready_kib` restored (e.g., `16320 KiB ready` on `W25Q128`), and `blackbox ls` shows `0 file(s)` (`flight_0001` sequence restarts cleanly).

---

## 3. Bench Recording & Tuning Verification (Optional)

To test logging or reproduce CPU/DShot timing without spinning props:
```bash
python3 tools/fc_shell.py "blackbox set mid" "blackbox record on"
sleep 4
python3 tools/fc_shell.py "blackbox record off"
sleep 1
python3 .agents/skills/cybflight-blackbox/scripts/blackbox_forensics.py status
```
Check `last session` for `0 drops, 0 enc-ovf  [ok]`.
