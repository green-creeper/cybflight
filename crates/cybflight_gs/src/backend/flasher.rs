use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc::{self, Receiver};
use std::thread;

#[derive(Debug, Clone)]
pub enum FlashEvent {
    Log(String),
    Finished { ok: bool, summary: String },
}

pub struct FlasherWorker {
    pub is_running: bool,
    pub progress: f32,
    pub status_text: String,
    pub log_lines: Vec<String>,
    pub dfu_detected: bool,
    pub dfu_device_info: String,
    rx: Option<Receiver<FlashEvent>>,
}

impl Default for FlasherWorker {
    fn default() -> Self {
        Self {
            is_running: false,
            progress: 0.0,
            status_text: "Ready".to_string(),
            log_lines: Vec::new(),
            dfu_detected: false,
            dfu_device_info: "Not scanned yet".to_string(),
            rx: None,
        }
    }
}

impl FlasherWorker {
    /// Probe `dfu-util -l` to check if an STM32 ROM DFU bootloader (`0483:df11`) is on the USB bus.
    pub fn refresh_dfu_status(&mut self) {
        match Command::new("dfu-util").arg("-l").output() {
            Ok(out) => {
                let text = String::from_utf8_lossy(&out.stdout);
                let mut found_line = None;
                for line in text.lines() {
                    if line.contains("0483:df11") && line.contains("alt=0") {
                        found_line = Some(line.trim().to_string());
                        break;
                    } else if line.contains("0483:df11") {
                        found_line = Some(line.trim().to_string());
                    }
                }
                if let Some(info) = found_line {
                    self.dfu_detected = true;
                    self.dfu_device_info = format!("STM32 DFU Bootloader Online ({info})");
                } else {
                    self.dfu_detected = false;
                    self.dfu_device_info =
                        "No 0483:df11 DFU device found (click 'Enter DFU' when connected over USB or hold BOOT button)".to_string();
                }
            }
            Err(e) => {
                self.dfu_detected = false;
                self.dfu_device_info = format!("dfu-util not found in PATH: {e}");
            }
        }
    }

    pub fn poll(&mut self) {
        let Some(rx) = &self.rx else {
            return;
        };
        let mut finished = false;
        while let Ok(ev) = rx.try_recv() {
            match ev {
                FlashEvent::Log(line) => {
                    if let Some(pct) = parse_dfu_progress_percent(&line) {
                        self.progress = pct / 100.0;
                    }
                    self.log_lines.push(line);
                    if self.log_lines.len() > 1000 {
                        self.log_lines.drain(0..200);
                    }
                }
                FlashEvent::Finished { ok, summary } => {
                    self.is_running = false;
                    self.progress = if ok { 1.0 } else { self.progress };
                    self.status_text = summary.clone();
                    self.log_lines.push(format!("=== {summary} ==="));
                    finished = true;
                }
            }
        }
        if finished {
            self.rx = None;
            self.refresh_dfu_status();
        }
    }

    /// Run `just build <vehicle>` followed by `just size <vehicle>`.
    pub fn start_build(
        &mut self,
        workspace_root: &Path,
        vehicle: &str,
        outer_loop_override: Option<&str>,
    ) {
        if self.is_running {
            return;
        }
        self.is_running = true;
        self.progress = 0.1;
        self.status_text = format!("Building firmware for '{vehicle}'...");
        self.log_lines.push(format!(
            "Starting firmware build for vehicle '{vehicle}' in {}...",
            workspace_root.display()
        ));

        let root = workspace_root.to_path_buf();
        let veh = vehicle.to_string();
        let outer = outer_loop_override.map(str::to_string);
        let (tx, rx) = mpsc::channel();
        self.rx = Some(rx);

        thread::spawn(move || {
            let mut cmd = Command::new("just");
            cmd.current_dir(&root).arg("size").arg(&veh);
            if let Some(ol) = &outer
                && !ol.is_empty()
                && ol != "default"
            {
                cmd.env("OUTER_LOOP", ol);
            }
            let ok = run_streaming_command(cmd, &tx);
            let summary = if ok {
                format!(
                    "Build succeeded for '{veh}' (target/thumbv7em-none-eabihf/release/cybflight.bin)"
                )
            } else {
                format!("Build failed for '{veh}'")
            };
            let _ = tx.send(FlashEvent::Finished { ok, summary });
        });
    }

    /// Run `just flash <vehicle>` (or `just flash-verify <vehicle>`).
    pub fn start_build_and_flash(
        &mut self,
        workspace_root: &Path,
        vehicle: &str,
        outer_loop_override: Option<&str>,
        verify: bool,
    ) {
        if self.is_running {
            return;
        }
        let recipe = if verify { "flash-verify" } else { "flash" };
        self.is_running = true;
        self.progress = 0.05;
        self.status_text = format!("Running `just {recipe} {vehicle}`...");
        self.log_lines
            .push(format!("Starting `just {recipe} {vehicle}`..."));

        let root = workspace_root.to_path_buf();
        let veh = vehicle.to_string();
        let outer = outer_loop_override.map(str::to_string);
        let recipe_owned = recipe.to_string();
        let (tx, rx) = mpsc::channel();
        self.rx = Some(rx);

        thread::spawn(move || {
            let mut cmd = Command::new("just");
            cmd.current_dir(&root).arg(&recipe_owned).arg(&veh);
            if let Some(ol) = &outer
                && !ol.is_empty()
                && ol != "default"
            {
                cmd.env("OUTER_LOOP", ol);
            }
            let ok = run_streaming_command(cmd, &tx);
            let summary = if ok {
                format!("`just {recipe_owned} {veh}` completed successfully!")
            } else {
                format!("`just {recipe_owned} {veh}` failed (check DFU status & log)")
            };
            let _ = tx.send(FlashEvent::Finished { ok, summary });
        });
    }

    /// Flash an existing `.bin` image directly via `dfu-util`.
    pub fn start_dfu_flash_bin(&mut self, bin_path: PathBuf) {
        if self.is_running {
            return;
        }
        self.is_running = true;
        self.progress = 0.05;
        self.status_text = format!("Flashing {} via dfu-util...", bin_path.display());
        self.log_lines.push(format!(
            "Flashing binary {} to 0x08000000 via dfu-util...",
            bin_path.display()
        ));

        let (tx, rx) = mpsc::channel();
        self.rx = Some(rx);

        thread::spawn(move || {
            let mut cmd = Command::new("dfu-util");
            cmd.args(["-d", "0483:df11", "-a", "0", "-s", "0x08000000:leave", "-D"])
                .arg(&bin_path);
            let ok = run_streaming_command(cmd, &tx);
            let summary = if ok {
                "DFU flash completed and board rebooted into application!".to_string()
            } else {
                "DFU flash failed (ensure board is in 0483:df11 DFU mode)".to_string()
            };
            let _ = tx.send(FlashEvent::Finished { ok, summary });
        });
    }
}

fn run_streaming_command(mut cmd: Command, tx: &mpsc::Sender<FlashEvent>) -> bool {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::{Duration, Instant};

    cmd.stdout(Stdio::piped()).stderr(Stdio::piped());
    let mut child = match cmd.spawn() {
        Ok(c) => c,
        Err(e) => {
            let _ = tx.send(FlashEvent::Log(format!("Failed to spawn process: {e}")));
            return false;
        }
    };

    let stdout = child.stdout.take();
    let stderr = child.stderr.take();

    let downloaded_ok = Arc::new(AtomicBool::new(false));
    let manifest_seen = Arc::new(AtomicBool::new(false));

    let tx_out = tx.clone();
    let dl_out = Arc::clone(&downloaded_ok);
    let mf_out = Arc::clone(&manifest_seen);
    let out_handle = stdout.map(|out| {
        thread::spawn(move || {
            let reader = BufReader::new(out);
            for line in reader.lines().map_while(Result::ok) {
                if line.contains("File downloaded successfully") {
                    dl_out.store(true, Ordering::Relaxed);
                }
                if line.contains("Transitioning to dfuMANIFEST state")
                    || line.contains("Submitting leave request")
                {
                    mf_out.store(true, Ordering::Relaxed);
                }
                let _ = tx_out.send(FlashEvent::Log(line));
            }
        })
    });

    let tx_err = tx.clone();
    let dl_err = Arc::clone(&downloaded_ok);
    let mf_err = Arc::clone(&manifest_seen);
    let err_handle = stderr.map(|err| {
        thread::spawn(move || {
            let reader = BufReader::new(err);
            for line in reader.lines().map_while(Result::ok) {
                if line.contains("File downloaded successfully") {
                    dl_err.store(true, Ordering::Relaxed);
                }
                if line.contains("Transitioning to dfuMANIFEST state")
                    || line.contains("Submitting leave request")
                {
                    mf_err.store(true, Ordering::Relaxed);
                }
                let _ = tx_err.send(FlashEvent::Log(line));
            }
        })
    });

    // Poll child exit so we can auto-reap macOS `dfu-util` if it hangs inside
    // `Transitioning to dfuMANIFEST state` after successfully downloading &
    // jumping the STM32 into application mode.
    let mut manifest_since: Option<Instant> = None;
    let mut reaped_after_manifest = false;
    let status = loop {
        match child.try_wait() {
            Ok(Some(s)) => break Ok(s),
            Ok(None) => {
                if downloaded_ok.load(Ordering::Relaxed) && manifest_seen.load(Ordering::Relaxed) {
                    let t0 = *manifest_since.get_or_insert_with(Instant::now);
                    if t0.elapsed() >= Duration::from_millis(1500) {
                        let _ = child.kill();
                        let s = child.wait();
                        reaped_after_manifest = true;
                        let _ = tx.send(FlashEvent::Log(
                            "macOS DFU leave completed (board rebooted out of DFU).".to_string(),
                        ));
                        break s;
                    }
                }
                thread::sleep(Duration::from_millis(50));
            }
            Err(e) => break Err(e),
        }
    };

    if let Some(h) = out_handle {
        let _ = h.join();
    }
    if let Some(h) = err_handle {
        let _ = h.join();
    }

    reaped_after_manifest
        || downloaded_ok.load(Ordering::Relaxed)
        || matches!(status, Ok(s) if s.success())
}

fn parse_dfu_progress_percent(line: &str) -> Option<f32> {
    // dfu-util prints lines like: `Download	[=============>           ]  54%       212992 bytes`
    let pct_pos = line.find('%')?;
    let before = line[..pct_pos].trim_end();
    let num_str = before.split_ascii_whitespace().last()?;
    num_str.parse::<f32>().ok()
}
