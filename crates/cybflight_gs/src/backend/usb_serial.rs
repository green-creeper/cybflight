use std::collections::BTreeSet;
use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serialport::{SerialPort, SerialPortType};

use super::DiscoveredPort;

const INITIAL_STREAMS: &[&str] = &["att", "imu1", "power", "dshot", "rc", "rcstats"];

#[derive(Default, Clone, Copy)]
struct DrainOutcome {
    saw_param_footer: bool,
    bytes_read: usize,
}

/// Events emitted by the USB CDC serial background worker.
#[derive(Debug)]
pub enum SerialEvent {
    Line(String),
    Connected(String),
    Disconnected(String),
    Error(String),
}

/// Active USB CDC connection to a physical Cybflight flight controller.
pub struct UsbSerialConnection {
    cmd_tx: Sender<String>,
    event_rx: Receiver<SerialEvent>,
    stop_flag: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl UsbSerialConnection {
    pub fn connect(port_name: &str, auto_stream: bool) -> Result<Self, String> {
        let mut port = serialport::new(port_name, 115_200)
            .timeout(Duration::from_millis(12))
            .open()
            .map_err(|e| format!("Failed to open {port_name}: {e}"))?;

        // Cybflight's USB CDC task waits for DTR before streaming/accepting shell I/O.
        let _ = port.write_data_terminal_ready(true);
        let _ = port.write_request_to_send(true);

        let (cmd_tx, cmd_rx) = mpsc::channel::<String>();
        let (event_tx, event_rx) = mpsc::channel::<SerialEvent>();
        let stop_flag = Arc::new(AtomicBool::new(false));
        let stop_clone = Arc::clone(&stop_flag);
        let port_label = port_name.to_string();

        let worker = thread::spawn(move || {
            let _ = event_tx.send(SerialEvent::Connected(port_label.clone()));

            let mut rx_buf = [0u8; 4096];
            let mut line_acc: Vec<u8> = Vec::with_capacity(512);
            let mut active_streams: BTreeSet<String> = BTreeSet::new();

            let outcome: Result<bool, String> = (|| {
                // 1. Silence any leftover streams and drain initial DTR banner cleanly.
                send_cmd_and_wait_prompt(
                    &mut *port,
                    "stream off",
                    Duration::from_millis(250),
                    &mut rx_buf,
                    &mut line_acc,
                    &event_tx,
                    Some(&stop_clone),
                )?;
                drain_for_duration(
                    &mut *port,
                    Duration::from_millis(20),
                    &mut rx_buf,
                    &mut line_acc,
                    &event_tx,
                    &stop_clone,
                )?;

                // 2. Query version + full parameter table BEFORE enabling high-rate streams
                //    so all 332+ lines transfer over uncontended USB CDC bulk IN.
                send_cmd_and_wait_prompt(
                    &mut *port,
                    "version",
                    Duration::from_millis(120),
                    &mut rx_buf,
                    &mut line_acc,
                    &event_tx,
                    Some(&stop_clone),
                )?;
                run_param_list_sync(
                    &mut *port,
                    "param list",
                    &active_streams,
                    &mut rx_buf,
                    &mut line_acc,
                    &event_tx,
                    &stop_clone,
                )?;

                // 3. Now start live telemetry streams if requested.
                if auto_stream && !stop_clone.load(Ordering::Relaxed) {
                    for &s in INITIAL_STREAMS {
                        active_streams.insert(s.to_string());
                    }
                    enable_streams_set(
                        &mut *port,
                        &active_streams,
                        &mut rx_buf,
                        &mut line_acc,
                        &event_tx,
                        &stop_clone,
                    )?;
                }

                if !stop_clone.load(Ordering::Relaxed) {
                    let _ = send_cmd_and_wait_prompt(
                        &mut *port,
                        "rcdiag",
                        Duration::from_millis(250),
                        &mut rx_buf,
                        &mut line_acc,
                        &event_tx,
                        Some(&stop_clone),
                    );
                }

                // 4. Main interleaved I/O loop with one-command-in-flight pacing so even
                //    hundreds of queued `param set` commands never overflow OS/USB rings.
                let mut saw_reboot = false;
                while !stop_clone.load(Ordering::Relaxed) {
                    for _ in 0..8 {
                        let Ok(raw_cmd) = cmd_rx.try_recv() else {
                            break;
                        };
                        let trimmed = raw_cmd.trim();
                        if trimmed.is_empty() {
                            continue;
                        }
                        if is_reboot_command(trimmed) {
                            send_reboot_and_flush(
                                &mut *port,
                                trimmed,
                                &mut rx_buf,
                                &mut line_acc,
                                &event_tx,
                            );
                            saw_reboot = true;
                            break;
                        }

                        let is_param_list =
                            trimmed == "param list" || trimmed.starts_with("param list ");

                        if is_param_list {
                            run_param_list_sync(
                                &mut *port,
                                trimmed,
                                &active_streams,
                                &mut rx_buf,
                                &mut line_acc,
                                &event_tx,
                                &stop_clone,
                            )?;
                        } else {
                            if trimmed == "stream off" {
                                active_streams.clear();
                            } else if let Some(mid) = trimmed
                                .strip_prefix("stream ")
                                .and_then(|s| s.strip_suffix(" on"))
                            {
                                let name = mid.trim();
                                if !name.is_empty() {
                                    active_streams.insert(name.to_string());
                                }
                            } else if let Some(mid) = trimmed
                                .strip_prefix("stream ")
                                .and_then(|s| s.strip_suffix(" off"))
                            {
                                active_streams.remove(mid.trim());
                            }
                            let wait_budget = if trimmed.starts_with("param save") {
                                // Flash sector erase on STM32F7 can stall up to ~1.5-2.0 s.
                                Duration::from_millis(2500)
                            } else {
                                Duration::from_millis(150)
                            };
                            send_cmd_and_wait_prompt(
                                &mut *port,
                                trimmed,
                                wait_budget,
                                &mut rx_buf,
                                &mut line_acc,
                                &event_tx,
                                Some(&stop_clone),
                            )?;
                        }
                    }

                    if saw_reboot {
                        break;
                    }

                    let _ = drain_available_rx(
                        &mut *port,
                        &mut rx_buf,
                        &mut line_acc,
                        &event_tx,
                        None,
                    )?;
                }

                // Drain, write, and flush any pending commands (e.g. `param save`, `reboot --dfu`)
                // queued right before `disconnect()` set `stop_flag`.
                if !saw_reboot {
                    saw_reboot = flush_pending_commands_on_stop(
                        &mut *port,
                        &cmd_rx,
                        &mut rx_buf,
                        &mut line_acc,
                        &event_tx,
                    );
                }

                Ok(saw_reboot)
            })();

            match outcome {
                Err(err_msg) => {
                    let _ = event_tx.send(SerialEvent::Error(err_msg));
                    let _ = event_tx.send(SerialEvent::Disconnected(port_label));
                    return;
                }
                Ok(saw_reboot) => {
                    // Turn off high-rate streams cleanly on user-requested disconnect unless rebooting.
                    if !saw_reboot {
                        let _ = port.write_all(b"stream off\n");
                        let _ = port.flush();
                    }
                }
            }
            let _ = event_tx.send(SerialEvent::Disconnected(port_label));
        });

        Ok(Self {
            cmd_tx,
            event_rx,
            stop_flag,
            worker: Some(worker),
        })
    }

    pub fn send_command(&self, cmd: &str) {
        let _ = self.cmd_tx.send(cmd.to_string());
    }

    pub fn poll_events(&self) -> Vec<SerialEvent> {
        let mut out = Vec::new();
        while let Ok(ev) = self.event_rx.try_recv() {
            out.push(ev);
        }
        out
    }

    pub fn disconnect(&mut self) {
        self.stop_flag.store(true, Ordering::Relaxed);
        if let Some(handle) = self.worker.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for UsbSerialConnection {
    fn drop(&mut self) {
        self.disconnect();
    }
}

#[derive(Default)]
struct CommandSyncState<'a> {
    expect_echo: Option<&'a str>,
    saw_echo: bool,
    saw_post_echo_prompt: bool,
}

impl<'a> CommandSyncState<'a> {
    fn for_command(clean: &'a str, line_acc: &mut Vec<u8>) -> Self {
        let trimmed = clean.trim();
        let need_echo = !trimmed.is_empty() && trimmed.len() <= 60;
        if !need_echo {
            // Strip any stale pre-command prompt so a blank/oversized command
            // waits for a genuinely new `> ` prompt after dispatch.
            if line_acc.as_slice() == b"> " || line_acc.as_slice() == b">" {
                line_acc.clear();
            } else if line_acc.starts_with(b"> ") {
                line_acc.drain(..2);
            }
        }
        Self {
            expect_echo: if need_echo { Some(trimmed) } else { None },
            saw_echo: !need_echo,
            saw_post_echo_prompt: false,
        }
    }
}

fn is_reboot_command(trimmed: &str) -> bool {
    trimmed == "reboot" || trimmed.starts_with("reboot ")
}

fn send_reboot_and_flush<P: std::io::Read + std::io::Write + ?Sized>(
    port: &mut P,
    cmd: &str,
    rx_buf: &mut [u8],
    line_acc: &mut Vec<u8>,
    event_tx: &Sender<SerialEvent>,
) {
    let clean = cmd.trim_end_matches(['\r', '\n']);
    let wire = format!("{clean}\n");
    let _ = write_all_interleaved(
        port,
        wire.as_bytes(),
        rx_buf,
        line_acc,
        event_tx,
        None,
        None,
    );
    let _ = port.flush();
    // Consume immediate echo / banner until USB CDC detaches or brief grace expires.
    let deadline = Instant::now() + Duration::from_millis(40);
    while Instant::now() < deadline {
        if drain_available_rx(port, rx_buf, line_acc, event_tx, None).is_err() {
            break;
        }
    }
}

/// Drains, writes, and flushes any remaining queued commands in `cmd_rx` before
/// tearing down the serial port so fast sequences like `reboot --dfu` + `disconnect()`
/// never drop the reboot command. Returns `true` if a reboot command was dispatched.
fn flush_pending_commands_on_stop<P: std::io::Read + std::io::Write + ?Sized>(
    port: &mut P,
    cmd_rx: &mpsc::Receiver<String>,
    rx_buf: &mut [u8],
    line_acc: &mut Vec<u8>,
    event_tx: &Sender<SerialEvent>,
) -> bool {
    while let Ok(raw_cmd) = cmd_rx.try_recv() {
        let trimmed = raw_cmd.trim();
        if trimmed.is_empty() || trimmed == "param list" || trimmed.starts_with("param list ") {
            continue;
        }
        if is_reboot_command(trimmed) {
            send_reboot_and_flush(port, trimmed, rx_buf, line_acc, event_tx);
            return true;
        }
        let wait_budget = if trimmed.starts_with("param save") {
            Duration::from_millis(2500)
        } else {
            Duration::from_millis(150)
        };
        let _ = send_cmd_and_wait_prompt(
            port,
            trimmed,
            wait_budget,
            rx_buf,
            line_acc,
            event_tx,
            None,
        );
    }
    false
}

/// Sends a single `\n`-terminated command and drains RX until the FC echoes the
/// command AND prints its post-dispatch `> ` shell prompt (or `timeout` expires),
/// guaranteeing at most 1 command in flight so OS TTY (`TTYHOG = 1024 B`) and USB
/// CDC buffers can never overflow.
fn send_cmd_and_wait_prompt<P: std::io::Read + std::io::Write + ?Sized>(
    port: &mut P,
    cmd: &str,
    timeout: Duration,
    rx_buf: &mut [u8],
    line_acc: &mut Vec<u8>,
    event_tx: &Sender<SerialEvent>,
    stop_flag: Option<&AtomicBool>,
) -> Result<(), String> {
    let clean = cmd.trim_end_matches(['\r', '\n']);
    let mut sync = CommandSyncState::for_command(clean, line_acc);
    let wire = format!("{clean}\n");
    write_all_interleaved(
        port,
        wire.as_bytes(),
        rx_buf,
        line_acc,
        event_tx,
        None,
        Some(&mut sync),
    )?;
    let _ = port.flush();

    let deadline = Instant::now() + timeout;
    while !sync.saw_post_echo_prompt
        && Instant::now() < deadline
        && !stop_flag.is_some_and(|f| f.load(Ordering::Relaxed))
    {
        let _ = drain_available_rx(port, rx_buf, line_acc, event_tx, Some(&mut sync))?;
    }
    Ok(())
}

/// Runs `param list` cleanly: temporarily pauses active streams so 200 Hz
/// telemetry lines cannot interleave with parameter lines, drains RX until the
/// `(N shown)` footer AND trailing `> ` prompt arrive, and restores streams afterward.
fn run_param_list_sync(
    port: &mut dyn SerialPort,
    cmd: &str,
    active_streams: &BTreeSet<String>,
    rx_buf: &mut [u8],
    line_acc: &mut Vec<u8>,
    event_tx: &Sender<SerialEvent>,
    stop_flag: &AtomicBool,
) -> Result<(), String> {
    let pause_and_resume = !active_streams.is_empty();
    if pause_and_resume {
        send_cmd_and_wait_prompt(
            port,
            "stream off",
            Duration::from_millis(100),
            rx_buf,
            line_acc,
            event_tx,
            Some(stop_flag),
        )?;
        drain_for_duration(
            port,
            Duration::from_millis(20),
            rx_buf,
            line_acc,
            event_tx,
            stop_flag,
        )?;
    }

    let clean = cmd.trim_end_matches(['\r', '\n']);
    let mut sync = CommandSyncState::for_command(clean, line_acc);
    let wire = format!("{clean}\n");
    write_all_interleaved(
        port,
        wire.as_bytes(),
        rx_buf,
        line_acc,
        event_tx,
        Some(stop_flag),
        Some(&mut sync),
    )?;

    let hard_deadline = Instant::now() + Duration::from_millis(5000);
    let mut idle_deadline = Instant::now() + Duration::from_millis(1200);
    let mut saw_footer = false;
    while Instant::now() < hard_deadline
        && Instant::now() < idle_deadline
        && !stop_flag.load(Ordering::Relaxed)
    {
        let d = drain_available_rx(port, rx_buf, line_acc, event_tx, Some(&mut sync))?;
        if d.saw_param_footer {
            saw_footer = true;
        }
        if saw_footer && sync.saw_post_echo_prompt {
            break;
        }
        if saw_footer {
            // Footer arrived; allow up to 60 ms to consume the trailing `> ` packet.
            let prompt_cap = Instant::now() + Duration::from_millis(60);
            if idle_deadline > prompt_cap {
                idle_deadline = prompt_cap;
            }
        } else if d.bytes_read > 0 {
            idle_deadline = Instant::now() + Duration::from_millis(800);
        }
    }

    if pause_and_resume && !stop_flag.load(Ordering::Relaxed) {
        enable_streams_set(port, active_streams, rx_buf, line_acc, event_tx, stop_flag)?;
    }

    Ok(())
}

fn enable_streams_set(
    port: &mut dyn SerialPort,
    active_streams: &BTreeSet<String>,
    rx_buf: &mut [u8],
    line_acc: &mut Vec<u8>,
    event_tx: &Sender<SerialEvent>,
    stop_flag: &AtomicBool,
) -> Result<(), String> {
    for stream_name in active_streams {
        if stop_flag.load(Ordering::Relaxed) {
            break;
        }
        let cmd = format!("stream {stream_name} on");
        send_cmd_and_wait_prompt(
            port,
            &cmd,
            Duration::from_millis(80),
            rx_buf,
            line_acc,
            event_tx,
            Some(stop_flag),
        )?;
    }
    Ok(())
}

fn drain_for_duration(
    port: &mut dyn SerialPort,
    dur: Duration,
    rx_buf: &mut [u8],
    line_acc: &mut Vec<u8>,
    event_tx: &Sender<SerialEvent>,
    stop_flag: &AtomicBool,
) -> Result<(), String> {
    let deadline = Instant::now() + dur;
    while Instant::now() < deadline && !stop_flag.load(Ordering::Relaxed) {
        let _ = drain_available_rx(port, rx_buf, line_acc, event_tx, None)?;
    }
    Ok(())
}

/// Writes `data` to `port`, retrying `TimedOut` / `WouldBlock` while draining
/// RX so USB CDC IN and OUT pipes can never deadlock against each other.
fn write_all_interleaved<P: std::io::Read + std::io::Write + ?Sized>(
    port: &mut P,
    mut data: &[u8],
    rx_buf: &mut [u8],
    line_acc: &mut Vec<u8>,
    event_tx: &Sender<SerialEvent>,
    stop_flag: Option<&AtomicBool>,
    mut sync: Option<&mut CommandSyncState<'_>>,
) -> Result<(), String> {
    let deadline = Instant::now() + Duration::from_millis(2000);
    while !data.is_empty() {
        if stop_flag.is_some_and(|f| f.load(Ordering::Relaxed)) {
            return Ok(());
        }
        match port.write(data) {
            Ok(n) if n > 0 => {
                data = &data[n..];
            }
            Ok(_) => {
                let _ = drain_available_rx(port, rx_buf, line_acc, event_tx, sync.as_deref_mut())?;
                if Instant::now() >= deadline {
                    return Err("Write error: USB serial write stalled (0 bytes)".to_string());
                }
            }
            Err(ref e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::TimedOut
                        | std::io::ErrorKind::WouldBlock
                        | std::io::ErrorKind::Interrupted
                ) =>
            {
                let _ = drain_available_rx(port, rx_buf, line_acc, event_tx, sync.as_deref_mut())?;
                if Instant::now() >= deadline {
                    return Err(format!("Write error: {e}"));
                }
            }
            Err(e) => {
                return Err(format!("Write error: {e}"));
            }
        }
    }
    Ok(())
}

/// Drains available RX chunks until the read timeout (`12 ms`) or up to 32
/// consecutive non-empty reads. Treats `Ok(0)` (POSIX EOF / `POLLHUP` when USB
/// CDC unenumerates on unplug or `reboot`) as a clean disconnect error rather
/// than spinning the worker thread at 100% CPU.
fn drain_available_rx<R: std::io::Read + ?Sized>(
    port: &mut R,
    rx_buf: &mut [u8],
    line_acc: &mut Vec<u8>,
    event_tx: &Sender<SerialEvent>,
    mut sync: Option<&mut CommandSyncState<'_>>,
) -> Result<DrainOutcome, String> {
    let mut out = DrainOutcome::default();
    for _ in 0..32 {
        match port.read(rx_buf) {
            Ok(n) if n > 0 => {
                out.bytes_read += n;
                if process_rx_chunk(&rx_buf[..n], line_acc, event_tx, sync.as_deref_mut()) {
                    out.saw_param_footer = true;
                }
            }
            Ok(0) if !rx_buf.is_empty() => {
                return Err("USB serial device disconnected".to_string());
            }
            Ok(_) => break,
            Err(ref e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::TimedOut
                        | std::io::ErrorKind::WouldBlock
                        | std::io::ErrorKind::Interrupted
                ) =>
            {
                break;
            }
            Err(e) => {
                return Err(format!("Serial read error: {e}"));
            }
        }
    }
    Ok(out)
}

fn process_rx_chunk(
    chunk: &[u8],
    line_acc: &mut Vec<u8>,
    event_tx: &Sender<SerialEvent>,
    mut sync: Option<&mut CommandSyncState<'_>>,
) -> bool {
    let mut saw_param_footer = false;
    for &b in chunk {
        if b == b'\n' {
            while line_acc.last() == Some(&b'\r') {
                line_acc.pop();
            }
            if !line_acc.is_empty() {
                let had_leading_prompt = line_acc.starts_with(b"> ") || line_acc.as_slice() == b">";
                let clean = String::from_utf8_lossy(line_acc).into_owned();
                line_acc.clear();
                let trimmed = clean.trim_start_matches(['>', ' ']).trim();
                if let Some(st) = sync.as_deref_mut() {
                    if st.saw_echo && had_leading_prompt {
                        st.saw_post_echo_prompt = true;
                    } else if !st.saw_echo
                        && let Some(expected) = st.expect_echo
                        && trimmed == expected
                    {
                        st.saw_echo = true;
                    }
                }
                if is_param_list_footer(trimmed) {
                    saw_param_footer = true;
                }
                let _ = event_tx.send(SerialEvent::Line(clean));
            }
        } else if b != b'\r' {
            line_acc.push(b);
            if line_acc.len() > 4096 {
                let clean = String::from_utf8_lossy(line_acc).into_owned();
                line_acc.clear();
                let _ = event_tx.send(SerialEvent::Line(clean));
            }
        }
    }
    if let Some(st) = sync
        && st.saw_echo
        && (line_acc.ends_with(b"> ") || line_acc.as_slice() == b">")
    {
        st.saw_post_echo_prompt = true;
    }
    saw_param_footer
}

pub fn is_param_list_footer(trimmed: &str) -> bool {
    if let Some(inner) = trimmed.strip_prefix('(').and_then(|s| s.strip_suffix(" shown)")) {
        return !inner.is_empty() && inner.chars().all(|c| c.is_ascii_digit());
    }
    trimmed.starts_with("no group matched ")
}

/// Enumerate candidate USB serial ports on macOS / Linux.
pub fn discover_serial_ports() -> Vec<DiscoveredPort> {
    let mut list = Vec::new();
    let Ok(ports) = serialport::available_ports() else {
        return list;
    };

    for p in ports {
        // On macOS, prefer `/dev/cu.usbmodem*`; skip Bluetooth debug ports
        if p.port_name.contains("Bluetooth") || p.port_name.contains("debug-console") {
            continue;
        }
        if cfg!(target_os = "macos") {
            // On macOS, prefer the call-out device (`/dev/cu.*`), which never
            // blocks on carrier detect, and skip `/dev/tty.*` duplicates.
            if p.port_name.starts_with("/dev/tty.") {
                continue;
            }
        } else if p.port_name.starts_with("/dev/cu.") {
            continue;
        }

        let (label, is_cybflight) = match &p.port_type {
            SerialPortType::UsbPort(usb) => {
                let prod = usb.product.as_deref().unwrap_or("USB Serial");
                let mfg = usb.manufacturer.as_deref().unwrap_or("");
                let is_cyb = (usb.vid == 0x0483 && usb.pid == 0x5740)
                    || prod.to_ascii_lowercase().contains("cybflight")
                    || mfg.to_ascii_lowercase().contains("cybflight");
                let tag = if is_cyb {
                    format!(
                        "{} — Cybflight FC ({:04x}:{:04x})",
                        p.port_name, usb.vid, usb.pid
                    )
                } else {
                    format!(
                        "{} — {} ({:04x}:{:04x})",
                        p.port_name, prod, usb.vid, usb.pid
                    )
                };
                (tag, is_cyb)
            }
            _ => {
                let is_modem = p.port_name.contains("usbmodem") || p.port_name.contains("ttyACM");
                (p.port_name.clone(), is_modem)
            }
        };

        list.push(DiscoveredPort {
            port_name: p.port_name,
            label,
            is_cybflight,
        });
    }

    // Sort Cybflight USB ports first
    list.sort_by(|a, b| {
        b.is_cybflight
            .cmp(&a.is_cybflight)
            .then(a.port_name.cmp(&b.port_name))
    });
    list
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn echo_does_not_prematurely_satisfy_post_dispatch_prompt() {
        let (tx, rx) = mpsc::channel();
        let mut line_acc = b"> ".to_vec();
        let mut sync = CommandSyncState::for_command("param save", &mut line_acc);

        // 1. FC echoes `param save\r\n` immediately BEFORE flash erase finishes.
        process_rx_chunk(b"param save\r\n", &mut line_acc, &tx, Some(&mut sync));
        assert!(sync.saw_echo);
        assert!(
            !sync.saw_post_echo_prompt,
            "echo attached to prior `> ` must not trigger prompt completion"
        );

        // 2. 1.5s later, flash erase completes and FC emits output + new `> `.
        process_rx_chunk(
            b"params saved to flash\r\n> ",
            &mut line_acc,
            &tx,
            Some(&mut sync),
        );
        assert!(sync.saw_post_echo_prompt);
        assert_eq!(line_acc, b"> ");

        let lines: Vec<String> = rx
            .try_iter()
            .filter_map(|ev| match ev {
                SerialEvent::Line(s) => Some(s),
                _ => None,
            })
            .collect();
        assert_eq!(lines, vec!["> param save", "params saved to flash"]);
    }

    #[test]
    fn interleaved_stream_lines_and_split_utf8_frame_cleanly() {
        let (tx, rx) = mpsc::channel();
        let mut line_acc = b"> ".to_vec();
        let mut sync = CommandSyncState::for_command("param get ixx", &mut line_acc);

        // In-flight stream line arrives before command echo
        process_rx_chunk(
            b"Imu(timestamp=1.0,accel_m_s2=[0,0,9.81],gyro_rad_s=[0,0,0],temp_c=30)\r\n",
            &mut line_acc,
            &tx,
            Some(&mut sync),
        );
        assert!(!sync.saw_echo);
        assert!(!sync.saw_post_echo_prompt);

        // Echo + response with multi-byte UTF-8 `[kg·m²]` split mid-character + prompt + stream line
        let full = "param get ixx\r\nixx = 0.000043 [kg·m²]\r\n> Imu(timestamp=2.0)\r\n".as_bytes();
        let split_at = full.iter().position(|&b| b == 0xC2).unwrap() + 1;
        process_rx_chunk(&full[..split_at], &mut line_acc, &tx, Some(&mut sync));
        assert!(sync.saw_echo);
        assert!(!sync.saw_post_echo_prompt);

        process_rx_chunk(&full[split_at..], &mut line_acc, &tx, Some(&mut sync));
        assert!(sync.saw_post_echo_prompt);

        let lines: Vec<String> = rx
            .try_iter()
            .filter_map(|ev| match ev {
                SerialEvent::Line(s) => Some(s),
                _ => None,
            })
            .collect();
        assert!(lines.iter().any(|l| l.contains("[kg·m²]")));
    }

    #[test]
    fn eof_zero_read_signals_disconnect_instead_of_spinning() {
        let (tx, _rx) = mpsc::channel();
        let mut rx_buf = [0u8; 64];
        let mut line_acc = Vec::new();
        let mut empty_eof: &[u8] = b"";
        let res = drain_available_rx(&mut empty_eof, &mut rx_buf, &mut line_acc, &tx, None);
        assert!(res.is_err());
    }

    #[test]
    fn shutdown_flush_writes_and_flushes_pending_dfu_reboot_command() {
        struct MockPort {
            rx: std::collections::VecDeque<u8>,
            tx: Vec<u8>,
            flushed: bool,
        }
        impl std::io::Read for MockPort {
            fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
                if self.rx.is_empty() {
                    return Ok(0);
                }
                let mut count = 0;
                while count < buf.len() {
                    if let Some(b) = self.rx.pop_front() {
                        buf[count] = b;
                        count += 1;
                    } else {
                        break;
                    }
                }
                Ok(count)
            }
        }
        impl std::io::Write for MockPort {
            fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
                self.tx.extend_from_slice(buf);
                Ok(buf.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                self.flushed = true;
                Ok(())
            }
        }

        let (cmd_tx, cmd_rx) = mpsc::channel();
        let (event_tx, _event_rx) = mpsc::channel();
        cmd_tx.send("param set rc_min_us 988".to_string()).unwrap();
        cmd_tx.send("reboot --dfu".to_string()).unwrap();

        let mut port = MockPort {
            rx: b"> param set rc_min_us 988\r\nok\r\n> ".iter().copied().collect(),
            tx: Vec::new(),
            flushed: false,
        };
        let mut rx_buf = [0u8; 64];
        let mut line_acc = b"> ".to_vec();

        let saw_reboot = flush_pending_commands_on_stop(
            &mut port,
            &cmd_rx,
            &mut rx_buf,
            &mut line_acc,
            &event_tx,
        );
        assert!(saw_reboot);
        assert!(port.flushed);
        assert_eq!(
            String::from_utf8_lossy(&port.tx),
            "param set rc_min_us 988\nreboot --dfu\n"
        );
    }
}
