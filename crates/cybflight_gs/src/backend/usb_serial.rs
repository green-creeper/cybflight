use std::io::{Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use serialport::SerialPortType;

use super::DiscoveredPort;

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
            .timeout(Duration::from_millis(20))
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

            // Wake shell prompt and query parameters + telemetry streams
            let init_cmds: &[&str] = if auto_stream {
                &[
                    "",
                    "version",
                    "param list",
                    "stream att on",
                    "stream imu1 on",
                    "stream power on",
                    "stream dshot on",
                ]
            } else {
                &["", "version", "param list"]
            };
            for cmd in init_cmds {
                let line = format!("{cmd}\r\n");
                let _ = port.write_all(line.as_bytes());
                let _ = port.flush();
                thread::sleep(Duration::from_millis(35));
            }

            let mut rx_buf = [0u8; 1024];
            let mut line_acc = String::with_capacity(512);

            while !stop_clone.load(Ordering::Relaxed) {
                // Drain outgoing commands
                while let Ok(cmd) = cmd_rx.try_recv() {
                    let payload = if cmd.ends_with('\n') {
                        cmd
                    } else {
                        format!("{cmd}\r\n")
                    };
                    if let Err(e) = port.write_all(payload.as_bytes()) {
                        let _ = event_tx.send(SerialEvent::Error(format!("Write error: {e}")));
                        let _ = event_tx.send(SerialEvent::Disconnected(port_label));
                        return;
                    }
                    let _ = port.flush();
                    thread::sleep(Duration::from_millis(8));
                }

                // Read incoming bytes
                match port.read(&mut rx_buf) {
                    Ok(n) if n > 0 => {
                        let chunk = String::from_utf8_lossy(&rx_buf[..n]);
                        for ch in chunk.chars() {
                            if ch == '\n' {
                                let clean = line_acc.trim_end_matches('\r').to_string();
                                line_acc.clear();
                                if !clean.is_empty() {
                                    let _ = event_tx.send(SerialEvent::Line(clean));
                                }
                            } else if ch != '\r' {
                                line_acc.push(ch);
                                if line_acc.len() > 2048 {
                                    let clean = std::mem::take(&mut line_acc);
                                    let _ = event_tx.send(SerialEvent::Line(clean));
                                }
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                    Err(e) => {
                        let _ =
                            event_tx.send(SerialEvent::Error(format!("Serial read error: {e}")));
                        let _ = event_tx.send(SerialEvent::Disconnected(port_label));
                        return;
                    }
                }
            }

            // Turn off high-rate streams cleanly on disconnect
            let _ = port.write_all(b"stream off\r\n");
            let _ = port.flush();
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

/// Enumerate candidate USB serial ports on macOS / Linux.
pub fn discover_serial_ports() -> Vec<DiscoveredPort> {
    let mut list = Vec::new();
    let Ok(ports) = serialport::available_ports() else {
        return list;
    };

    for p in ports {
        // On macOS, prefer `/dev/tty.*` or `/dev/cu.usbmodem*`; skip Bluetooth debug ports
        if p.port_name.contains("Bluetooth") || p.port_name.contains("debug-console") {
            continue;
        }
        if p.port_name.starts_with("/dev/cu.") {
            // Keep `/dev/tty.*` to avoid duplicate entries per device on macOS unless no tty exists
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
