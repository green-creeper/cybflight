use core::fmt::Write;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_usb::{
    class::cdc_acm::CdcAcmClass,
    driver::{Driver, EndpointError},
};

pub mod complete;
pub mod format;

// ---------------------------------------------------------------------------
// WriteBuf — core::fmt::Write adapter for fixed byte slices
// ---------------------------------------------------------------------------
pub struct WriteBuf<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> WriteBuf<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}

impl Write for WriteBuf<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = &mut self.buf[self.pos..];
        if bytes.len() > remaining.len() {
            return Err(core::fmt::Error);
        }
        remaining[..bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
        Ok(())
    }
}

/// Write `data` to the CDC class in ≤64-byte packets.
///
/// A zero-length packet (ZLP) is appended when `data` is an exact multiple of
/// 64 bytes, signalling end-of-transfer to the USB host.
pub async fn write_all<'d>(
    class: &mut CdcAcmClass<'d, impl Driver<'d>>,
    data: &[u8],
) -> Result<(), EndpointError> {
    for chunk in data.chunks(64) {
        class.write_packet(chunk).await?;
    }
    if !data.is_empty() && data.len().is_multiple_of(64) {
        class.write_packet(&[]).await?;
    }
    Ok(())
}

/// Coalesces many small shell lines (such as `param list` and `param diff`)
/// into full 64-byte USB Full-Speed bulk packets instead of emitting one short
/// packet per line (which terminates each host URB early and throttles large
/// dumps to ~1 line per 1 ms USB SOF frame).
pub struct BufferedUsbWriter<'a, 'd, D: Driver<'d>> {
    class: &'a mut CdcAcmClass<'d, D>,
    buf: [u8; 256],
    pos: usize,
    sent_exact_64: bool,
}

impl<'a, 'd, D: Driver<'d>> BufferedUsbWriter<'a, 'd, D> {
    pub fn new(class: &'a mut CdcAcmClass<'d, D>) -> Self {
        Self {
            class,
            buf: [0u8; 256],
            pos: 0,
            sent_exact_64: false,
        }
    }

    pub async fn push(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let mut rem = data;
        while !rem.is_empty() {
            let space = self.buf.len() - self.pos;
            let take = rem.len().min(space);
            self.buf[self.pos..self.pos + take].copy_from_slice(&rem[..take]);
            self.pos += take;
            rem = &rem[take..];
            if self.pos == self.buf.len() {
                for chunk in self.buf.chunks_exact(64) {
                    self.class.write_packet(chunk).await?;
                }
                self.pos = 0;
                self.sent_exact_64 = true;
            }
        }
        Ok(())
    }

    pub async fn finish(self) -> Result<(), EndpointError> {
        if self.pos > 0 {
            write_all(self.class, &self.buf[..self.pos]).await
        } else if self.sent_exact_64 {
            self.class.write_packet(&[]).await
        } else {
            Ok(())
        }
    }
}

pub struct ShellLine {
    buf: [u8; 256],
    len: usize,
}

impl ShellLine {
    pub fn new() -> Self {
        Self {
            buf: [0; 256],
            len: 0,
        }
    }

    /// Write into the line via a closure, then seal it in one borrow.
    pub fn format<F: FnOnce(&mut WriteBuf<'_>)>(&mut self, f: F) {
        let mut w = WriteBuf::new(&mut self.buf);
        f(&mut w);
        self.len = w.as_slice().len();
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl Default for ShellLine {
    fn default() -> Self {
        Self::new()
    }
}

pub static SHELL_OUT: Channel<CriticalSectionRawMutex, ShellLine, 8> = Channel::new();

/// Best-effort push of an `ERROR: …` line onto the shell output queue.
/// Silently drops if the queue is full or the shell consumer hasn't
/// started yet — the caller has bigger problems than a missed line.
///
/// Used by tasks that need to surface a degraded-but-alive condition
/// (e.g. subscriber-slot exhaustion at boot) without panicking the FCU.
pub fn shell_err(msg: &str) {
    let mut line = ShellLine::new();
    line.format(|w| {
        let _ = w.write_str("ERROR: ");
        let _ = w.write_str(msg);
        let _ = w.write_str("\r\n");
    });
    SHELL_OUT.try_send(line).ok();
}
