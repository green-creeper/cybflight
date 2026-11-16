//! Backend-agnostic storage types and trait for the flight blackbox.

use embedded_io_async::Write;

/// Max filename length surfaced in [`EntrySummary`].
pub const MAX_FILENAME: usize = 32;
pub type EntryName = heapless::String<MAX_FILENAME>;

/// One root-level recording entry.
#[derive(Clone)]
pub struct EntrySummary {
    pub name: EntryName,
    pub size: u32,
}

/// Outcome of deleting all flight logs via `blackbox clean`.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct CleanSummary {
    /// Files deleted.
    pub removed: u32,
    /// True if deletion stopped part-way due to an I/O fault.
    pub aborted: bool,
}

/// Unified blackbox storage error across SDMMC/FAT and SPI-NOR backends.
#[derive(Clone, Copy, defmt::Format)]
pub enum OpError {
    NoSubscriberSlot,
    NoMessageInTimeout,
    CardAcquire,
    StorageInit,
    NoPartitionTable,
    NoFatPartition,
    PartitionIo,
    Mount,
    Create,
    Write,
    PartialWrite,
    Unmount,
    DiskFull,
    NotFound,
    Open,
    Read,
    Remove,
    Aborted,
}

/// Pluggable file payload written inside a recording session.
#[allow(async_fn_in_trait)]
pub trait FileBody {
    /// Write the body. Return total bytes written so the caller knows the
    /// finalized file size without a separate stat.
    async fn write<W: Write>(&mut self, w: &mut W) -> Result<u32, W::Error>;
}

/// Read-side abort marker returned by [`FileSink`] (disconnection, timeout, or armed).
pub struct SinkAbort;

/// Streaming read consumer used by `blackbox get`.
#[allow(async_fn_in_trait)]
pub trait FileSink {
    /// Announce the exact byte count that will follow.
    async fn begin(&mut self, total: u32) -> Result<(), SinkAbort>;
    /// Consume one sequential chunk of file content (up to 512 bytes).
    async fn data(&mut self, chunk: &[u8]) -> Result<(), SinkAbort>;
}

/// Session-level storage I/O statistics surfaced by `blackbox status`.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct IoStats {
    pub writes: u32,
    pub blocks_written: u32,
    pub max_batch: u16,
    pub reads: u32,
    pub blocks_read: u32,
    pub fallbacks: u32,
}

/// Live summary of onboard SPI-NOR storage (`W25Q128`) for `blackbox status`.
#[derive(Clone, Copy, Default)]
pub struct NorStorageInfo {
    pub jedec: [u8; 3],
    pub capacity_kib: u32,
    pub used_kib: u32,
    pub ready_kib: u32,
    pub files: u16,
    pub formatting: bool,
}

/// Storage backend seam consumed by `blackbox_task` and `FlightRecorder`.
#[allow(async_fn_in_trait)]
pub trait BlackboxBackend {
    async fn idle_maintenance(&mut self) {}
    async fn scan_highest_seq(&mut self, prefix: &str, suffix: &str) -> Result<u32, OpError>;
    async fn write_session<P: FileBody>(
        &mut self,
        path: &str,
        body: &mut P,
    ) -> Result<u32, OpError>;
    fn last_io_stats(&self) -> IoStats;
    async fn scan_root_files<F>(&mut self, visit: &mut F) -> Result<(), OpError>
    where
        F: FnMut(&str, u32);
    async fn read_file<S: FileSink>(
        &mut self,
        path: &str,
        offset: u32,
        max_len: Option<u32>,
        sink: &mut S,
    ) -> Result<u32, OpError>;
    async fn remove_file(&mut self, path: &str) -> Result<(), OpError>;
    async fn remove_flight_logs(&mut self) -> Result<CleanSummary, OpError>;
}

/// Parse `"<prefix>NNNN<suffix>"` (case-insensitive) into `NNNN`.
pub fn parse_seq(name: &str, prefix: &str, suffix: &str) -> Option<u32> {
    if name.len() < prefix.len() + suffix.len() {
        return None;
    }
    let (head, rest) = name.split_at(prefix.len());
    if !head.eq_ignore_ascii_case(prefix) {
        return None;
    }
    let (mid, tail) = rest.split_at(rest.len() - suffix.len());
    if !tail.eq_ignore_ascii_case(suffix) {
        return None;
    }
    mid.parse().ok()
}

/// True for recorder files (`flight_NNNN.mcap`, case-insensitive).
pub fn is_flight_log(name: &str) -> bool {
    parse_seq(name, "flight_", ".mcap").is_some()
}
