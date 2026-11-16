//! Blackbox / flight-data-recorder subsystem.
//!
//! Single capture path:
//!
//! - **`recorder`** — opens `flight_<NNNN>.mcap` on the rising edge
//!   of [`should_record`], runs a multi-topic capture loop until the
//!   predicate falls, closes cleanly. Triggered by either the real
//!   arm path (`motors::IS_ARMED`, set via RC arm switch / failsafe)
//!   or the bench path (`RECORDER_HOLD`, set by `blackbox record on`
//!   in the shell).
//!
//! ## Architecture
//!
//! ```text
//!   shell `blackbox record on` ─┐
//!                               ├─▶ blackbox_task ─▶ recorder
//!   IS_ARMED edge ──────────────┘                          │
//!                                                          ▼
//!                                  fat::write_file<P: FileBody>
//!                                                          │
//!                                  embedded-fatfs / partitions
//!                                                          │
//!                                  SdmmcBlockStore::open_session
//!                                                          │
//!                                  embassy-stm32 SDMMC
//! ```
//!
//! - `mcap`, `cbor`, `fat`, `sdmmc_block` are the layered building blocks.
//! - `topics::*` holds per-topic schema + CBOR encoder.
//! - `record_set` is the small/mid/large/none tier classifier.
//! - Shell file ops (`blackbox ls/get/rm`) reach the card through
//!   request/result Signals serviced by `blackbox_task` between
//!   recording sessions; `get` streams file bytes back through
//!   [`GET_STREAM`].
//! - The whole subsystem is gated on `bsp::HAS_BLACKBOX_STORAGE` (a
//!   compile-time `const bool`) — no Cargo feature flag.

pub mod cbor;
#[cfg(not(feature = "board_flywoof745"))]
pub mod fat;
#[cfg(not(feature = "board_flywoof745"))]
pub mod lazy_stream;
pub mod mcap;
#[cfg(feature = "board_flywoof745")]
pub mod nor_store;
pub mod record_set;
pub mod recorder;
#[cfg(not(feature = "board_flywoof745"))]
pub mod sdmmc_block;
pub mod topics;
pub mod types;

#[cfg(feature = "board_flywoof745")]
pub use nor_store::NorBlackboxStore;
pub use record_set::{RecordSet, current as record_set, set as set_record_set};
#[cfg(not(feature = "board_flywoof745"))]
pub use sdmmc_block::SdmmcBlockStore;
pub use types::{
    BlackboxBackend, CleanSummary, EntryName, EntrySummary, FileBody, FileSink, IoStats,
    MAX_FILENAME, NorStorageInfo, OpError, SinkAbort, is_flight_log, parse_seq,
};

#[cfg(feature = "board_flywoof745")]
pub type BoardBlackboxStore = NorBlackboxStore;
#[cfg(not(feature = "board_flywoof745"))]
pub type BoardBlackboxStore = SdmmcBlockStore;

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{Either, Either4, select, select4};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, with_timeout};

/// Manual record-toggle for bench testing. When `true`, the recorder
/// behaves as if `IS_ARMED` were `true` — opens a `flight_NNNN.mcap`
/// file and logs until either this flag clears or `IS_ARMED` is also
/// false. Set by the `blackbox record on/off` shell command.
///
/// This bypass exists so the recorder can be exercised on the bench
/// without touching `ARM_STATE` (which would also spin DShot output).
pub static RECORDER_HOLD: AtomicBool = AtomicBool::new(false);

/// True if the recorder should be capturing right now — `IS_ARMED`
/// from real flight OR the manual hold flag.
#[inline]
pub fn should_record() -> bool {
    crate::motors::IS_ARMED.load(Ordering::Acquire) || RECORDER_HOLD.load(Ordering::Acquire)
}

/// True while `blackbox_task` is inside `recorder::run_session`
/// (including the 3 s post-disarm grace window).
pub static SESSION_ACTIVE: AtomicBool = AtomicBool::new(false);

/// True whenever IMU blackbox mirrors should publish (`should_record`
/// or inside an active session's post-disarm grace window).
#[inline]
pub fn is_capturing() -> bool {
    should_record() || SESSION_ACTIVE.load(Ordering::Acquire)
}

// ─── Request / result signals ───────────────────────────────────────────

/// Raised by `blackbox ls`. Payload is unit; the request itself
/// carries no parameters (always lists the FAT root).
pub static LS_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// Outcome of one `blackbox ls` op.
pub static LS_RESULT: Signal<CriticalSectionRawMutex, LsReport> = Signal::new();

/// Maximum number of entries surfaced through one `LsReport::Ok`.
/// Beyond this we stop populating but keep counting, and set
/// `truncated = true` so the shell can warn the user.
pub const LS_MAX_ENTRIES: usize = 16;

/// Result of one `blackbox ls` op. `Busy` covers the recorder-active
/// case — concurrent SDMMC access from the shell while the recorder
/// owns the peripheral would race.
#[derive(Clone)]
pub enum LsReport {
    Ok {
        entries: heapless::Vec<EntrySummary, LS_MAX_ENTRIES>,
        /// True if the FAT root held more than `LS_MAX_ENTRIES`
        /// files; the surplus is silently dropped.
        truncated: bool,
        /// Total file count seen during the scan (not capped).
        total_files: u32,
    },
    Failed(OpError),
    Busy,
}

/// Raised by `blackbox get`. Names a root-directory file plus the
/// byte window to stream (`len = None` means "to EOF").
pub struct GetRequest {
    pub name: EntryName,
    pub offset: u32,
    pub len: Option<u32>,
}
pub static GET_REQUEST: Signal<CriticalSectionRawMutex, GetRequest> = Signal::new();

/// Header outcome of one `blackbox get` op — everything knowable
/// before the first data byte. `Ok` means the size header may be
/// sent and [`GET_STREAM`] chunks will follow; every other variant
/// terminates the op with no binary payload.
#[derive(Clone)]
pub enum GetReport {
    Ok { size: u32 },
    NotFound,
    Failed(OpError),
    Busy,
}
pub static GET_RESULT: Signal<CriticalSectionRawMutex, GetReport> = Signal::new();

/// Chunk payload size for [`GET_STREAM`]. Matches the FAT read
/// granularity (one BufStream block per chunk).
pub const GET_CHUNK_LEN: usize = 512;

/// One unit of `blackbox get` file flow, task → shell.
pub enum GetChunk {
    Data {
        buf: [u8; GET_CHUNK_LEN],
        len: u16,
    },
    /// Clean end of stream: CRC-32 of every streamed byte.
    End {
        crc32: u32,
    },
    /// Mid-stream fault; the transfer is dead and the shell should
    /// print the reason in-band (the size header is already out).
    Err(OpError),
}

/// File-content conduit for `blackbox get`. Depth 2 = double
/// buffering (the task reads block N+1 from the card while the shell
/// drains block N over USB); USB FS is the throughput bottleneck, so
/// deeper buys nothing. RAM cost ≈ 1.1 KB static.
pub static GET_STREAM: Channel<CriticalSectionRawMutex, GetChunk, 2> = Channel::new();

/// Bound on every `GET_STREAM.send` so a dead shell (host stopped
/// reading, USB unplugged) can never wedge `blackbox_task` — it also
/// services arm-edges and must return to its loop.
const GET_SEND_TIMEOUT: Duration = Duration::from_secs(5);

/// Raised by `blackbox rm`. Payload is the root-directory file name.
pub static RM_REQUEST: Signal<CriticalSectionRawMutex, EntryName> = Signal::new();

/// Outcome of one `blackbox rm` op.
#[derive(Clone)]
pub enum RmReport {
    Ok,
    NotFound,
    Failed(OpError),
    Busy,
}
pub static RM_RESULT: Signal<CriticalSectionRawMutex, RmReport> = Signal::new();

/// Raised by `blackbox clean`: delete every `flight_NNNN.mcap` in the
/// FAT root (nothing else on the card is touched).
pub static CLEAN_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Outcome of one `blackbox clean` op.
#[derive(Clone)]
pub enum CleanReport {
    Ok(CleanSummary),
    Failed(OpError),
    Busy,
}
pub static CLEAN_RESULT: Signal<CriticalSectionRawMutex, CleanReport> = Signal::new();

/// Outcome of the most recent recording session this boot, for
/// `blackbox status` (Stage 7: the user can see what the last session
/// did without a defmt console). `None` until the first session
/// closes. Written by `blackbox_task` right after `run_session`
/// returns; read synchronously by the shell.
#[derive(Clone, Copy)]
pub struct LastSession {
    /// Sequence number NNNN of `flight_NNNN.mcap`.
    pub seq: u32,
    pub bytes: u32,
    pub messages: u32,
    pub drops: u32,
    pub encode_overflows: u32,
    /// `None` = clean close. `Some(PartialWrite)` = mid-session fault
    /// but a truncated log was salvaged under this name. Any other
    /// `Some(e)` = session failed; counters above are 0 (the summary
    /// died with the session).
    pub error: Option<OpError>,
    /// Backend-level I/O counters for the session.
    pub io: IoStats,
}

pub static LAST_SESSION: Mutex<CriticalSectionRawMutex, Cell<Option<LastSession>>> =
    Mutex::new(Cell::new(None));

// ─── Arm-edge detection ─────────────────────────────────────────────────
//
// 20 ms poll cadence + 40 ms confirmation window matches the
// Betaflight blackbox debounce. Cheap, no extra channel needed:
// IS_ARMED is the established source of truth for the rest of the
// firmware.

const ARM_POLL_INTERVAL: Duration = Duration::from_millis(20);
const ARM_DEBOUNCE_POLLS: u8 = 2; // 2 × 20 ms = 40 ms

// ─── Task ───────────────────────────────────────────────────────────────

/// Owns the SDMMC peripheral for the duration of the firmware. Four
/// triggers wake it up:
///
/// 1. Arm-edge — runs `recorder::run_session` until disarm (or the
///    user clears `RECORDER_HOLD`).
/// 2. `LS_REQUEST` — scans the FAT root and returns an `LsReport`.
/// 3. `GET_REQUEST` — streams one file through `GET_STREAM`.
/// 4. `RM_REQUEST` — deletes one file.
/// 5. `CLEAN_REQUEST` — deletes every `flight_NNNN.mcap`.
///
/// All file ops are rejected with `Busy` if the recorder is
/// currently active; while a recording session runs the task is
/// blocked inside `run_session` and requests queue (then get dropped
/// as stale when the session ends).
#[embassy_executor::task]
pub async fn blackbox_task(mut store: BoardBlackboxStore) -> ! {
    defmt::info!("blackbox: ready");
    let mut prev_record = should_record();
    let mut debounce: u8 = 0;
    let mut flight_seq: Option<u32> = None;

    loop {
        match select4(
            Timer::after(ARM_POLL_INTERVAL),
            LS_REQUEST.wait(),
            GET_REQUEST.wait(),
            select(RM_REQUEST.wait(), CLEAN_REQUEST.wait()),
        )
        .await
        {
            Either4::First(()) => {
                let now_record = should_record();
                if now_record && !prev_record {
                    debounce = debounce.saturating_add(1);
                    if debounce >= ARM_DEBOUNCE_POLLS {
                        debounce = 0;
                        let rs = record_set::current();
                        if !rs.enabled() {
                            defmt::info!(
                                "blackbox: record-edge (record_set=none, session skipped)"
                            );
                        } else {
                            match next_seq(&mut store, &mut flight_seq).await {
                                Ok(seq) => {
                                    defmt::info!(
                                        "blackbox: record-edge → session {} (record_set={})",
                                        seq,
                                        rs.name(),
                                    );
                                    SESSION_ACTIVE.store(true, Ordering::Release);
                                    let outcome = recorder::run_session(&mut store, seq, rs).await;
                                    SESSION_ACTIVE.store(false, Ordering::Release);
                                    let io = store.last_io_stats();
                                    defmt::info!(
                                        "blackbox: io: {} writes / {} blocks (max batch {}), \
                                         {} reads / {} blocks, {} fallbacks",
                                        io.writes,
                                        io.blocks_written,
                                        io.max_batch,
                                        io.reads,
                                        io.blocks_read,
                                        io.fallbacks,
                                    );
                                    match outcome {
                                        Ok(s) => {
                                            defmt::info!(
                                                "blackbox: closed /{} ({} bytes, {} msgs, \
                                                 {} drops, {} enc-overflows)",
                                                s.file_name.as_str(),
                                                s.bytes,
                                                s.messages,
                                                s.drops,
                                                s.encode_overflows,
                                            );
                                            LAST_SESSION.lock(|c| {
                                                c.set(Some(LastSession {
                                                    seq,
                                                    bytes: s.bytes,
                                                    messages: s.messages,
                                                    drops: s.drops,
                                                    encode_overflows: s.encode_overflows,
                                                    error: None,
                                                    io,
                                                }))
                                            });
                                        }
                                        Err(OpError::PartialWrite) => {
                                            defmt::warn!(
                                                "blackbox: session {} faulted; \
                                                 partial log kept (seq not recycled)",
                                                seq,
                                            );
                                            LAST_SESSION.lock(|c| {
                                                c.set(Some(LastSession {
                                                    seq,
                                                    bytes: 0,
                                                    messages: 0,
                                                    drops: 0,
                                                    encode_overflows: 0,
                                                    error: Some(OpError::PartialWrite),
                                                    io,
                                                }))
                                            });
                                        }
                                        Err(e) => {
                                            defmt::warn!("blackbox: session failed: {:?}", e);
                                            LAST_SESSION.lock(|c| {
                                                c.set(Some(LastSession {
                                                    seq,
                                                    bytes: 0,
                                                    messages: 0,
                                                    drops: 0,
                                                    encode_overflows: 0,
                                                    error: Some(e),
                                                    io,
                                                }))
                                            });
                                            if let Some(s) = flight_seq.as_mut() {
                                                *s = s.saturating_sub(1);
                                            }
                                        }
                                    }
                                }
                                Err(e) => {
                                    defmt::warn!("blackbox: seq scan failed: {:?}", e)
                                }
                            }
                        }
                        prev_record = should_record();
                        let _ = LS_REQUEST.try_take();
                        let _ = GET_REQUEST.try_take();
                        let _ = RM_REQUEST.try_take();
                        let _ = CLEAN_REQUEST.try_take();
                    }
                } else {
                    debounce = 0;
                    prev_record = now_record;
                    if !now_record && !crate::motors::IS_ARMED.load(Ordering::Acquire) {
                        store.idle_maintenance().await;
                    }
                }
            }
            Either4::Second(()) => {
                if should_record() {
                    defmt::warn!("blackbox: ls rejected (recorder active)");
                    LS_RESULT.signal(LsReport::Busy);
                    continue;
                }
                LS_RESULT.signal(run_ls(&mut store).await);
            }
            Either4::Third(req) => {
                if should_record() {
                    defmt::warn!("blackbox: get rejected (recorder active)");
                    GET_RESULT.signal(GetReport::Busy);
                    continue;
                }
                run_get(&mut store, &req).await;
            }
            Either4::Fourth(Either::First(name)) => {
                if should_record() {
                    defmt::warn!("blackbox: rm rejected (recorder active)");
                    RM_RESULT.signal(RmReport::Busy);
                    continue;
                }
                RM_RESULT.signal(run_rm(&mut store, name.as_str()).await);
            }
            Either4::Fourth(Either::Second(())) => {
                if should_record() {
                    defmt::warn!("blackbox: clean rejected (recorder active)");
                    CLEAN_RESULT.signal(CleanReport::Busy);
                    continue;
                }
                let report = match store.remove_flight_logs().await {
                    Ok(s) => CleanReport::Ok(s),
                    Err(e) => CleanReport::Failed(e),
                };
                if matches!(report, CleanReport::Ok(_)) {
                    flight_seq = None;
                }
                CLEAN_RESULT.signal(report);
            }
        }
    }
}

async fn next_seq<S: BlackboxBackend>(
    store: &mut S,
    cache: &mut Option<u32>,
) -> Result<u32, OpError> {
    let highest = match *cache {
        Some(h) => h,
        None => store.scan_highest_seq("flight_", ".mcap").await?,
    };
    let next = highest.saturating_add(1);
    *cache = Some(next);
    Ok(next)
}

async fn run_ls<S: BlackboxBackend>(store: &mut S) -> LsReport {
    let mut entries: heapless::Vec<EntrySummary, LS_MAX_ENTRIES> = heapless::Vec::new();
    let mut total: u32 = 0;
    let mut truncated = false;
    let mut visit = |name: &str, size: u32| {
        total = total.saturating_add(1);
        if entries.is_full() {
            truncated = true;
            return;
        }
        let mut n: EntryName = heapless::String::new();
        for c in name.chars() {
            if n.push(c).is_err() {
                break;
            }
        }
        let _ = entries.push(EntrySummary { name: n, size });
    };
    match store.scan_root_files(&mut visit).await {
        Ok(()) => LsReport::Ok {
            entries,
            truncated,
            total_files: total,
        },
        Err(e) => LsReport::Failed(e),
    }
}

/// [`FileSink`] that forwards a file to the shell through
/// [`GET_RESULT`] (size header) and [`GET_STREAM`] (content), CRC-ing
/// every byte on the way past.
struct ShellGetSink {
    crc: cybflight_core::crc32::Hasher,
    started: bool,
}

impl FileSink for ShellGetSink {
    async fn begin(&mut self, total: u32) -> Result<(), SinkAbort> {
        self.started = true;
        GET_RESULT.signal(GetReport::Ok { size: total });
        Ok(())
    }

    async fn data(&mut self, chunk: &[u8]) -> Result<(), SinkAbort> {
        if crate::motors::IS_ARMED.load(Ordering::Acquire) {
            defmt::warn!("blackbox: get aborted (armed mid-transfer)");
            return Err(SinkAbort);
        }
        self.crc.update(chunk);
        let mut buf = [0u8; GET_CHUNK_LEN];
        let len = chunk.len().min(GET_CHUNK_LEN);
        buf[..len].copy_from_slice(&chunk[..len]);
        with_timeout(
            GET_SEND_TIMEOUT,
            GET_STREAM.send(GetChunk::Data {
                buf,
                len: len as u16,
            }),
        )
        .await
        .map_err(|_| SinkAbort)
    }
}

async fn run_get<S: BlackboxBackend>(store: &mut S, req: &GetRequest) {
    while GET_STREAM.try_receive().is_ok() {}
    let mut sink = ShellGetSink {
        crc: cybflight_core::crc32::Hasher::new(),
        started: false,
    };
    match store.read_file(req.name.as_str(), req.offset, req.len, &mut sink).await {
        Ok(_) => {
            let crc32 = sink.crc.finalize();
            let _ = with_timeout(GET_SEND_TIMEOUT, GET_STREAM.send(GetChunk::End { crc32 })).await;
        }
        Err(e) if sink.started => {
            let _ = GET_STREAM.try_send(GetChunk::Err(e));
        }
        Err(OpError::NotFound) => GET_RESULT.signal(GetReport::NotFound),
        Err(e) => GET_RESULT.signal(GetReport::Failed(e)),
    }
}

async fn run_rm<S: BlackboxBackend>(store: &mut S, path: &str) -> RmReport {
    match store.remove_file(path).await {
        Ok(()) => RmReport::Ok,
        Err(OpError::NotFound) => RmReport::NotFound,
        Err(e) => RmReport::Failed(e),
    }
}
