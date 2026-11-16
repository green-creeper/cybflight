//! Flash-native, zero-in-flight-erase SPI-NOR blackbox store (`W25Q128` on `SPI1`).
//!
//! ## Flash Layout (`16 MiB` = `256 × 64 KiB` blocks)
//! - Block `0` (`0x0000_0000..0x0001_0000`, `256` pages):
//!   - Page `0`: Volume header (`CYBBNOR1`, version, capacity, CRC-32) — written once on format.
//!   - Pages `1..=16`: Superblock clean markers (`CYBC`), one `256 B` page per `1 MiB` (`16 × 64 KiB`)
//!     chunk — programmed **once** per chunk (`NOP = 1`) once all blocks in that chunk are verified erased.
//!   - Pages `17..=253`: `79` file slots × `3` dedicated pages (`OpenPage`, `ClosePage`, `TombPage`),
//!     each programmed at most **once** (`NOP = 1`) — zero multi-writes per page.
//! - Blocks `1..255` (`0x0001_0000..0x0100_0000`): `4 KiB`-aligned contiguous `.mcap` payloads.
//!
//! ## Real-Time Safety
//! - `idle_maintenance()` formats unformatted parts and pre-erases one `64 KiB` block per tick
//!   only while disarmed and idle (`!IS_ARMED && !should_record()`), yielding every `5 ms`.
//! - During armed flight, `NorMcapWriter` performs **zero sector/block erases** and writes
//!   exclusively to pre-erased `256 B` pages over preemptible blocking SPI.

use core::cell::Cell;

use cybflight_core::crc32::Hasher;
use cybflight_drivers::w25q128::{BLOCK64_SIZE, FlashError, PAGE_SIZE, SECTOR_SIZE, W25q128};
use embassy_futures::yield_now;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant};
use embedded_io_async::Write;
use static_cell::StaticCell;

use super::mcap;
use super::types::{
    BlackboxBackend, CleanSummary, EntryName, FileBody, FileSink, IoStats, NorStorageInfo,
    OpError, parse_seq,
};
use crate::hal::gpio::Output;
use crate::hal::mode::Blocking;
use crate::hal::spi::{Spi, mode::Master};

const VOL_MAGIC: [u8; 8] = *b"CYBBNOR1";
const VOL_VERSION: u32 = 1;
const CHUNK_MAGIC: [u8; 4] = *b"CYBC";
const OPEN_MAGIC: [u8; 4] = *b"CYBO";
const CLOSE_MAGIC: [u8; 4] = *b"CYBZ";
const TOMB_MAGIC: [u8; 4] = *b"CYBT";

const DATA_START_OFFSET: u32 = BLOCK64_SIZE as u32;
const BLOCKS_PER_CHUNK: usize = 16;
const MAX_CHUNKS: usize = 16;
const CHUNK_PAGE_BASE: u32 = 1;
const SLOT_PAGE_BASE: u32 = 17;
const MAX_SLOTS: usize = 79;

const TRAILER_RESERVE_BYTES: u32 = 1024;
const MIN_SESSION_BYTES: u32 = 16 * 1024;
const FLAG_PARTIAL: u32 = 1;
const RING_PAGES: usize = 16; // 4 KiB burst buffer (absorbs full MCAP schema header)

/// Live snapshot of onboard NOR flash state for `blackbox status`.
pub static NOR_STORAGE_INFO: Mutex<CriticalSectionRawMutex, Cell<NorStorageInfo>> =
    Mutex::new(Cell::new(NorStorageInfo {
        jedec: [0; 3],
        capacity_kib: 0,
        used_kib: 0,
        ready_kib: 0,
        files: 0,
        formatting: false,
    }));

type NorDriver = W25q128<Spi<'static, Blocking, Master>, Output<'static>, Delay>;

/// Log-structured MCAP backend over `W25Q128` SPI-NOR flash.
pub struct NorBlackboxStore {
    flash: NorDriver,
    ring: &'static mut [[u8; PAGE_SIZE]; RING_PAGES],
    initialized: bool,
    formatted: bool,
    jedec: [u8; 3],
    capacity_bytes: u32,
    total_blocks: usize,
    num_chunks: usize,
    chunk_clean: [bool; MAX_CHUNKS],
    next_slot: usize,
    live_files: u16,
    write_head: u32,
    erased_end: u32,
    next_clean_block: usize,
    last_stats: IoStats,
}

#[derive(Clone, Copy)]
enum SlotState {
    Empty,
    Unclosed {
        data_offset: u32,
    },
    Closed {
        name_len: u8,
        name: [u8; 32],
        data_offset: u32,
        byte_len: u32,
        deleted: bool,
    },
    Corrupt,
}

impl NorBlackboxStore {
    pub fn new(spi: Spi<'static, Blocking, Master>, cs: Output<'static>) -> Self {
        static RING_BUF: StaticCell<[[u8; PAGE_SIZE]; RING_PAGES]> = StaticCell::new();
        let ring = RING_BUF.init([[0u8; PAGE_SIZE]; RING_PAGES]);
        Self {
            flash: W25q128::new(spi, cs, Delay),
            ring,
            initialized: false,
            formatted: false,
            jedec: [0; 3],
            capacity_bytes: 0,
            total_blocks: 0,
            num_chunks: 0,
            chunk_clean: [false; MAX_CHUNKS],
            next_slot: 0,
            live_files: 0,
            write_head: DATA_START_OFFSET,
            erased_end: DATA_START_OFFSET,
            next_clean_block: 1,
            last_stats: IoStats::default(),
        }
    }

    fn publish_info(&self) {
        let used_bytes = self.write_head.saturating_sub(DATA_START_OFFSET);
        let ready_bytes = self.erased_end.saturating_sub(self.write_head);
        let info = NorStorageInfo {
            jedec: self.jedec,
            capacity_kib: self.capacity_bytes / 1024,
            used_kib: used_bytes / 1024,
            ready_kib: ready_bytes / 1024,
            files: self.live_files,
            formatting: self.initialized && (!self.formatted || self.next_clean_block < self.total_blocks),
        };
        NOR_STORAGE_INFO.lock(|c| c.set(info));
    }

    async fn ensure_hardware_init(&mut self) -> Result<(), OpError> {
        if self.initialized {
            return Ok(());
        }
        let jedec = self.flash.init().await.map_err(|e| {
            defmt::warn!("blackbox/nor: W25Q128 init failed: {:?}", e);
            OpError::StorageInit
        })?;
        self.jedec = jedec.as_bytes();
        self.capacity_bytes = self.flash.capacity_bytes();
        self.total_blocks = ((self.capacity_bytes as usize) / BLOCK64_SIZE).min(256);
        self.num_chunks = self.total_blocks.div_ceil(BLOCKS_PER_CHUNK).min(MAX_CHUNKS);
        self.initialized = true;
        defmt::info!(
            "blackbox/nor: JEDEC {:02x}:{:02x}:{:02x} ({} KiB, {} blocks)",
            self.jedec[0],
            self.jedec[1],
            self.jedec[2],
            self.capacity_bytes / 1024,
            self.total_blocks,
        );
        self.scan_volume().await?;
        self.publish_info();
        Ok(())
    }

    async fn ensure_ready_disarmed(&mut self) -> Result<(), OpError> {
        self.ensure_hardware_init().await?;
        if !self.formatted {
            if is_armed() {
                return Err(OpError::StorageInit);
            }
            self.format_initial().await?;
        }
        Ok(())
    }

    async fn format_initial(&mut self) -> Result<(), OpError> {
        defmt::info!("blackbox/nor: formatting volume (Block 0 + Block 1)");
        self.flash.erase_block_64k(0).await.map_err(|_| OpError::StorageInit)?;
        if is_armed() {
            return Err(OpError::Aborted);
        }
        self.flash
            .erase_block_64k(DATA_START_OFFSET)
            .await
            .map_err(|_| OpError::StorageInit)?;

        self.write_volume_header().await?;

        self.formatted = true;
        self.chunk_clean = [false; MAX_CHUNKS];
        self.next_slot = 0;
        self.live_files = 0;
        self.write_head = DATA_START_OFFSET;
        self.erased_end = DATA_START_OFFSET + BLOCK64_SIZE as u32;
        self.next_clean_block = 2.min(self.total_blocks);
        self.publish_info();
        defmt::info!("blackbox/nor: format complete (64 KiB immediately ready)");
        Ok(())
    }

    async fn write_volume_header(&mut self) -> Result<(), OpError> {
        let mut hdr = [0u8; 24];
        hdr[0..8].copy_from_slice(&VOL_MAGIC);
        hdr[8..12].copy_from_slice(&VOL_VERSION.to_le_bytes());
        hdr[12..16].copy_from_slice(&self.capacity_bytes.to_le_bytes());
        hdr[16..20].copy_from_slice(&(self.total_blocks as u32).to_le_bytes());
        let crc = crc32_of(&hdr[0..20]);
        hdr[20..24].copy_from_slice(&crc.to_le_bytes());
        self.flash
            .write_page_slice(0, &hdr)
            .await
            .map_err(|_| OpError::Write)?;
        self.flash.flush().await.map_err(|_| OpError::Write)
    }

    async fn write_chunk_marker(&mut self, chunk_idx: usize) -> Result<(), OpError> {
        let mut buf = [0u8; 12];
        buf[0..4].copy_from_slice(&CHUNK_MAGIC);
        buf[4..8].copy_from_slice(&(chunk_idx as u32).to_le_bytes());
        let crc = crc32_of(&buf[0..8]);
        buf[8..12].copy_from_slice(&crc.to_le_bytes());
        let addr = (CHUNK_PAGE_BASE + chunk_idx as u32) * PAGE_SIZE as u32;
        self.flash
            .write_page_slice(addr, &buf)
            .await
            .map_err(|_| OpError::Write)?;
        self.flash.flush().await.map_err(|_| OpError::Write)
    }

    async fn scan_volume(&mut self) -> Result<(), OpError> {
        let mut hdr = [0u8; 24];
        self.flash.read_data(0, &mut hdr).await.map_err(|_| OpError::Read)?;
        if &hdr[0..8] != &VOL_MAGIC
            || u32::from_le_bytes(hdr[8..12].try_into().unwrap()) != VOL_VERSION
            || u32::from_le_bytes(hdr[12..16].try_into().unwrap()) != self.capacity_bytes
            || crc32_of(&hdr[0..20]) != u32::from_le_bytes(hdr[20..24].try_into().unwrap())
        {
            self.formatted = false;
            self.chunk_clean = [false; MAX_CHUNKS];
            self.next_slot = 0;
            self.live_files = 0;
            self.write_head = DATA_START_OFFSET;
            self.erased_end = DATA_START_OFFSET;
            self.next_clean_block = 1;
            return Ok(());
        }

        self.formatted = true;
        self.chunk_clean = [false; MAX_CHUNKS];
        for c in 0..self.num_chunks {
            let mut m = [0u8; 12];
            let addr = (CHUNK_PAGE_BASE + c as u32) * PAGE_SIZE as u32;
            self.flash.read_data(addr, &mut m).await.map_err(|_| OpError::Read)?;
            let expected = crc32_of(&m[0..8]);
            if &m[0..4] == &CHUNK_MAGIC
                && u32::from_le_bytes(m[4..8].try_into().unwrap()) == c as u32
                && expected == u32::from_le_bytes(m[8..12].try_into().unwrap())
            {
                self.chunk_clean[c] = true;
            }
        }

        self.write_head = DATA_START_OFFSET;
        self.next_slot = 0;
        self.live_files = 0;
        let mut last_unclosed_offset: Option<u32> = None;

        for slot in 0..MAX_SLOTS {
            match self.read_slot(slot).await? {
                SlotState::Empty => break,
                SlotState::Closed {
                    data_offset,
                    byte_len,
                    deleted,
                    ..
                } => {
                    self.next_slot = slot + 1;
                    let end = align_up_4k(data_offset.saturating_add(byte_len));
                    if end > self.write_head {
                        self.write_head = end.min(self.capacity_bytes);
                    }
                    if !deleted {
                        self.live_files += 1;
                    }
                    last_unclosed_offset = None;
                }
                SlotState::Unclosed { data_offset } => {
                    self.next_slot = slot + 1;
                    if data_offset > self.write_head {
                        self.write_head = data_offset;
                    }
                    last_unclosed_offset = Some(data_offset);
                }
                SlotState::Corrupt => {
                    self.next_slot = slot + 1;
                }
            }
        }

        // Advance `write_head` past any non-erased 4 KiB sectors from an unclosed crash session.
        if let Some(off) = last_unclosed_offset {
            let scan_from = off.max(self.write_head);
            let clean_pos = self.find_first_erased_sector(scan_from).await?;
            self.write_head = clean_pos.max(self.write_head);
        }

        self.recompute_erased_frontier().await?;
        Ok(())
    }

    async fn recompute_erased_frontier(&mut self) -> Result<(), OpError> {
        let mut head = self.write_head.min(self.capacity_bytes);
        let head_block = (head as usize) / BLOCK64_SIZE;
        let head_chunk = head_block / BLOCKS_PER_CHUNK;

        let mut contiguous = head;
        let mut next_block = self.total_blocks;

        if head_chunk < self.num_chunks {
            if self.chunk_clean[head_chunk] {
                contiguous = ((head_chunk + 1) * BLOCKS_PER_CHUNK * BLOCK64_SIZE) as u32;
                contiguous = contiguous.min(self.capacity_bytes);
                let mut c = head_chunk + 1;
                while c < self.num_chunks && self.chunk_clean[c] {
                    contiguous = (((c + 1) * BLOCKS_PER_CHUNK * BLOCK64_SIZE) as u32)
                        .min(self.capacity_bytes);
                    c += 1;
                }
                next_block = (c * BLOCKS_PER_CHUNK).min(self.total_blocks);
            } else {
                // Partially written/erased chunk: check if the tail of `head_block`
                // after `head` is already erased so small hops don't wait to erase `head_block`.
                let block_end = (((head_block + 1) * BLOCK64_SIZE) as u32).min(self.capacity_bytes);
                if (head as usize) % BLOCK64_SIZE != 0
                    && head < block_end
                    && self.range_is_erased(head, block_end).await?
                {
                    contiguous = block_end;
                    next_block = (head_block + 1).min(self.total_blocks);
                } else {
                    next_block = head.div_ceil(BLOCK64_SIZE as u32) as usize;
                    if next_block >= self.total_blocks {
                        next_block = self.total_blocks;
                    }
                    head = contiguous;
                }
            }
        }

        self.erased_end = contiguous.max(head);
        self.next_clean_block = next_block.max(head.div_ceil(BLOCK64_SIZE as u32) as usize);
        Ok(())
    }

    async fn read_slot(&mut self, slot: usize) -> Result<SlotState, OpError> {
        let base_page = SLOT_PAGE_BASE + (slot as u32) * 3;
        let mut open = [0u8; 48];
        self.flash
            .read_data(base_page * PAGE_SIZE as u32, &mut open)
            .await
            .map_err(|_| OpError::Read)?;
        if open[0..4] == [0xFF; 4] {
            return Ok(SlotState::Empty);
        }
        if &open[0..4] != &OPEN_MAGIC
            || crc32_of(&open[0..44]) != u32::from_le_bytes(open[44..48].try_into().unwrap())
        {
            return Ok(SlotState::Corrupt);
        }
        let name_len = open[4].min(32);
        let data_offset = u32::from_le_bytes(open[8..12].try_into().unwrap());
        let mut name = [0u8; 32];
        name.copy_from_slice(&open[12..44]);

        let mut close = [0u8; 16];
        self.flash
            .read_data((base_page + 1) * PAGE_SIZE as u32, &mut close)
            .await
            .map_err(|_| OpError::Read)?;
        if close[0..4] == [0xFF; 4] {
            return Ok(SlotState::Unclosed { data_offset });
        }
        if &close[0..4] != &CLOSE_MAGIC
            || crc32_of(&close[0..12]) != u32::from_le_bytes(close[12..16].try_into().unwrap())
        {
            return Ok(SlotState::Unclosed { data_offset });
        }
        let byte_len = u32::from_le_bytes(close[4..8].try_into().unwrap());

        let mut tomb = [0u8; 8];
        self.flash
            .read_data((base_page + 2) * PAGE_SIZE as u32, &mut tomb)
            .await
            .map_err(|_| OpError::Read)?;
        let deleted = &tomb[0..4] == &TOMB_MAGIC
            && crc32_of(&tomb[0..4]) == u32::from_le_bytes(tomb[4..8].try_into().unwrap());

        Ok(SlotState::Closed {
            name_len,
            name,
            data_offset,
            byte_len,
            deleted,
        })
    }

    async fn range_is_erased(&mut self, start: u32, end: u32) -> Result<bool, OpError> {
        let mut buf = [0u8; 512];
        let mut pos = start;
        let mut reads = 0u32;
        while pos < end {
            let chunk = (end - pos).min(buf.len() as u32) as usize;
            self.flash
                .read_data(pos, &mut buf[..chunk])
                .await
                .map_err(|_| OpError::Read)?;
            if buf[..chunk].iter().any(|&b| b != 0xFF) {
                return Ok(false);
            }
            pos += chunk as u32;
            reads += 1;
            if reads & 15 == 0 {
                yield_now().await;
            }
        }
        Ok(true)
    }

    async fn find_first_erased_sector(&mut self, start: u32) -> Result<u32, OpError> {
        let mut pos = align_up_4k(start);
        while pos + SECTOR_SIZE as u32 <= self.capacity_bytes {
            if self.range_is_erased(pos, pos + SECTOR_SIZE as u32).await? {
                return Ok(pos);
            }
            pos += SECTOR_SIZE as u32;
        }
        Ok(self.capacity_bytes)
    }

    async fn find_live_slot(&mut self, path: &str) -> Result<Option<(usize, u32, u32)>, OpError> {
        let target = path.trim_start_matches('/');
        for slot in 0..self.next_slot {
            if let SlotState::Closed {
                name_len,
                name,
                data_offset,
                byte_len,
                deleted,
            } = self.read_slot(slot).await?
            {
                if deleted {
                    continue;
                }
                if let Ok(s) = core::str::from_utf8(&name[..name_len as usize]) {
                    if s.eq_ignore_ascii_case(target) {
                        return Ok(Some((slot, data_offset, byte_len)));
                    }
                }
            }
        }
        Ok(None)
    }

    async fn reclaim_all_disarmed(&mut self) -> Result<(), OpError> {
        if is_armed() {
            return Err(OpError::Aborted);
        }
        let dirty_blocks = self.write_head.div_ceil(BLOCK64_SIZE as u32).max(1) as usize;
        let dirty_blocks = dirty_blocks.min(self.total_blocks);

        let old_chunk_clean = self.chunk_clean;
        let old_erased_blocks = (self.erased_end as usize) / BLOCK64_SIZE;

        // Erase Block 0 (metadata) + dirty data blocks 1..dirty_blocks.
        self.flash.erase_block_64k(0).await.map_err(|_| OpError::Remove)?;
        for b in 1..dirty_blocks {
            if is_armed() {
                // Safe partial state: Block 0 is erased so next mount will auto-format cleanly.
                self.formatted = false;
                self.publish_info();
                return Err(OpError::Aborted);
            }
            self.flash
                .erase_block_64k((b * BLOCK64_SIZE) as u32)
                .await
                .map_err(|_| OpError::Remove)?;
        }

        self.write_volume_header().await?;

        self.formatted = true;
        self.next_slot = 0;
        self.live_files = 0;
        self.write_head = DATA_START_OFFSET;
        self.chunk_clean = [false; MAX_CHUNKS];

        // Preserve any untouched clean 1 MiB chunks (or chunks newly made clean by erasing `1..dirty_blocks`).
        for c in 0..self.num_chunks {
            let c_start = (c * BLOCKS_PER_CHUNK).max(1);
            let c_end = ((c + 1) * BLOCKS_PER_CHUNK).min(self.total_blocks);
            let clean = old_chunk_clean[c]
                || c_end <= dirty_blocks
                || c_end <= old_erased_blocks;
            if clean && c_start < self.total_blocks {
                self.write_chunk_marker(c).await?;
                self.chunk_clean[c] = true;
            }
        }

        // Compute contiguous erased frontier and resume background verification where needed.
        let mut c = 0usize;
        while c < self.num_chunks && self.chunk_clean[c] {
            c += 1;
        }
        let clean_up_to_blocks = (c * BLOCKS_PER_CHUNK)
            .max(dirty_blocks)
            .max(old_erased_blocks)
            .min(self.total_blocks);
        self.erased_end = ((clean_up_to_blocks * BLOCK64_SIZE) as u32).max(DATA_START_OFFSET);
        self.next_clean_block = clean_up_to_blocks.max(1);
        self.publish_info();
        Ok(())
    }

    #[inline]
    async fn flush_flash_coop(&mut self) -> Result<(), FlashError> {
        wait_flash_ready_coop(&mut self.flash).await
    }
}

async fn wait_flash_ready_coop(flash: &mut NorDriver) -> Result<(), FlashError> {
    let start = Instant::now();
    while !flash.poll_ready()? {
        if start.elapsed() > Duration::from_millis(20) {
            return Err(FlashError::Timeout);
        }
        yield_now().await;
    }
    Ok(())
}

impl BlackboxBackend for NorBlackboxStore {
    async fn idle_maintenance(&mut self) {
        if is_armed() {
            return;
        }
        if !self.initialized && self.ensure_hardware_init().await.is_err() {
            self.publish_info();
            return;
        }
        if !self.formatted {
            let _ = self.format_initial().await;
            return;
        }
        if self.next_clean_block >= self.total_blocks {
            return;
        }

        let b = self.next_clean_block;
        let addr = (b * BLOCK64_SIZE) as u32;
        let end = (addr + BLOCK64_SIZE as u32).min(self.capacity_bytes);
        match self.range_is_erased(addr, end).await {
            Ok(true) => {}
            Ok(false) => {
                if self.flash.erase_block_64k(addr).await.is_err() {
                    return;
                }
            }
            Err(_) => return,
        }

        self.next_clean_block = b + 1;
        if end > self.erased_end {
            self.erased_end = end;
        }

        let c = b / BLOCKS_PER_CHUNK;
        let chunk_end = (((c + 1) * BLOCKS_PER_CHUNK).min(self.total_blocks) * BLOCK64_SIZE) as u32;
        if c < self.num_chunks && !self.chunk_clean[c] && self.erased_end >= chunk_end {
            if self.write_chunk_marker(c).await.is_ok() {
                self.chunk_clean[c] = true;
            }
        }
        self.publish_info();
    }

    async fn scan_highest_seq(&mut self, prefix: &str, suffix: &str) -> Result<u32, OpError> {
        self.ensure_ready_disarmed().await?;
        let mut highest = 0u32;
        self.scan_root_files(&mut |name, _| {
            if let Some(n) = parse_seq(name, prefix, suffix) {
                if n > highest {
                    highest = n;
                }
            }
        })
        .await?;
        Ok(highest)
    }

    async fn write_session<P: FileBody>(
        &mut self,
        path: &str,
        body: &mut P,
    ) -> Result<u32, OpError> {
        self.ensure_ready_disarmed().await?;
        self.last_stats = IoStats::default();

        if self.next_slot >= MAX_SLOTS {
            return Err(OpError::DiskFull);
        }
        let avail = self.erased_end.saturating_sub(self.write_head);
        if avail < MIN_SESSION_BYTES {
            return Err(OpError::DiskFull);
        }

        let slot = self.next_slot;
        let data_offset = self.write_head;
        let base_page = SLOT_PAGE_BASE + (slot as u32) * 3;

        // Commit OpenPage before streaming so crashes never lose the allocation frontier.
        let clean_name = path.trim_start_matches('/');
        let name_bytes = clean_name.as_bytes();
        let name_len = name_bytes.len().min(32);
        let mut open = [0u8; 48];
        open[0..4].copy_from_slice(&OPEN_MAGIC);
        open[4] = name_len as u8;
        open[8..12].copy_from_slice(&data_offset.to_le_bytes());
        open[12..12 + name_len].copy_from_slice(&name_bytes[..name_len]);
        let open_crc = crc32_of(&open[0..44]);
        open[44..48].copy_from_slice(&open_crc.to_le_bytes());
        self.flash
            .write_page_slice(base_page * PAGE_SIZE as u32, &open)
            .await
            .map_err(|_| OpError::Create)?;
        self.flush_flash_coop().await.map_err(|_| OpError::Create)?;

        self.next_slot = slot + 1;
        let mut stats = IoStats::default();
        stats.writes = 1;

        let soft_limit = avail - TRAILER_RESERVE_BYTES;
        let mut writer = NorMcapWriter::new(
            &mut self.flash,
            &mut self.ring,
            data_offset,
            soft_limit,
            avail,
            &mut stats,
        );

        let body_res = body.write(&mut writer).await;
        let (final_bytes, partial, io_err) = match body_res {
            Ok(n) => {
                let tail_res = writer.finish_final_page().await;
                if let Err(_) = tail_res {
                    (n, true, Some(OpError::Write))
                } else {
                    (writer.bytes_written(), false, None)
                }
            }
            Err(NorWriteError::DiskFull) => {
                // Clean record boundary before `DiskFull`: unlock the 1 KiB reserve and append
                // DataEnd + Footer + Magic so the partial log passes `mcap doctor`.
                writer.unlock_trailer();
                let trailer_ok = async {
                    mcap::write_data_end(&mut writer, 0).await?;
                    mcap::write_footer(&mut writer, 0, 0, 0).await?;
                    mcap::write_magic(&mut writer).await
                }
                .await
                .is_ok();
                let finish_ok = writer.finish_final_page().await.is_ok();
                let n = writer.bytes_written();
                if trailer_ok && finish_ok && n > 0 {
                    (n, true, Some(OpError::PartialWrite))
                } else {
                    (n, true, Some(OpError::Write))
                }
            }
            Err(NorWriteError::Io) => {
                let n = writer.bytes_written();
                let _ = self.flash.flush().await;
                (n, true, Some(OpError::Write))
            }
        };

        // Seal ClosePage whenever bytes landed cleanly (including salvaged PartialWrite).
        if io_err.is_none() || matches!(io_err, Some(OpError::PartialWrite)) {
            let mut close = [0u8; 16];
            close[0..4].copy_from_slice(&CLOSE_MAGIC);
            close[4..8].copy_from_slice(&final_bytes.to_le_bytes());
            close[8..12].copy_from_slice(&(if partial { FLAG_PARTIAL } else { 0u32 }).to_le_bytes());
            let close_crc = crc32_of(&close[0..12]);
            close[12..16].copy_from_slice(&close_crc.to_le_bytes());
            if self
                .flash
                .write_page_slice((base_page + 1) * PAGE_SIZE as u32, &close)
                .await
                .is_ok()
                && self.flush_flash_coop().await.is_ok()
            {
                stats.writes += 1;
                self.live_files += 1;
            }
        }

        self.write_head = align_up_4k(data_offset.saturating_add(final_bytes.max(1)))
            .min(self.capacity_bytes);
        if self.write_head > self.erased_end {
            self.erased_end = self.write_head;
        }
        self.next_clean_block = self
            .next_clean_block
            .max(self.write_head.div_ceil(BLOCK64_SIZE as u32) as usize);
        self.last_stats = stats;
        self.publish_info();

        match io_err {
            None => Ok(final_bytes),
            Some(e) => Err(e),
        }
    }

    fn last_io_stats(&self) -> IoStats {
        self.last_stats
    }

    async fn scan_root_files<F>(&mut self, visit: &mut F) -> Result<(), OpError>
    where
        F: FnMut(&str, u32),
    {
        self.ensure_hardware_init().await?;
        if !self.formatted {
            return Ok(());
        }
        for slot in 0..self.next_slot {
            if let SlotState::Closed {
                name_len,
                name,
                byte_len,
                deleted,
                ..
            } = self.read_slot(slot).await?
            {
                if deleted {
                    continue;
                }
                let mut entry: EntryName = heapless::String::new();
                for &b in &name[..name_len as usize] {
                    let c = if b.is_ascii() { b as char } else { '?' };
                    if entry.push(c).is_err() {
                        break;
                    }
                }
                visit(entry.as_str(), byte_len);
            }
        }
        Ok(())
    }

    async fn read_file<S: FileSink>(
        &mut self,
        path: &str,
        offset: u32,
        max_len: Option<u32>,
        sink: &mut S,
    ) -> Result<u32, OpError> {
        self.ensure_hardware_init().await?;
        if !self.formatted {
            return Err(OpError::NotFound);
        }
        let Some((_, data_offset, byte_len)) = self.find_live_slot(path).await? else {
            return Err(OpError::NotFound);
        };

        let clamped_off = offset.min(byte_len);
        let avail = byte_len - clamped_off;
        let to_send = max_len.map_or(avail, |m| m.min(avail));

        sink.begin(to_send).await.map_err(|_| OpError::Aborted)?;
        let mut buf = [0u8; 512];
        let mut sent = 0u32;
        while sent < to_send {
            let chunk = (to_send - sent).min(buf.len() as u32) as usize;
            self.flash
                .read_data(data_offset + clamped_off + sent, &mut buf[..chunk])
                .await
                .map_err(|_| OpError::Read)?;
            sink.data(&buf[..chunk]).await.map_err(|_| OpError::Aborted)?;
            sent += chunk as u32;
            if (sent & 4095) == 0 {
                yield_now().await;
            }
        }
        Ok(sent)
    }

    async fn remove_file(&mut self, path: &str) -> Result<(), OpError> {
        self.ensure_hardware_init().await?;
        if !self.formatted {
            return Err(OpError::NotFound);
        }
        if is_armed() {
            return Err(OpError::Aborted);
        }
        let Some((slot, _, _)) = self.find_live_slot(path).await? else {
            return Err(OpError::NotFound);
        };
        let base_page = SLOT_PAGE_BASE + (slot as u32) * 3;
        let mut tomb = [0u8; 8];
        tomb[0..4].copy_from_slice(&TOMB_MAGIC);
        let crc = crc32_of(&tomb[0..4]);
        tomb[4..8].copy_from_slice(&crc.to_le_bytes());
        self.flash
            .write_page_slice((base_page + 2) * PAGE_SIZE as u32, &tomb)
            .await
            .map_err(|_| OpError::Remove)?;
        self.flash.flush().await.map_err(|_| OpError::Remove)?;
        self.live_files = self.live_files.saturating_sub(1);
        self.publish_info();

        // If removing the last file freed the entire volume, immediately reclaim
        // dirty blocks so `blackbox-pull --all --delete` recovers flash capacity.
        if self.live_files == 0 && !is_armed() {
            self.reclaim_all_disarmed().await?;
        }
        Ok(())
    }

    async fn remove_flight_logs(&mut self) -> Result<CleanSummary, OpError> {
        self.ensure_hardware_init().await?;
        if !self.formatted {
            return Ok(CleanSummary::default());
        }
        if is_armed() {
            return Err(OpError::Aborted);
        }
        let removed = self.live_files as u32;
        self.reclaim_all_disarmed().await?;
        Ok(CleanSummary {
            removed,
            aborted: false,
        })
    }
}

#[derive(Clone, Copy)]
enum McapTrack {
    Magic(u8),
    Op,
    Len { idx: u8, acc: u64 },
    Body(u64),
}

#[derive(Clone, Copy, Debug)]
enum NorWriteError {
    DiskFull,
    Io,
}

impl core::fmt::Display for NorWriteError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::DiskFull => f.write_str("nor flash capacity reached"),
            Self::Io => f.write_str("nor flash SPI I/O error"),
        }
    }
}

impl core::error::Error for NorWriteError {}

impl embedded_io_async::Error for NorWriteError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

/// Zero-erase `embedded_io_async::Write` adapter buffering 256-byte NOR pages
/// in a 4 KiB ring and halting strictly on MCAP record boundaries before the
/// reserved trailer.
struct NorMcapWriter<'a> {
    flash: &'a mut NorDriver,
    ring: &'a mut [[u8; PAGE_SIZE]; RING_PAGES],
    base_addr: u32,
    soft_limit: u32,
    hard_limit: u32,
    bytes_written: u32,
    flushed_bytes: u32,
    read_idx: usize,
    write_idx: usize,
    full_pages: usize,
    write_off: usize,
    track: McapTrack,
    trailer_unlocked: bool,
    stats: &'a mut IoStats,
}

impl<'a> NorMcapWriter<'a> {
    fn new(
        flash: &'a mut NorDriver,
        ring: &'a mut [[u8; PAGE_SIZE]; RING_PAGES],
        base_addr: u32,
        soft_limit: u32,
        hard_limit: u32,
        stats: &'a mut IoStats,
    ) -> Self {
        Self {
            flash,
            ring,
            base_addr,
            soft_limit,
            hard_limit,
            bytes_written: 0,
            flushed_bytes: 0,
            read_idx: 0,
            write_idx: 0,
            full_pages: 0,
            write_off: 0,
            track: McapTrack::Magic(0),
            trailer_unlocked: false,
            stats,
        }
    }

    #[inline]
    fn bytes_written(&self) -> u32 {
        self.bytes_written
    }

    #[inline]
    fn unlock_trailer(&mut self) {
        self.trailer_unlocked = true;
    }

    async fn wait_flash_ready_coop(&mut self) -> Result<(), NorWriteError> {
        wait_flash_ready_coop(self.flash)
            .await
            .map_err(|_| NorWriteError::Io)
    }

    async fn finish_page_if_full(&mut self) -> Result<(), NorWriteError> {
        if self.write_off == PAGE_SIZE {
            self.write_off = 0;
            self.write_idx = (self.write_idx + 1) & (RING_PAGES - 1);
            self.full_pages += 1;
            self.pump_nonblocking().await?;
        }
        Ok(())
    }

    async fn kick_next_full_page(&mut self) -> Result<(), NorWriteError> {
        let addr = self.base_addr + self.flushed_bytes;
        let page = &self.ring[self.read_idx];
        self.flash
            .write_page_slice(addr, page)
            .await
            .map_err(|e| match e {
                FlashError::OutOfBounds => NorWriteError::DiskFull,
                _ => NorWriteError::Io,
            })?;
        self.read_idx = (self.read_idx + 1) & (RING_PAGES - 1);
        self.full_pages -= 1;
        self.flushed_bytes += PAGE_SIZE as u32;
        self.stats.writes = self.stats.writes.saturating_add(1);
        self.stats.blocks_written = self.stats.blocks_written.saturating_add(1);
        self.stats.max_batch = 1;
        // Let peer thread-mode tasks (iwdg, mahony, usb) run while silicon programs.
        yield_now().await;
        Ok(())
    }

    async fn pump_nonblocking(&mut self) -> Result<(), NorWriteError> {
        while self.full_pages > 0 {
            if !self.flash.poll_ready().map_err(|_| NorWriteError::Io)? {
                break;
            }
            self.kick_next_full_page().await?;
        }
        Ok(())
    }

    async fn ensure_ring_space(&mut self) -> Result<(), NorWriteError> {
        if self.full_pages == RING_PAGES {
            self.wait_flash_ready_coop().await?;
            self.kick_next_full_page().await?;
        }
        Ok(())
    }

    async fn finish_final_page(&mut self) -> Result<(), NorWriteError> {
        while self.full_pages > 0 {
            self.wait_flash_ready_coop().await?;
            self.kick_next_full_page().await?;
        }
        if self.write_off > 0 {
            self.wait_flash_ready_coop().await?;
            let addr = self.base_addr + self.flushed_bytes;
            let tail = &self.ring[self.write_idx][..self.write_off];
            self.flash
                .write_page_slice(addr, tail)
                .await
                .map_err(|_| NorWriteError::Io)?;
            self.flushed_bytes += self.write_off as u32;
            self.write_off = 0;
            self.stats.writes = self.stats.writes.saturating_add(1);
            self.stats.blocks_written = self.stats.blocks_written.saturating_add(1);
            self.stats.max_batch = 1;
        }
        self.wait_flash_ready_coop().await
    }

    fn advance_mcap_tracker(&mut self, byte: u8) -> bool {
        match self.track {
            McapTrack::Magic(n) => {
                if n + 1 >= 8 {
                    self.track = McapTrack::Op;
                } else {
                    self.track = McapTrack::Magic(n + 1);
                }
                true
            }
            McapTrack::Op => {
                if !self.trailer_unlocked && self.bytes_written >= self.soft_limit {
                    return false;
                }
                self.track = McapTrack::Len { idx: 0, acc: 0 };
                true
            }
            McapTrack::Len { idx, mut acc } => {
                acc |= (byte as u64) << (idx * 8);
                if idx + 1 >= 8 {
                    self.track = if acc == 0 {
                        McapTrack::Op
                    } else {
                        McapTrack::Body(acc)
                    };
                } else {
                    self.track = McapTrack::Len { idx: idx + 1, acc };
                }
                true
            }
            McapTrack::Body(rem) => {
                if rem <= 1 {
                    self.track = McapTrack::Op;
                } else {
                    self.track = McapTrack::Body(rem - 1);
                }
                true
            }
        }
    }
}

impl embedded_io_async::ErrorType for NorMcapWriter<'_> {
    type Error = NorWriteError;
}

impl Write for NorMcapWriter<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut accepted = 0usize;
        while accepted < buf.len() {
            if self.bytes_written >= self.hard_limit {
                return if accepted > 0 {
                    Ok(accepted)
                } else {
                    Err(NorWriteError::DiskFull)
                };
            }
            if let McapTrack::Body(rem) = self.track {
                self.ensure_ring_space().await?;
                let max_input = buf.len() - accepted;
                let max_page = PAGE_SIZE - self.write_off;
                let max_hard = (self.hard_limit - self.bytes_written) as usize;
                let max_rem = rem.min(usize::MAX as u64) as usize;
                let n = max_input.min(max_page).min(max_hard).min(max_rem);
                self.ring[self.write_idx][self.write_off..self.write_off + n]
                    .copy_from_slice(&buf[accepted..accepted + n]);
                self.write_off += n;
                self.bytes_written += n as u32;
                accepted += n;
                self.track = if rem == n as u64 {
                    McapTrack::Op
                } else {
                    McapTrack::Body(rem - n as u64)
                };
                self.finish_page_if_full().await?;
                continue;
            }
            if matches!(self.track, McapTrack::Op)
                && buf.len() - accepted >= 9
                && PAGE_SIZE - self.write_off >= 9
                && (self.hard_limit - self.bytes_written) >= 9
            {
                if !self.trailer_unlocked && self.bytes_written >= self.soft_limit {
                    return if accepted > 0 {
                        Ok(accepted)
                    } else {
                        Err(NorWriteError::DiskFull)
                    };
                }
                self.ensure_ring_space().await?;
                let body_len =
                    u64::from_le_bytes(buf[accepted + 1..accepted + 9].try_into().unwrap());
                self.ring[self.write_idx][self.write_off..self.write_off + 9]
                    .copy_from_slice(&buf[accepted..accepted + 9]);
                self.write_off += 9;
                self.bytes_written += 9;
                accepted += 9;
                self.track = if body_len == 0 {
                    McapTrack::Op
                } else {
                    McapTrack::Body(body_len)
                };
                self.finish_page_if_full().await?;
                continue;
            }
            let b = buf[accepted];
            if !self.advance_mcap_tracker(b) {
                return if accepted > 0 {
                    Ok(accepted)
                } else {
                    Err(NorWriteError::DiskFull)
                };
            }
            self.ensure_ring_space().await?;
            self.ring[self.write_idx][self.write_off] = b;
            self.write_off += 1;
            self.bytes_written += 1;
            accepted += 1;
            self.finish_page_if_full().await?;
        }
        Ok(accepted)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.pump_nonblocking().await
    }
}

#[inline]
fn is_armed() -> bool {
    crate::motors::IS_ARMED.load(core::sync::atomic::Ordering::Acquire)
}

#[inline]
fn align_up_4k(v: u32) -> u32 {
    let mask = (SECTOR_SIZE as u32) - 1;
    v.saturating_add(mask) & !mask
}

#[inline]
fn crc32_of(data: &[u8]) -> u32 {
    let mut h = Hasher::new();
    h.update(data);
    h.finalize()
}
