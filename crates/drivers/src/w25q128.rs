//! Async JEDEC SPI-NOR flash driver (`W25Q128` and pin/command-compatible parts).
//!
//! Designed for onboard 16 MiB (128 Mbit) blackbox storage chips connected over
//! standard 24-bit SPI (`0x9F` JEDEC ID, `0x0B` Fast Read, `0x02` Page Program,
//! `0x20` 4 KiB Sector Erase, `0xD8` 64 KiB Block Erase).

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;
use embedded_hal_async::delay::DelayNs;

pub const PAGE_SIZE: usize = 256;
pub const SECTOR_SIZE: usize = 4096;
pub const BLOCK64_SIZE: usize = 65536;
pub const MAX_24BIT_BYTES: u32 = 16 * 1024 * 1024;

const CMD_WRITE_STATUS_REG1: u8 = 0x01;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_READ_STATUS_REG1: u8 = 0x05;
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_FAST_READ: u8 = 0x0B;
const CMD_SECTOR_ERASE_4K: u8 = 0x20;
const CMD_ENABLE_RESET: u8 = 0x66;
const CMD_RESET_DEVICE: u8 = 0x99;
const CMD_JEDEC_ID: u8 = 0x9F;
const CMD_RELEASE_POWER_DOWN: u8 = 0xAB;
const CMD_BLOCK_ERASE_64K: u8 = 0xD8;

const SR1_BUSY: u8 = 0x01;
const SR1_WEL: u8 = 0x02;
const SR1_BP_MASK: u8 = 0x3C;

/// 3-byte JEDEC manufacturer/device identifier returned by command `0x9F`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct JedecId {
    pub mfg: u8,
    pub mem_type: u8,
    pub cap_exp: u8,
}

impl JedecId {
    #[inline]
    pub const fn as_bytes(self) -> [u8; 3] {
        [self.mfg, self.mem_type, self.cap_exp]
    }
}

/// Errors produced by [`W25q128`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FlashError {
    Spi,
    Pin,
    Timeout,
    InvalidJedec([u8; 3]),
    WriteEnableFailed,
    OutOfBounds,
    PageCrossed,
}

/// Async SPI-NOR flash driver over synchronous `SpiBus<u8>` + async `DelayNs`.
///
/// Using a synchronous SPI bus inside short CS-guarded transactions keeps every
/// SPI burst preemptible by higher-priority flight interrupts while yielding
/// during internal flash program/erase busy polls via `DelayNs`.
pub struct W25q128<SPI, CS, D> {
    spi: SPI,
    cs: CS,
    delay: D,
    jedec: Option<JedecId>,
    capacity_bytes: u32,
    program_in_flight: bool,
}

impl<SPI, CS, D> W25q128<SPI, CS, D>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    D: DelayNs,
{
    /// Construct a new uninitialized driver handle (`init()` must be called first).
    pub fn new(spi: SPI, cs: CS, delay: D) -> Self {
        Self {
            spi,
            cs,
            delay,
            jedec: None,
            capacity_bytes: 0,
            program_in_flight: false,
        }
    }

    #[inline]
    pub fn capacity_bytes(&self) -> u32 {
        self.capacity_bytes
    }

    #[inline]
    pub fn jedec_id(&self) -> Option<JedecId> {
        self.jedec
    }

    /// Execute `f` with `CS` asserted low, flush the SPI peripheral before
    /// deasserting `CS`, and always drive `CS` high again even if an I/O error
    /// occurred mid-transaction.
    fn with_cs<R>(
        &mut self,
        f: impl FnOnce(&mut SPI) -> Result<R, SPI::Error>,
    ) -> Result<R, FlashError> {
        self.cs.set_low().map_err(|_| FlashError::Pin)?;
        let op_res = f(&mut self.spi);
        let flush_res = self.spi.flush();
        let cs_res = self.cs.set_high();
        let val = op_res.map_err(|_| FlashError::Spi)?;
        flush_res.map_err(|_| FlashError::Spi)?;
        cs_res.map_err(|_| FlashError::Pin)?;
        Ok(val)
    }

    fn read_status_reg1(&mut self) -> Result<u8, FlashError> {
        let mut rx = [0u8; 2];
        self.with_cs(|spi| spi.transfer(&mut rx, &[CMD_READ_STATUS_REG1, 0x00]))?;
        Ok(rx[1])
    }

    async fn wait_ready(&mut self, poll_us: u32, max_polls: u32) -> Result<(), FlashError> {
        for _ in 0..max_polls {
            if (self.read_status_reg1()? & SR1_BUSY) == 0 {
                return Ok(());
            }
            self.delay.delay_us(poll_us).await;
        }
        Err(FlashError::Timeout)
    }

    /// Non-blocking readiness check for an in-flight pipelined page program.
    ///
    /// Returns `Ok(true)` immediately when no page program is in flight or once
    /// `SR1.BUSY` clears in silicon; returns `Ok(false)` while silicon is still busy.
    pub fn poll_ready(&mut self) -> Result<bool, FlashError> {
        if !self.program_in_flight {
            return Ok(true);
        }
        if (self.read_status_reg1()? & SR1_BUSY) == 0 {
            self.program_in_flight = false;
            return Ok(true);
        }
        Ok(false)
    }

    async fn ensure_ready(&mut self) -> Result<(), FlashError> {
        if self.poll_ready()? {
            return Ok(());
        }
        self.wait_ready(250, 100).await?;
        self.program_in_flight = false;
        Ok(())
    }

    /// Wait for any in-flight pipelined page program to complete in silicon.
    pub async fn flush(&mut self) -> Result<(), FlashError> {
        self.ensure_ready().await
    }

    fn write_enable(&mut self) -> Result<(), FlashError> {
        self.with_cs(|spi| spi.write(&[CMD_WRITE_ENABLE]))?;
        if (self.read_status_reg1()? & SR1_WEL) == 0 {
            return Err(FlashError::WriteEnableFailed);
        }
        Ok(())
    }

    /// Wake the flash chip from power-down, soft-reset any interrupted sequence,
    /// validate its JEDEC ID, compute 24-bit addressable capacity, and clear
    /// non-volatile block-protect bits if set.
    pub async fn init(&mut self) -> Result<JedecId, FlashError> {
        self.jedec = None;
        self.capacity_bytes = 0;
        self.program_in_flight = false;

        self.cs.set_high().map_err(|_| FlashError::Pin)?;
        self.delay.delay_us(50).await;

        self.with_cs(|spi| spi.write(&[CMD_RELEASE_POWER_DOWN]))?;
        self.delay.delay_us(35).await;

        self.with_cs(|spi| spi.write(&[CMD_ENABLE_RESET]))?;
        self.with_cs(|spi| spi.write(&[CMD_RESET_DEVICE]))?;
        self.delay.delay_us(50).await;

        self.wait_ready(1_000, 100).await?;

        let mut rx = [0u8; 4];
        self.with_cs(|spi| spi.transfer(&mut rx, &[CMD_JEDEC_ID, 0x00, 0x00, 0x00]))?;
        let raw = [rx[1], rx[2], rx[3]];

        // Accept standard JEDEC SPI-NOR devices from 1 MiB (0x14) upward; reject
        // floating / shorted MISO (`0x00` or `0xFF`).
        if raw[0] == 0x00 || raw[0] == 0xFF || !(0x14..=0x20).contains(&raw[2]) {
            return Err(FlashError::InvalidJedec(raw));
        }

        let jedec = JedecId {
            mfg: raw[0],
            mem_type: raw[1],
            cap_exp: raw[2],
        };
        let capacity_bytes = if jedec.cap_exp >= 24 {
            MAX_24BIT_BYTES
        } else {
            1u32 << jedec.cap_exp
        };

        self.jedec = Some(jedec);
        self.capacity_bytes = capacity_bytes;

        let sr1 = self.read_status_reg1()?;
        if (sr1 & SR1_BP_MASK) != 0 {
            self.write_enable()?;
            self.with_cs(|spi| spi.write(&[CMD_WRITE_STATUS_REG1, 0x00]))?;
            self.wait_ready(2_000, 200).await?;
        }

        Ok(jedec)
    }

    fn check_bounds(&self, addr: u32, len: usize) -> Result<(), FlashError> {
        let len_u32 = u32::try_from(len).map_err(|_| FlashError::OutOfBounds)?;
        match addr.checked_add(len_u32) {
            Some(end) if end <= self.capacity_bytes => Ok(()),
            _ => Err(FlashError::OutOfBounds),
        }
    }

    /// Read `buf.len()` bytes starting at 24-bit byte address `addr`.
    pub async fn read_data(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), FlashError> {
        self.check_bounds(addr, buf.len())?;
        if buf.is_empty() {
            return Ok(());
        }
        self.ensure_ready().await?;
        let a = addr.to_be_bytes();
        let hdr = [CMD_FAST_READ, a[1], a[2], a[3], 0x00];
        self.with_cs(|spi| {
            spi.write(&hdr)?;
            spi.read(buf)
        })
    }

    /// Program `data` within a single 256-byte page starting at `addr`.
    ///
    /// Pipelines the silicon page-program busy interval (`t_PP`): once the SPI
    /// payload finishes, this returns immediately so the caller can encode or
    /// buffer the next page while flash programs in the background, and any
    /// subsequent read, write, erase, or [`Self::flush`] waits for completion.
    pub async fn write_page_slice(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashError> {
        self.check_bounds(addr, data.len())?;
        if data.is_empty() {
            return Ok(());
        }
        let page_off = (addr as usize) & (PAGE_SIZE - 1);
        if data.len() > PAGE_SIZE || page_off + data.len() > PAGE_SIZE {
            return Err(FlashError::PageCrossed);
        }

        self.ensure_ready().await?;
        self.write_enable()?;
        let a = addr.to_be_bytes();
        let hdr = [CMD_PAGE_PROGRAM, a[1], a[2], a[3]];
        self.with_cs(|spi| {
            spi.write(&hdr)?;
            spi.write(data)
        })?;
        self.program_in_flight = true;
        Ok(())
    }

    /// Program an arbitrary byte slice `data` starting at `addr`, splitting across
    /// 256-byte page boundaries automatically and flushing the final page.
    pub async fn write_bytes(&mut self, mut addr: u32, mut data: &[u8]) -> Result<(), FlashError> {
        self.check_bounds(addr, data.len())?;
        while !data.is_empty() {
            let page_off = (addr as usize) & (PAGE_SIZE - 1);
            let chunk = (PAGE_SIZE - page_off).min(data.len());
            self.write_page_slice(addr, &data[..chunk]).await?;
            addr += chunk as u32;
            data = &data[chunk..];
        }
        self.flush().await
    }

    /// Erase one 4 KiB sector (`addr` must be 4 KiB aligned).
    pub async fn erase_sector_4k(&mut self, addr: u32) -> Result<(), FlashError> {
        if (addr & (SECTOR_SIZE as u32 - 1)) != 0 {
            return Err(FlashError::OutOfBounds);
        }
        self.check_bounds(addr, SECTOR_SIZE)?;
        self.ensure_ready().await?;
        self.write_enable()?;
        let a = addr.to_be_bytes();
        let cmd = [CMD_SECTOR_ERASE_4K, a[1], a[2], a[3]];
        self.with_cs(|spi| spi.write(&cmd))?;
        self.wait_ready(2_000, 800).await
    }

    /// Erase one 64 KiB block (`addr` must be 64 KiB aligned).
    pub async fn erase_block_64k(&mut self, addr: u32) -> Result<(), FlashError> {
        if (addr & (BLOCK64_SIZE as u32 - 1)) != 0 {
            return Err(FlashError::OutOfBounds);
        }
        self.check_bounds(addr, BLOCK64_SIZE)?;
        self.ensure_ready().await?;
        self.write_enable()?;
        let a = addr.to_be_bytes();
        let cmd = [CMD_BLOCK_ERASE_64K, a[1], a[2], a[3]];
        self.with_cs(|spi| spi.write(&cmd))?;
        self.wait_ready(5_000, 3_000).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
    use embedded_hal::digital::ErrorType as PinErrorType;
    use embedded_hal::spi::{Error as SpiErrorTrait, ErrorKind, ErrorType as SpiErrorType};

    fn noop_waker() -> Waker {
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        fn noop(_: *const ()) {}
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
        unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
    }

    fn block_on<F: Future>(fut: F) -> F::Output {
        let mut fut = core::pin::pin!(fut);
        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);
        loop {
            match Pin::as_mut(&mut fut).poll(&mut cx) {
                Poll::Ready(v) => return v,
                Poll::Pending => {}
            }
        }
    }

    struct DummyDelay;
    impl DelayNs for DummyDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    #[derive(Debug, Clone, Copy)]
    struct MockSpiErr;
    impl SpiErrorTrait for MockSpiErr {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    struct SimNor {
        mem: Vec<u8>,
        jedec: [u8; 3],
        sr1: u8,
        cs_low: bool,
        busy_polls_left: u8,
        reset_armed: bool,
        tx_buf: Vec<u8>,
        read_cursor: Option<(u32, usize)>,
        last_flush_while_cs_low: bool,
        fail_next_flush: bool,
    }

    impl SimNor {
        fn new(cap_exp: u8, bp_bits: u8) -> Self {
            let size = if cap_exp >= 24 {
                MAX_24BIT_BYTES as usize
            } else {
                1usize << cap_exp
            };
            Self {
                mem: vec![0xFF; size],
                jedec: [0xEF, 0x40, cap_exp],
                sr1: bp_bits & SR1_BP_MASK,
                cs_low: false,
                busy_polls_left: 0,
                reset_armed: false,
                tx_buf: Vec::new(),
                read_cursor: None,
                last_flush_while_cs_low: false,
                fail_next_flush: false,
            }
        }

        fn begin_cs(&mut self) {
            self.cs_low = true;
            self.tx_buf.clear();
            self.read_cursor = None;
            self.last_flush_while_cs_low = false;
        }

        fn end_cs(&mut self) {
            if !self.cs_low {
                return;
            }
            self.cs_low = false;
            if self.tx_buf.is_empty() {
                return;
            }
            let cmd = self.tx_buf[0];
            if cmd != CMD_RESET_DEVICE {
                self.reset_armed = false;
            }
            match cmd {
                CMD_RELEASE_POWER_DOWN => {}
                CMD_ENABLE_RESET => {
                    self.reset_armed = true;
                }
                CMD_RESET_DEVICE => {
                    if self.reset_armed {
                        self.sr1 &= !SR1_WEL;
                    }
                    self.reset_armed = false;
                }
                CMD_WRITE_ENABLE if self.busy_polls_left == 0 => {
                    self.sr1 |= SR1_WEL;
                }
                CMD_WRITE_STATUS_REG1 if self.busy_polls_left == 0 && self.tx_buf.len() == 2 => {
                    if (self.sr1 & SR1_WEL) != 0 {
                        self.sr1 = (self.sr1 & !SR1_BP_MASK) | (self.tx_buf[1] & SR1_BP_MASK);
                        self.sr1 &= !SR1_WEL;
                        self.busy_polls_left = 1;
                    }
                }
                CMD_PAGE_PROGRAM if self.busy_polls_left == 0 && self.tx_buf.len() >= 5 => {
                    if (self.sr1 & SR1_WEL) != 0 && (self.sr1 & SR1_BP_MASK) == 0 {
                        let base = u32::from_be_bytes([
                            0,
                            self.tx_buf[1],
                            self.tx_buf[2],
                            self.tx_buf[3],
                        ]);
                        let page_base = (base as usize) & !(PAGE_SIZE - 1);
                        let mut page_off = (base as usize) & (PAGE_SIZE - 1);
                        // Real SPI NOR wraps within the 256 B page!
                        for &b in &self.tx_buf[4..] {
                            self.mem[page_base + page_off] &= b;
                            page_off = (page_off + 1) & (PAGE_SIZE - 1);
                        }
                        self.busy_polls_left = 2;
                    }
                    self.sr1 &= !SR1_WEL;
                }
                CMD_SECTOR_ERASE_4K if self.busy_polls_left == 0 && self.tx_buf.len() == 4 => {
                    if (self.sr1 & SR1_WEL) != 0 && (self.sr1 & SR1_BP_MASK) == 0 {
                        let base = u32::from_be_bytes([
                            0,
                            self.tx_buf[1],
                            self.tx_buf[2],
                            self.tx_buf[3],
                        ]) as usize
                            & !(SECTOR_SIZE - 1);
                        self.mem[base..base + SECTOR_SIZE].fill(0xFF);
                        self.busy_polls_left = 3;
                    }
                    self.sr1 &= !SR1_WEL;
                }
                CMD_BLOCK_ERASE_64K if self.busy_polls_left == 0 && self.tx_buf.len() == 4 => {
                    if (self.sr1 & SR1_WEL) != 0 && (self.sr1 & SR1_BP_MASK) == 0 {
                        let base = u32::from_be_bytes([
                            0,
                            self.tx_buf[1],
                            self.tx_buf[2],
                            self.tx_buf[3],
                        ]) as usize
                            & !(BLOCK64_SIZE - 1);
                        self.mem[base..base + BLOCK64_SIZE].fill(0xFF);
                        self.busy_polls_left = 4;
                    }
                    self.sr1 &= !SR1_WEL;
                }
                _ => {}
            }
            self.tx_buf.clear();
            self.read_cursor = None;
        }

        fn clock_byte(&mut self, tx: u8) -> u8 {
            let idx = self.tx_buf.len();
            self.tx_buf.push(tx);
            if idx == 0 {
                return 0xFF;
            }
            match self.tx_buf[0] {
                CMD_JEDEC_ID => match idx {
                    1 => self.jedec[0],
                    2 => self.jedec[1],
                    3 => self.jedec[2],
                    _ => 0xFF,
                },
                CMD_READ_STATUS_REG1 => {
                    if self.busy_polls_left > 0 {
                        self.busy_polls_left -= 1;
                        self.sr1 | SR1_BUSY
                    } else {
                        self.sr1 & !SR1_BUSY
                    }
                }
                CMD_FAST_READ if self.busy_polls_left == 0 => {
                    if idx < 4 {
                        0xFF
                    } else if idx == 4 {
                        let addr = u32::from_be_bytes([
                            0,
                            self.tx_buf[1],
                            self.tx_buf[2],
                            self.tx_buf[3],
                        ]);
                        self.read_cursor = Some((addr, 0));
                        0xFF
                    } else if let Some((addr, off)) = &mut self.read_cursor {
                        let pos = ((*addr as usize) + *off) % self.mem.len();
                        *off += 1;
                        self.mem[pos]
                    } else {
                        0xFF
                    }
                }
                _ => 0xFF,
            }
        }
    }

    use std::cell::RefCell;
    use std::rc::Rc;

    #[derive(Clone)]
    struct SharedSim(Rc<RefCell<SimNor>>);

    struct MockSpi(SharedSim);
    impl SpiErrorType for MockSpi {
        type Error = MockSpiErr;
    }
    impl SpiBus<u8> for MockSpi {
        fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            let mut s = self.0.0.borrow_mut();
            for w in words.iter_mut() {
                *w = s.clock_byte(0x00);
            }
            Ok(())
        }
        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            let mut s = self.0.0.borrow_mut();
            for &b in words {
                s.clock_byte(b);
            }
            Ok(())
        }
        fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            let mut s = self.0.0.borrow_mut();
            let len = read.len().max(write.len());
            for i in 0..len {
                let b = write.get(i).copied().unwrap_or(0x00);
                let r = s.clock_byte(b);
                if let Some(slot) = read.get_mut(i) {
                    *slot = r;
                }
            }
            Ok(())
        }
        fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            let mut s = self.0.0.borrow_mut();
            for w in words.iter_mut() {
                *w = s.clock_byte(*w);
            }
            Ok(())
        }
        fn flush(&mut self) -> Result<(), Self::Error> {
            let mut s = self.0.0.borrow_mut();
            s.last_flush_while_cs_low = s.cs_low;
            if s.fail_next_flush {
                s.fail_next_flush = false;
                return Err(MockSpiErr);
            }
            Ok(())
        }
    }

    struct MockCs(SharedSim);
    impl PinErrorType for MockCs {
        type Error = Infallible;
    }
    impl OutputPin for MockCs {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.0.0.borrow_mut().begin_cs();
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.0.0.borrow_mut().end_cs();
            Ok(())
        }
    }

    fn make_driver(cap_exp: u8, bp: u8) -> (W25q128<MockSpi, MockCs, DummyDelay>, SharedSim) {
        let sim = SharedSim(Rc::new(RefCell::new(SimNor::new(cap_exp, bp))));
        (
            W25q128::new(MockSpi(sim.clone()), MockCs(sim.clone()), DummyDelay),
            sim,
        )
    }

    #[test]
    fn init_clears_bp_and_reports_16mib() {
        let (mut flash, sim) = make_driver(0x18, 0x1C);
        let id = block_on(flash.init()).expect("init should succeed");
        assert_eq!(id.as_bytes(), [0xEF, 0x40, 0x18]);
        assert_eq!(flash.capacity_bytes(), MAX_24BIT_BYTES);
        assert_eq!(sim.0.borrow().sr1 & SR1_BP_MASK, 0);
        assert!(sim.0.borrow().last_flush_while_cs_low);
    }

    #[test]
    fn rejects_invalid_jedec() {
        for bad in [[0x00, 0x00, 0x00], [0xFF, 0xFF, 0xFF], [0xEF, 0x40, 0x10]] {
            let (mut flash, sim) = make_driver(0x18, 0);
            sim.0.borrow_mut().jedec = bad;
            assert_eq!(block_on(flash.init()), Err(FlashError::InvalidJedec(bad)));
            assert_eq!(flash.capacity_bytes(), 0);
        }
    }

    #[test]
    fn bitwise_and_program_and_erases() {
        let (mut flash, _sim) = make_driver(0x14, 0); // 1 MiB for fast test
        block_on(flash.init()).unwrap();

        // Program across page boundary (256B page starts at 0x100).
        let payload: Vec<u8> = (0..600u32).map(|i| (i.wrapping_mul(37) ^ 0xA5) as u8).collect();
        block_on(flash.write_bytes(0x0001_0080, &payload)).unwrap();

        let mut rb = vec![0u8; payload.len()];
        block_on(flash.read_data(0x0001_0080, &mut rb)).unwrap();
        assert_eq!(rb, payload);

        // Second program without erase only clears bits (old &= new).
        block_on(flash.write_page_slice(0x0001_0000, &[0xAA, 0x55])).unwrap();
        block_on(flash.write_page_slice(0x0001_0000, &[0xF0, 0x0F])).unwrap();
        let mut two = [0u8; 2];
        block_on(flash.read_data(0x0001_0000, &mut two)).unwrap();
        assert_eq!(two, [0xAA & 0xF0, 0x55 & 0x0F]);

        // Page-crossing slice must fail before touching flash.
        assert_eq!(
            block_on(flash.write_page_slice(0x0001_00FE, &[1, 2, 3, 4])),
            Err(FlashError::PageCrossed)
        );

        // 4 KiB erase resets sector 0x0001_0000 to 0xFF while keeping sector 0x0001_1000.
        block_on(flash.erase_sector_4k(0x0001_0000)).unwrap();
        let mut sec0 = [0u8; 4];
        block_on(flash.read_data(0x0001_0080, &mut sec0)).unwrap();
        assert_eq!(sec0, [0xFF; 4]);

        // Unaligned erase rejected.
        assert_eq!(
            block_on(flash.erase_sector_4k(0x0001_0100)),
            Err(FlashError::OutOfBounds)
        );

        // 64 KiB block erase resets the whole block.
        block_on(flash.erase_block_64k(0x0001_0000)).unwrap();
        let mut rb2 = vec![0u8; payload.len()];
        block_on(flash.read_data(0x0001_0080, &mut rb2)).unwrap();
        assert!(rb2.iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn pipelined_page_writes_overlap_and_flush_before_read_or_erase() {
        let (mut flash, sim) = make_driver(0x14, 0);
        block_on(flash.init()).unwrap();

        // `write_page_slice` returns with silicon still busy (`busy_polls_left == 2`).
        block_on(flash.write_page_slice(0x0000_1000, &[0x12, 0x34])).unwrap();
        assert_eq!(sim.0.borrow().busy_polls_left, 2);
        assert!(!flash.poll_ready().unwrap());
        assert!(!flash.poll_ready().unwrap());
        assert!(flash.poll_ready().unwrap());
        assert_eq!(sim.0.borrow().busy_polls_left, 0);

        // Second write leaves silicon busy again so the next write tests automatic drain.
        block_on(flash.write_page_slice(0x0000_1080, &[0xAA])).unwrap();
        assert_eq!(sim.0.borrow().busy_polls_left, 2);

        // Next page write drains the prior busy interval before programming.
        block_on(flash.write_page_slice(0x0000_1100, &[0x56, 0x78])).unwrap();
        assert_eq!(sim.0.borrow().busy_polls_left, 2);

        // Read drains busy interval and reads committed bytes.
        let mut buf = [0u8; 2];
        block_on(flash.read_data(0x0000_1000, &mut buf)).unwrap();
        assert_eq!(sim.0.borrow().busy_polls_left, 0);
        assert_eq!(buf, [0x12, 0x34]);

        let mut single = [0u8; 1];
        block_on(flash.read_data(0x0000_1080, &mut single)).unwrap();
        assert_eq!(single, [0xAA]);

        block_on(flash.read_data(0x0000_1100, &mut buf)).unwrap();
        assert_eq!(buf, [0x56, 0x78]);

        // Explicit `flush()` clears in-flight program busy state.
        block_on(flash.write_page_slice(0x0000_1200, &[0xAB])).unwrap();
        assert_eq!(sim.0.borrow().busy_polls_left, 2);
        block_on(flash.flush()).unwrap();
        assert_eq!(sim.0.borrow().busy_polls_left, 0);

        // Erase drains an in-flight program before issuing sector erase.
        block_on(flash.write_page_slice(0x0000_1300, &[0xCD])).unwrap();
        assert_eq!(sim.0.borrow().busy_polls_left, 2);
        block_on(flash.erase_sector_4k(0x0000_1000)).unwrap();
        assert_eq!(sim.0.borrow().busy_polls_left, 0);
        let mut erased = [0u8; 1];
        block_on(flash.read_data(0x0000_1300, &mut erased)).unwrap();
        assert_eq!(erased, [0xFF]);
    }

    #[test]
    fn cs_always_deasserted_after_spi_flush_error() {
        let (mut flash, sim) = make_driver(0x14, 0);
        block_on(flash.init()).unwrap();
        sim.0.borrow_mut().fail_next_flush = true;
        let mut buf = [0u8; 4];
        assert_eq!(block_on(flash.read_data(0, &mut buf)), Err(FlashError::Spi));
        assert!(!sim.0.borrow().cs_low);
    }
}
