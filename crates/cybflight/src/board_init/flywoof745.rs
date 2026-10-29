//! Board init for the Flywoo GOKU GN 745 45A AIO 32-bit V3 (`STM32F745VG`).
//!
//! Hardware wired here:
//! - Primary IMU on SPI4 (`PE2` SCK, `PE5` MISO, `PE6` MOSI, `PE4` CS, `PE1` EXTI1):
//!   auto-probes MPU6000 (V3), ICM42688P, or BMI270.
//! - I2C1 (`PB6` SCL, `PB7` SDA): SPL06 barometer (`0x76`) + QMC5883L compass (`0x0D`).
//! - Dual-timer Bidirectional DShot600:
//!   - M1 (Rear-Right CW):  `PB0`  `TIM3_CH3` AF2 (`DMA1_CH7`, req 5)
//!   - M2 (Front-Right CCW): `PB1`  `TIM3_CH4` AF2 (`DMA1_CH2`, req 5)
//!   - M3 (Rear-Left CCW):  `PE9`  `TIM1_CH1` AF1 (`DMA2_CH1`, req 6)
//!   - M4 (Front-Left CW):  `PE11` `TIM1_CH2` AF1 (`DMA2_CH2`, req 6)
//! - SerialRx (ELRS CRSF / GHST) via `bsp::PORT_SERIAL_RX` (default `USART2`, `PD5/PD6`).
//! - Offboard / ESP Bridge DMA UART via `bsp::PORT_ESP_BRIDGE` (default `USART1`, `PA9/PA10` @ 460800 baud).
//! - GPS via `bsp::PORT_GPS` (default `USART6`, `PC6/PC7`).
//! - Battery VBAT (`PC3`, `ADC3_IN13`) & Current (`PC2`, `ADC3_IN12`).

use cybflight_drivers::baro::spl06::Spl06;
#[cfg(feature = "est_pos_gps")]
use cybflight_drivers::gps::GpsDriver;
use cybflight_drivers::imu::bmi270::Bmi270;
use cybflight_drivers::imu::icm426xx::Icm426xx;
use cybflight_drivers::imu::mpu6x00::Mpu6x00;
use cybflight_drivers::imu::{DetectedImu, ReadImu, probe_imu_raw};
use cybflight_drivers::led::Led;
use cybflight_drivers::mag::Qmc5883l;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::{InterruptExecutor, SendSpawner, Spawner};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer, with_timeout};
use static_cell::StaticCell;

use crate::bsp;
use crate::hal;
use crate::motors::{DshotQuadConfig, MotorTimerConfig};
use crate::sensors::baro::BaroReader;
#[cfg(feature = "est_pos_gps")]
use crate::sensors::gps::GpsRunner;
use crate::sensors::imu::{
    ImuReader, SpiBusMtx, bmi270_reader_task, icm_reader_task, mpu_reader_task,
};
use crate::sensors::mag::{I2cBusMtx, MagReader};
use crate::sensors::power::{PowerMonitor, power_task};
use crate::status;
use crate::usb_serial;
use hal::gpio::{AfType, Flex, OutputType, Speed};
use hal::spi::{self, Spi};
use hal::time::Hertz;
use hal::timer::low_level::Timer as LLTimer;

// Buffered serial UARTs used as role candidates (SerialRx / GPS).
hal::bind_interrupts!(struct SerialIrqs {
    USART2 => hal::usart::BufferedInterruptHandler<hal::peripherals::USART2>;
    USART3 => hal::usart::BufferedInterruptHandler<hal::peripherals::USART3>;
    UART4  => hal::usart::BufferedInterruptHandler<hal::peripherals::UART4>;
    UART5  => hal::usart::BufferedInterruptHandler<hal::peripherals::UART5>;
    USART6 => hal::usart::BufferedInterruptHandler<hal::peripherals::USART6>;
});

// I2C1 interrupts for SPL06 baro + QMC5883L mag.
hal::bind_interrupts!(struct I2c1Irqs {
    I2C1_EV => hal::i2c::EventInterruptHandler<hal::peripherals::I2C1>;
    I2C1_ER => hal::i2c::ErrorInterruptHandler<hal::peripherals::I2C1>;
});

// USART1 interrupt for Offboard / ESP bridge (DMA UART).
hal::bind_interrupts!(struct Usart1Irqs {
    USART1 => hal::usart::InterruptHandler<hal::peripherals::USART1>;
});

// Dedicated P8 interrupt executor for the 8 kHz IMU reader (strictly between
// P6 `EXECUTOR_HIGH` DShot and P10 `EXECUTOR_CTRL` INDI/Failsafe/CRSF), so a
// 126..210 us INDI WLS step on P10 can never stall 125 us MPU-6000 DRDY reads.
use crate::hal::interrupt;
static EXECUTOR_IMU: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CAN2_RX1() {
    unsafe { EXECUTOR_IMU.on_interrupt() }
}

pub async fn init(
    spawner: &Spawner,
    ctrl_spawner: &SendSpawner,
    high_spawner: &SendSpawner,
    board: bsp::Board,
) {
    let imu_spawner = {
        use hal::interrupt::{self, InterruptExt, Priority};
        interrupt::CAN2_RX1.set_priority(Priority::P8);
        EXECUTOR_IMU.start(interrupt::CAN2_RX1)
    };

    // --- Load vehicle parameters from flash (or defaults) ---
    crate::params::init_from_flash(board.internal_flash);

    // --- Status LED (PA2) ---
    let led0 = Led::new(board.leds.led0, false);
    spawner.spawn(status::task(led0)).unwrap();

    // --- USB CDC serial ---
    spawner
        .spawn(usb_serial::task(
            board.usb.usb_otg_fs,
            board.usb.dp,
            board.usb.dm,
        ))
        .unwrap();

    // Wait for power to stabilize before touching SPI devices.
    Timer::after_millis(100).await;

    // Start SPI4 at 1 MHz (54 MHz / 64 = 843.75 kHz) to satisfy the MPU-6000
    // <= 1 MHz register-write specification during WHO_AM_I probe & config,
    // then switch to high-speed SPI (6.75 MHz / 13.5 MHz) for 8 kHz DMA reads.
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(1_000_000);
    spi_config.mode = spi::MODE_3;

    // --- IMU1 on SPI4 (PE2 SCK / PE5 MISO / PE6 MOSI, CS=PE4, DRDY=PE1) ---
    let mut spi4 = Spi::new(
        board.spi.spi4,
        board.spi.spi4_sck,
        board.spi.spi4_mosi,
        board.spi.spi4_miso,
        board.spi.spi4_tx_dma,
        board.spi.spi4_rx_dma,
        spi_config,
    );
    let mut cs = board.sensors.gyro1_cs;
    let detected = probe_imu_raw(&mut spi4, &mut cs).await;
    defmt::info!("FLYWOOF745 IMU probe on SPI4: {:?}", detected);

    static SPI4_BUS: StaticCell<SpiBusMtx> = StaticCell::new();
    let spi4_bus = SPI4_BUS.init(Mutex::new(spi4));
    let dev4 = SpiDevice::new(spi4_bus, cs);

    let mut delay = embassy_time::Delay;
    let sensor_params = crate::params::get().sensors;
    let accel_cutoff_hz = sensor_params.imu_accel_lpf_hz;
    let gyro_cutoff_hz = sensor_params.imu_gyro_lpf_hz;
    let mag_hard_iron =
        nalgebra::Vector3::from(crate::params::get().airframe.install.mag_hard_iron);

    match detected {
        Ok(DetectedImu::Mpu6000) | Ok(DetectedImu::Mpu6500) => {
            if crate::rates::IMU_ODR_HZ != 8000.0 {
                defmt::error!(
                    "imu_1khz build but detected MPU6x00 (8 kHz only) — IMU1 reader not spawned"
                );
            } else {
                match Mpu6x00::new(dev4, board.sensors.gyro1_drdy, &mut delay).await {
                    Ok(imu1) => {
                        let mut fast_cfg = spi::Config::default();
                        // PCLK2 = 54 MHz; embassy-stm32's integer division `54_000_000 / freq`
                        // maps `13_500_000` -> `4` (`Br::DIV4` = 13.5 MHz), safely within
                        // MPU-6000's 20 MHz sensor-read spec (note: `20_000_000` truncated to `2` = 27 MHz!).
                        fast_cfg.frequency = Hertz(13_500_000);
                        fast_cfg.mode = spi::MODE_3;
                        let _ = spi4_bus.lock().await.set_config(&fast_cfg);
                        defmt::info!("IMU1 init OK (MPU6000 on SPI4 @ 13.5 MHz)");
                        // Pass `None` for IMU_1_RAW: this AIO has no blackbox SD/flash storage,
                        // avoiding 8,000 unconsumed critical-section PubSub writes/sec.
                        imu_spawner
                            .spawn(mpu_reader_task(
                                ImuReader::new(
                                    imu1,
                                    board.sensors.gyro1_align,
                                    accel_cutoff_hz,
                                    gyro_cutoff_hz,
                                ),
                                &crate::sensors::IMU_1,
                                None,
                            ))
                            .unwrap();
                    }
                    Err(e) => defmt::error!("IMU1 MPU6000 init failed: {}", e),
                }
            }
        }
        Ok(DetectedImu::Icm42605)
        | Ok(DetectedImu::Icm42622P)
        | Ok(DetectedImu::Icm42688P)
        | Ok(DetectedImu::Iim42652)
        | Ok(DetectedImu::Iim42653) => {
            match Icm426xx::new(
                dev4,
                board.sensors.gyro1_drdy,
                &mut delay,
                crate::rates::ICM_ODR,
            )
            .await
            {
                Ok(imu1) => {
                    let mut fast_cfg = spi::Config::default();
                    // 54_000_000 / 13_500_000 = 4 -> Br::DIV4 = 13.5 MHz (<= 24 MHz spec)
                    fast_cfg.frequency = Hertz(13_500_000);
                    fast_cfg.mode = spi::MODE_3;
                    let _ = spi4_bus.lock().await.set_config(&fast_cfg);
                    defmt::info!("IMU1 init OK (ICM426xx on SPI4 @ 13.5 MHz)");
                    defmt::assert!(
                        imu1.sample_rate_hz() == crate::rates::IMU_ODR_HZ,
                        "IMU1 ODR does not match rates::IMU_ODR_HZ"
                    );
                    imu_spawner
                        .spawn(icm_reader_task(
                            ImuReader::new(
                                imu1,
                                board.sensors.gyro1_align,
                                accel_cutoff_hz,
                                gyro_cutoff_hz,
                            ),
                            &crate::sensors::IMU_1,
                            None,
                        ))
                        .unwrap();
                }
                Err(e) => defmt::error!("IMU1 ICM init failed: {}", e),
            }
        }
        Err(0x24) => {
            // BMI270 WHO_AM_I = 0x24
            match Bmi270::new(dev4, board.sensors.gyro1_drdy, embassy_time::Delay).await {
                Ok(imu1) => {
                    let mut fast_cfg = spi::Config::default();
                    // 54_000_000 / 6_750_000 = 8 -> Br::DIV8 = 6.75 MHz (<= 10 MHz spec)
                    fast_cfg.frequency = Hertz(6_750_000);
                    fast_cfg.mode = spi::MODE_3;
                    let _ = spi4_bus.lock().await.set_config(&fast_cfg);
                    defmt::info!("IMU1 init OK (BMI270 on SPI4 @ 6.75 MHz)");
                    imu_spawner
                        .spawn(bmi270_reader_task(
                            ImuReader::new(
                                imu1,
                                // BMI270 revision uses ROTATION_PITCH_180 in FlywooF745 hwdef -> Cw180Deg in FLU
                                crate::SensorAlign::Cw180Deg,
                                accel_cutoff_hz,
                                gyro_cutoff_hz,
                            ),
                            &crate::sensors::IMU_1,
                            None,
                        ))
                        .unwrap();
                }
                Err(e) => defmt::error!("IMU1 BMI270 init failed: {}", e),
            }
        }
        Err(id) => {
            defmt::error!("Unknown IMU on SPI4: WHO_AM_I={:#x}", id);
        }
    }

    let _ = &ctrl_spawner;
    static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0u8; 256])[..];
    let rx_buf = &mut RX_BUF.init([0u8; 256])[..];
    let mut rc_uart_cfg = hal::usart::Config::default();
    rc_uart_cfg.baudrate = 420_000;
    rc_uart_cfg.rx_pull = hal::gpio::Pull::Up;

    // --- SerialRx (CRSF / ELRS) ---
    #[cfg(feature = "rx_crsf")]
    match bsp::PORT_SERIAL_RX {
        bsp::SerialPortId::Usart2 => {
            match hal::usart::BufferedUart::new(
                board.serial.usart2,
                board.serial.usart2_rx,
                board.serial.usart2_tx,
                tx_buf,
                rx_buf,
                SerialIrqs,
                rc_uart_cfg,
            ) {
                Ok(uart) => {
                    defmt::info!("CRSF USART2 init OK (T2=PD5, R2=PD6)");
                    ctrl_spawner
                        .spawn(crate::sensors::rc::crsf_runner::crsf_task(uart))
                        .unwrap_or_else(|e| defmt::error!("Failed to spawn CRSF task: {}", e));
                }
                Err(e) => defmt::error!("CRSF USART2 init failed: {}", e),
            }
        }
        bsp::SerialPortId::Uart4 => {
            match hal::usart::BufferedUart::new(
                board.serial.uart4,
                board.serial.uart4_rx,
                board.serial.uart4_tx,
                tx_buf,
                rx_buf,
                SerialIrqs,
                rc_uart_cfg,
            ) {
                Ok(uart) => {
                    defmt::info!("CRSF UART4 init OK (T4=PA0, R4=PA1)");
                    ctrl_spawner
                        .spawn(crate::sensors::rc::crsf_runner::crsf_task(uart))
                        .unwrap_or_else(|e| defmt::error!("Failed to spawn CRSF task: {}", e));
                }
                Err(e) => defmt::error!("CRSF UART4 init failed: {}", e),
            }
        }
        bsp::SerialPortId::Uart5 => {
            match hal::usart::BufferedUart::new(
                board.serial.uart5,
                board.serial.uart5_rx,
                board.serial.uart5_tx,
                tx_buf,
                rx_buf,
                SerialIrqs,
                rc_uart_cfg,
            ) {
                Ok(uart) => {
                    defmt::info!("CRSF UART5 init OK (T5=PC12, R5=PD2)");
                    ctrl_spawner
                        .spawn(crate::sensors::rc::crsf_runner::crsf_task(uart))
                        .unwrap_or_else(|e| defmt::error!("Failed to spawn CRSF task: {}", e));
                }
                Err(e) => defmt::error!("CRSF UART5 init failed: {}", e),
            }
        }
        _ => defmt::warn!("CRSF: PORT_SERIAL_RX is not a supported SerialRx port on this board"),
    }

    #[cfg(feature = "rx_ghst")]
    match bsp::PORT_SERIAL_RX {
        bsp::SerialPortId::Usart2 => {
            match hal::usart::BufferedUart::new_half_duplex(
                board.serial.usart2,
                board.serial.usart2_tx,
                SerialIrqs,
                tx_buf,
                rx_buf,
                rc_uart_cfg,
                hal::usart::HalfDuplexReadback::NoReadback,
            ) {
                Ok(uart) => {
                    defmt::info!("GHST USART2 half-duplex init OK (T2=PD5)");
                    ctrl_spawner
                        .spawn(crate::sensors::rc::ghst_runner::ghst_task(uart))
                        .unwrap_or_else(|e| defmt::error!("Failed to spawn GHST task: {}", e));
                }
                Err(e) => defmt::error!("GHST USART2 init failed: {}", e),
            }
        }
        _ => defmt::warn!("GHST: PORT_SERIAL_RX is not a supported SerialRx port on this board"),
    }

    // --- GPS (`PORT_GPS`, default USART6 PC6/PC7) ---
    #[cfg(feature = "est_pos_gps")]
    {
        use crate::sensors::gps::{GPS_HEALTH, GpsHealth};
        GPS_HEALTH.lock(|c| c.set(GpsHealth::Initializing));
        match bsp::PORT_GPS {
            bsp::SerialPortId::Usart6 => {
                defmt::info!("GPS: starting USART6 init (T6=PC6, R6=PC7)");
                static GPS_TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
                static GPS_RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
                let tx_buf = &mut GPS_TX_BUF.init([0u8; 256])[..];
                let rx_buf = &mut GPS_RX_BUF.init([0u8; 1024])[..];
                let mut uart_config = hal::usart::Config::default();
                uart_config.baudrate = 115_200;
                uart_config.rx_pull = hal::gpio::Pull::Up;
                match hal::usart::BufferedUart::new(
                    board.serial.usart6,
                    board.serial.usart6_rx,
                    board.serial.usart6_tx,
                    tx_buf,
                    rx_buf,
                    SerialIrqs,
                    uart_config,
                ) {
                    Ok(uart) => init_gps_uart(uart, spawner).await,
                    Err(e) => {
                        GPS_HEALTH.lock(|c| c.set(GpsHealth::UartInitFailed));
                        defmt::error!("GPS USART6 init failed: {}", e);
                    }
                }
            }
            bsp::SerialPortId::Usart3 => {
                defmt::info!("GPS: starting USART3 init (T3=PB10, R3=PB11)");
                static GPS_TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
                static GPS_RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
                let tx_buf = &mut GPS_TX_BUF.init([0u8; 256])[..];
                let rx_buf = &mut GPS_RX_BUF.init([0u8; 1024])[..];
                let mut uart_config = hal::usart::Config::default();
                uart_config.baudrate = 115_200;
                uart_config.rx_pull = hal::gpio::Pull::Up;
                match hal::usart::BufferedUart::new(
                    board.serial.usart3,
                    board.serial.usart3_rx,
                    board.serial.usart3_tx,
                    tx_buf,
                    rx_buf,
                    SerialIrqs,
                    uart_config,
                ) {
                    Ok(uart) => init_gps_uart(uart, spawner).await,
                    Err(e) => {
                        GPS_HEALTH.lock(|c| c.set(GpsHealth::UartInitFailed));
                        defmt::error!("GPS USART3 init failed: {}", e);
                    }
                }
            }
            _ => {
                GPS_HEALTH.lock(|c| c.set(GpsHealth::UartInitFailed));
                defmt::warn!("GPS: PORT_GPS is not a supported GPS port on this board");
            }
        }
    }

    // --- I2C1 shared bus (PB6 SCL, PB7 SDA): SPL06 baro (0x76) + QMC5883L mag (0x0D) ---
    // Per docs/sensor_bus_sharing.md: initialize ALL devices on a shared I2C bus
    // before spawning ANY sensor reader tasks.
    {
        // Unstick I2C1 (`PB6` SCL, `PB7` SDA) before `I2c::new` in case a warm
        // MCU reset (`reboot`, `reboot --dfu`, or `IWDG`) interrupted an
        // in-flight QMC5883L (200 Hz) or SPL06 (32 Hz) byte read while 3.3V/5V
        // remained powered, leaving a slave holding SDA low (I2C spec §3.1.16 /
        // ArduPilot `I2CBus::clear_bus`: open-drain SCL recovery clocks + STOP).
        let mut i2c1_scl = board.i2c.i2c1_scl;
        let mut i2c1_sda = board.i2c.i2c1_sda;
        {
            let mut scl = Flex::new(i2c1_scl.reborrow());
            let mut sda = Flex::new(i2c1_sda.reborrow());
            scl.set_high();
            sda.set_high();
            scl.set_as_input_output_pull(Speed::Medium, hal::gpio::Pull::Up);
            sda.set_as_input_output_pull(Speed::Medium, hal::gpio::Pull::Up);
            Timer::after_micros(20).await;
            for _round in 0..2 {
                // Clock 9 full SCL pulses with SDA released high (NXP UM10204 §3.1.16)
                // so any slave mid-read shifts out all remaining data bits and samples
                // Master NACK (SDA=1) on the 9th clock, rather than breaking on an
                // intermediate '1' data bit whose next SCL falling edge shifts a '0'.
                sda.set_high();
                Timer::after_micros(20).await;
                for _ in 0..9 {
                    scl.set_low();
                    Timer::after_micros(20).await;
                    scl.set_high();
                    Timer::after_micros(20).await;
                }
                // Generate clean I2C STOP condition (SDA Low -> High while SCL is High)
                scl.set_low();
                Timer::after_micros(20).await;
                sda.set_low();
                Timer::after_micros(20).await;
                scl.set_high();
                Timer::after_micros(20).await;
                sda.set_high();
                Timer::after_micros(20).await;
                if sda.is_high() && scl.is_high() {
                    break;
                }
            }
        }

        static I2C1_BUS: StaticCell<I2cBusMtx> = StaticCell::new();
        let mut i2c_config = hal::i2c::Config::default();
        i2c_config.frequency = Hertz(400_000);
        i2c_config.sda_pullup = true;
        i2c_config.scl_pullup = true;
        // `50 ms` balances both constraints:
        // - 200x a 0.25 ms 8-byte 400 kHz transfer (avoids false Thread-Executor
        //   scheduling timeouts that `20 ms` hit under P6/P8/P10 preemption), and
        // - `2 * 50 ms = 100 ms < 141 ms` worst-case 47 kHz LSI `IWDG` margin even
        //   if an external GPS-mast `I2C1` wire shorts `SCL`/`SDA` low in flight
        //   (where `embassy-stm32`'s `master_write`/`wait_tc` busy-spin up to
        //   `timeout`). Any aborted `write_read` (`TC=1, AUTOEND=0, BUSY=1`) is
        //   immediately cleared by `reset_i2c1_if_wedged()`.
        i2c_config.timeout = Duration::from_millis(50);
        let i2c1 = hal::i2c::I2c::new(
            board.i2c.i2c1,
            i2c1_scl,
            i2c1_sda,
            I2c1Irqs,
            board.i2c.i2c1_tx_dma,
            board.i2c.i2c1_rx_dma,
            i2c_config,
        );
        Timer::after_millis(5).await;
        let i2c1_bus: &'static I2cBusMtx = I2C1_BUS.init(Mutex::<NoopRawMutex, _>::new(i2c1));

        // 1. Probe & init QMC5883L external compass (0x0D) — bounded per-op by `i2c_config.timeout`
        // (never wrap `embassy-stm32` `I2c` calls in an outer `with_timeout` that could cancel
        // `write_read` between `write_dma_internal(send_stop=false)` and `read_dma_internal`).
        let mut probe_dev = I2cDevice::new(i2c1_bus);
        let qmc_found = Qmc5883l::probe(&mut probe_dev).await;
        reset_i2c1_if_wedged();
        let mag_opt = if qmc_found {
            defmt::info!("I2C1: QMC5883L found, initializing...");
            let dev = I2cDevice::new(i2c1_bus);
            let mut delay = embassy_time::Delay;
            match Qmc5883l::new(dev, &mut delay).await {
                Ok(mag) => {
                    defmt::info!("QMC5883L init OK");
                    Some(mag)
                }
                Err(e) => {
                    reset_i2c1_if_wedged();
                    defmt::warn!("QMC5883L init failed: {}", e);
                    None
                }
            }
        } else {
            defmt::warn!("QMC5883L not detected on I2C1 (addr 0x0D)");
            None
        };

        // 2. Probe & init SPL06 barometer (0x76)
        let mut probe_dev = I2cDevice::new(i2c1_bus);
        let spl_found = Spl06::probe_i2c(&mut probe_dev, bsp::sensors::BARO_1_I2C_ADDR).await;
        reset_i2c1_if_wedged();
        let baro_opt = if spl_found {
            defmt::info!(
                "I2C1: SPL06 found at {:#x}, initializing...",
                bsp::sensors::BARO_1_I2C_ADDR
            );
            let dev = I2cDevice::new(i2c1_bus);
            let mut delay = embassy_time::Delay;
            match Spl06::new_i2c(dev, bsp::sensors::BARO_1_I2C_ADDR, &mut delay).await {
                Ok(baro) => {
                    defmt::info!("SPL06 init OK");
                    Some(baro)
                }
                Err(e) => {
                    reset_i2c1_if_wedged();
                    defmt::warn!("SPL06 init failed: {}", e);
                    None
                }
            }
        } else {
            defmt::warn!(
                "SPL06 not detected on I2C1 (addr {:#x})",
                bsp::sensors::BARO_1_I2C_ADDR
            );
            None
        };

        // 3. Spawn tasks only after all shared-bus device initialization completes
        if let Some(mag) = mag_opt {
            spawner
                .spawn(crate::sensors::mag::qmc5883l_mag_task(MagReader::new(
                    mag,
                    board.sensors.mag_align,
                    mag_hard_iron,
                )))
                .unwrap_or_else(|e| defmt::error!("Failed to spawn QMC5883L task: {}", e));
        }
        if let Some(baro) = baro_opt {
            spawner
                .spawn(crate::sensors::baro::spl06_i2c_baro_task(
                    BaroReader::new(baro),
                    &crate::sensors::BARO_1,
                ))
                .unwrap_or_else(|e| defmt::error!("Failed to spawn SPL06 baro task: {}", e));
        }
    }

    // --- Offboard / ESP bridge (`PORT_ESP_BRIDGE`, default USART1 PA9/PA10 @ 460800 baud) ---
    match bsp::PORT_ESP_BRIDGE {
        bsp::SerialPortId::Usart1 => {
            let mut uart_config = hal::usart::Config::default();
            uart_config.baudrate = 460_800;
            uart_config.rx_pull = hal::gpio::Pull::Up;

            match hal::usart::Uart::new(
                board.serial.usart1,
                board.serial.usart1_rx,
                board.serial.usart1_tx,
                Usart1Irqs,
                board.serial.usart1_tx_dma,
                board.serial.usart1_rx_dma,
                uart_config,
            ) {
                Ok(uart) => {
                    let (tx, rx) = uart.split();
                    defmt::info!("ESP/Offboard bridge USART1 init OK (DMA, 460800 baud)");
                    spawner
                        .spawn(crate::comm::esp_bridge::esp_bridge_rx_task(rx))
                        .unwrap_or_else(|e| defmt::error!("Failed to spawn ESP bridge RX: {}", e));
                    spawner
                        .spawn(crate::comm::esp_bridge::esp_bridge_tx_task(tx))
                        .unwrap_or_else(|e| defmt::error!("Failed to spawn ESP bridge TX: {}", e));
                }
                Err(e) => defmt::error!("ESP bridge USART1 init failed: {}", e),
            }
        }
        _ => defmt::warn!("ESP bridge: PORT_ESP_BRIDGE is not a supported port on this board"),
    }

    // --- Dual-timer Bidirectional DShot600 motor output (TIM3 + TIM1) ---
    let timer3 = LLTimer::new(board.motors.tim3);
    let tim3_regs = timer3.regs_gp16();
    core::mem::forget(timer3);

    // TIM1 is an advanced-control timer — enable BDTR.MOE so outputs are active.
    let timer1 = LLTimer::new(board.motors.tim1);
    let tim1_regs = timer1.regs_gp16();
    timer1.regs_advanced().bdtr().modify(|w| w.set_moe(true));
    core::mem::forget(timer1);

    let af = AfType::output(OutputType::PushPull, Speed::Low);
    macro_rules! pin_af {
        ($pin:expr, $af_num:expr) => {{
            let mut flex = Flex::new($pin);
            flex.set_as_af_unchecked($af_num, af);
            core::mem::forget(flex);
        }};
    }
    pin_af!(board.motors.m1, 2); // PB0  AF2 = TIM3_CH3 (M1 Rear-Right CW)
    pin_af!(board.motors.m2, 2); // PB1  AF2 = TIM3_CH4 (M2 Front-Right CCW)
    pin_af!(board.motors.m3, 1); // PE9  AF1 = TIM1_CH1 (M3 Rear-Left CCW)
    pin_af!(board.motors.m4, 1); // PE11 AF1 = TIM1_CH2 (M4 Front-Left CW)

    let dshot_config = DshotQuadConfig {
        motors: [
            MotorTimerConfig {
                timer_regs: tim3_regs,
                channel_index: 2,
                dma_request: 5, // DMA1 Stream 7 Channel 5 = TIM3_CH3
                gpio_port: hal::pac::GPIOB,
                gpio_pin: 0,
                af_number: 2,
            },
            MotorTimerConfig {
                timer_regs: tim3_regs,
                channel_index: 3,
                dma_request: 5, // DMA1 Stream 2 Channel 5 = TIM3_CH4
                gpio_port: hal::pac::GPIOB,
                gpio_pin: 1,
                af_number: 2,
            },
            MotorTimerConfig {
                timer_regs: tim1_regs,
                channel_index: 0,
                dma_request: 6, // DMA2 Stream 1 Channel 6 = TIM1_CH1
                gpio_port: hal::pac::GPIOE,
                gpio_pin: 9,
                af_number: 1,
            },
            MotorTimerConfig {
                timer_regs: tim1_regs,
                channel_index: 1,
                dma_request: 6, // DMA2 Stream 2 Channel 6 = TIM1_CH2
                gpio_port: hal::pac::GPIOE,
                gpio_pin: 11,
                af_number: 1,
            },
        ],
        timers: [tim3_regs, tim1_regs],
        timer_count: 2,
    };

    high_spawner
        .spawn(crate::motors::dshot::dshot_task(
            dshot_config,
            board.motors.m1_dma,
            board.motors.m2_dma,
            board.motors.m3_dma,
            board.motors.m4_dma,
        ))
        .unwrap_or_else(|_| defmt::error!("Failed to spawn DShot task"));

    // --- Power monitoring (ADC3: VBAT = PC3 IN13, CURR = PC2 IN12) ---
    let power_mon = PowerMonitor::new(
        board.adc.adc3,
        board.adc.vbat,
        board.adc.curr,
        crate::sensors::power::AdcInput {
            gpioc_pin: 3,
            channel: 13,
        },
        crate::sensors::power::AdcInput {
            gpioc_pin: 2,
            channel: 12,
        },
        bsp::POWER_CAL,
    );
    spawner
        .spawn(power_task(power_mon))
        .unwrap_or_else(|_| defmt::error!("Failed to spawn power task"));
}

#[cfg(all(feature = "est_pos_gps", not(feature = "gps_unicore")))]
async fn init_gps_uart(mut uart: hal::usart::BufferedUart<'static>, spawner: &Spawner) {
    use core::sync::atomic::Ordering;
    use crate::sensors::gps::{
        GPS_DETECTED_BAUD, GPS_HEALTH, GPS_PROBE_RX_BYTES, GpsHealth, err_kind,
    };
    use embedded_io_async::{ErrorType, Read, Write};

    struct CountingRw<'a, T>(&'a mut T);
    impl<T: ErrorType> ErrorType for CountingRw<'_, T> {
        type Error = T::Error;
    }
    impl<T: Read> Read for CountingRw<'_, T> {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = self.0.read(buf).await?;
            GPS_PROBE_RX_BYTES.fetch_add(n as u32, Ordering::Relaxed);
            Ok(n)
        }
    }
    impl<T: Write> Write for CountingRw<'_, T> {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.0.write(buf).await
        }
        async fn flush(&mut self) -> Result<(), Self::Error> {
            self.0.flush().await
        }
    }

    GPS_PROBE_RX_BYTES.store(0, Ordering::Relaxed);
    GPS_DETECTED_BAUD.store(0, Ordering::Relaxed);
    Timer::after_millis(250).await;

    const CANDIDATE_BAUDS: [u32; 5] = [115_200, 230_400, 38_400, 57_600, 9_600];
    let mut last_err = None;

    for &baud in &CANDIDATE_BAUDS {
        let _ = uart.set_baudrate(baud);
        // Drain any stale / wrong-baud framing bytes from the RX ring
        let mut drain_buf = [0u8; 64];
        while let Ok(Ok(n)) = with_timeout(
            Duration::from_millis(10),
            embedded_io_async::Read::read(&mut uart, &mut drain_buf),
        )
        .await
        {
            if n == 0 {
                break;
            }
            GPS_PROBE_RX_BYTES.fetch_add(n as u32, Ordering::Relaxed);
        }

        let mut rw = CountingRw(&mut uart);
        match with_timeout(Duration::from_millis(550), GpsDriver::configure_once(&mut rw)).await {
            Ok(Ok(())) => {
                GPS_DETECTED_BAUD.store(baud, Ordering::Relaxed);
                defmt::info!("GPS u-blox init OK at {} baud", baud);
                spawner
                    .spawn(crate::sensors::gps::gps_task(GpsRunner::new(
                        GpsDriver::from_configured(uart),
                    )))
                    .unwrap_or_else(|e| defmt::error!("Failed to spawn GPS task: {}", e));
                return;
            }
            Ok(Err(e)) => {
                last_err = Some(err_kind(&e));
            }
            Err(_) => {}
        }
    }

    if let Some(k) = last_err {
        GPS_HEALTH.lock(|c| c.set(GpsHealth::InitFailed(k)));
        defmt::warn!("GPS init failed across all candidate baudrates");
    } else {
        GPS_HEALTH.lock(|c| c.set(GpsHealth::InitTimedOut));
        defmt::warn!(
            "GPS init timed out across all baudrates (rx_bytes={})",
            GPS_PROBE_RX_BYTES.load(Ordering::Relaxed)
        );
    }
}

#[cfg(all(feature = "est_pos_gps", feature = "gps_unicore"))]
async fn init_gps_uart(uart: hal::usart::BufferedUart<'static>, spawner: &Spawner) {
    use core::sync::atomic::Ordering;
    use crate::sensors::gps::{GPS_DETECTED_BAUD, GPS_HEALTH, GpsHealth, err_kind};
    let mut delay = embassy_time::Delay;
    match with_timeout(Duration::from_secs(3), GpsDriver::new(uart, &mut delay)).await {
        Ok(Ok(gps)) => {
            GPS_DETECTED_BAUD.store(115_200, Ordering::Relaxed);
            defmt::info!("GPS Unicore init OK at 115200 baud");
            spawner
                .spawn(crate::sensors::gps::gps_task(GpsRunner::new(gps)))
                .unwrap_or_else(|e| defmt::error!("Failed to spawn GPS task: {}", e));
        }
        Ok(Err(e)) => {
            GPS_HEALTH.lock(|c| c.set(GpsHealth::InitFailed(err_kind(&e))));
        }
        Err(_) => {
            GPS_HEALTH.lock(|c| c.set(GpsHealth::InitTimedOut));
        }
    }
}

/// Clear a wedged STM32F7 `I2C1` peripheral state machine (`RM0385 §30.4.6`)
/// when called synchronously between `I2cDevice` transactions on the Thread
/// Executor (zero active `I2C1_BUS` lock holders).
///
/// If `embassy-stm32` `i2c/v2.rs::write_read` ever aborts on timeout or bus
/// error after `write_dma_internal(..., send_stop = false)`, the peripheral
/// remains stuck in `TC=1, AUTOEND=0, ISR.BUSY=1` stretching `SCL` low. Toggling
/// `CR1.PE` `0 -> 1` (held low for >= 3 APB1 clocks; `216` CPU cycles = `1 us`
/// = `54` APB1 clocks at `54 MHz`) resets all internal `I2C1` state machines and
/// `ISR` flags while preserving `TIMINGR`.
pub fn reset_i2c1_if_wedged() {
    let regs = hal::pac::I2C1;
    let isr = regs.isr().read();
    if isr.busy() || isr.tc() || isr.tcr() || regs.cr2().read().start() {
        critical_section::with(|_| {
            regs.cr1().modify(|w| {
                w.set_txdmaen(false);
                w.set_rxdmaen(false);
                w.set_tcie(false);
                w.set_nackie(false);
                w.set_errie(false);
                w.set_pe(false);
            });
            let _ = regs.cr1().read();
        });
        cortex_m::asm::delay(216);
        critical_section::with(|_| {
            regs.cr1().modify(|w| w.set_pe(true));
            let _ = regs.cr1().read();
        });
    }
}
