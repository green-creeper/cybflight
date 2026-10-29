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
use embassy_executor::{SendSpawner, Spawner};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
#[cfg(feature = "est_pos_gps")]
use embassy_time::{Duration, with_timeout};
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

pub async fn init(
    spawner: &Spawner,
    ctrl_spawner: &SendSpawner,
    high_spawner: &SendSpawner,
    board: bsp::Board,
) {
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

    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(8_000_000);
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
                        defmt::info!("IMU1 init OK (MPU6000 on SPI4)");
                        ctrl_spawner
                            .spawn(mpu_reader_task(
                                ImuReader::new(
                                    imu1,
                                    board.sensors.gyro1_align,
                                    accel_cutoff_hz,
                                    gyro_cutoff_hz,
                                ),
                                &crate::sensors::IMU_1,
                                Some(&crate::sensors::IMU_1_RAW),
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
                    defmt::info!("IMU1 init OK (ICM426xx on SPI4)");
                    defmt::assert!(
                        imu1.sample_rate_hz() == crate::rates::IMU_ODR_HZ,
                        "IMU1 ODR does not match rates::IMU_ODR_HZ"
                    );
                    ctrl_spawner
                        .spawn(icm_reader_task(
                            ImuReader::new(
                                imu1,
                                board.sensors.gyro1_align,
                                accel_cutoff_hz,
                                gyro_cutoff_hz,
                            ),
                            &crate::sensors::IMU_1,
                            Some(&crate::sensors::IMU_1_RAW),
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
                    defmt::info!("IMU1 init OK (BMI270 on SPI4)");
                    ctrl_spawner
                        .spawn(bmi270_reader_task(
                            ImuReader::new(
                                imu1,
                                board.sensors.gyro1_align,
                                accel_cutoff_hz,
                                gyro_cutoff_hz,
                            ),
                            &crate::sensors::IMU_1,
                            Some(&crate::sensors::IMU_1_RAW),
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

    static TX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0u8; 128])[..];
    let rx_buf = &mut RX_BUF.init([0u8; 128])[..];
    let mut rc_uart_cfg = hal::usart::Config::default();
    rc_uart_cfg.baudrate = 420_000;

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
        use crate::sensors::gps::{GPS_HEALTH, GpsHealth, err_kind};
        GPS_HEALTH.lock(|c| c.set(GpsHealth::Initializing));
        match bsp::PORT_GPS {
            bsp::SerialPortId::Usart6 => {
                defmt::info!("GPS: starting USART6 init (T6=PC6, R6=PC7)");
                static GPS_TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
                static GPS_RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
                let tx_buf = &mut GPS_TX_BUF.init([0u8; 256])[..];
                let rx_buf = &mut GPS_RX_BUF.init([0u8; 256])[..];
                let mut uart_config = hal::usart::Config::default();
                uart_config.baudrate = 115_200;
                match hal::usart::BufferedUart::new(
                    board.serial.usart6,
                    board.serial.usart6_rx,
                    board.serial.usart6_tx,
                    tx_buf,
                    rx_buf,
                    SerialIrqs,
                    uart_config,
                ) {
                    Ok(uart) => {
                        let mut delay = embassy_time::Delay;
                        match with_timeout(Duration::from_secs(3), GpsDriver::new(uart, &mut delay))
                            .await
                        {
                            Ok(Ok(gps)) => {
                                defmt::info!("GPS u-blox init OK on USART6");
                                spawner
                                    .spawn(crate::sensors::gps::gps_task(GpsRunner::new(gps)))
                                    .unwrap_or_else(|e| {
                                        defmt::error!("Failed to spawn GPS task: {}", e)
                                    });
                            }
                            Ok(Err(e)) => {
                                GPS_HEALTH.lock(|c| c.set(GpsHealth::InitFailed(err_kind(&e))));
                                defmt::warn!("GPS init failed: {}", e);
                            }
                            Err(_) => {
                                GPS_HEALTH.lock(|c| c.set(GpsHealth::InitTimedOut));
                                defmt::warn!("GPS init timed out (no module?)");
                            }
                        }
                    }
                    Err(e) => {
                        GPS_HEALTH.lock(|c| c.set(GpsHealth::UartInitFailed));
                        defmt::error!("GPS USART6 init failed: {}", e);
                    }
                }
            }
            bsp::SerialPortId::Usart3 => {
                defmt::info!("GPS: starting USART3 init (T3=PB10, R3=PB11)");
                static GPS_TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
                static GPS_RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
                let tx_buf = &mut GPS_TX_BUF.init([0u8; 256])[..];
                let rx_buf = &mut GPS_RX_BUF.init([0u8; 256])[..];
                let mut uart_config = hal::usart::Config::default();
                uart_config.baudrate = 115_200;
                match hal::usart::BufferedUart::new(
                    board.serial.usart3,
                    board.serial.usart3_rx,
                    board.serial.usart3_tx,
                    tx_buf,
                    rx_buf,
                    SerialIrqs,
                    uart_config,
                ) {
                    Ok(uart) => {
                        let mut delay = embassy_time::Delay;
                        match with_timeout(Duration::from_secs(3), GpsDriver::new(uart, &mut delay))
                            .await
                        {
                            Ok(Ok(gps)) => {
                                defmt::info!("GPS u-blox init OK on USART3");
                                spawner
                                    .spawn(crate::sensors::gps::gps_task(GpsRunner::new(gps)))
                                    .unwrap_or_else(|e| {
                                        defmt::error!("Failed to spawn GPS task: {}", e)
                                    });
                            }
                            Ok(Err(e)) => {
                                GPS_HEALTH.lock(|c| c.set(GpsHealth::InitFailed(err_kind(&e))));
                                defmt::warn!("GPS init failed: {}", e);
                            }
                            Err(_) => {
                                GPS_HEALTH.lock(|c| c.set(GpsHealth::InitTimedOut));
                                defmt::warn!("GPS init timed out (no module?)");
                            }
                        }
                    }
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
        static I2C1_BUS: StaticCell<I2cBusMtx> = StaticCell::new();
        let mut i2c_config = hal::i2c::Config::default();
        i2c_config.frequency = Hertz(400_000);
        let i2c1 = hal::i2c::I2c::new(
            board.i2c.i2c1,
            board.i2c.i2c1_scl,
            board.i2c.i2c1_sda,
            I2c1Irqs,
            board.i2c.i2c1_tx_dma,
            board.i2c.i2c1_rx_dma,
            i2c_config,
        );
        let i2c1_bus: &'static I2cBusMtx = I2C1_BUS.init(Mutex::<NoopRawMutex, _>::new(i2c1));

        // 1. Probe & init QMC5883L external compass (0x0D)
        let mut probe_dev = I2cDevice::new(i2c1_bus);
        let mag_opt = if Qmc5883l::probe(&mut probe_dev).await {
            defmt::info!("I2C1: QMC5883L found, initializing...");
            let dev = I2cDevice::new(i2c1_bus);
            let mut delay = embassy_time::Delay;
            match Qmc5883l::new(dev, &mut delay).await {
                Ok(mag) => {
                    defmt::info!("QMC5883L init OK");
                    Some(mag)
                }
                Err(e) => {
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
        let baro_opt = if Spl06::probe_i2c(&mut probe_dev, bsp::sensors::BARO_1_I2C_ADDR).await {
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
