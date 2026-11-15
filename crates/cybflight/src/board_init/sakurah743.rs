#[cfg(feature = "est_pos_gps")]
use cybflight_drivers::gps::GpsDriver;
use cybflight_drivers::imu::ReadImu;
use cybflight_drivers::imu::icm426xx::Icm426xx;
use cybflight_drivers::led::Led;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::{SendSpawner, Spawner};
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
#[cfg(feature = "est_pos_gps")]
use embassy_time::{Duration, with_timeout};
use static_cell::StaticCell;

use crate::bsp;
use crate::hal;
use crate::motors::{DshotQuadConfig, MotorTimerConfig};
#[cfg(feature = "est_pos_gps")]
use crate::sensors::gps::{GPS_HEALTH, GpsHealth, GpsRunner, err_kind};
use crate::sensors::imu::{ImuReader, SpiBusMtx, icm_reader_task};
use crate::sensors::power::{PowerMonitor, power_task};
use crate::status;
use crate::usb_serial;
use hal::gpio::{AfType, Flex, OutputType, Speed};
use hal::spi::{self, Spi};
use hal::time::Hertz;
use hal::timer::low_level::Timer as LLTimer;

// Single interrupt struct covering all buffered serial UARTs used as role
// candidates. board_init dispatches based on the bsp::PORT_* constants
// defined in the BSP port mapping table; LLVM eliminates dead match arms
// since the discriminants are compile-time constants.
hal::bind_interrupts!(struct SerialIrqs {
    UART4 => hal::usart::BufferedInterruptHandler<hal::peripherals::UART4>;
    USART3 => hal::usart::BufferedInterruptHandler<hal::peripherals::USART3>;
    USART2 => hal::usart::BufferedInterruptHandler<hal::peripherals::USART2>;
    UART7 => hal::usart::BufferedInterruptHandler<hal::peripherals::UART7>;
});

// Bind USART1 interrupt for ESP bridge (DMA UART)
hal::bind_interrupts!(struct Usart1Irqs {
    USART1 => hal::usart::InterruptHandler<hal::peripherals::USART1>;
});

// Bind SDMMC1 interrupt for the touch_sd / blackbox storage pipeline.
// Gated on the BSP capability flag, not a Cargo feature — boards
// without storage compile out the binding cleanly.
hal::bind_interrupts!(struct SdmmcIrqs {
    SDMMC1 => hal::sdmmc::InterruptHandler<hal::peripherals::SDMMC1>;
});

/// Board initialization.
///
/// Spawner routing:
/// - `spawner` (thread): LED, USB, RC reader, mag, baro, GPS, ESP bridge.
///   These tolerate latency; they run alongside the mission planner.
/// - `ctrl_spawner` (interrupt P10): **IMU readers**. Same executor as
///   the downstream attitude filter, ESKF, INDI and MPC — so the whole
///   sensor→estimation→control chain preempts any thread work (notably
///   the trajectory planner's BFGS solve). The SPI bus mutex type
///   (`SpiBusMtx`) was upgraded from `NoopRawMutex` to
///   `CriticalSectionRawMutex` to make the task `Send` for the
///   interrupt-executor spawn.
/// - `high_spawner` (interrupt P6): DShot motor output.
pub async fn init(
    spawner: &Spawner,
    ctrl_spawner: &SendSpawner,
    high_spawner: &SendSpawner,
    board: bsp::Board,
) {
    {
        use hal::interrupt::{self, InterruptExt, Priority};
        // Keep slow comms/SDMMC below P10 ctrl and EXTI2 DRDY at P5.
        interrupt::EXTI2.set_priority(Priority::P5);
        interrupt::OTG_FS.set_priority(Priority::P12);
        interrupt::USART1.set_priority(Priority::P12);
        interrupt::USART2.set_priority(Priority::P12);
        interrupt::USART3.set_priority(Priority::P12);
        interrupt::UART4.set_priority(Priority::P12);
        interrupt::UART7.set_priority(Priority::P12);
        interrupt::SDMMC1.set_priority(Priority::P12);
    }

    // --- Load vehicle parameters from flash (or defaults) ---
    crate::params::init_from_flash(board.internal_flash);

    // Sensor topology params (IMU LPF cutoffs) — from the
    // vehicle YAML bake plus any flash overrides. Captured once at init;
    // changes need a reboot.
    let sensor_params = crate::params::get().sensors;
    let accel_cutoff_hz = sensor_params.imu_accel_lpf_hz;
    let gyro_cutoff_hz = sensor_params.imu_gyro_lpf_hz;

    // --- LEDs: turn off led1 & led2, use led0 for status ---
    let mut led1 = Led::new(board.leds.led1, false);
    let mut led2 = Led::new(board.leds.led2, false);
    led1.off();
    led2.off();
    let led0 = Led::new(board.leds.led0, false);
    spawner.spawn(status::task(led0)).unwrap();

    // --- External arm LED (Speedybee 2812 on PA8 / TIM1_CH1 / DMA1_CH7) ---
    // Seed the live atomic from flash, then spawn the WS2812 driver task.
    // The strip stays off until the operator runs `led on` AND the vehicle
    // arms — see `arm_led::compose_frame`.
    crate::arm_led::ARM_LED_ENABLED.store(
        crate::params::get().system.arm_led_enabled,
        core::sync::atomic::Ordering::Relaxed,
    );
    // LED strip is on TIM1_CH1 (PA8/AF1). TIM1 is an advanced-control timer:
    // enable its RCC clock, set BDTR.MOE (required for the output stage), and
    // keep the clock alive (forget) before handing the GP16 regs to the driver.
    let led_timer = LLTimer::new(board.motors.tim1);
    let led_tim_regs = led_timer.regs_gp16();
    led_timer.regs_advanced().bdtr().modify(|w| w.set_moe(true));
    core::mem::forget(led_timer);
    let arm_led_strip = crate::arm_led::ws2812::Ws2812::new(
        led_tim_regs,
        board.motors.led_strip,
        1, // AF1 = TIM1_CH1
        0, // CH1
        board.motors.dma1_ch7,
        11, // DMAMUX request: TIM1_CH1
    );
    spawner
        .spawn(crate::arm_led::task(arm_led_strip))
        .unwrap_or_else(|e| defmt::error!("Failed to spawn arm LED task: {}", e));

    // --- USB CDC serial ---
    spawner
        .spawn(usb_serial::task(
            board.usb.usb_otg_fs,
            board.usb.dp,
            board.usb.dm,
        ))
        .unwrap();

    // --- Blackbox / flight-data-recorder pipeline ---
    //
    // SDMMC1 (4-bit) on SAKURAH743 → SdmmcBlockStore → blackbox_task.
    // The task owns the SDMMC peripheral for its lifetime and waits
    // on shell-driven request signals (touch_sd, skeleton, ...);
    // nothing touches the card until the user invokes a blackbox op.
    //
    // Compile-time gate: `bsp::HAS_BLACKBOX_STORAGE` is a `const bool`
    // so the entire branch is dead-code-eliminated on boards without
    // storage. No Cargo feature required.
    if bsp::HAS_BLACKBOX_STORAGE {
        let mut sdmmc_cfg = hal::sdmmc::Config::default();
        sdmmc_cfg.data_transfer_timeout = 5_000_000; // generous for low-end cards
        let sdmmc = hal::sdmmc::Sdmmc::new_4bit(
            board.sdio.sdio,
            SdmmcIrqs,
            board.sdio.ck,
            board.sdio.cmd,
            board.sdio.d0,
            board.sdio.d1,
            board.sdio.d2,
            board.sdio.d3,
            sdmmc_cfg,
        );
        let store = crate::blackbox::SdmmcBlockStore::new(sdmmc);
        if let Err(e) = spawner.spawn(crate::blackbox::blackbox_task(store)) {
            defmt::error!("blackbox: spawn failed: {:?}", defmt::Debug2Format(&e));
        }
    }

    // Wait for power to stabilize before touching SPI devices.
    Timer::after_millis(100).await;

    // SPI4 carries IMU1 alone, 24 MHz (ICM-42688-P datasheet max). The
    // 15-byte TEMP/ACCEL/GYRO burst is ~5 µs of bus time against the
    // 125 µs 8 kHz period. History: at the original 1 MHz (with a second
    // INT_STATUS transaction per sample) it was 136 µs — longer than the
    // period — so the reader silently ran at ~2.5 kHz on every build
    // (`imurate`). 24 MHz boot-looped (IWDG) on 2026-08-24 with that
    // two-transaction driver; it was re-benched and kept after the
    // driver moved to a single burst + latched DRDY (docs/imu_fifo.md).
    // If it ever boot-loops again, 8 MHz (foxeer/micoair) is the known-
    // good fallback. embassy picks the nearest prescaler <= 24 MHz.
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(24_000_000);
    spi_config.mode = spi::MODE_3;

    // --- IMU1: ICM42688P on SPI4 (PE12/13/14, CS=PE11, DRDY=PB2) ---
    static SPI4_BUS: StaticCell<SpiBusMtx> = StaticCell::new();
    let spi4 = Spi::new(
        board.spi.spi4,
        board.spi.spi4_sck,
        board.spi.spi4_mosi,
        board.spi.spi4_miso,
        board.spi.spi4_tx_dma,
        board.spi.spi4_rx_dma,
        spi_config,
    );
    let spi4_bus = SPI4_BUS.init(Mutex::new(spi4));
    let dev4 = SpiDevice::new(spi4_bus, board.sensors.gyro1_cs);

    let mut delay = embassy_time::Delay;
    match Icm426xx::new(
        dev4,
        board.sensors.gyro1_drdy,
        &mut delay,
        crate::rates::ICM_ODR,
    )
    .await
    {
        Ok(imu1) => {
            defmt::info!("IMU1 init OK");
            defmt::assert!(
                imu1.sample_rate_hz() == crate::rates::IMU_ODR_HZ,
                "IMU1 ODR does not match rates::IMU_ODR_HZ"
            );
            // IMU reader → control executor (P10). Fresh gyro/accel
            // samples preempt any thread-executor work, including the
            // trajectory planner's BFGS solve.
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
                .unwrap_or_else(|e| defmt::error!("Failed to spawn IMU1 reader task: {}", e));
        }
        Err(e) => defmt::error!("IMU1 init failed: {}", e),
    }

    // --- SerialRx: bsp::PORT_SERIAL_RX selects the UART. ---
    // To move SerialRx to a different UART: change PORT_SERIAL_RX in bsp/sakurah743/src/lib.rs.
    // LLVM eliminates the dead match arms since PORT_SERIAL_RX is a compile-time constant.
    #[cfg(feature = "rx_crsf")]
    match bsp::PORT_SERIAL_RX {
        bsp::SerialPortId::Uart4 => {
            static TX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
            static RX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
            let tx_buf = &mut TX_BUF.init([0u8; 128])[..];
            let rx_buf = &mut RX_BUF.init([0u8; 128])[..];
            let mut uart_config = hal::usart::Config::default();
            uart_config.baudrate = 420_000;
            match hal::usart::BufferedUart::new(
                board.serial.uart4,
                board.serial.uart4_rx,
                board.serial.uart4_tx,
                tx_buf,
                rx_buf,
                SerialIrqs,
                uart_config,
            ) {
                Ok(uart) => {
                    defmt::info!("CRSF UART4 init OK");
                    // CRSF parser on ctrl_spawner (P10): disarm-switch
                    // edge detection lives here (sensors/rc.rs:91-108),
                    // so running on P10 means the emergency-disarm path
                    // cannot be blocked by a thread-resident planner
                    // solve. Worst-case disarm latency ~2-3 ms.
                    ctrl_spawner
                        .spawn(crate::sensors::rc::crsf_runner::crsf_task(uart))
                        .unwrap_or_else(|e| defmt::error!("Failed to spawn CRSF task: {}", e));
                }
                Err(e) => defmt::error!("CRSF UART4 init failed: {}", e),
            }
        }
        _ => defmt::warn!("CRSF: PORT_SERIAL_RX is not a supported SerialRx port on this board"),
    }

    // --- GPS: bsp::PORT_GPS selects the UART. ---
    //
    // Gated on `est_pos_gps`: the receiver init talks to the module with a
    // 30 s timeout that empirically resolves to ~3 s when no module answers,
    // adding that latency to every mocap-only boot for no benefit. With the
    // gate, a non-GPS build skips UART setup entirely and `gpshealth`
    // reports `GpsHealth::NotConfigured`.
    #[cfg(feature = "est_pos_gps")]
    {
        // u-blox runs at 230_400 on this board; the UM982 is configured for
        // 115_200 on its COM1 (the port wired to USART3).
        #[cfg(not(feature = "gps_unicore"))]
        const GPS_BAUD: u32 = 230_400;
        #[cfg(feature = "gps_unicore")]
        const GPS_BAUD: u32 = 115_200;
        GPS_HEALTH.lock(|c| c.set(GpsHealth::Initializing));
        match bsp::PORT_GPS {
            bsp::SerialPortId::Usart3 => {
                defmt::info!("GPS: starting USART3 init");
                static GPS_TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
                static GPS_RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
                let tx_buf = &mut GPS_TX_BUF.init([0u8; 256])[..];
                let rx_buf = &mut GPS_RX_BUF.init([0u8; 1024])[..];
                let mut uart_config = hal::usart::Config::default();
                uart_config.baudrate = GPS_BAUD;
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
                        defmt::info!("GPS: USART3 ready, initializing receiver...");
                        let mut delay = embassy_time::Delay;
                        match with_timeout(
                            Duration::from_secs(30),
                            GpsDriver::new(uart, &mut delay),
                        )
                        .await
                        {
                            Ok(Ok(gps)) => {
                                defmt::info!("GPS receiver init OK");
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

    // --- ESP bridge: bsp::PORT_ESP_BRIDGE selects the UART. ---
    match bsp::PORT_ESP_BRIDGE {
        bsp::SerialPortId::Usart1 => {
            let mut uart_config = hal::usart::Config::default();
            uart_config.baudrate = 921_600;

            match hal::usart::Uart::new(
                board.serial.usart1,
                board.serial.usart1_rx,
                board.serial.usart1_tx,
                Usart1Irqs,
                board.motors.dma1_ch4,
                board.motors.dma1_ch5,
                uart_config,
            ) {
                Ok(uart) => {
                    let (tx, rx) = uart.split();
                    defmt::info!("ESP bridge USART1 init OK (DMA)");
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

    // --- DShot motor output ---

    // Enable RCC for motor timers (Timer::new enables clock + reset)
    let timer5 = LLTimer::new(board.motors.tim5);
    let timer3 = LLTimer::new(board.motors.tim3);
    let tim5_regs = timer5.regs_gp16();
    let tim3_regs = timer3.regs_gp16();
    // Prevent drop from disabling RCC clocks
    core::mem::forget(timer5);
    core::mem::forget(timer3);

    // Configure GPIO as timer AF (board-specific pins + AF numbers)
    let af = AfType::output(OutputType::PushPull, Speed::Low);
    macro_rules! pin_af {
        ($pin:expr, $af_num:expr) => {{
            let mut flex = Flex::new($pin);
            flex.set_as_af_unchecked($af_num, af);
            core::mem::forget(flex);
        }};
    }
    pin_af!(board.motors.m1, 2); // PA2 AF2 = TIM5_CH3
    pin_af!(board.motors.m2, 2); // PA3 AF2 = TIM5_CH4
    pin_af!(board.motors.m3, 2); // PB1 AF2 = TIM3_CH4
    pin_af!(board.motors.m4, 2); // PB0 AF2 = TIM3_CH3

    let dshot_config = DshotQuadConfig {
        motors: [
            MotorTimerConfig {
                timer_regs: tim5_regs,
                channel_index: 2,
                dma_request: 57,
                gpio_port: hal::pac::GPIOA,
                gpio_pin: 2,
            }, // M1: PA2 TIM5_CH3
            MotorTimerConfig {
                timer_regs: tim5_regs,
                channel_index: 3,
                dma_request: 58,
                gpio_port: hal::pac::GPIOA,
                gpio_pin: 3,
            }, // M2: PA3 TIM5_CH4
            MotorTimerConfig {
                timer_regs: tim3_regs,
                channel_index: 3,
                dma_request: 26,
                gpio_port: hal::pac::GPIOB,
                gpio_pin: 1,
            }, // M3: PB1 TIM3_CH4
            MotorTimerConfig {
                timer_regs: tim3_regs,
                channel_index: 2,
                dma_request: 25,
                gpio_port: hal::pac::GPIOB,
                gpio_pin: 0,
            }, // M4: PB0 TIM3_CH3
        ],
        timers: [tim5_regs, tim3_regs],
        timer_count: 2,
    };

    high_spawner
        .spawn(crate::motors::dshot::dshot_task(
            dshot_config,
            board.motors.dma1_ch0,
            board.motors.dma1_ch1,
            board.motors.dma1_ch2,
            board.motors.dma1_ch3,
        ))
        .unwrap_or_else(|_| defmt::error!("Failed to spawn DShot task"));

    // --- Power monitoring (ADC3) ---
    // Placed after DShot so motor control starts even if ADC init has issues.
    // SAKURAH743: VBAT=PC3 (ADC3_INP1), CURR=PC2 (ADC3_INP0).
    let power_mon = PowerMonitor::new(
        board.adc.adc3,
        board.adc.vbat,
        board.adc.curr,
        crate::sensors::power::AdcInput { channel: 1 },
        crate::sensors::power::AdcInput { channel: 0 },
        bsp::POWER_CAL,
    );
    spawner
        .spawn(power_task(power_mon))
        .unwrap_or_else(|_| defmt::error!("Failed to spawn power task"));
}
