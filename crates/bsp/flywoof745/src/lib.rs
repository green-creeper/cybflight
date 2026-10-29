#![no_std]

pub use embassy_stm32 as hal;

use hal::exti;
use hal::gpio::{Level, Output, Pull, Speed};
use hal::time::Hertz;
use hal::{Config, Peripherals, bind_interrupts};

pub use bsp_types::{
    DmaHint, MotorMeta, PowerCalibration, SensorAlign, SerialPortId, TimerChannel, TimerId,
};
pub use cybflight_drivers::beeper::Beeper;

/// Board name matching Flywoo GOKU GN 745 AIO target.
pub const BOARD_NAME: &str = "FLYWOOF745";
/// Manufacturer ID from Betaflight target.
pub const MANUFACTURER_ID: &str = "FLWO";

/// Flywoo GOKU GN 745 AIO beeper on PD15 is active-high (not inverted).
pub const BEEPER_INVERTED: bool = false;

pub const IMU_COUNT: usize = 1;
pub const BARO_COUNT: usize = 1;
pub const HAS_MAG: bool = true;
pub const HAS_OSD: bool = true;
pub const HAS_FLASH: bool = true;
pub const HAS_GPS: bool = true;
pub const HAS_SDCARD: bool = false;
/// SPI-NOR flash (W25Q128 on SPI1) is present, no SDMMC slot.
pub const HAS_BLACKBOX_STORAGE: bool = false;
/// STM32F745 has 4 KB backup SRAM at 0x4002_4000.
pub const HAS_BACKUP_SRAM: bool = true;
pub const LED_COUNT: usize = 1;

/// Primary gyro native output data rate (Hz). MPU6000 / ICM42688P run at 8 kHz.
pub const PRIMARY_GYRO_ODR_HZ: f32 = 8000.0;

/// DMA stream types used for the 4 DShot motor channels on STM32F745:
/// - M1: PB0 (TIM3_CH3) -> DMA1_CH7 (Stream 7, channel 5)
/// - M2: PB1 (TIM3_CH4) -> DMA1_CH2 (Stream 2, channel 5)
/// - M3: PE9 (TIM1_CH1) -> DMA2_CH1 (Stream 1, channel 6)
/// - M4: PE11 (TIM1_CH2) -> DMA2_CH2 (Stream 2, channel 6)
pub type DshotDmaM1 = hal::peripherals::DMA1_CH7;
pub type DshotDmaM2 = hal::peripherals::DMA1_CH2;
pub type DshotDmaM3 = hal::peripherals::DMA2_CH1;
pub type DshotDmaM4 = hal::peripherals::DMA2_CH2;

/// ADC calibration matching `goku_ardu_params.params` (`BATT_VOLT_MULT = 10.9`, `BATT_AMP_PERVLT = 28.5`).
pub const POWER_CAL: PowerCalibration = PowerCalibration {
    voltage_scale: 109,
    voltage_divider: 10,
    voltage_multiplier: 1,
    current_scale: 351,
    current_offset: 0,
};

/// Default UART role assignments (can also be overridden via runtime params `serial_*`):
/// - `USART2` (PD5 TX / PD6 RX): CRSF / ELRS receiver
/// - `USART6` (PC6 TX / PC7 RX): GPS (`SERIAL6_PROTOCOL = 5` in `goku_ardu_params.params`)
/// - `USART1` (PA9 TX / PA10 RX): Offboard / ESP bridge DMA UART (460800 / 921600 baud)
pub const PORT_SERIAL_RX: SerialPortId = SerialPortId::Usart2;
pub const PORT_GPS: SerialPortId = SerialPortId::Usart6;
pub const PORT_ESP_BRIDGE: SerialPortId = SerialPortId::Usart1;

pub mod sensors {
    /// Primary IMU on SPI4 (MPU6000 on V3, also probes ICM42688P and BMI270).
    pub const GYRO_1: &str = "MPU6000";
    /// Barometer on I2C1 (SPL06 at 0x76).
    pub const BARO_1: &str = "SPL06";
    pub const BARO_1_I2C_ADDR: u8 = 0x76;
    /// External compass on I2C1 (QMC5883L at 0x0D).
    pub const MAG: &str = "QMC5883L";
    pub const MAG_I2C_ADDR: u8 = 0x0D;
}

/// Motor metadata table in Betaflight QuadX motor index order (M1..M4):
/// - M1 (Rear-Right CW):  PB0  TIM3_CH3 AF2 (DMA1 Stream 7, Ch 5)
/// - M2 (Front-Right CCW): PB1  TIM3_CH4 AF2 (DMA1 Stream 2, Ch 5)
/// - M3 (Rear-Left CCW):  PE9  TIM1_CH1 AF1 (DMA2 Stream 1, Ch 6)
/// - M4 (Front-Left CW):  PE11 TIM1_CH2 AF1 (DMA2 Stream 2, Ch 6)
pub const MOTOR_META: [MotorMeta; 4] = [
    MotorMeta {
        timer: TimerId::Tim3,
        channel: TimerChannel::Ch3,
        af: 2,
        dma: Some(DmaHint {
            stream: 7,
            request: 5,
        }),
    },
    MotorMeta {
        timer: TimerId::Tim3,
        channel: TimerChannel::Ch4,
        af: 2,
        dma: Some(DmaHint {
            stream: 2,
            request: 5,
        }),
    },
    MotorMeta {
        timer: TimerId::Tim1,
        channel: TimerChannel::Ch1,
        af: 1,
        dma: Some(DmaHint {
            stream: 1,
            request: 6,
        }),
    },
    MotorMeta {
        timer: TimerId::Tim1,
        channel: TimerChannel::Ch2,
        af: 1,
        dma: Some(DmaHint {
            stream: 2,
            request: 6,
        }),
    },
];

// EXTI1 interrupt for Gyro DRDY on PE1:
bind_interrupts!(pub struct ExtiIrqs {
    EXTI1 => exti::InterruptHandler<hal::interrupt::typelevel::EXTI1>;
});

pub struct Leds {
    pub led0: Output<'static>,
}

pub struct SpiPins {
    // SPI1 (PA5 SCK / PA6 MISO / PA7 MOSI): W25Q128 16MB SPI flash (CS = PA4)
    pub spi1: hal::Peri<'static, hal::peripherals::SPI1>,
    pub spi1_sck: hal::Peri<'static, hal::peripherals::PA5>,
    pub spi1_miso: hal::Peri<'static, hal::peripherals::PA6>,
    pub spi1_mosi: hal::Peri<'static, hal::peripherals::PA7>,

    // SPI2 (PB13 SCK / PB14 MISO / PB15 MOSI): AT7456E OSD (CS = PB12)
    pub spi2: hal::Peri<'static, hal::peripherals::SPI2>,
    pub spi2_sck: hal::Peri<'static, hal::peripherals::PB13>,
    pub spi2_miso: hal::Peri<'static, hal::peripherals::PB14>,
    pub spi2_mosi: hal::Peri<'static, hal::peripherals::PB15>,

    // SPI4 (PE2 SCK / PE5 MISO / PE6 MOSI): Primary IMU (CS = PE4, DRDY = PE1)
    // DMA2 Stream 4 (ch 5) TX, DMA2 Stream 0 (ch 4) RX
    pub spi4: hal::Peri<'static, hal::peripherals::SPI4>,
    pub spi4_sck: hal::Peri<'static, hal::peripherals::PE2>,
    pub spi4_miso: hal::Peri<'static, hal::peripherals::PE5>,
    pub spi4_mosi: hal::Peri<'static, hal::peripherals::PE6>,
    pub spi4_tx_dma: hal::Peri<'static, hal::peripherals::DMA2_CH4>,
    pub spi4_rx_dma: hal::Peri<'static, hal::peripherals::DMA2_CH0>,
}

pub struct I2cPins {
    // I2C1 (PB6 SCL / PB7 SDA): SPL06 baro (0x76) + QMC5883L mag (0x0D)
    // DMA1 Stream 6 (ch 1) TX, DMA1 Stream 0 (ch 1) RX
    pub i2c1: hal::Peri<'static, hal::peripherals::I2C1>,
    pub i2c1_scl: hal::Peri<'static, hal::peripherals::PB6>,
    pub i2c1_sda: hal::Peri<'static, hal::peripherals::PB7>,
    pub i2c1_tx_dma: hal::Peri<'static, hal::peripherals::DMA1_CH6>,
    pub i2c1_rx_dma: hal::Peri<'static, hal::peripherals::DMA1_CH0>,
}

pub struct SerialPins {
    pub usart1: hal::Peri<'static, hal::peripherals::USART1>,
    pub usart1_tx: hal::Peri<'static, hal::peripherals::PA9>,
    pub usart1_rx: hal::Peri<'static, hal::peripherals::PA10>,
    pub usart1_tx_dma: hal::Peri<'static, hal::peripherals::DMA2_CH7>,
    pub usart1_rx_dma: hal::Peri<'static, hal::peripherals::DMA2_CH5>,

    pub usart2: hal::Peri<'static, hal::peripherals::USART2>,
    pub usart2_tx: hal::Peri<'static, hal::peripherals::PD5>,
    pub usart2_rx: hal::Peri<'static, hal::peripherals::PD6>,

    pub usart3: hal::Peri<'static, hal::peripherals::USART3>,
    pub usart3_tx: hal::Peri<'static, hal::peripherals::PB10>,
    pub usart3_rx: hal::Peri<'static, hal::peripherals::PB11>,

    pub uart4: hal::Peri<'static, hal::peripherals::UART4>,
    pub uart4_tx: hal::Peri<'static, hal::peripherals::PA0>,
    pub uart4_rx: hal::Peri<'static, hal::peripherals::PA1>,

    pub uart5: hal::Peri<'static, hal::peripherals::UART5>,
    pub uart5_tx: hal::Peri<'static, hal::peripherals::PC12>,
    pub uart5_rx: hal::Peri<'static, hal::peripherals::PD2>,

    pub usart6: hal::Peri<'static, hal::peripherals::USART6>,
    pub usart6_tx: hal::Peri<'static, hal::peripherals::PC6>,
    pub usart6_rx: hal::Peri<'static, hal::peripherals::PC7>,
}

pub struct AdcPins {
    pub adc3: hal::Peri<'static, hal::peripherals::ADC3>,
    pub vbat: hal::Peri<'static, hal::peripherals::PC3>, // ADC3_IN13
    pub curr: hal::Peri<'static, hal::peripherals::PC2>, // ADC3_IN12
    pub rssi: hal::Peri<'static, hal::peripherals::PC5>, // ADC3_IN15
}

pub struct MotorPins {
    pub tim3: hal::Peri<'static, hal::peripherals::TIM3>,
    pub tim1: hal::Peri<'static, hal::peripherals::TIM1>,
    pub tim4: hal::Peri<'static, hal::peripherals::TIM4>,

    pub m1: hal::Peri<'static, hal::peripherals::PB0>, // TIM3_CH3 (AF2)
    pub m2: hal::Peri<'static, hal::peripherals::PB1>, // TIM3_CH4 (AF2)
    pub m3: hal::Peri<'static, hal::peripherals::PE9>, // TIM1_CH1 (AF1)
    pub m4: hal::Peri<'static, hal::peripherals::PE11>, // TIM1_CH2 (AF1)

    pub led_strip: hal::Peri<'static, hal::peripherals::PD12>, // TIM4_CH1 (AF2)

    pub m1_dma: hal::Peri<'static, DshotDmaM1>,
    pub m2_dma: hal::Peri<'static, DshotDmaM2>,
    pub m3_dma: hal::Peri<'static, DshotDmaM3>,
    pub m4_dma: hal::Peri<'static, DshotDmaM4>,
}

pub struct SensorPins {
    pub gyro1_cs: Output<'static>,
    pub gyro1_drdy: exti::ExtiInput<'static>,
    pub gyro1_align: SensorAlign,
    pub mag_i2c_addr: u8,
    pub mag_align: SensorAlign,
}

pub struct FlashPins {
    pub flash_cs: Output<'static>,
}

pub struct OsdPins {
    pub osd_cs: Output<'static>,
}

pub struct UsbPins {
    pub usb_otg_fs: hal::Peri<'static, hal::peripherals::USB_OTG_FS>,
    pub dp: hal::Peri<'static, hal::peripherals::PA12>,
    pub dm: hal::Peri<'static, hal::peripherals::PA11>,
}

bind_interrupts!(pub struct UsbIrqs {
    OTG_FS => hal::usb::InterruptHandler<hal::peripherals::USB_OTG_FS>;
});

pub struct Board {
    pub leds: Leds,
    pub beeper: Beeper<Output<'static>>,

    pub spi: SpiPins,
    pub i2c: I2cPins,
    pub serial: SerialPins,
    pub adc: AdcPins,
    pub motors: MotorPins,
    pub sensors: SensorPins,
    pub flash: FlashPins,
    pub osd: OsdPins,
    pub usb: UsbPins,

    pub internal_flash: hal::Peri<'static, hal::peripherals::FLASH>,
}

/// SYSCLK = 216 MHz on STM32F745VG (8 MHz HSE / 4 * 216 / 2).
pub const SYSCLK_HZ: u32 = 216_000_000;

/// APB2 = HCLK/4 = 54 MHz -> APB2 timer kernel clock = 2 * 54 MHz = 108 MHz.
/// 108 MHz is an exact 9x multiple of the 12 MHz DShot600 timer clock, matching APB1!
pub const APB2_TIMER_HZ: u32 = 108_000_000;

/// APB1 = HCLK/4 = 54 MHz -> APB1 timer kernel clock = 2 * 54 MHz = 108 MHz.
pub const APB1_TIMER_HZ: u32 = 108_000_000;

fn board_config() -> Config {
    let mut config = Config::default();
    {
        use hal::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,   // 8 MHz / 4 = 2 MHz
            mul: PllMul::MUL216,       // 2 MHz * 216 = 432 MHz VCO
            divp: Some(PllPDiv::DIV2), // 432 MHz / 2 = 216 MHz SYSCLK
            divq: Some(PllQDiv::DIV9), // 432 MHz / 9 = 48 MHz USB OTG FS
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1; // 216 MHz HCLK
        config.rcc.apb1_pre = APBPrescaler::DIV4; // 54 MHz PCLK1 (108 MHz TIM3)
        config.rcc.apb2_pre = APBPrescaler::DIV4; // 54 MHz PCLK2 (108 MHz TIM1)
    }
    config
}

pub fn init() -> (Board, hal::usart::UartTx<'static, hal::mode::Blocking>) {
    let p: Peripherals = hal::init(board_config());

    let leds = Leds {
        led0: Output::new(p.PA2, Level::Low, Speed::Low),
    };

    let beeper_pin = Output::new(
        p.PD15,
        if BEEPER_INVERTED {
            Level::High
        } else {
            Level::Low
        },
        Speed::Low,
    );
    let beeper = Beeper::new(beeper_pin, BEEPER_INVERTED);

    let gyro1_cs = Output::new(p.PE4, Level::High, Speed::VeryHigh);
    let flash_cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let osd_cs = Output::new(p.PB12, Level::High, Speed::VeryHigh);

    let gyro1_drdy = exti::ExtiInput::new(p.PE1, p.EXTI1, Pull::None, ExtiIrqs);

    let usb = UsbPins {
        usb_otg_fs: p.USB_OTG_FS,
        dp: p.PA12,
        dm: p.PA11,
    };

    let spi = SpiPins {
        spi1: p.SPI1,
        spi1_sck: p.PA5,
        spi1_miso: p.PA6,
        spi1_mosi: p.PA7,

        spi2: p.SPI2,
        spi2_sck: p.PB13,
        spi2_miso: p.PB14,
        spi2_mosi: p.PB15,

        spi4: p.SPI4,
        spi4_sck: p.PE2,
        spi4_miso: p.PE5,
        spi4_mosi: p.PE6,
        spi4_tx_dma: p.DMA2_CH4,
        spi4_rx_dma: p.DMA2_CH0,
    };

    let i2c = I2cPins {
        i2c1: p.I2C1,
        i2c1_scl: p.PB6,
        i2c1_sda: p.PB7,
        i2c1_tx_dma: p.DMA1_CH6,
        i2c1_rx_dma: p.DMA1_CH0,
    };

    let serial = SerialPins {
        usart1: p.USART1,
        usart1_tx: p.PA9,
        usart1_rx: p.PA10,
        usart1_tx_dma: p.DMA2_CH7,
        usart1_rx_dma: p.DMA2_CH5,

        usart2: p.USART2,
        usart2_tx: p.PD5,
        usart2_rx: p.PD6,

        usart3: p.USART3,
        usart3_tx: p.PB10,
        usart3_rx: p.PB11,

        uart4: p.UART4,
        uart4_tx: p.PA0,
        uart4_rx: p.PA1,

        uart5: p.UART5,
        uart5_tx: p.PC12,
        uart5_rx: p.PD2,

        usart6: p.USART6,
        usart6_tx: p.PC6,
        usart6_rx: p.PC7,
    };

    let adc = AdcPins {
        adc3: p.ADC3,
        vbat: p.PC3,
        curr: p.PC2,
        rssi: p.PC5,
    };

    let motors = MotorPins {
        tim3: p.TIM3,
        tim1: p.TIM1,
        tim4: p.TIM4,
        m1: p.PB0,
        m2: p.PB1,
        m3: p.PE9,
        m4: p.PE11,
        led_strip: p.PD12,
        m1_dma: p.DMA1_CH7,
        m2_dma: p.DMA1_CH2,
        m3_dma: p.DMA2_CH1,
        m4_dma: p.DMA2_CH2,
    };

    let sensors = SensorPins {
        gyro1_cs,
        gyro1_drdy,
        // Base alignment Cw0Deg; arbitrary 3D mounting rotation (e.g. 45° / 225°
        // diamond AIO yaw) is applied via runtime params `imu_rpy_r/p/y` in
        // `vehicles/goku_f745.yaml` or `param set`.
        gyro1_align: SensorAlign::Cw0Deg,
        mag_i2c_addr: sensors::MAG_I2C_ADDR,
        mag_align: SensorAlign::Cw0DegFlip,
    };

    let flash = FlashPins { flash_cs };
    let osd = OsdPins { osd_cs };

    // defmt hardware logger on UART7 TX (PE8) at 921600 baud
    let mut defmt_uart_config = hal::usart::Config::default();
    defmt_uart_config.baudrate = 921_600;
    let defmt_uart = hal::usart::UartTx::new_blocking(p.UART7, p.PE8, defmt_uart_config)
        .expect("defmt UART7 init failed");

    (
        Board {
            leds,
            beeper,
            spi,
            i2c,
            serial,
            adc,
            motors,
            sensors,
            flash,
            osd,
            usb,
            internal_flash: p.FLASH,
        },
        defmt_uart,
    )
}
