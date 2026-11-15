#![no_std]

/// Sensor orientation tags from Betaflight — full set of alignment variants.
#[derive(Copy, Clone, Debug, Eq, PartialEq, defmt::Format)]
pub enum SensorAlign {
    Default,
    Cw0Deg,
    Cw90Deg,
    Cw180Deg,
    Cw270Deg,
    Cw0DegFlip,
    Cw90DegFlip,
    Cw180DegFlip,
    Cw270DegFlip,
}

/// A concrete serial port ID — full STM32H7 superset.
#[derive(Copy, Clone, Debug, Eq, PartialEq, defmt::Format)]
pub enum SerialPortId {
    Usart1,
    Usart2,
    Usart3,
    Uart4,
    Uart5,
    Usart6,
    Uart7,
    Uart8,
    Lpuart1,
}

/// Battery/current ADC calibration constants from Betaflight target config.
#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct PowerCalibration {
    /// Betaflight `vbatscale` (dimensionless ADC→voltage scale factor).
    pub voltage_scale: u16,
    /// Betaflight `vbatresdivval` (resistor divider ratio denominator).
    pub voltage_divider: u16,
    /// Betaflight `vbatresdivmultiplier` (extra multiplier, usually 1).
    pub voltage_multiplier: u16,
    /// Betaflight `current_meter_scale` (mV per 10 A).
    pub current_scale: u16,
    /// Betaflight `current_meter_offset` (centiamps).
    pub current_offset: i16,
}
