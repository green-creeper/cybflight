//! DShot600 bidirectional protocol layer.
//!
//! Pure protocol logic: frame encoding, GCR telemetry decode, and eRPM/EDT
//! interpretation. Hardware-agnostic and fully host-testable.

pub mod frame;
pub mod gcr;
pub mod telemetry;

// ---------------------------------------------------------------------------
// DShot command values (from dshot_command.h)
// ---------------------------------------------------------------------------

pub const DSHOT_CMD_MOTOR_STOP: u16 = 0;

// ---------------------------------------------------------------------------
// Throttle range
// ---------------------------------------------------------------------------

pub const DSHOT_MIN_THROTTLE: u16 = 48;
pub const DSHOT_MAX_THROTTLE: u16 = 2047;

// ---------------------------------------------------------------------------
// Buffer sizes
// ---------------------------------------------------------------------------

/// DMA buffer: 16 data bits + 2 trailing reset words.
pub const DSHOT_DMA_BUFFER_SIZE: usize = 18;

/// Minimum number of edges in a valid GCR telemetry packet.
pub const MIN_GCR_EDGES: usize = 7;

/// Maximum number of edges in a valid GCR telemetry packet.
pub const MAX_GCR_EDGES: usize = 22;

/// GCR ticks per bit at DShot600 (12 MHz clock, 750 kbit/s = 5/4 × 600 kbit/s).
/// 12_000_000 / 750_000 = 16 ticks.
pub const DSHOT600_GCR_TICKS_PER_BIT: u32 = 16;
