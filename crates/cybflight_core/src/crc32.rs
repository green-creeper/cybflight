//! Compact IEEE 802.3 CRC-32 (`64 B` nibble LUT).
//!
//! Replaces `crc32fast`'s `16 KiB` slicing-by-16 lookup table to save Flash
//! `.rodata` on `512 KiB` MCUs while producing identical CRC-32 checksums.
//! Zero allocations, `no_std`, and safe inside `#[panic_handler]` / `HardFault`.

const fn make_nibble_table() -> [u32; 16] {
    let mut table = [0u32; 16];
    let mut i = 0;
    while i < 16 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 4 {
            c = if (c & 1) != 0 {
                0xEDB8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
            k += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
}

const NIBBLE_TABLE: [u32; 16] = make_nibble_table();

/// Incremental IEEE 802.3 CRC-32 hasher.
#[derive(Clone, Copy, Debug)]
pub struct Hasher {
    state: u32,
}

impl Default for Hasher {
    fn default() -> Self {
        Self::new()
    }
}

impl Hasher {
    /// Start a fresh CRC-32 computation.
    pub const fn new() -> Self {
        Self { state: 0xFFFF_FFFF }
    }

    /// Feed `bytes` into the running CRC-32 state.
    pub const fn update_const(mut self, bytes: &[u8]) -> Self {
        let mut i = 0;
        while i < bytes.len() {
            let b = bytes[i];
            self.state =
                NIBBLE_TABLE[((self.state ^ (b as u32)) & 0x0F) as usize] ^ (self.state >> 4);
            self.state =
                NIBBLE_TABLE[((self.state ^ ((b >> 4) as u32)) & 0x0F) as usize] ^ (self.state >> 4);
            i += 1;
        }
        self
    }

    /// Feed `bytes` into the hasher.
    pub fn update(&mut self, bytes: &[u8]) {
        *self = self.update_const(bytes);
    }

    /// Finalize and return the IEEE 802.3 CRC-32 digest.
    pub const fn finalize(self) -> u32 {
        !self.state
    }
}

/// Compute the IEEE 802.3 CRC-32 digest of `bytes` in a single pass.
pub const fn hash(bytes: &[u8]) -> u32 {
    Hasher::new().update_const(bytes).finalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_and_check_vector() {
        assert_eq!(hash(b""), 0x0000_0000);
        assert_eq!(hash(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn incremental_matches_oneshot() {
        let msg = b"cybflight-nor-blackbox-crc32-check";
        let mut h = Hasher::new();
        h.update(&msg[..7]);
        h.update(&msg[7..19]);
        h.update(&msg[19..]);
        assert_eq!(h.finalize(), hash(msg));
    }
}
