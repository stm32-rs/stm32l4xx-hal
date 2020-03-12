//! CRC calculation unit

use crate::rcc;
use crate::stm32::CRC;
use core::ptr;

/// Extension trait to constrain the FLASH peripheral
pub trait CrcExt {
    /// Constrains the CRC peripheral to play nicely with the other abstractions
    fn constrain(self, ahb1: &mut rcc::AHB1) -> Config;
}

impl CrcExt for CRC {
    fn constrain(self, ahb1: &mut rcc::AHB1) -> Config {
        // Enable power to CRC unit
        ahb1.enr().modify(|_, w| w.crcen().set_bit());

        // Default values
        Config {
            initial_value: 0xffff_ffff,
            polynomial: Polynomial::L32(0x04c1_1db7),
            input_bit_reversal: None,
            output_bit_reversal: false,
        }
    }
}

pub enum Polynomial {
    L7(u8),
    L8(u8),
    L16(u16),
    L32(u32),
}

pub enum BitReversal {
    ByByte,
    ByHalfWord,
    ByWord,
}

/// CRC configuration
pub struct Config {
    initial_value: u32,
    polynomial: Polynomial,
    input_bit_reversal: Option<BitReversal>,
    output_bit_reversal: bool,
}

impl Config {
    pub fn initial_value(mut self, init: u32) -> Self {
        self.initial_value = init;

        self
    }

    pub fn polynomial(mut self, polynomial: Polynomial) -> Self {
        self.polynomial = polynomial;

        self
    }

    pub fn input_bit_reversal(mut self, rev: BitReversal) -> Self {
        self.input_bit_reversal = Some(rev);

        self
    }

    pub fn output_bit_reversal(mut self, rev: bool) -> Self {
        self.output_bit_reversal = rev;

        self
    }

    pub fn freeze(self) -> Crc {
        let crc = unsafe { &(*CRC::ptr()) };

        let (poly, poly_bits) = match self.polynomial {
            Polynomial::L7(val) => ((val & 0x7f) as u32, 0b11),
            Polynomial::L8(val) => (val as u32, 0b10),
            Polynomial::L16(val) => (val as u32, 0b01),
            Polynomial::L32(val) => (val, 0b00),
        };

        let in_rev_bits = match self.input_bit_reversal {
            None => 0b00,
            Some(BitReversal::ByByte) => 0b01,
            Some(BitReversal::ByHalfWord) => 0b10,
            Some(BitReversal::ByWord) => 0b11,
        };

        crc.init
            .write(|w| unsafe { w.crc_init().bits(self.initial_value) });

        crc.pol.write(|w| unsafe { w.bits(poly) });

        crc.cr.write(|w| {
            unsafe {
                w.rev_in()
                    .bits(in_rev_bits)
                    .polysize()
                    .bits(poly_bits)
                    .reset()
                    .set_bit();
            }

            if self.output_bit_reversal {
                w.rev_out().set_bit()
            } else {
                w.rev_out().clear_bit()
            }
        });

        Crc {}
    }
}

/// Constrained FLASH peripheral
pub struct Crc {}

impl Crc {
    /// This will reset the Crc to its initial condition
    #[inline]
    pub fn reset(&mut self) {
        let crc = unsafe { &(*CRC::ptr()) };

        crc.cr.write(|w| w.reset().set_bit());
    }

    /// Feed the Crc with data, this will internally optimize for word writes
    pub fn feed(&mut self, data: &[u8]) {
        let crc = unsafe { &(*CRC::ptr()) };
        for byte in data {
            unsafe {
                ptr::write_volatile(&crc.dr as *const _ as *mut u8, *byte);
            }
        }

        // TODO: Why does this corrupt the DWARF output of the generated elf? Ah, compiler bug.
        // Will be reported.
        //
        // let (prefix, words, suffix) = unsafe { data.align_to::<u32>() };

        // // Workaround with svd2rust, it does not generate the byte interface to the DR register
        // unsafe {
        //     for byte in prefix {
        //         ptr::write_volatile(&crc.dr as *const _ as *mut u8, *byte);
        //     }

        //     for word in words {
        //         crc.dr.write(|w| w.bits(*word));
        //     }

        //     for byte in suffix {
        //         ptr::write_volatile(&crc.dr as *const _ as *mut u8, *byte);
        //     }
        // }
    }

    /// Get the result of the CRC, depending on the polynomial chosen only a certain amount of the
    /// bits are the result. This will reset the Crc peripheral after use.
    ///
    /// TODO: Fix this?
    #[inline]
    pub fn result(&mut self) -> u32 {
        let ret = self.peek_result();

        self.reset();

        ret
    }

    /// Get a peed at the result of the CRC, depending on the polynomial chosen only a certain
    /// amount of the bits are the result.
    #[inline]
    pub fn peek_result(&self) -> u32 {
        let crc = unsafe { &(*CRC::ptr()) };

        crc.dr.read().bits()
    }
}
