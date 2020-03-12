//! Flash memory module
//!
//! Example usage of flash programming interface:
//!
//! ```
//! fn program_region(mut flash: flash::Parts) -> Result<(), flash::FlashError> {
//!     // Unlock the flashing module
//!     let mut prog = flash.keyr.unlock_flash(&mut flash.sr, &mut flash.cr)?;
//!
//!     let page = flash::FlashPage(5);
//!
//!     // Perform the erase and programing operation
//!     prog.erase_page(page)?;
//!     let data = [
//!         0x1111_1112_1113_1114,
//!         0x2221_2222_2223_2224,
//!         0x3331_3332_3333_3334,
//!     ];
//!     prog.program_dword(page.to_address(), &data)?;
//!
//!     // Check result (not needed, but done for this example)
//!     let addr = page.to_address() as *const u64;
//!     assert!(unsafe { core::ptr::read(addr) } == data[0]);
//!     assert!(unsafe { core::ptr::read(addr.offset(1)) } == data[1]);
//!     assert!(unsafe { core::ptr::read(addr.offset(2)) } == data[2]);
//!
//!     Ok(())
//! }
//! ```

#![deny(missing_docs)]

use crate::rcc::Clocks;
use crate::stm32::{flash, FLASH};
use core::{ops::Drop, ptr};

/// Extension trait to constrain the FLASH peripheral
pub trait FlashExt {
    /// Constrains the FLASH peripheral to play nicely with the other abstractions
    fn constrain(self) -> Parts;
}

impl FlashExt for FLASH {
    fn constrain(self) -> Parts {
        Parts {
            acr: ACR {},
            pdkeyr: PDKEYR {},
            keyr: KEYR {},
            optkeyr: OPTKEYR {},
            sr: SR {},
            cr: CR {},
            eccr: ECCR {},
            pcrop1sr: PCROP1SR {},
            pcrop1er: PCROP1ER {},
            wrp1ar: WRP1AR {},
            wrp1br: WRP1BR {},
        }
    }
}

/// Constrained FLASH peripheral
pub struct Parts {
    /// Opaque ACR register
    pub acr: ACR,
    /// Opaque PDKEYR register
    pub pdkeyr: PDKEYR,
    /// Opaque KEYR register
    pub keyr: KEYR,
    /// Opaque OPTKEYR register
    pub optkeyr: OPTKEYR,
    /// Opaque SR register
    pub sr: SR,
    /// Opaque SR register
    pub cr: CR,
    /// Opaque ECCR register
    pub eccr: ECCR,
    /// Opaque PCROP1SR register
    pub pcrop1sr: PCROP1SR,
    /// Opaque PCROP1ER register
    pub pcrop1er: PCROP1ER,
    /// Opaque WRP1AR register
    pub wrp1ar: WRP1AR,
    /// Opaque WRP1BR register
    pub wrp1br: WRP1BR,
}

macro_rules! generate_register {
    ($a:ident, $b:ident, $name:expr) => {
        #[doc = "Opaque "]
        #[doc = $name]
        #[doc = " register"]
        pub struct $a;


        impl $a {
            #[allow(unused)]
            pub(crate) fn $b(&mut self) -> &flash::$a {
                // NOTE(unsafe) this proxy grants exclusive access to this register
                unsafe { &(*FLASH::ptr()).$b }
            }
        }
    };

    ($a:ident, $b:ident) => {
        generate_register!($a, $b, stringify!($a));
    };
}

generate_register!(ACR, acr);
generate_register!(PDKEYR, pdkeyr);
generate_register!(KEYR, keyr);
generate_register!(OPTKEYR, optkeyr);
generate_register!(SR, sr);
generate_register!(CR, cr);
generate_register!(ECCR, eccr);
generate_register!(PCROP1SR, pcrop1sr);
generate_register!(PCROP1ER, pcrop1er);
generate_register!(WRP1AR, wrp1ar);
generate_register!(WRP1BR, wrp1br);

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

impl KEYR {
    /// Unlock the flash registers via KEYR to access the flash programming
    pub fn unlock_flash<'a>(
        &'a mut self,
        sr: &'a mut SR,
        cr: &'a mut CR,
    ) -> Result<FlashProgramming<'a>, FlashError> {
        let keyr = self.keyr();
        unsafe {
            keyr.write(|w| w.bits(FLASH_KEY1));
            keyr.write(|w| w.bits(FLASH_KEY2));
        }

        if cr.cr().read().lock().bit_is_clear() {
            Ok(FlashProgramming { sr, cr })
        } else {
            Err(FlashError::UnableToUnlock)
        }
    }
}

/// Flash page representation where each flash page represents a region of 2048 bytes. The flash
/// controller can only erase on a page basis.
#[derive(Copy, Clone, Debug)]
pub struct FlashPage(pub u8);

impl FlashPage {
    /// This gives the starting address of a flash page in physical address
    pub fn to_address(&self) -> usize {
        0x0800_0000 + self.0 as usize * 2048
    }
}

/// Flash programming interface
pub struct FlashProgramming<'a> {
    sr: &'a mut SR,
    cr: &'a mut CR,
}

impl<'a> Drop for FlashProgramming<'a> {
    fn drop(&mut self) {
        // Lock on drop
        self.lock();
    }
}

impl<'a> FlashProgramming<'a> {
    /// Lock the flash memory controller
    fn lock(&mut self) {
        self.cr.cr().modify(|_, w| w.lock().set_bit());
    }

    /// Wait till last flash operation is complete
    fn wait(&mut self) -> Result<(), FlashError> {
        while self.sr.sr().read().bsy().bit_is_set() {}

        self.status()
    }

    /// Check flash status
    pub fn status(&mut self) -> Result<(), FlashError> {
        let sr = self.sr.sr().read();

        if sr.bsy().bit_is_set() {
            Err(FlashError::Busy)
        } else if sr.pgaerr().bit_is_set() {
            Err(FlashError::AlignmentError)
        } else if sr.progerr().bit_is_set() {
            Err(FlashError::ProgrammingError)
        } else if sr.wrperr().bit_is_set() {
            Err(FlashError::WriteProtectionError)
        } else {
            Ok(())
        }
    }

    /// Erase specified flash page
    pub fn erase_page(&mut self, page: FlashPage) -> Result<(), FlashError> {
        self.cr
            .cr()
            .modify(|_, w| unsafe { w.pnb().bits(page.0).per().set_bit() });
        self.cr.cr().modify(|_, w| w.start().set_bit());

        let res = self.wait();

        self.cr.cr().modify(|_, w| w.per().clear_bit());

        res
    }

    /// Program double-word (64-bit) value at a specified address. `address` must be an address of
    /// a location in the flash memory aligned to 8 bytes.
    pub fn program_dword(&mut self, address: usize, data: &[u64]) -> Result<(), FlashError> {
        // NB: The check for alignment of the address, and that the flash is erased is made by the
        // flash controller. The `wait` function will return the proper error codes.
        let mut address = address as *mut u32;

        self.cr.cr().modify(|_, w| w.pg().set_bit());

        for dword in data {
            unsafe {
                ptr::write_volatile(address, *dword as u32);
                ptr::write_volatile(address.add(1), (*dword >> 32) as u32);

                address = address.add(2);
            }

            self.wait()?;

            if self.sr.sr().read().eop().bit_is_set() {
                self.sr.sr().modify(|_, w| w.eop().clear_bit());
            }
        }

        self.cr.cr().modify(|_, w| w.pg().clear_bit());

        Ok(())
    }

    /// Erase all flash pages, note that this will erase the current running program if it is not
    /// called from a program running in RAM.
    pub fn erase_all_pages(&mut self) -> Result<(), FlashError> {
        self.cr.cr().modify(|_, w| w.mer1().set_bit());
        self.cr.cr().modify(|_, w| w.start().set_bit());

        let res = self.wait();

        self.cr.cr().modify(|_, w| w.mer1().clear_bit());

        res
    }
}

/// Flash operation errors
#[derive(Copy, Clone, Debug)]
pub enum FlashError {
    /// The unlock procedure failed
    UnableToUnlock,
    /// Address to be programmed contains a value different from '0xFFFF_FFFF_FFFF_FFFF'
    /// before programming
    ProgrammingError,
    /// Alignment error, i.e. the address is not 64-bit aligned
    AlignmentError,
    /// Programming a write-protected address of the Flash memory
    WriteProtectionError,
    /// Programming and erase controller is busy
    Busy,
}
