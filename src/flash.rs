//! Flash memory module
//!
//! Example usage of flash programming interface:
//!
//! ```
//! fn program_region(mut flash: flash::Parts) -> Result<(), flash::Error> {
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
//!     prog.write_native(page.to_address(), &data)?;
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

use crate::stm32::{flash, FLASH};
use crate::traits::flash as flash_trait;
use core::convert::TryFrom;
use core::{ops::Drop, ptr};
pub use flash_trait::{Error, FlashPage, Read, WriteErase};

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
    ) -> Result<FlashProgramming<'a>, Error> {
        let keyr = self.keyr();
        unsafe {
            keyr.write(|w| w.bits(FLASH_KEY1));
            keyr.write(|w| w.bits(FLASH_KEY2));
        }

        if cr.cr().read().lock().bit_is_clear() {
            Ok(FlashProgramming { sr, cr })
        } else {
            Err(Error::Failure)
        }
    }
}

impl FlashPage {
    /// This gives the starting address of a flash page in physical address
    pub const fn to_address(&self) -> usize {
        0x0800_0000 + self.0 * 2048
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

impl<'a> Read for FlashProgramming<'a> {
    type NativeType = u8;

    #[inline]
    fn read_native(&self, address: usize, array: &mut [Self::NativeType]) {
        assert!(address % core::mem::align_of::<Self::NativeType>() == 0);

        let mut address = address as *const Self::NativeType;

        for data in array {
            unsafe {
                *data = ptr::read(address);
                address = address.add(1);
            }
        }
    }
}

impl<'a> WriteErase for FlashProgramming<'a> {
    type NativeType = u64;

    fn status(&self) -> flash_trait::Result {
        let sr = unsafe { &(*FLASH::ptr()).sr }.read();

        if sr.bsy().bit_is_set() {
            Err(flash_trait::Error::Busy)
        } else if sr.pgaerr().bit_is_set() {
            Err(flash_trait::Error::Illegal)
        } else if sr.progerr().bit_is_set() {
            Err(flash_trait::Error::Illegal)
        } else if sr.wrperr().bit_is_set() {
            Err(flash_trait::Error::Illegal)
        } else {
            Ok(())
        }
    }

    fn erase_page(&mut self, page: flash_trait::FlashPage) -> flash_trait::Result {
        self.cr
            .cr()
            .modify(|_, w| unsafe { w.pnb().bits(u8::try_from(page.0).unwrap()).per().set_bit() });
        self.cr.cr().modify(|_, w| w.start().set_bit());

        let res = self.wait();

        self.cr.cr().modify(|_, w| w.per().clear_bit());

        res
    }

    fn write_native(&mut self, address: usize, array: &[Self::NativeType]) -> flash_trait::Result {
        assert!(address % core::mem::align_of::<Self::NativeType>() == 0);

        // NB: The check for alignment of the address, and that the flash is erased is made by the
        // flash controller. The `wait` function will return the proper error codes.
        let mut address = address as *mut u32;

        self.cr.cr().modify(|_, w| w.pg().set_bit());

        for dword in array {
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
}

impl<'a> FlashProgramming<'a> {
    /// Lock the flash memory controller
    fn lock(&mut self) {
        self.cr.cr().modify(|_, w| w.lock().set_bit());
    }

    /// Wait till last flash operation is complete
    fn wait(&mut self) -> flash_trait::Result {
        while self.sr.sr().read().bsy().bit_is_set() {}

        self.status()
    }

    /// Erase all flash pages, note that this will erase the current running program if it is not
    /// called from a program running in RAM.
    pub fn erase_all_pages(&mut self) -> flash_trait::Result {
        self.cr.cr().modify(|_, w| w.mer1().set_bit());
        self.cr.cr().modify(|_, w| w.start().set_bit());

        let res = self.wait();

        self.cr.cr().modify(|_, w| w.mer1().clear_bit());

        res
    }
}
