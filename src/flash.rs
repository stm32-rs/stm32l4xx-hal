//! Flash memory

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
    ) -> Result<FlashProgramming<'a>, ()> {
        let keyr = self.keyr();
        unsafe {
            keyr.write(|w| w.bits(FLASH_KEY1));
            keyr.write(|w| w.bits(FLASH_KEY2));
        }

        if cr.cr().read().lock().bit_is_clear() {
            Ok(FlashProgramming { sr, cr })
        } else {
            Err(())
        }
    }
}

/// The flash page
pub struct FlashPage(pub u8);

impl FlashPage {
    /// Flash page to physical address
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
    pub fn program_dword(&mut self, address: usize, data: u64) -> Result<(), FlashError> {
        if address & 0x7 != 0 {
            return Err(FlashError::AlignmentError);
        }

        if unsafe { ptr::read(address as *const u64) } != 0xFFFF_FFFF_FFFF_FFFF {
            return Err(FlashError::ProgrammingError);
        }

        self.cr.cr().modify(|_, w| w.pg().set_bit());

        unsafe {
            ptr::write_volatile(address as *mut u32, data as u32);
            ptr::write_volatile((address + 4) as *mut u32, (data >> 32) as u32);
        }

        let res = self.wait();

        self.cr.cr().modify(|_, w| w.pg().clear_bit());

        res
    }

    // TODO:
    // pub fn fast_program(&self, address: usize, data: &[u64]) -> Result<(), FlashError> {
    //     unimplemented!()
    // }

    /// Erase all flash pages
    pub fn erase_all_pages(&mut self) -> Result<(), FlashError> {
        self.cr.cr().modify(|_, w| w.mer1().set_bit());
        self.cr.cr().modify(|_, w| w.start().set_bit());

        let res = self.wait();

        self.cr.cr().modify(|_, w| w.mer1().clear_bit());

        res
    }
}

/// Flash errors
#[derive(Copy, Clone, Debug)]
pub enum FlashError {
    /// Timeout while waiting for the completion of the operation
    Timeout,
    /// Address to be programmed contains a value different from '0xFFFF_FFFF_FFFF_FFFF'
    /// before programming
    ProgrammingError,
    /// Alignment error
    AlignmentError,
    /// Programming a write-protected address of the Flash memory
    WriteProtectionError,
    // /// Fast programming error
    // FastProgrammingError,
    /// Programming and erase controller is busy
    Busy,
}
