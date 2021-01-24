//! Read and write onboard flash memory. Different from `flash.rs`, in that it
//! doesn't rely on constraining  the FLASH register block, and its code style
//! more directly matches the datasheet.
//!

use crate::pac::FLASH;
use core;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

#[derive(Copy, Clone, Debug)]
pub enum Error {
    /// Flash controller is not done yet
    Busy,
    /// Error detected (by command execution, or because no command could be executed)
    Illegal,
    /// Set during read if ECC decoding logic detects correctable or uncorrectable error
    EccError,
    /// Page number is out of range
    PageOutOfRange,
    /// (Legal) command failed
    Failure,
}

pub struct Flash {
    pub(crate) regs: FLASH,
}

impl Flash {
    pub fn new(regs: FLASH) -> Self {
        Self { regs }
    }
    /// Unlock the flash memory, allowing writes. See L4 Reference manual, section 3.3.5
    pub fn unlock(&mut self) {
        self.regs.keyr.write(|w| unsafe { w.bits(FLASH_KEY1) });
        self.regs.keyr.write(|w| unsafe { w.bits(FLASH_KEY2) });

        while self.regs.cr.read().lock().bit_is_set() {}
    }

    /// Lock the flash memory, allowing writes.
    pub fn lock(&mut self) {
        self.regs.cr.modify(|_, w| w.lock().set_bit());
    }

    /// Erase an entire page. See L4 Reference manual, section 3.3.5.
    /// For why this is required, reference L4 RM, section 3.3.7:
    /// "Programming in a previously programmed address is not allowed except if the data to write
    /// is full zero, and any attempt will set PROGERR flag in the Flash status register
    /// (FLASH_SR)."
    pub fn erase_page(&mut self, page: usize) -> Result<(), Error> {
        self.unlock();

        // 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the Flash
        // status register (FLASH_SR).
        let sr = self.regs.sr.read();
        if sr.bsy().bit_is_set() {
            self.lock();
            return Err(Error::Busy);
        }

        // 2. Check and clear all error programming flags due to a previous programming. If not,
        // PGSERR is set.
        if sr.pgaerr().bit_is_set() || sr.progerr().bit_is_set() || sr.wrperr().bit_is_set() {
            self.lock();
            return Err(Error::Illegal);
        }

        // 3. Set the PER bit and select the page you wish to erase (PNB) with the associated bank
        // (BKER) in the Flash control register (FLASH_CR).
        match page {
            0..=255 => {
                self.regs.cr.modify(|_, w| unsafe {
                    w.bker().clear_bit().pnb().bits(page as u8).per().set_bit()
                });
            }
            256..=511 => {
                self.regs.cr.modify(|_, w| unsafe {
                    w.bker()
                        .set_bit()
                        .pnb()
                        .bits((page - 256) as u8)
                        .per()
                        .set_bit()
                });
            }
            _ => {
                return Err(Error::PageOutOfRange);
            }
        }

        // 4. Set the STRT bit in the FLASH_CR register.
        self.regs.cr.modify(|_, w| w.start().set_bit());

        // 5. Wait for the BSY bit to be cleared in the FLASH_SR register.
        while self.regs.sr.read().bsy().bit_is_set() {}

        self.lock();

        Ok(())
    }

    /// Write the contents of a page. Must be erased first. See L4 RM, section 3.3.7.
    pub fn write_page(&mut self, page: usize, data: &[u64]) -> Result<(), Error> {
        // todo: DRY from `erase_page`.
        // The Flash memory programming sequence in standard mode is as follows:
        // 1. Check that no Flash main memory operation is ongoing by checking the BSY bit in the
        // Flash status register (FLASH_SR).
        self.unlock();

        let sr = self.regs.sr.read();
        if sr.bsy().bit_is_set() {
            self.lock();
            return Err(Error::Busy);
        }

        // 2. Check and clear all error programming flags due to a previous programming. If not,
        // PGSERR is set.
        if sr.pgaerr().bit_is_set() || sr.progerr().bit_is_set() || sr.wrperr().bit_is_set() {
            self.lock();
            return Err(Error::Illegal);
        }

        // 3. Set the PG bit in the Flash control register (FLASH_CR).
        self.regs.cr.modify(|_, w| w.pg().set_bit());

        // 4. Perform the data write operation at the desired memory address, inside main memory
        // block or OTP area. Only double word can be programmed.
        // – Write a first word in an address aligned with double word
        // – Write the second word
        let mut address = page_to_address(page) as *mut u32;

        for dword in data {
            unsafe {
                core::ptr::write_volatile(address, *dword as u32);
                core::ptr::write_volatile(address.add(1), (*dword >> 32) as u32);

                address = address.add(2);
            }

            // 5. Wait until the BSY bit is cleared in the FLASH_SR register.
            while self.regs.sr.read().bsy().bit_is_set() {}

            if self.regs.sr.read().eop().bit_is_set() {
                self.regs.sr.modify(|_, w| w.eop().clear_bit());
            }

            // 6. Check that EOP flag is set in the FLASH_SR register (meaning that the programming
            // operation has succeed), and clear it by software.
            if self.regs.sr.read().eop().bit_is_set() {
                self.regs.sr.modify(|_, w| w.eop().clear_bit());
            }
        }

        // 7. Clear the PG bit in the FLASH_CR register if there no more programming request
        // anymore.
        self.regs.cr.modify(|_, w| w.pg().clear_bit());

        self.lock();

        Ok(())
    }

    /// Read data for a page at a given offset.
    pub fn read(&self, page: usize, offset: isize) -> u64 {
        let addr = page_to_address(page) as *const u64;
        unsafe { core::ptr::read(addr.offset(offset)) }
    }

    // todo: Erase bank method.
}

fn page_to_address(page: usize) -> usize {
    // todo: u64? *u64?
    0x0800_0000 + page as usize * 2048
}
