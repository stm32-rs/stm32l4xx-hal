//! Flash memory module
//!
//! Example usage of flash programming interface:
//!
//! ```
//! fn program_region(mut flash: flash::McuFlash) -> Result<(), flash::Error> {
//!     // Unlock the flashing module
//!     let mut prog = flash.unlock()?;
//!     let (start, end) = prog.range();
//!
//!     let page = start + Address(1024);
//!
//!     // Perform the erase and programing operation
//!     let data = [
//!         0x1111_1112_1113_1114,
//!         0x2221_2222_2223_2224,
//!         0x3331_3332_3333_3334,
//!     ];
//!     prog.try_write(page, &data)?;
//!
//!     // Check result (not needed, but done for this example)
//!     let addr = page.0 as *const u64;
//!     assert!(unsafe { core::ptr::read(addr) } == data[0]);
//!     assert!(unsafe { core::ptr::read(addr.offset(1)) } == data[1]);
//!     assert!(unsafe { core::ptr::read(addr.offset(2)) } == data[2]);
//!
//!     Ok(())
//! }
//! ```

#![deny(missing_docs)]

use crate::stm32::{flash, FLASH};
use core::convert::TryInto;
use core::{mem, ops::Drop, ptr};
use embedded_hal::storage::{Address, BitSubset, IterableByOverlaps, ReadWrite, Region};

#[derive(Clone, Copy, Debug)]
/// Size of the MCU flash
pub enum FlashVariant {
    /// 1MB flash size
    #[cfg(any(feature = "stm32l4x1", feature = "stm32l4x5", feature = "stm32l4x6"))]
    Size1024KB = 1024,
    /// 512KB flash size
    #[cfg(any(
        feature = "stm32l4x1",
        feature = "stm32l4x2",
        feature = "stm32l4x5",
        feature = "stm32l4x6"
    ))]
    Size512KB = 512,
    /// 256KB flash size
    #[cfg(any(
        feature = "stm32l4x1",
        feature = "stm32l4x2",
        feature = "stm32l4x3",
        feature = "stm32l4x5",
        feature = "stm32l4x6"
    ))]
    Size256KB = 256,
    #[cfg(any(feature = "stm32l4x1", feature = "stm32l4x2", feature = "stm32l4x3",))]
    Size128KB = 128,
    #[cfg(feature = "stm32l4x2")]
    Size64KB = 64,
}

impl FlashVariant {
    const fn bytes(self) -> u32 {
        self as u32 * 1024
    }
}

/// Error type of flash peripheral
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

/// Extension trait to constrain the FLASH peripheral
pub trait FlashExt {
    /// Constrains the FLASH peripheral to play nicely with the other abstractions
    fn constrain(self, variant: FlashVariant) -> McuFlash;
}

impl FlashExt for FLASH {
    fn constrain(self, variant: FlashVariant) -> McuFlash {
        McuFlash {
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
            memory_map: MemoryMap(variant),
        }
    }
}

/// Constrained FLASH peripheral
pub struct McuFlash {
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
    /// Memory map of the given Flash variant
    memory_map: MemoryMap,
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

impl McuFlash {
    /// Unlock the flash registers via KEYR to access the flash programming
    pub fn unlock<'a>(&'a mut self) -> Result<FlashProgramming<'a>, Error> {
        let keyr = self.keyr.keyr();
        unsafe {
            keyr.write(|w| w.bits(FLASH_KEY1));
            keyr.write(|w| w.bits(FLASH_KEY2));
        }

        let cr = &mut self.cr;
        if cr.cr().read().lock().bit_is_clear() {
            let sr = &mut self.sr;
            Ok(FlashProgramming {
                sr,
                cr,
                memory_map: self.memory_map,
            })
        } else {
            Err(Error::Failure)
        }
    }
}

/// Flash programming interface
pub struct FlashProgramming<'a> {
    sr: &'a mut SR,
    cr: &'a mut CR,
    memory_map: MemoryMap,
}

impl<'a> FlashProgramming<'a> {
    fn status(&mut self) -> Result<(), Error> {
        let sr = self.sr.sr().read();

        if sr.bsy().bit_is_set() {
            Err(Error::Busy)
        } else if sr.pgaerr().bit_is_set() {
            Err(Error::Illegal)
        } else if sr.progerr().bit_is_set() {
            Err(Error::Illegal)
        } else if sr.wrperr().bit_is_set() {
            Err(Error::Illegal)
        } else {
            Ok(())
        }
    }

    /// Lock the flash memory controller
    fn lock(&mut self) {
        self.cr.cr().modify(|_, w| w.lock().set_bit());
    }

    /// Wait till last flash operation is complete
    fn wait(&mut self) -> Result<(), Error> {
        while self.sr.sr().read().bsy().bit_is_set() {}
        self.status()
    }

    /// Erase all flash pages, note that this will erase the current running program if it is not
    /// called from a program running in RAM.
    pub fn erase_all_pages(&mut self) -> Result<(), Error> {
        self.cr.cr().modify(|_, w| w.mer1().set_bit());
        self.cr.cr().modify(|_, w| w.start().set_bit());

        let res = self.wait();

        self.cr.cr().modify(|_, w| w.mer1().clear_bit());

        res
    }

    fn erase_page(&mut self, page: &Page) -> Result<(), Error> {
        let page_number = (MemoryMap::start() + page.location).0 / page.size as u32;
        match page.area {
            Area::Main(MemoryBank::Bank1) => {
                self.cr.cr().modify(|_, w| unsafe {
                    w.bker()
                        .clear_bit()
                        .pnb()
                        .bits(page_number as u8)
                        .per()
                        .set_bit()
                });
            }
            Area::Main(MemoryBank::Bank2) => {
                self.cr.cr().modify(|_, w| unsafe {
                    w.bker()
                        .set_bit()
                        .pnb()
                        .bits((page_number - 256) as u8)
                        .per()
                        .set_bit()
                });
            }
            // TODO: Handle Area::SystemMemory, Area::OneTimeProgrammable & Area::OptionBytes
            _ => {
                return Err(Error::PageOutOfRange);
            }
        }

        self.cr.cr().modify(|_, w| w.start().set_bit());

        let res = self.wait();

        self.cr.cr().modify(|_, w| w.per().clear_bit());

        res
    }

    fn write_native(&mut self, data: &[u64], aligned_address: usize) -> Result<(), Error> {
        // NB: The check for alignment of the address, and that the flash is erased is made by the
        // flash controller. The `wait` function will return the proper error codes.
        let mut address = aligned_address as *mut u32;

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

    fn write_bytes(&mut self, data: &[u8], address: Address) -> Result<(), Error> {
        let address = address.0 as usize;
        let address_offset = address % mem::align_of::<u64>();
        let unaligned_size = (mem::size_of::<u64>() - address_offset) % mem::size_of::<u64>();

        if unaligned_size > 0 {
            let unaligned_data = &data[..unaligned_size];
            // Handle unaligned address data, make it into a native write
            let mut data = 0xffff_ffff_ffff_ffffu64;
            for b in unaligned_data {
                data = (data >> 8) | ((*b as u64) << 56);
            }

            let unaligned_address = address - address_offset;
            let native = &[data];
            self.write_native(native, unaligned_address)?;
        }

        // Handle aligned address data
        let aligned_data = &data[unaligned_size..];
        let mut aligned_address = if unaligned_size > 0 {
            address - address_offset + mem::size_of::<u64>()
        } else {
            address
        };

        let mut chunks = aligned_data.chunks_exact(mem::size_of::<u64>());

        while let Some(exact_chunk) = chunks.next() {
            // Write chunks
            let native = &[u64::from_ne_bytes(exact_chunk.try_into().unwrap())];
            self.write_native(native, aligned_address)?;
            aligned_address += mem::size_of::<u64>();
        }

        let rem = chunks.remainder();

        if rem.len() > 0 {
            let mut data = 0xffff_ffff_ffff_ffffu64;
            // Write remainder
            for b in rem.iter().rev() {
                data = (data << 8) | *b as u64;
            }

            let native = &[data];
            self.write_native(native, aligned_address)?;
        }

        Ok(())
    }
}

impl<'a> Drop for FlashProgramming<'a> {
    fn drop(&mut self) {
        // Lock on drop
        self.lock();
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
/// Describes which flash area is being addressed
enum Area {
    /// Main flash area
    Main(MemoryBank),
    /// System memory flash area
    SystemMemory,
    /// OTA (One time programmable) flash area
    OneTimeProgrammable,
    /// Option bytes flash area
    OptionBytes,
}

#[derive(Copy, Clone, Debug, PartialEq)]
/// Describes which memory bank is being addressed
enum MemoryBank {
    /// Memory bank 1
    Bank1,
    /// Memory bank 2
    Bank2,
}

#[derive(Copy, Clone, Debug, PartialEq)]
/// Flash page, as being the smallest entity to erase
struct Page {
    area: Area,
    location: Address,
    size: usize,
}

const MAX_PAGE_SIZE: usize = 2048;

#[derive(Clone, Copy)]
/// Memory map describing the sections and pages of the flash
struct MemoryMap(FlashVariant);

impl MemoryMap {
    /// Get an iterator over all the possible pages of the flash memory
    fn pages(&self) -> impl Iterator<Item = Page> {
        let page_size = 2048;
        let last_page = (self.0.bytes() / page_size) - 1;

        (0..last_page).map(move |page_number| {
            let bank = if page_number < last_page / 2 {
                MemoryBank::Bank1
            } else {
                MemoryBank::Bank2
            };

            Page::new(
                Area::Main(bank),
                Self::start() + Address(page_number * page_size),
                page_size as usize,
            )
        })
    }

    /// The base address (start) of the flash memory
    pub const fn start() -> Address {
        Address(0x0800_0000)
    }

    /// The end address of the flash memory
    pub const fn end(&self) -> Address {
        Address(0x0800_0000 + self.0.bytes() - 1)
    }
}

impl Region for Page {
    fn contains(&self, address: Address) -> bool {
        (self.location <= address) && (self.end() > address)
    }
}

impl Page {
    /// Construct a new `Page`
    const fn new(area: Area, location: Address, size: usize) -> Self {
        Self {
            area,
            location,
            size,
        }
    }

    /// The end address of the page
    const fn end(&self) -> Address {
        Address(self.location.0 + self.size as u32 - 1)
    }
}

impl<'a> ReadWrite for FlashProgramming<'a> {
    type Error = Error;

    fn try_read(&mut self, address: Address, bytes: &mut [u8]) -> nb::Result<(), Self::Error> {
        let mut address = address.0 as *const u8;

        for data in bytes {
            unsafe {
                *data = ptr::read(address);
                address = address.add(1);
            }
        }
        Ok(())
    }

    fn try_write(&mut self, address: Address, bytes: &[u8]) -> nb::Result<(), Self::Error> {
        self.write_bytes(bytes, address)?;
        // for (block, page, addr) in self.memory_map.pages().overlaps(bytes, address) {
        //     let merge_buffer = &mut [0u8; MAX_PAGE_SIZE][0..page.size];
        //     let offset_into_page = addr.0.saturating_sub(page.location.0) as usize;

        //     self.try_read(page.location, merge_buffer)?;

        //     if block.is_subset_of(&merge_buffer[offset_into_page..page.size]) {
        //         self.write_bytes(block, addr)?;
        //     } else {
        //         self.erase_page(&page)?;
        //         merge_buffer
        //             .iter_mut()
        //             .skip(offset_into_page)
        //             .zip(block)
        //             .for_each(|(byte, input)| *byte = *input);
        //         self.write_bytes(merge_buffer, page.location)?;
        //     }
        // }

        Ok(())
    }

    fn range(&self) -> (Address, Address) {
        (MemoryMap::start(), self.memory_map.end())
    }

    fn try_erase(&mut self, from: Address, to: Address) -> nb::Result<(), Self::Error> {
        self.memory_map
            .pages()
            .skip_while(|page| !page.contains(from))
            .take_while(|page| page.contains(to))
            .try_for_each(|page| self.erase_page(&page).map_err(nb::Error::Other))
    }
}
