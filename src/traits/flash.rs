/// Flash page representation where each flash page represents a region of 2048 bytes. The flash
/// controller can only erase on a page basis.
#[derive(Copy, Clone, Debug)]
pub struct FlashPage(pub usize);

/// Flash operation error
#[derive(Copy, Clone, Debug)]
pub enum Error {
    /// Flash controller is not done yet
    Busy,
    /// Error detected (by command execution, or because no command could be executed)
    Illegal,
    /// Set during read if ECC decoding logic detects correctable or uncorrectable error
    EccError,
    /// (Legal) command failed
    Failure,
}

/// A type alias for the result of a Flash operation.
pub type Result = core::result::Result<(), Error>;

pub trait Read {
    /// Native type of the flash for reading with the correct alignment of the memory
    ///
    /// Can be `u8`, `u16`, `u32`, ..., or any user defined type
    type NativeType;

    /// Read from the flash memory using the native interface
    fn read_native(&self, address: usize, array: &mut [Self::NativeType]);

    // /// read a buffer of bytes from memory
    // /// checks that the address and buffer size are multiples of native
    // /// FLASH ReadSize.
    // fn read(&self, address: usize, buf: &mut [u8]) {
    //     // TODO: offer a version without restrictions?
    //     // can round down address, round up buffer length,
    //     // but where to get the buffer from?
    //     assert!(buf.len() % ReadSize::to_usize() == 0);
    //     assert!(address % ReadSize::to_usize() == 0);

    //     for i in (0..buf.len()).step_by(ReadSize::to_usize()) {
    //         self.read_native(
    //             address + i,
    //             GenericArray::from_mut_slice(&mut buf[i..i + ReadSize::to_usize()]),
    //         );
    //     }
    // }
}

pub trait WriteErase {
    /// Native type of the flash for writing with the correct alignment
    ///
    /// Can be `u8`, `u16`, `u32`, ..., or any user defined type
    type NativeType;

    /// check flash status
    fn status(&self) -> Result;

    /// Erase specified flash page.
    fn erase_page(&mut self, page: FlashPage) -> Result;

    /// The smallest possible write, depends on platform
    fn write_native(&mut self, address: usize, array: &[Self::NativeType]) -> Result;

    // fn write(&mut self, address: usize, data: &[u8]) -> Result {
    //     let write_size = WriteSize::to_usize();
    //     assert!(data.len() % write_size == 0);
    //     assert!(address % write_size == 0);

    //     for i in (0..data.len()).step_by(write_size) {
    //         self.write_native(
    //             address + i,
    //             GenericArray::from_slice(&data[i..i + write_size]),
    //             // cs,
    //         )?;
    //     }
    //     Ok(())
    // }
}
