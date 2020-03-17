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
    /// Native type of the flash for reading with the correct alignment of the memory and size
    ///
    /// Can be `u8`, `u16`, `u32`, ..., or any user defined type
    type NativeType;

    /// Read from the flash memory using the native interface
    fn read_native(&self, address: usize, array: &mut [Self::NativeType]);

    /// Read a buffer of bytes from memory
    fn read(&self, address: usize, buf: &mut [u8]);
}

pub trait WriteErase {
    /// Native type of the flash for writing with the correct alignment and size
    ///
    /// Can be `u8`, `u16`, `u32`, ..., or any user defined type
    type NativeType;

    /// check flash status
    fn status(&self) -> Result;

    /// Erase specified flash page.
    fn erase_page(&mut self, page: FlashPage) -> Result;

    /// The smallest possible write, depends on platform
    fn write_native(&mut self, address: usize, array: &[Self::NativeType]) -> Result;

    /// Read a buffer of bytes to memory, this uses the native writes internally and if it's not
    /// the same length and a set of native writes the write will be padded to fill a native write.
    fn write(&mut self, address: usize, data: &[u8]) -> Result;
}
