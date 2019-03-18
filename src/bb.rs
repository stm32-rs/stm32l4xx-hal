//! Bit banding

// seems this is not a HAL trait (yet?)
// TODO: why is it used
//
// "The processor memory map includes two bit-band regions.
// These occupy the lowest 1MB of the SRAM and Peripheral memory regions respectively.
// These bit-band regions map each word in an alias region of memory to a bit
// in a bit-band region of memory."
//
// This module only handles peripheral, not SRAM bit-banding

use core::ptr;

pub fn clear<T>(register: *const T, bit: u8) {
    write(register, bit, false);
}

pub fn set<T>(register: *const T, bit: u8) {
    write(register, bit, true);
}

pub fn write<T>(register: *const T, bit: u8, set: bool) {
    let addr = register as usize;

    // Peripheral memory starts at 0x4000_0000, the first megabyte is aliased.
    //
    // Bit-band region is first megabyte
    assert!(addr >= 0x4000_0000 && addr <= 0x4010_0000);
    assert!(bit < 32);

    let bit = bit as usize;
    // bit_word_addr = bit_band_base + (byte_offset x 32) + (bit_number × 4)
    let bb_addr = (0x4200_0000 + (addr - 0x4000_0000) * 32) + 4 * bit;
    unsafe { ptr::write_volatile(bb_addr as *mut u32, if set { 1 } else { 0 }) }
}
