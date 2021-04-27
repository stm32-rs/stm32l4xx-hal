//! Direct Memory Access Engine

#![allow(dead_code)]

use core::fmt;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::ops::DerefMut;
use core::ptr;
use core::slice;
use core::sync::atomic::{compiler_fence, Ordering};

use crate::rcc::AHB1;
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};
use stable_deref_trait::StableDeref;

#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    Overrun,
    BufferError,
}

pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Half {
    First,
    Second,
}

pub trait CharacterMatch {
    /// Checks to see if the peripheral has detected a character match and
    /// clears the flag
    fn check_character_match(&mut self, clear: bool) -> bool;
}

/// Frame reader "worker", access and handling of frame reads is made through this structure.
pub struct FrameReader<BUFFER, PAYLOAD, const N: usize>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
{
    buffer: BUFFER,
    payload: PAYLOAD,
    matching_character: u8,
}

impl<BUFFER, PAYLOAD, const N: usize> FrameReader<BUFFER, PAYLOAD, N>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
{
    pub(crate) fn new(
        buffer: BUFFER,
        payload: PAYLOAD,
        matching_character: u8,
    ) -> FrameReader<BUFFER, PAYLOAD, N> {
        Self {
            buffer,
            payload,
            matching_character,
        }
    }
}

impl<BUFFER, PAYLOAD, CHANNEL, const N: usize> FrameReader<BUFFER, RxDma<PAYLOAD, CHANNEL>, N>
where
    PAYLOAD: CharacterMatch,
{
    /// Checks to see if the peripheral has detected a character match and
    /// clears the flag
    pub fn check_character_match(&mut self, clear: bool) -> bool {
        self.payload.payload.check_character_match(clear)
    }
}

/// Frame sender "worker", access and handling of frame transmissions is made through this
/// structure.
pub struct FrameSender<BUFFER, PAYLOAD, const N: usize>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
{
    buffer: Option<BUFFER>,
    payload: PAYLOAD,
}

impl<BUFFER, PAYLOAD, const N: usize> FrameSender<BUFFER, PAYLOAD, N>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
{
    pub(crate) fn new(payload: PAYLOAD) -> FrameSender<BUFFER, PAYLOAD, N> {
        Self {
            buffer: None,
            payload,
        }
    }
}

/// Data type for holding data frames for the Serial.
///
/// Internally used uninitialized storage, making this storage zero cost to create. It can also be
/// used with, for example, [`heapless::pool`] to create a pool of serial frames.
///
/// [`heapless::pool`]: https://docs.rs/heapless/0.5.3/heapless/pool/index.html
pub struct DMAFrame<const N: usize> {
    len: u16,
    buf: [MaybeUninit<u8>; N],
}

impl<const N: usize> fmt::Debug for DMAFrame<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.read())
    }
}

impl<const N: usize> fmt::Write for DMAFrame<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let free = self.free();

        if s.len() > free {
            Err(fmt::Error)
        } else {
            self.write_slice(s.as_bytes());
            Ok(())
        }
    }
}

impl<const N: usize> Default for DMAFrame<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> DMAFrame<N> {
    const INIT: MaybeUninit<u8> = MaybeUninit::uninit();
    /// Creates a new node for the Serial DMA
    #[inline]
    pub const fn new() -> Self {
        // Create an uninitialized array of `MaybeUninit<u8>`.
        Self {
            len: 0,
            buf: [Self::INIT; N],
        }
    }

    /// Gives a `&mut [u8]` slice to write into with the maximum size, the `commit` method
    /// must then be used to set the actual number of bytes written.
    ///
    /// Note that this function internally first zeros the uninitialized part of the node's buffer.
    pub fn write(&mut self) -> &mut [u8] {
        // Initialize remaining memory with a safe value
        for elem in &mut self.buf[self.len as usize..] {
            *elem = MaybeUninit::new(0);
        }

        self.len = self.max_len() as u16;

        // NOTE(unsafe): This is safe as the operation above set the entire buffer to a valid state
        unsafe { slice::from_raw_parts_mut(self.buf.as_mut_ptr() as *mut _, self.max_len()) }
    }

    /// Used to shrink the current size of the frame, used in conjunction with `write`.
    #[inline]
    pub fn commit(&mut self, shrink_to: usize) {
        // Only shrinking is allowed to remain safe with the `MaybeUninit`
        if shrink_to < self.len as _ {
            self.len = shrink_to as _;
        }
    }

    /// Gives an uninitialized `&mut [MaybeUninit<u8>]` slice to write into, the `set_len` method
    /// must then be used to set the actual number of bytes written.
    #[inline]
    pub fn write_uninit(&mut self) -> &mut [MaybeUninit<u8>; N] {
        &mut self.buf
    }

    /// Used to set the current size of the frame, used in conjunction with `write_uninit` to have an
    /// interface for uninitialized memory. Use with care!
    ///
    /// # Safety
    ///
    /// NOTE(unsafe): This must be set so that the final buffer is only referencing initialized
    /// memory.
    #[inline]
    pub unsafe fn set_len(&mut self, len: usize) {
        assert!(len <= self.max_len());
        self.len = len as _;
    }

    /// Used to write data into the node, and returns how many bytes were written from `buf`.
    ///
    /// If the node is already partially filled, this will continue filling the node.
    pub fn write_slice(&mut self, buf: &[u8]) -> usize {
        let count = buf.len().min(self.free());

        // Used to write data into the `MaybeUninit`
        // NOTE(unsafe): Safe based on the size check above
        unsafe {
            ptr::copy_nonoverlapping(
                buf.as_ptr(),
                (self.buf.as_mut_ptr() as *mut u8).add(self.len.into()),
                count,
            );
        }

        self.len += count as u16;

        count
    }

    /// Clear the node of all data making it empty
    #[inline]
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Returns a readable slice which maps to the buffers internal data
    #[inline]
    pub fn read(&self) -> &[u8] {
        // NOTE(unsafe): Safe as it uses the internal length of valid data
        unsafe { slice::from_raw_parts(self.buf.as_ptr() as *const _, self.len as usize) }
    }

    /// Returns a readable mutable slice which maps to the buffers internal data
    #[inline]
    pub fn read_mut(&mut self) -> &mut [u8] {
        // NOTE(unsafe): Safe as it uses the internal length of valid data
        unsafe { slice::from_raw_parts_mut(self.buf.as_mut_ptr() as *mut _, self.len as usize) }
    }

    /// Reads how many bytes are available
    #[inline]
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Reads how many bytes are free
    #[inline]
    pub fn free(&self) -> usize {
        self.max_len() - self.len as usize
    }

    /// Get the max length of the frame
    #[inline]
    pub fn max_len(&self) -> usize {
        N
    }

    /// Checks if the frame is empty
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[inline]
    pub(crate) unsafe fn buffer_address_for_dma(&self) -> u32 {
        self.buf.as_ptr() as u32
    }

    #[inline]
    pub(crate) fn buffer_as_ptr(&self) -> *const MaybeUninit<u8> {
        self.buf.as_ptr()
    }

    #[inline]
    pub(crate) fn buffer_as_mut_ptr(&mut self) -> *mut MaybeUninit<u8> {
        self.buf.as_mut_ptr()
    }
}

impl<const N: usize> AsRef<[u8]> for DMAFrame<N> {
    #[inline]
    fn as_ref(&self) -> &[u8] {
        self.read()
    }
}

pub struct CircBuffer<BUFFER, PAYLOAD>
where
    BUFFER: 'static,
{
    buffer: &'static mut [BUFFER; 2],
    payload: PAYLOAD,
    readable_half: Half,
    consumed_offset: usize,
}

impl<BUFFER, PAYLOAD> CircBuffer<BUFFER, PAYLOAD>
where
    &'static mut [BUFFER; 2]: StaticWriteBuffer,
    BUFFER: 'static,
{
    pub(crate) fn new(buf: &'static mut [BUFFER; 2], payload: PAYLOAD) -> Self {
        CircBuffer {
            buffer: buf,
            payload,
            readable_half: Half::Second,
            consumed_offset: 0,
        }
    }
}

pub trait DmaExt {
    type Channels;

    fn split(self, ahb: &mut AHB1) -> Self::Channels;
}

pub trait TransferPayload {
    fn start(&mut self);
    fn stop(&mut self);
}

pub struct Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    _mode: PhantomData<MODE>,
    buffer: BUFFER,
    payload: PAYLOAD,
}

impl<BUFFER, PAYLOAD> Transfer<R, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn r(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn w(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<MODE, BUFFER, PAYLOAD> Drop for Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    fn drop(&mut self) {
        self.payload.stop();
        compiler_fence(Ordering::SeqCst);
    }
}

/// Read transfer
pub struct R;

/// Write transfer
pub struct W;

macro_rules! dma {
    ($($DMAX:ident: ($dmaX:ident, $dmaXen:ident, $dmaXrst:ident, {
        $($CX:ident: (
            $ccrX:ident,
            $CCRX:ident,
            $cndtrX:ident,
            $CNDTRX:ident,
            $cparX:ident,
            $CPARX:ident,
            $cmarX:ident,
            $CMARX:ident,
            $htifX:ident,
            $tcifX:ident,
            $chtifX:ident,
            $ctcifX:ident,
            $cgifX:ident,
            $teifX:ident,
            $cteifX:ident
        ),)+
    }),)+) => {
        $(
            pub mod $dmaX {
                use core::sync::atomic::{self, Ordering};
                use crate::stm32::{$DMAX, dma1};
                use core::ops::DerefMut;
                use core::ptr;
                use stable_deref_trait::StableDeref;

                use crate::dma::{CircBuffer, FrameReader, CharacterMatch, FrameSender, DMAFrame, DmaExt, Error, Event, Half, Transfer, W, R, RxDma, TxDma, TransferPayload};
                use crate::rcc::AHB1;

                #[allow(clippy::manual_non_exhaustive)]
                pub struct Channels((), $(pub $CX),+);

                $(
                    /// A singleton that represents a single DMAx channel (channel X in this case)
                    ///
                    /// This singleton has exclusive access to the registers of the DMAx channel X
                    pub struct $CX;

                    impl $CX {
                        /// Associated peripheral `address`
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        #[inline]
                        pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
                            self.cpar().write(|w|
                                unsafe { w.pa().bits(address) }
                            );
                            self.ccr().modify(|_, w| w.pinc().bit(inc) );
                        }

                        /// `address` where from/to data will be read/write
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        #[inline]
                        pub fn set_memory_address(&mut self, address: u32, inc: bool) {
                            self.cmar().write(|w|
                                unsafe { w.ma().bits(address) }
                            );
                            self.ccr().modify(|_, w| w.minc().bit(inc) );
                        }

                        /// Number of bytes to transfer
                        #[inline]
                        pub fn set_transfer_length(&mut self, len: u16) {
                            self.cndtr().write(|w| w.ndt().bits(len));
                        }

                        /// Starts the DMA transfer
                        #[inline]
                        pub fn start(&mut self) {
                            self.ccr().modify(|_, w| w.en().set_bit() );
                        }

                        /// Stops the DMA transfer
                        #[inline]
                        pub fn stop(&mut self) {
                            self.ifcr().write(|w| w.$cgifX().set_bit());
                            self.ccr().modify(|_, w| w.en().clear_bit() );
                        }

                        /// Returns `true` if there's a transfer in progress
                        #[inline]
                        pub fn in_progress(&self) -> bool {
                            self.isr().$tcifX().bit_is_clear()
                        }

                        #[inline]
                        pub fn listen(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => self.ccr().modify(|_, w| w.htie().set_bit()),
                                Event::TransferComplete => {
                                    self.ccr().modify(|_, w| w.tcie().set_bit())
                                }
                            }
                        }

                        #[inline]
                        pub fn unlisten(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => {
                                    self.ccr().modify(|_, w| w.htie().clear_bit())
                                },
                                Event::TransferComplete => {
                                    self.ccr().modify(|_, w| w.tcie().clear_bit())
                                }
                            }
                        }

                        #[inline]
                        pub(crate) fn isr(&self) -> dma1::isr::R {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).isr.read() }
                        }

                        #[inline]
                        pub(crate) fn ifcr(&self) -> &dma1::IFCR {
                            unsafe { &(*$DMAX::ptr()).ifcr }
                        }

                        #[inline]
                        pub(crate) fn ccr(&mut self) -> &dma1::$CCRX {
                            unsafe { &(*$DMAX::ptr()).$ccrX }
                        }

                        #[inline]
                        pub(crate) fn cndtr(&mut self) -> &dma1::$CNDTRX {
                            unsafe { &(*$DMAX::ptr()).$cndtrX }
                        }

                        #[inline]
                        pub(crate) fn cpar(&mut self) -> &dma1::$CPARX {
                            unsafe { &(*$DMAX::ptr()).$cparX }
                        }

                        #[inline]
                        pub(crate) fn cmar(&mut self) -> &dma1::$CMARX {
                            unsafe { &(*$DMAX::ptr()).$cmarX }
                        }

                        #[inline]
                        pub(crate) fn cselr(&mut self) -> &dma1::CSELR {
                            unsafe { &(*$DMAX::ptr()).cselr }
                        }

                        #[inline]
                        pub(crate) fn get_cndtr(&self) -> u32 {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).$cndtrX.read().bits() }
                        }

                    }

                    impl<BUFFER, PAYLOAD, const N: usize> FrameSender<BUFFER, TxDma<PAYLOAD, $CX>, N>
                    where
                        BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
                        TxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        /// This method should be called in the transfer complete interrupt of the
                        /// DMA, will return the sent frame if the transfer was truly completed.
                        pub fn transfer_complete_interrupt(
                            &mut self,
                        ) -> Option<BUFFER> {

                            // Clear ISR flag (Transfer Complete)
                            if !self.payload.channel.in_progress() {
                                self.payload.channel.ifcr().write(|w| w.$ctcifX().set_bit());
                            } else {
                                // The old transfer is not complete
                                return None;
                            }

                            self.payload.channel.stop();

                            // NOTE(compiler_fence) operations on the DMA should not be reordered
                            // before the next statement, takes the buffer from the DMA transfer.
                            atomic::compiler_fence(Ordering::SeqCst);

                            // Return the old buffer for the user to do what they want with it
                            self.buffer.take()
                        }

                        /// Returns `true` if there is an ongoing transfer.
                        #[inline]
                        pub fn ongoing_transfer(&self) -> bool {
                            self.buffer.is_some()
                        }

                        /// Send a frame. Will return `Err(frame)` if there was already an ongoing
                        /// transaction or if the buffer has not been read out.
                        pub fn send(
                            &mut self,
                            frame: BUFFER,
                        ) -> Result<(), BUFFER> {
                            if self.ongoing_transfer() {
                                // The old transfer is not complete
                                return Err(frame);
                            }

                            let new_buf = &*frame;
                            self.payload.channel.set_memory_address(new_buf.buffer_as_ptr() as u32, true);
                            self.payload.channel.set_transfer_length(new_buf.len() as u16);

                            // If there has been an error, clear the error flag to let the next
                            // transaction start
                            if self.payload.channel.isr().$teifX().bit_is_set() {
                                self.payload.channel.ifcr().write(|w| w.$cteifX().set_bit());
                            }

                            // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                            // the next statement, which starts the DMA transfer
                            atomic::compiler_fence(Ordering::Release);

                            self.payload.channel.start();

                            self.buffer = Some(frame);

                            Ok(())
                        }
                    }

                    impl<BUFFER, PAYLOAD, const N: usize> FrameReader<BUFFER, RxDma<PAYLOAD, $CX>, N>
                    where
                        BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        /// This function should be called from the transfer complete interrupt of
                        /// the corresponding DMA channel.
                        ///
                        /// Returns the full buffer received by the USART.
                        #[inline]
                        pub fn transfer_complete_interrupt(&mut self, next_frame: BUFFER) -> BUFFER {
                            self.internal_interrupt(next_frame, false)
                        }

                        /// This function should be called from the character match interrupt of
                        /// the corresponding USART
                        ///
                        /// Returns the buffer received by the USART, including the matching
                        /// character.
                        #[inline]
                        pub fn character_match_interrupt(&mut self, next_frame: BUFFER) -> BUFFER {
                            self.internal_interrupt(next_frame, true)
                        }

                        /// This function should be called from the receiver timeout interrupt of
                        /// the corresponding USART
                        ///
                        /// Returns the buffer received by the USART.
                        #[inline]
                        pub fn receiver_timeout_interrupt(&mut self, next_frame: BUFFER) -> BUFFER {
                            self.internal_interrupt(next_frame, false)
                        }

                        fn internal_interrupt(
                            &mut self,
                            mut next_frame: BUFFER,
                            character_match_interrupt: bool,
                        ) -> BUFFER {
                            let old_buf = &mut *self.buffer;
                            let new_buf = &mut *next_frame;
                            new_buf.clear();

                            // Clear ISR flag (Transfer Complete)
                            if !self.payload.channel.in_progress() {
                                self.payload.channel.ifcr().write(|w| w.$ctcifX().set_bit());
                            } else if character_match_interrupt {
                                // 1. If DMA not done and there was a character match interrupt,
                                // let the DMA flush a little and then halt transfer.
                                //
                                // This is to alleviate the race condition between the character
                                // match interrupt and the DMA memory transfer.
                                let left_in_buffer = self.payload.channel.get_cndtr() as usize;

                                for _ in 0..5 {
                                    let now_left = self.payload.channel.get_cndtr() as usize;

                                    if left_in_buffer - now_left >= 4 {
                                        // We have gotten 4 extra characters flushed
                                        break;
                                    }
                                }
                            }

                            self.payload.channel.stop();

                            // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                            // the next statement, which starts a new DMA transfer
                            atomic::compiler_fence(Ordering::SeqCst);

                            let left_in_buffer = self.payload.channel.get_cndtr() as usize;
                            let got_data_len = old_buf.max_len() - left_in_buffer; // How many bytes we got
                            unsafe {
                                old_buf.set_len(got_data_len);
                            }

                            // 2. Check DMA race condition by finding matched character, and that
                            //    the length is larger than 0
                            let len = if character_match_interrupt && got_data_len > 0 {
                                let search_buf = old_buf.read();

                                // Search from the end
                                let ch = self.matching_character;
                                if let Some(pos) = search_buf.iter().rposition(|&x| x == ch) {
                                    pos+1
                                } else {
                                    // No character match found
                                    0
                                }
                            } else {
                                old_buf.len()
                            };

                            // 3. Start DMA again
                            let diff = if len < got_data_len {
                                // We got some extra characters in the from the new frame, move
                                // them into the new buffer
                                let diff = got_data_len - len;

                                let new_buf_ptr = new_buf.buffer_as_mut_ptr();
                                let old_buf_ptr = old_buf.buffer_as_ptr();

                                // new_buf[0..diff].copy_from_slice(&old_buf[len..got_data_len]);
                                unsafe {
                                    ptr::copy_nonoverlapping(old_buf_ptr.add(len), new_buf_ptr, diff);
                                }

                                diff
                            } else {
                                0
                            };

                            self.payload.channel.set_memory_address(unsafe { new_buf.buffer_as_ptr().add(diff) } as u32, true);
                            self.payload.channel.set_transfer_length((new_buf.max_len() - diff) as u16);
                            let received_buffer = core::mem::replace(&mut self.buffer, next_frame);

                            // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                            // the next statement, which starts the DMA transfer
                            atomic::compiler_fence(Ordering::Release);

                            self.payload.channel.start();

                            // 4. Return full frame
                            received_buffer
                        }
                    }

                    impl<B, PAYLOAD> CircBuffer<B, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {

                        /// Return the partial contents of the buffer half being written
                        pub fn partial_peek<R, F, T>(&mut self, f: F) -> Result<R, Error>
                            where
                            F: FnOnce(&[T], Half) -> Result<(usize, R), ()>,
                            B: AsRef<[T]>,
                        {
                            // this inverts expectation and returns the half being _written_
                            let buf = match self.readable_half {
                                Half::First => &self.buffer[1],
                                Half::Second => &self.buffer[0],
                            };
                            //                          ,- half-buffer
                            //    [ x x x x y y y y y z | z z z z z z z z z z ]
                            //                       ^- pending=11
                            let pending = self.payload.channel.get_cndtr() as usize; // available bytes in _whole_ buffer
                            let slice = buf.as_ref();
                            let capacity = slice.len(); // capacity of _half_ a buffer
                            //     <--- capacity=10 --->
                            //    [ x x x x y y y y y z | z z z z z z z z z z ]
                            let pending = if pending > capacity {
                                pending - capacity
                            } else {
                                pending
                            };
                            //                          ,- half-buffer
                            //    [ x x x x y y y y y z | z z z z z z z z z z ]
                            //                       ^- pending=1
                            let end = capacity - pending;
                            //    [ x x x x y y y y y z | z z z z z z z z z z ]
                            //                       ^- end=9
                            //             ^- consumed_offset=4
                            //             [y y y y y] <-- slice
                            let slice = &buf.as_ref()[self.consumed_offset..end];
                            match f(slice, self.readable_half) {
                                Ok((l, r)) => { self.consumed_offset += l; Ok(r) },
                                Err(_) => Err(Error::BufferError),
                            }
                        }

                        /// Peeks into the readable half of the buffer
                        /// Returns the result of the closure
                        pub fn peek<R, F, T>(&mut self, f: F) -> Result<R, Error>
                            where
                            F: FnOnce(&[T], Half) -> R,
                            B: AsRef<[T]>,
                        {
                            let half_being_read = self.readable_half()?;
                            let buf = match half_being_read {
                                Half::First => &self.buffer[0],
                                Half::Second => &self.buffer[1],
                            };
                            let slice = &buf.as_ref()[self.consumed_offset..];
                            self.consumed_offset = 0;
                            Ok(f(slice, half_being_read))
                        }

                        /// Returns the `Half` of the buffer that can be read
                        pub fn readable_half(&mut self) -> Result<Half, Error> {
                            let isr = self.payload.channel.isr();
                            let first_half_is_done = isr.$htifX().bit_is_set();
                            let second_half_is_done = isr.$tcifX().bit_is_set();

                            if first_half_is_done && second_half_is_done {
                                return Err(Error::Overrun);
                            }

                            let last_read_half = self.readable_half;

                            Ok(match last_read_half {
                                Half::First => {
                                    if second_half_is_done {
                                        self.payload.channel.ifcr().write(|w| w.$ctcifX().set_bit());

                                        self.readable_half = Half::Second;
                                        Half::Second
                                    } else {
                                        last_read_half
                                    }
                                }
                                Half::Second => {
                                    if first_half_is_done {
                                        self.payload.channel.ifcr().write(|w| w.$chtifX().set_bit());

                                        self.readable_half = Half::First;
                                        Half::First
                                    } else {
                                        last_read_half
                                    }
                                }
                            })
                        }

                        /// Stops the transfer and returns the underlying buffer and RxDma
                        pub fn stop(mut self) -> (&'static mut [B; 2], RxDma<PAYLOAD, $CX>) {
                            self.payload.stop();

                            (self.buffer, self.payload)
                        }
                    }

                    impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn is_done(&self) -> bool {
                            !self.payload.channel.in_progress()
                        }

                        pub fn wait(mut self) -> (BUFFER, RxDma<PAYLOAD, $CX>) {
                            // XXX should we check for transfer errors here?
                            // The manual says "A DMA transfer error can be generated by reading
                            // from or writing to a reserved address space". I think it's impossible
                            // to get to that state with our type safe API and *safe* Rust.
                            while !self.is_done() {}

                            self.payload.stop();

                            // TODO can we weaken this compiler barrier?
                            // NOTE(compiler_fence) operations on `buffer` should not be reordered
                            // before the previous statement, which marks the DMA transfer as done
                            atomic::compiler_fence(Ordering::SeqCst);

                            // `Transfer` needs to have a `Drop` implementation, because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                            // and `mem::forget`.
                            //
                            // NOTE(unsafe) There is no panic branch between getting the resources
                            // and forgetting `self`.
                            unsafe {
                                let buffer = ptr::read(&self.buffer);
                                let payload = ptr::read(&self.payload);
                                core::mem::forget(self);
                                (buffer, payload)
                            }
                        }
                    }

                    impl<BUFFER, PAYLOAD> Transfer<R, BUFFER, TxDma<PAYLOAD, $CX>>
                    where
                        TxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn is_done(&self) -> bool {
                            !self.payload.channel.in_progress()
                        }

                        pub fn wait(mut self) -> (BUFFER, TxDma<PAYLOAD, $CX>) {
                            // XXX should we check for transfer errors here?
                            // The manual says "A DMA transfer error can be generated by reading
                            // from or writing to a reserved address space". I think it's impossible
                            // to get to that state with our type safe API and *safe* Rust.
                            while !self.is_done() {}

                            self.payload.stop();

                            // TODO can we weaken this compiler barrier?
                            // NOTE(compiler_fence) operations on `buffer` should not be reordered
                            // before the previous statement, which marks the DMA transfer as done
                            atomic::compiler_fence(Ordering::SeqCst);

                            // `Transfer` needs to have a `Drop` implementation, because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                            // and `mem::forget`.
                            //
                            // NOTE(unsafe) There is no panic branch between getting the resources
                            // and forgetting `self`.
                            unsafe {
                                let buffer = ptr::read(&self.buffer);
                                let payload = ptr::read(&self.payload);
                                core::mem::forget(self);
                                (buffer, payload)
                            }
                        }
                    }

                    impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn peek<T>(&self) -> &[T]
                        where
                            BUFFER: AsRef<[T]>,
                        {
                            let pending = self.payload.channel.get_cndtr() as usize;

                            let capacity = self.buffer.as_ref().len();

                            &self.buffer.as_ref()[..(capacity - pending)]
                        }
                    }

                    impl<BUFFER, PAYLOAD> Transfer<R, BUFFER, TxDma<PAYLOAD, $CX>>
                    where
                        TxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn peek<T>(&self) -> &[T]
                        where
                            BUFFER: AsRef<[T]>
                        {
                            let pending = self.payload.channel.get_cndtr() as usize;

                            let capacity = self.buffer.as_ref().len();

                            &self.buffer.as_ref()[..(capacity - pending)]
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Channels = Channels;

                    fn split(self, ahb: &mut AHB1) -> Channels {
                        ahb.enr().modify(|_, w| w.$dmaXen().set_bit());

                        // reset the DMA control registers (stops all on-going transfers)
                        $(
                            self.$ccrX.reset();
                        )+

                        Channels((), $($CX { }),+)
                    }
                }
            }
        )+
    }
}

dma! {
    DMA1: (dma1, dma1en, dma1rst, {
        C1: (
            ccr1, CCR1,
            cndtr1, CNDTR1,
            cpar1, CPAR1,
            cmar1, CMAR1,
            htif1, tcif1,
            chtif1, ctcif1, cgif1,
            teif1, cteif1
        ),
        C2: (
            ccr2, CCR2,
            cndtr2, CNDTR2,
            cpar2, CPAR2,
            cmar2, CMAR2,
            htif2, tcif2,
            chtif2, ctcif2, cgif2,
            teif2, cteif2
        ),
        C3: (
            ccr3, CCR3,
            cndtr3, CNDTR3,
            cpar3, CPAR3,
            cmar3, CMAR3,
            htif3, tcif3,
            chtif3, ctcif3, cgif3,
            teif3, cteif3
        ),
        C4: (
            ccr4, CCR4,
            cndtr4, CNDTR4,
            cpar4, CPAR4,
            cmar4, CMAR4,
            htif4, tcif4,
            chtif4, ctcif4, cgif4,
            teif4, cteif4
        ),
        C5: (
            ccr5, CCR5,
            cndtr5, CNDTR5,
            cpar5, CPAR5,
            cmar5, CMAR5,
            htif5, tcif5,
            chtif5, ctcif5, cgif5,
            teif5, cteif5
        ),
        C6: (
            ccr6, CCR6,
            cndtr6, CNDTR6,
            cpar6, CPAR6,
            cmar6, CMAR6,
            htif6, tcif6,
            chtif6, ctcif6, cgif6,
            teif6, cteif6
        ),
        C7: (
            ccr7, CCR7,
            cndtr7, CNDTR7,
            cpar7, CPAR7,
            cmar7, CMAR7,
            htif7, tcif7,
            chtif7, ctcif7, cgif7,
            teif7, cteif7
        ),
    }),
    DMA2: (dma2, dma2en, dma2rst, {
        C1: (
            ccr1, CCR1,
            cndtr1, CNDTR1,
            cpar1, CPAR1,
            cmar1, CMAR1,
            htif1, tcif1,
            chtif1, ctcif1, cgif1,
            teif1, cteif1
        ),
        C2: (
            ccr2, CCR2,
            cndtr2, CNDTR2,
            cpar2, CPAR2,
            cmar2, CMAR2,
            htif2, tcif2,
            chtif2, ctcif2, cgif2,
            teif2, cteif2
        ),
        C3: (
            ccr3, CCR3,
            cndtr3, CNDTR3,
            cpar3, CPAR3,
            cmar3, CMAR3,
            htif3, tcif3,
            chtif3, ctcif3, cgif3,
            teif3, cteif3
        ),
        C4: (
            ccr4, CCR4,
            cndtr4, CNDTR4,
            cpar4, CPAR4,
            cmar4, CMAR4,
            htif4, tcif4,
            chtif4, ctcif4, cgif4,
            teif4, cteif4
        ),
        C5: (
            ccr5, CCR5,
            cndtr5, CNDTR5,
            cpar5, CPAR5,
            cmar5, CMAR5,
            htif5, tcif5,
            chtif5, ctcif5, cgif5,
            teif5, cteif5
        ),
        C6: (
            ccr6, CCR6,
            cndtr6, CNDTR6,
            cpar6, CPAR6,
            cmar6, CMAR6,
            htif6, tcif6,
            chtif6, ctcif6, cgif6,
            teif6, cteif6
        ),
        C7: (
            ccr7, CCR7,
            cndtr7, CNDTR7,
            cpar7, CPAR7,
            cmar7, CMAR7,
            htif7, tcif7,
            chtif7, ctcif7, cgif7,
            teif7, cteif7
        ),
    }),
}

/// DMA Receiver
pub struct RxDma<PAYLOAD, RXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: RXCH,
}

/// DMA Transmitter
pub struct TxDma<PAYLOAD, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: TXCH,
}

/// DMA Receiver/Transmitter
pub struct RxTxDma<PAYLOAD, RXCH, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub rxchannel: RXCH,
    pub txchannel: TXCH,
}

pub trait Receive {
    type RxChannel;
    type TransmittedWord;
}

pub trait Transmit {
    type TxChannel;
    type ReceivedWord;
}

/// Trait for circular DMA readings from peripheral to memory.
pub trait CircReadDma<B, RS>: Receive
where
    &'static mut [B; 2]: StaticWriteBuffer<Word = RS>,
    B: 'static,
    Self: core::marker::Sized,
{
    fn circ_read(self, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self>;
}

/// Trait for DMA readings from peripheral to memory.
pub trait ReadDma<B, RS>: Receive
where
    B: StaticWriteBuffer<Word = RS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn read(self, buffer: B) -> Transfer<W, B, Self>;
}

/// Trait for DMA writing from memory to peripheral.
pub trait WriteDma<B, TS>: Transmit
where
    B: StaticReadBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn write(self, buffer: B) -> Transfer<R, B, Self>;
}
