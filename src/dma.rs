//! Direct Memory Access Engine

#![allow(dead_code)]

use core::fmt;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::ops::DerefMut;
use core::ptr;
use core::slice;
use core::sync::atomic::{compiler_fence, Ordering};
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

use crate::rcc::AHB1;
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

pub trait CharacterMatch {
    /// Checks to see if the peripheral has detected a character match and
    /// clears the flag
    fn check_character_match(&mut self, clear: bool) -> bool;
}

pub trait ReceiverTimeout {
    /// Check to see if the peripheral has detected a
    /// receiver timeout and clears the flag
    fn check_receiver_timeout(&mut self, clear: bool) -> bool;
}

pub trait OperationError<O, E> {
    /// Check to see if the peripheral has detected some
    /// sort of error while performing an operation
    fn check_operation_error(&mut self) -> Result<O, E>;
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
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
    PAYLOAD: CharacterMatch,
{
    /// Checks to see if the peripheral has detected a character match and
    /// clears the flag
    pub fn check_character_match(&mut self, clear: bool) -> bool {
        self.payload.payload.check_character_match(clear)
    }
}

impl<BUFFER, PAYLOAD, CHANNEL, const N: usize> FrameReader<BUFFER, RxDma<PAYLOAD, CHANNEL>, N>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
    PAYLOAD: ReceiverTimeout,
{
    pub fn check_receiver_timeout(&mut self, clear: bool) -> bool {
        self.payload.payload.check_receiver_timeout(clear)
    }
}

impl<BUFFER, PAYLOAD, CHANNEL, const N: usize> FrameReader<BUFFER, RxDma<PAYLOAD, CHANNEL>, N>
where
    BUFFER: Sized + StableDeref<Target = DMAFrame<N>> + DerefMut + 'static,
{
    pub fn check_operation_error<O, E>(&mut self) -> Result<O, E>
    where
        PAYLOAD: OperationError<O, E>,
    {
        self.payload.payload.check_operation_error()
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
    buffer: &'static mut BUFFER,
    payload: PAYLOAD,
    read_index: usize,
    write_previous: usize,
}

impl<BUFFER, PAYLOAD> CircBuffer<BUFFER, PAYLOAD>
where
    &'static mut BUFFER: StaticWriteBuffer,
    BUFFER: 'static,
{
    pub(crate) fn new(buf: &'static mut BUFFER, payload: PAYLOAD) -> Self {
        CircBuffer {
            buffer: buf,
            payload,
            read_index: 0,
            write_previous: 0,
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

impl<BUFFER, PAYLOAD> Transfer<RW, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn rw(buffer: BUFFER, payload: PAYLOAD) -> Self {
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

impl<MODE, BUFFER, PAYLOAD> Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn extract_inner_without_drop(self) -> (BUFFER, PAYLOAD) {
        // `Transfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            // We cannot use mem::replace as we do not have valid objects to replace with
            let buffer = ptr::read(&self.buffer);
            let payload = ptr::read(&self.payload);
            core::mem::forget(self);
            (buffer, payload)
        }
    }
}

/// Read transfer
pub struct R;

/// Write transfer
pub struct W;

/// Read/Write transfer
pub struct RW;

macro_rules! for_all_pairs {
    ($mac:ident: $($x:ident)*) => {
        // Duplicate the list
        for_all_pairs!(@inner $mac: $($x)*; $($x)*);
    };

    // The end of iteration: we exhausted the list
    (@inner $mac:ident: ; $($x:ident)*) => {};

    // The head/tail recursion: pick the first element of the first list
    // and recursively do it for the tail.
    (@inner $mac:ident: $head:ident $($tail:ident)*; $($x:ident)*) => {
        $(
            $mac!($head $x);
        )*
        for_all_pairs!(@inner $mac: $($tail)*; $($x)*);
    };
}

macro_rules! rx_tx_channel_mapping {
    ($CH_A:ident $CH_B:ident) => {
        impl<BUFFER, PAYLOAD> Transfer<RW, BUFFER, RxTxDma<PAYLOAD, $CH_A, $CH_B>>
        where
            RxTxDma<PAYLOAD, $CH_A, $CH_B>: TransferPayload,
        {
            pub fn is_done(&self) -> bool {
                !self.payload.rx_channel.in_progress() && !self.payload.tx_channel.in_progress()
            }

            pub fn wait(mut self) -> (BUFFER, RxTxDma<PAYLOAD, $CH_A, $CH_B>) {
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

                // `Transfer` has a `Drop` implementation because we accept
                // managed buffers that can free their memory on drop. Because of that
                // we can't move out of the `Transfer`'s fields directly.
                self.extract_inner_without_drop()
            }
        }

        impl<BUFFER, PAYLOAD> Transfer<RW, BUFFER, RxTxDma<PAYLOAD, $CH_A, $CH_B>>
        where
            RxTxDma<PAYLOAD, $CH_A, $CH_B>: TransferPayload,
        {
            pub fn peek<T>(&self) -> &[T]
            where
                BUFFER: AsRef<[T]>,
            {
                let pending = self.payload.rx_channel.get_cndtr() as usize;

                let capacity = self.buffer.as_ref().len();

                &self.buffer.as_ref()[..(capacity - pending)]
            }
        }
    };
}

macro_rules! dma {
    ($($DMAX:ident: ($dmaX:ident, {
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

                use crate::dma::{CircBuffer, FrameReader, FrameSender, DMAFrame, DmaExt, Error, Event, Transfer, W, R, RW, RxDma, RxTxDma, TxDma, TransferPayload};
                use crate::rcc::{AHB1, Enable};

                #[allow(clippy::manual_non_exhaustive)]
                pub struct Channels((), $(pub $CX),+);

                for_all_pairs!(rx_tx_channel_mapping: $($CX)+);

                $(
                    /// A singleton that represents a single DMAx channel (channel X in this case)
                    ///
                    /// This singleton has exclusive access to the registers of the DMAx channel X
                    pub struct $CX;

                    impl $CX {
                        /// Associated peripheral `address`
                        ///
                        /// `inc` indicates whether the address will be incremented after every transfer
                        #[inline]
                        pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
                            self.cpar().write(|w|
                                unsafe { w.pa().bits(address) }
                            );
                            self.ccr().modify(|_, w| w.pinc().bit(inc) );
                        }

                        /// `address` where from/to data will be read/write
                        ///
                        /// `inc` indicates whether the address will be incremented after every transfer
                        #[inline]
                        pub fn set_memory_address(&mut self, address: u32, inc: bool) {
                            self.cmar().write(|w|
                                unsafe { w.ma().bits(address) }
                            );
                            self.ccr().modify(|_, w| w.minc().bit(inc) );
                        }

                        /// The amount of transfers that makes up one transaction
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

                        #[cfg(not(any(
                            // feature = "stm32l4p5",
                            // feature = "stm32l4q5",
                            // feature = "stm32l4r5",
                            // feature = "stm32l4s5",
                            // feature = "stm32l4r7",
                            // feature = "stm32l4s7",
                            feature = "stm32l4r9",
                            feature = "stm32l4s9"
                        )))]
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
                            let got_data_len = old_buf.max_len() - left_in_buffer; // How many transfers were completed = how many bytes are available
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
                            unsafe { old_buf.set_len(got_data_len - diff) };
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
                        /// Determines if the write index passed the given `mark` when it moved
                        /// from `previous` to `current` with wrapping behaviour at `wrap`.
                        /// When `current` reaches `mark` (`current == mark`), this method already
                        /// returns `true`.
                        fn passed_mark(&self, mut previous: usize, mut current: usize, mark: usize, wrap: usize) -> bool {
                            // We have three indexes mark (m), previous (p) and current (c) so
                            // there are fac(3)=6 possibilities how those can be ordered. For both
                            // cases (passed, !passed), there are three wrapped variations each:
                            // !passed: 1. m <= p <= c, 2. c < m <= p, 3. p <= c < m
                            //  passed: 1. m <= c <  p, 2. p < m <= c, 3. c <  p < m
                            // By enforcing p >= m and c >= m (reverting the wrap), we only have to
                            // check the first case.
                            if previous < mark {
                                previous += wrap;
                            }
                            if current < mark {
                                current += wrap;
                            }
                            current < previous && current >= mark
                        }

                        /// Reads and removes the available contents of the dma buffer into `buf`.
                        /// Returns `Err(Error::Overrun)` if an overrun is detected but there is no
                        /// guarantee that every overrun can be detected.
                        /// On success, returns the number of words written to `buf`.
                        pub fn read<T>(&mut self, buf: &mut [T]) -> Result<usize, Error>
                            where
                            B: AsRef<[T]>,
                            T: Copy,
                        {
                            // We do our best to figure out when an overrun occurred but without
                            // risking false positives.
                            //
                            // One possibility to detect an overrun is by examining the read- and
                            // write-indexes: If the write-index passes the read-index, that is an
                            // overrun condition because an unread value is overwritten. This check
                            // can fail if the write-index passed the read-index but due to
                            // wrapping, this can not be detected. For example, this can happen if
                            // `capacity` many words were written since the last read which looks
                            // like no word has been written.
                            //
                            // Another possibility to detect overruns is by checking the
                            // TransferComplete and HalfTransferComplete flags. For example, the
                            // TransferComplete flag is set when the write-index wraps around so
                            // whenever we encounter this flag the new write-index should be
                            // smaller than the previous one. If it is not, more than `capacity`
                            // many words must have been written which definitely must be an
                            // overrun. Another possibility to formulate this condition is to check
                            // wheter the write-index passed index 0. A similar condition can be
                            // formulated for the HalfTransferComplete flag, i.e. check whether the
                            // write-index passed index capacity/2.
                            //
                            // Unfortunately, even both checks together can not guarantee that we
                            // detect all overruns.
                            // Example:
                            // read = 2, write = 3, 2*capacity-2 words written => write = 1.
                            let capacity = self.buffer.as_ref().len();
                            // We read the flags before reading the current write-index because if
                            // another word is written between those two accesses, this ordering
                            // prevents a false positive overrun error.
                            let isr = self.payload.channel.isr();
                            let half_complete_flag = isr.$htifX().bit_is_set();
                            if half_complete_flag {
                                self.payload.channel.ifcr().write(|w| w.$chtifX().set_bit());
                            }
                            let transfer_complete_flag = isr.$tcifX().bit_is_set();
                            if transfer_complete_flag {
                                self.payload.channel.ifcr().write(|w| w.$ctcifX().set_bit());
                            }
                            let write_current = capacity - self.payload.channel.get_cndtr() as usize;
                            // Copy the data before examining the overrun conditions. If the
                            // overrun happens shortly after the flags and write-index were read,
                            // we can not detect it anyways. So we can only hope that we have
                            // already read the word(s) that will be overwritten.
                            let available = if write_current >= self.read_index {
                                write_current - self.read_index
                            } else {
                                capacity + write_current - self.read_index
                            };
                            let read_len = core::cmp::min(available, buf.len());
                            if self.read_index + read_len <= capacity {
                                // non-wrapping read
                                buf[..read_len].copy_from_slice(&self.buffer.as_ref()[self.read_index..self.read_index+read_len]);
                            } else {
                                // wrapping read
                                let first_read_len = capacity - self.read_index;
                                let second_read_len = read_len - first_read_len;
                                buf[..first_read_len].copy_from_slice(&self.buffer.as_ref()[self.read_index..]);
                                buf[first_read_len..read_len].copy_from_slice(&self.buffer.as_ref()[..second_read_len]);
                            }
                            // For checking the overrun conditions, it is important that we use the
                            // old read_index so do not increment it yet but check overrun
                            // conditions first.
                            // For odd buffer sizes, the half-complete flag is set at
                            // ceil(capacity/2).
                            let overrun =
                                self.passed_mark(self.write_previous, write_current, self.read_index, capacity)
                                || (transfer_complete_flag && !self.passed_mark(self.write_previous, write_current, 0, capacity))
                                || (half_complete_flag && !self.passed_mark(self.write_previous, write_current, (capacity+1)/2, capacity));
                            self.write_previous = write_current;
                            if overrun {
                                self.read_index = write_current;
                                Err(Error::Overrun)
                            } else {
                                self.read_index += read_len;
                                if self.read_index >= capacity {
                                    self.read_index -= capacity;
                                }
                                Ok(read_len)
                            }
                        }

                        /// Stops the transfer and returns the underlying buffer and RxDma
                        pub fn stop(mut self) -> (&'static mut B, RxDma<PAYLOAD, $CX>) {
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

                            // `Transfer` has a `Drop` implementation because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields directly.
                            self.extract_inner_without_drop()
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

                            // `Transfer` has a `Drop` implementation because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields directly.
                            self.extract_inner_without_drop()
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
                        <$DMAX>::enable(ahb);

                        #[cfg(any(
                            // feature = "stm32l4p5",
                            // feature = "stm32l4q5",
                            // feature = "stm32l4r5",
                            // feature = "stm32l4s5",
                            // feature = "stm32l4r7",
                            // feature = "stm32l4s7",
                            feature = "stm32l4r9",
                            feature = "stm32l4s9"
                        ))]
                        ahb.enr().modify(|_, w| w.dmamux1en().set_bit());

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
    DMA1: (dma1, {
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
    DMA2: (dma2, {
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
    pub rx_channel: RXCH,
    pub tx_channel: TXCH,
}

pub trait Receive {
    type RxChannel;
    type TransmittedWord;
}

pub trait Transmit {
    type TxChannel;
    type ReceivedWord;
}

pub trait ReceiveTransmit {
    type RxChannel;
    type TxChannel;
    type TransferedWord;
}

/// Trait for circular DMA readings from peripheral to memory.
pub trait CircReadDma<B, RS>: Receive
where
    &'static mut B: StaticWriteBuffer<Word = RS>,
    B: 'static,
    Self: core::marker::Sized,
{
    fn circ_read(self, buffer: &'static mut B) -> CircBuffer<B, Self>;
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

/// Trait for DMA simultaneously writing and reading between memory and peripheral.
pub trait TransferDma<B, TS>: ReceiveTransmit
where
    B: StaticWriteBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn transfer(self, buffer: B) -> Transfer<RW, B, Self>;
}
