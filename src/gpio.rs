//! General Purpose Input / Output

pub use crate::hal::digital::v2::PinState;
use crate::hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};
use core::convert::Infallible;
use core::marker::PhantomData;

use crate::rcc::AHB2;

mod convert;
pub use convert::PinMode;
mod partially_erased;
pub use partially_erased::{PEPin, PartiallyErasedPin};
mod exti;
pub use exti::ExtiPin;
mod erased;
pub use erased::{EPin, ErasedPin};

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, ahb: &mut AHB2) -> Self::Parts;
}

/// Input mode (type state)
pub struct Input<IOCONF = Floating> {
    _mode: PhantomData<IOCONF>,
}

/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<OTYPE = PushPull> {
    _mode: PhantomData<OTYPE>,
}

/// Push pull output (type state)
pub struct PushPull;
/// Open drain output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;

pub type Debugger = Alternate<0, PushPull>;

/// GPIO Pin speed selection
pub enum Speed {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3,
}

pub trait PinExt {
    type Mode;
    /// Return pin number
    fn pin_id(&self) -> u8;
    /// Return port number
    fn port_id(&self) -> u8;
}

/// Alternate mode (type state)
pub struct Alternate<const A: u8, OTYPE = PushPull> {
    _mode: PhantomData<OTYPE>,
}

#[derive(Debug, PartialEq)]
pub enum Edge {
    Rising,
    Falling,
    RisingFalling,
}

mod marker {
    /// Marker trait that show if `ExtiPin` can be implemented
    pub trait Interruptable {}
}

impl<MODE> marker::Interruptable for Output<MODE> {}
impl<MODE> marker::Interruptable for Input<MODE> {}

/// Opaque MODER register
pub struct MODER<const P: char> {
    _0: (),
}

impl<const P: char> MODER<P> {
    pub(crate) fn new() -> Self {
        Self { _0: () }
    }
}

/// Opaque OTYPER register
pub struct OTYPER<const P: char> {
    _0: (),
}

impl<const P: char> OTYPER<P> {
    pub(crate) fn new() -> Self {
        Self { _0: () }
    }
}

/// Opaque OSPEEDR register
pub struct OSPEEDR<const P: char> {
    _0: (),
}
impl<const P: char> OSPEEDR<P> {
    pub(crate) fn new() -> Self {
        Self { _0: () }
    }
}

/// Opaque PUPDR register
pub struct PUPDR<const P: char> {
    _0: (),
}

impl<const P: char> PUPDR<P> {
    pub(crate) fn new() -> Self {
        Self { _0: () }
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PXx:ident, $port_id:literal, $({ $pwrenable:expr },)? [
        $($PXi:ident: ($pxi:ident, $i:expr $(, $MODE:ty)?),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use crate::stm32::$GPIOX;

            use crate::rcc::{AHB2, Enable, Reset};
            use super::{Afr, Analog, GpioExt, Pin, MODER, OTYPER, OSPEEDR, PUPDR};

            /// GPIO parts
            pub struct Parts {
                /// Opaque AFRH register
                pub afrh: Afr<$port_id, true>,
                /// Opaque AFRL register
                pub afrl: Afr<$port_id, false>,
                /// Opaque MODER register
                pub moder: MODER<$port_id>,
                /// Opaque OTYPER register
                pub otyper: OTYPER<$port_id>,
                /// Opaque OSPEEDR register
                pub ospeedr: OSPEEDR<$port_id>,
                /// Opaque PUPDR register
                pub pupdr: PUPDR<$port_id>,
                $(
                    /// Pin
                    pub $pxi: $PXi $(<$MODE>)?,
                )+
            }

            $(
                pub type $PXi<MODE = Analog> = Pin<$port_id, $i, MODE>;
            )+

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, ahb: &mut AHB2) -> Parts {
                    <$GPIOX>::enable(ahb);
                    <$GPIOX>::reset(ahb);
                    $($pwrenable)?

                    Parts {
                        afrh: Afr(()),
                        afrl: Afr(()),
                        moder: MODER::new(),
                        otyper: OTYPER::new(),
                        ospeedr: OSPEEDR::new(),
                        pupdr: PUPDR::new(),
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }
            }
        }

        pub use $gpiox::{
            $($PXi,)*
        };
    }
}

/// Generic pin type
///
/// - `P` is port name: `A` for GPIOA, `B` for GPIOB, etc.
/// - `N` is pin number: from `0` to `15`.
/// - `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
pub struct Pin<const P: char, const N: u8, MODE = Analog> {
    _mode: PhantomData<MODE>,
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }
}

impl<const P: char, const N: u8, MODE> PinExt for Pin<P, N, MODE> {
    type Mode = MODE;

    #[inline(always)]
    fn pin_id(&self) -> u8 {
        N
    }
    #[inline(always)]
    fn port_id(&self) -> u8 {
        P as u8 - b'A'
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Output<MODE>> {
    /// Set pin speed
    pub fn set_speed(self, speed: Speed) -> Self {
        let offset = 2 * { N };

        unsafe {
            (*Gpio::<P>::ptr())
                .ospeedr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset)))
        };

        self
    }
}

impl<const P: char, const N: u8> Pin<P, N, Output<OpenDrain>> {
    /// Enables / disables the internal pull up
    pub fn internal_pull_up(&mut self, _pupdr: &mut PUPDR<P>, on: bool) {
        let offset = 2 * { N };
        let value = if on { 0b01 } else { 0b00 };
        unsafe {
            (*Gpio::<P>::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)))
        };
    }
}

impl<const P: char, const N: u8, const A: u8, MODE> Pin<P, N, Alternate<A, MODE>> {
    /// Set pin speed
    pub fn set_speed(self, speed: Speed) -> Self {
        let offset = 2 * { N };

        unsafe {
            (*Gpio::<P>::ptr())
                .ospeedr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset)))
        };

        self
    }

    /// Enables / disables the internal pull up
    pub fn internal_pull_up(&mut self, _pupdr: &mut PUPDR<P>, on: bool) {
        let offset = 2 * { N };
        let value = if on { 0b01 } else { 0b00 };
        unsafe {
            (*Gpio::<P>::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)))
        };
    }
}

impl<const P: char, const N: u8, const A: u8> Pin<P, N, Alternate<A, PushPull>> {
    /// Turns pin alternate configuration pin into open drain
    pub fn set_open_drain(self) -> Pin<P, N, Alternate<A, OpenDrain>> {
        let offset = { N };
        unsafe {
            (*Gpio::<P>::ptr())
                .otyper
                .modify(|r, w| w.bits(r.bits() | (1 << offset)))
        };

        Pin::new()
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// Erases the pin number from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase_number(self) -> PEPin<P, MODE> {
        PEPin::new(N)
    }

    /// Erases the pin number and the port from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase(self) -> EPin<MODE> {
        EPin::new(P as u8 - b'A', N)
    }
}

// Internal helper functions
//
// NOTE: The functions in this impl block are "safe", but they
// are callable when the pin is in modes where they don't make
// sense.
impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// Set the output of the pin regardless of its mode.
    /// Primarily used to set the output value of the pin
    /// before changing its mode to an output to avoid
    /// a short spike of an incorrect value
    #[inline(always)]
    fn _set_state(&mut self, state: PinState) {
        match state {
            PinState::High => self._set_high(),
            PinState::Low => self._set_low(),
        }
    }
    #[inline(always)]
    fn _set_high(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << N)) }
    }
    #[inline(always)]
    fn _set_low(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << (16 + N))) }
    }

    #[inline(always)]
    fn _is_set_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Gpio::<P>::ptr()).odr.read().bits() & (1 << N) == 0 }
    }
    #[inline(always)]
    fn _is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Gpio::<P>::ptr()).idr.read().bits() & (1 << N) == 0 }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Output<MODE>> {
    #[inline]
    pub fn set_high(&mut self) {
        self._set_high()
    }
    #[inline]
    pub fn set_low(&mut self) {
        self._set_low()
    }
    #[inline(always)]
    pub fn get_state(&self) -> PinState {
        if self._is_set_low() {
            PinState::Low
        } else {
            PinState::High
        }
    }
    #[inline(always)]
    pub fn set_state(&mut self, state: PinState) {
        match state {
            PinState::Low => self._set_low(),
            PinState::High => self._set_high(),
        }
    }
    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self._is_set_low()
    }
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self._is_set_low()
    }
    #[inline]
    pub fn toggle(&mut self) {
        if self._is_set_low() {
            self._set_high()
        } else {
            self._set_low()
        }
    }
}

impl<const P: char, const N: u8, MODE> OutputPin for Pin<P, N, Output<MODE>> {
    type Error = Infallible;
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<const P: char, const N: u8, MODE> StatefulOutputPin for Pin<P, N, Output<MODE>> {
    #[inline]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }
    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<const P: char, const N: u8, MODE> ToggleableOutputPin for Pin<P, N, Output<MODE>> {
    type Error = Infallible;

    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Input<MODE>> {
    #[inline]
    pub fn is_high(&self) -> bool {
        !self._is_low()
    }
    #[inline]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

impl<const P: char, const N: u8, MODE> InputPin for Pin<P, N, Input<MODE>> {
    type Error = Infallible;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

impl<const P: char, const N: u8> Pin<P, N, Output<OpenDrain>> {
    #[inline]
    pub fn is_high(&self) -> bool {
        !self._is_low()
    }
    #[inline]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

impl<const P: char, const N: u8> InputPin for Pin<P, N, Output<OpenDrain>> {
    type Error = Infallible;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

/// Opaque AFR register
pub struct Afr<const P: char, const H: bool>(());

/// Represents high or low configuration register
pub trait HL {
    /// Configuration register associated to pin
    type Afr;
}

macro_rules! cr {
    ($high:literal: [$($i:literal),+]) => {
        $(
            impl<const P: char, MODE> HL for Pin<P, $i, MODE> {
                type Afr = Afr<P, $high>;
            }
        )+
    }
}

cr!(false: [0, 1, 2, 3, 4, 5, 6, 7]);
cr!(true: [8, 9, 10, 11, 12, 13, 14, 15]);

gpio!(GPIOA, gpioa, PAx, 'A', [
    PA0: (pa0, 0, Analog),
    PA1: (pa1, 1, Analog),
    PA2: (pa2, 2, Analog),
    PA3: (pa3, 3, Analog),
    PA4: (pa4, 4, Analog),
    PA5: (pa5, 5, Analog),
    PA6: (pa6, 6, Analog),
    PA7: (pa7, 7, Analog),
    PA8: (pa8, 8, Analog),
    PA9: (pa9, 9, Analog),
    PA10: (pa10, 10, Analog),
    PA11: (pa11, 11, Analog),
    PA12: (pa12, 12, Analog),
    PA13: (pa13, 13, super::Debugger), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, super::Debugger), // SWCLK, PullDown
    PA15: (pa15, 15, super::Debugger), // JTDI, PullUp
]);

gpio!(GPIOB, gpiob, PBx, 'B', [
    PB0: (pb0, 0, Analog),
    PB1: (pb1, 1, Analog),
    PB2: (pb2, 2, Analog),
    PB3: (pb3, 3, super::Debugger), // SWO
    PB4: (pb4, 4, super::Debugger), // JTRST, PullUp
    PB5: (pb5, 5, Analog),
    PB6: (pb6, 6, Analog),
    PB7: (pb7, 7, Analog),
    PB8: (pb8, 8, Analog),
    PB9: (pb9, 9, Analog),
    PB10: (pb10, 10, Analog),
    PB11: (pb11, 11, Analog),
    PB12: (pb12, 12, Analog),
    PB13: (pb13, 13, Analog),
    PB14: (pb14, 14, Analog),
    PB15: (pb15, 15, Analog),
]);

gpio!(GPIOC, gpioc, PCx, 'C', [
    PC0: (pc0, 0, Analog),
    PC1: (pc1, 1, Analog),
    PC2: (pc2, 2, Analog),
    PC3: (pc3, 3, Analog),
    PC4: (pc4, 4, Analog),
    PC5: (pc5, 5, Analog),
    PC6: (pc6, 6, Analog),
    PC7: (pc7, 7, Analog),
    PC8: (pc8, 8, Analog),
    PC9: (pc9, 9, Analog),
    PC10: (pc10, 10, Analog),
    PC11: (pc11, 11, Analog),
    PC12: (pc12, 12, Analog),
    PC13: (pc13, 13, Analog),
    PC14: (pc14, 14, Analog),
    PC15: (pc15, 15, Analog),
]);

gpio!(GPIOD, gpiod, PDx, 'D', [
    PD0: (pd0, 0, Analog),
    PD1: (pd1, 1, Analog),
    PD2: (pd2, 2, Analog),
    PD3: (pd3, 3, Analog),
    PD4: (pd4, 4, Analog),
    PD5: (pd5, 5, Analog),
    PD6: (pd6, 6, Analog),
    PD7: (pd7, 7, Analog),
    PD8: (pd8, 8, Analog),
    PD9: (pd9, 9, Analog),
    PD10: (pd10, 10, Analog),
    PD11: (pd11, 11, Analog),
    PD12: (pd12, 12, Analog),
    PD13: (pd13, 13, Analog),
    PD14: (pd14, 14, Analog),
    PD15: (pd15, 15, Analog),
]);

gpio!(GPIOE, gpioe, PEx, 'E', [
    PE0: (pe0, 0, Analog),
    PE1: (pe1, 1, Analog),
    PE2: (pe2, 2, Analog),
    PE3: (pe3, 3, Analog),
    PE4: (pe4, 4, Analog),
    PE5: (pe5, 5, Analog),
    PE6: (pe6, 6, Analog),
    PE7: (pe7, 7, Analog),
    PE8: (pe8, 8, Analog),
    PE9: (pe9, 9, Analog),
    PE10: (pe10, 10, Analog),
    PE11: (pe11, 11, Analog),
    PE12: (pe12, 12, Analog),
    PE13: (pe13, 13, Analog),
    PE14: (pe14, 14, Analog),
    PE15: (pe15, 15, Analog),
]);

#[cfg(any(
    // feature = "stm32l471",  // missing PAC support for Port G
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    // feature = "stm32l4r5",
    // feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
))]
gpio!(GPIOF, gpiof, PFx, 'F', [
    PF0: (pf0, 0, Analog),
    PF1: (pf1, 1, Analog),
    PF2: (pf2, 2, Analog),
    PF3: (pf3, 3, Analog),
    PF4: (pf4, 4, Analog),
    PF5: (pf5, 5, Analog),
    PF6: (pf6, 6, Analog),
    PF7: (pf7, 7, Analog),
    PF8: (pf8, 8, Analog),
    PF9: (pf9, 9, Analog),
    PF10: (pf10, 10, Analog),
    PF11: (pf11, 11, Analog),
    PF12: (pf12, 12, Analog),
    PF13: (pf13, 13, Analog),
    PF14: (pf14, 14, Analog),
    PF15: (pf15, 15, Analog),
]);
#[cfg(any(
    // feature = "stm32l471",  // missing PAC support for Port G
    feature = "stm32l475",
    feature = "stm32l476",
    feature = "stm32l485",
    feature = "stm32l486",
    feature = "stm32l496",
    feature = "stm32l4a6",
    // feature = "stm32l4p5",
    // feature = "stm32l4q5",
    // feature = "stm32l4r5",
    // feature = "stm32l4s5",
    // feature = "stm32l4r7",
    // feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9",
))]
gpio!(GPIOG, gpiog, PGx, 'G',
    { unsafe { (*crate::pac::PWR::ptr()).cr2.modify(|_,w| w.iosv().set_bit()); } },
[
    PG0: (pg0, 0, Analog),
    PG1: (pg1, 1, Analog),
    PG2: (pg2, 2, Analog),
    PG3: (pg3, 3, Analog),
    PG4: (pg4, 4, Analog),
    PG5: (pg5, 5, Analog),
    PG6: (pg6, 6, Analog),
    PG7: (pg7, 7, Analog),
    PG8: (pg8, 8, Analog),
    PG9: (pg9, 9, Analog),
    PG10: (pg10, 10, Analog),
    PG11: (pg11, 11, Analog),
    PG12: (pg12, 12, Analog),
    PG13: (pg13, 13, Analog),
    PG14: (pg14, 14, Analog),
    PG15: (pg15, 15, Analog),
]);

#[cfg(any(
  // feature = "stm32l471",  // missing PAC support for Port H
  feature = "stm32l475",
  feature = "stm32l476",
  feature = "stm32l485",
  feature = "stm32l486",
  feature = "stm32l496",
  feature = "stm32l4a6",
  // feature = "stm32l4p5",
  // feature = "stm32l4q5",
  // feature = "stm32l4r5",
  // feature = "stm32l4s5",
  // feature = "stm32l4r7",
  // feature = "stm32l4s7",
  feature = "stm32l4r9",
  feature = "stm32l4s9",
))]
gpio!(GPIOH, gpioh, PHx, 'H', 7, [
    PH0: (ph0, 0, Analog),
    PH1: (ph1, 1, Analog),
    PH2: (ph2, 2, Analog),
    PH3: (ph3, 3, Analog),
    PH4: (ph4, 4, Analog),
    PH5: (ph5, 5, Analog),
    PH6: (ph6, 6, Analog),
    PH7: (ph7, 7, Analog),
    PH8: (ph8, 8, Analog),
    PH9: (ph9, 9, Analog),
    PH10: (ph10, 10, Analog),
    PH11: (ph11, 11, Analog),
    PH12: (ph12, 12, Analog),
    PH13: (ph13, 13, Analog),
    PH14: (ph14, 14, Analog),
    PH15: (ph15, 15, Analog),
]);

struct Gpio<const P: char>;
impl<const P: char> Gpio<P> {
    const fn ptr() -> *const crate::pac::gpioa::RegisterBlock {
        match P {
            'A' => crate::pac::GPIOA::ptr(),
            'B' => crate::pac::GPIOB::ptr() as _,
            'C' => crate::pac::GPIOC::ptr() as _,
            'D' => crate::pac::GPIOD::ptr() as _,
            'E' => crate::pac::GPIOE::ptr() as _,
            #[cfg(any(
                // feature = "stm32l471",  // missing PAC support for Port F
                feature = "stm32l475",
                feature = "stm32l476",
                feature = "stm32l485",
                feature = "stm32l486",
                feature = "stm32l496",
                feature = "stm32l4a6",
                // feature = "stm32l4p5",
                // feature = "stm32l4q5",
                // feature = "stm32l4r5",
                // feature = "stm32l4s5",
                // feature = "stm32l4r7",
                // feature = "stm32l4s7",
                feature = "stm32l4r9",
                feature = "stm32l4s9",
            ))]
            'F' => crate::pac::GPIOF::ptr() as _,
            #[cfg(any(
                // feature = "stm32l471",  // missing PAC support for Port G
                feature = "stm32l475",
                feature = "stm32l476",
                feature = "stm32l485",
                feature = "stm32l486",
                feature = "stm32l496",
                feature = "stm32l4a6",
                // feature = "stm32l4p5",
                // feature = "stm32l4q5",
                // feature = "stm32l4r5",
                // feature = "stm32l4s5",
                // feature = "stm32l4r7",
                // feature = "stm32l4s7",
                feature = "stm32l4r9",
                feature = "stm32l4s9",
            ))]
            'G' => crate::pac::GPIOG::ptr() as _,
            #[cfg(any(
              // feature = "stm32l471",  // missing PAC support for Port G
              feature = "stm32l475",
              feature = "stm32l476",
              feature = "stm32l485",
              feature = "stm32l486",
              feature = "stm32l496",
              feature = "stm32l4a6",
              // feature = "stm32l4p5",
              // feature = "stm32l4q5",
              // feature = "stm32l4r5",
              // feature = "stm32l4s5",
              // feature = "stm32l4r7",
              // feature = "stm32l4s7",
              feature = "stm32l4r9",
              feature = "stm32l4s9",
            ))]
            'H' => crate::pac::GPIOH::ptr() as _,
            _ => core::ptr::null(),
        }
    }
}
