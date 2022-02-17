//! General Purpose Input / Output

pub use crate::hal::digital::v2::PinState;
use crate::hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};
use core::convert::Infallible;
use core::marker::PhantomData;

use crate::pac::{self, EXTI, SYSCFG};
use crate::rcc::{Enable, AHB2, APB2};

mod convert;

mod partially_erased;
pub use partially_erased::{PEPin, PartiallyErasedPin};
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
pub struct Input<IOCONF> {
    _mode: PhantomData<IOCONF>,
}

/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<OTYPE> {
    _mode: PhantomData<OTYPE>,
}

/// Push pull output (type state)
pub struct PushPull;
/// Open drain output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;

pub type Debugger = Alternate<PushPull, 0>;

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
pub struct Alternate<MODE, const A: u8> {
    _mode: PhantomData<MODE>,
}

#[derive(Debug, PartialEq)]
pub enum Edge {
    Rising,
    Falling,
    RisingFalling,
}

mod sealed {
    /// Marker trait that show if `ExtiPin` can be implemented
    pub trait Interruptable {}
}

use sealed::Interruptable;
impl<MODE> Interruptable for Output<MODE> {}
impl<MODE> Interruptable for Input<MODE> {}

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG, apb2: &mut APB2);
    fn trigger_on_edge(&mut self, exti: &mut EXTI, level: Edge);
    fn enable_interrupt(&mut self, exti: &mut EXTI);
    fn disable_interrupt(&mut self, exti: &mut EXTI);
    fn clear_interrupt_pending_bit(&mut self);
    fn check_interrupt(&self) -> bool;
}

impl<PIN> ExtiPin for PIN
where
    PIN: PinExt,
    PIN::Mode: Interruptable,
{
    /// Make corresponding EXTI line sensitive to this pin
    #[inline(always)]
    fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG, apb2: &mut APB2) {
        // SYSCFG clock must be enabled in order to do register writes
        SYSCFG::enable(apb2);

        let i = self.pin_id();
        let port = self.port_id() as u32;
        let offset = 4 * (i % 4);
        match i {
            0..=3 => {
                syscfg.exticr1.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            4..=7 => {
                syscfg.exticr2.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            8..=11 => {
                syscfg.exticr3.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            12..=15 => {
                syscfg.exticr4.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            _ => unreachable!(),
        }
    }

    /// Generate interrupt on rising edge, falling edge or both
    #[inline(always)]
    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
        let i = self.pin_id();
        match edge {
            Edge::Rising => {
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::Falling => {
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::RisingFalling => {
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
            }
        }
    }

    /// Enable external interrupts from this pin.
    #[inline(always)]
    fn enable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr1
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
    }

    /// Disable external interrupts from this pin
    #[inline(always)]
    fn disable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr1
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
    }

    /// Clear the interrupt pending bit for this pin
    #[inline(always)]
    fn clear_interrupt_pending_bit(&mut self) {
        unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << self.pin_id())) };
    }

    /// Reads the interrupt pending bit for this pin
    #[inline(always)]
    fn check_interrupt(&self) -> bool {
        unsafe { ((*EXTI::ptr()).pr1.read().bits() & (1 << self.pin_id())) != 0 }
    }
}

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
    ($GPIOX:ident, $gpiox:ident, $PXx:ident, $port_id:literal, $extigpionr:expr, $({ $pwrenable:expr },)? [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $HL:ident, $exticri:ident),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use crate::stm32::$GPIOX;

            use crate::rcc::{AHB2, Enable, Reset};
            use super::{Afr, Analog, GpioExt, Pin, H8, L8, MODER, OTYPER, OSPEEDR, PUPDR};

            /// GPIO parts
            pub struct Parts {
                /// Opaque AFRH register
                pub afrh: Afr<H8, $port_id>,
                /// Opaque AFRL register
                pub afrl: Afr<L8, $port_id>,
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
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            $(
                pub type $PXi<MODE> = Pin<MODE, $HL, $port_id, $i>;
            )+

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, ahb: &mut AHB2) -> Parts {
                    <$GPIOX>::enable(ahb);
                    <$GPIOX>::reset(ahb);
                    $($pwrenable)?

                    Parts {
                        afrh: Afr::new(),
                        afrl: Afr::new(),
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
/// - `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
/// - `HL` represents high 8 or low 8 pin.
/// - `P` is port name: `A` for GPIOA, `B` for GPIOB, etc.
/// - `N` is pin number: from `0` to `15`.
pub struct Pin<MODE, HL, const P: char, const N: u8> {
    _mode: PhantomData<(MODE, HL)>,
}

impl<MODE, HL, const P: char, const N: u8> Pin<MODE, HL, P, N> {
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }
}

impl<MODE, HL, const P: char, const N: u8> PinExt for Pin<MODE, HL, P, N> {
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

impl<MODE, HL, const P: char, const N: u8> Pin<Output<MODE>, HL, P, N> {
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

impl<HL, const P: char, const N: u8> Pin<Output<OpenDrain>, HL, P, N> {
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

impl<MODE, HL, const P: char, const N: u8, const A: u8> Pin<Alternate<MODE, A>, HL, P, N> {
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

impl<HL, const P: char, const N: u8, const A: u8> Pin<Alternate<PushPull, A>, HL, P, N> {
    /// Turns pin alternate configuration pin into open drain
    pub fn set_open_drain(self) -> Pin<Alternate<OpenDrain, A>, HL, P, N> {
        let offset = { N };
        unsafe {
            (*Gpio::<P>::ptr())
                .otyper
                .modify(|r, w| w.bits(r.bits() | (1 << offset)))
        };

        Pin::new()
    }
}

impl<MODE, HL, const P: char, const N: u8> Pin<MODE, HL, P, N> {
    /// Erases the pin number from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase_number(self) -> PEPin<MODE, P> {
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
impl<MODE, HL, const P: char, const N: u8> Pin<MODE, HL, P, N> {
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

impl<MODE, HL, const P: char, const N: u8> Pin<Output<MODE>, HL, P, N> {
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

impl<MODE, HL, const P: char, const N: u8> OutputPin for Pin<Output<MODE>, HL, P, N> {
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

impl<MODE, HL, const P: char, const N: u8> StatefulOutputPin for Pin<Output<MODE>, HL, P, N> {
    #[inline]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }
    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<MODE, HL, const P: char, const N: u8> ToggleableOutputPin for Pin<Output<MODE>, HL, P, N> {
    type Error = Infallible;

    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<MODE, HL, const P: char, const N: u8> Pin<Input<MODE>, HL, P, N> {
    #[inline]
    pub fn is_high(&self) -> bool {
        !self._is_low()
    }
    #[inline]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

impl<MODE, HL, const P: char, const N: u8> InputPin for Pin<Input<MODE>, HL, P, N> {
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

impl<HL, const P: char, const N: u8> Pin<Output<OpenDrain>, HL, P, N> {
    #[inline]
    pub fn is_high(&self) -> bool {
        !self._is_low()
    }
    #[inline]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

impl<HL, const P: char, const N: u8> InputPin for Pin<Output<OpenDrain>, HL, P, N> {
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
pub struct Afr<HL, const P: char> {
    _afr: PhantomData<HL>,
}

impl<HL, const P: char> Afr<HL, P> {
    pub(crate) fn new() -> Self {
        Self { _afr: PhantomData }
    }
}

macro_rules! af {
    ($HL:ident, $AFR:ident, $afr:ident) => {
        #[doc(hidden)]
        pub struct $HL {
            _0: (),
        }

        impl<const P: char> Afr<$HL, P> {
            #[allow(dead_code)]
            pub(crate) fn afr(&mut self) -> &pac::gpioa::$AFR {
                unsafe { &(*Gpio::<P>::ptr()).$afr }
            }
        }
    };
}

af!(H8, AFRH, afrh);
af!(L8, AFRL, afrl);

gpio!(GPIOA, gpioa, PAx, 'A', 0, [
    PA0: (pa0, 0, Analog, L8, exticr1),
    PA1: (pa1, 1, Analog, L8, exticr1),
    PA2: (pa2, 2, Analog, L8, exticr1),
    PA3: (pa3, 3, Analog, L8, exticr1),
    PA4: (pa4, 4, Analog, L8, exticr2),
    PA5: (pa5, 5, Analog, L8, exticr2),
    PA6: (pa6, 6, Analog, L8, exticr2),
    PA7: (pa7, 7, Analog, L8, exticr2),
    PA8: (pa8, 8, Analog, H8, exticr3),
    PA9: (pa9, 9, Analog, H8, exticr3),
    PA10: (pa10, 10, Analog, H8, exticr3),
    PA11: (pa11, 11, Analog, H8, exticr3),
    PA12: (pa12, 12, Analog, H8, exticr4),
    PA13: (pa13, 13, super::Debugger, H8, exticr4), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, super::Debugger, H8, exticr4), // SWCLK, PullDown
    PA15: (pa15, 15, super::Debugger, H8, exticr4), // JTDI, PullUp
]);

gpio!(GPIOB, gpiob, PBx, 'B', 1, [
    PB0: (pb0, 0, Analog, L8, exticr1),
    PB1: (pb1, 1, Analog, L8, exticr1),
    PB2: (pb2, 2, Analog, L8, exticr1),
    PB3: (pb3, 3, super::Debugger, L8, exticr1), // SWO
    PB4: (pb4, 4, super::Debugger, L8, exticr2), // JTRST, PullUp
    PB5: (pb5, 5, Analog, L8, exticr2),
    PB6: (pb6, 6, Analog, L8, exticr2),
    PB7: (pb7, 7, Analog, L8, exticr2),
    PB8: (pb8, 8, Analog, H8, exticr3),
    PB9: (pb9, 9, Analog, H8, exticr3),
    PB10: (pb10, 10, Analog, H8, exticr3),
    PB11: (pb11, 11, Analog, H8, exticr3),
    PB12: (pb12, 12, Analog, H8, exticr4),
    PB13: (pb13, 13, Analog, H8, exticr4),
    PB14: (pb14, 14, Analog, H8, exticr4),
    PB15: (pb15, 15, Analog, H8, exticr4),
]);

gpio!(GPIOC, gpioc, PCx, 'C', 2, [
    PC0: (pc0, 0, Analog, L8, exticr1),
    PC1: (pc1, 1, Analog, L8, exticr1),
    PC2: (pc2, 2, Analog, L8, exticr1),
    PC3: (pc3, 3, Analog, L8, exticr1),
    PC4: (pc4, 4, Analog, L8, exticr2),
    PC5: (pc5, 5, Analog, L8, exticr2),
    PC6: (pc6, 6, Analog, L8, exticr2),
    PC7: (pc7, 7, Analog, L8, exticr2),
    PC8: (pc8, 8, Analog, H8, exticr3),
    PC9: (pc9, 9, Analog, H8, exticr3),
    PC10: (pc10, 10, Analog, H8, exticr3),
    PC11: (pc11, 11, Analog, H8, exticr3),
    PC12: (pc12, 12, Analog, H8, exticr4),
    PC13: (pc13, 13, Analog, H8, exticr4),
    PC14: (pc14, 14, Analog, H8, exticr4),
    PC15: (pc15, 15, Analog, H8, exticr4),
]);

gpio!(GPIOD, gpiod, PDx, 'D', 3, [
    PD0: (pd0, 0, Analog, L8, exticr1),
    PD1: (pd1, 1, Analog, L8, exticr1),
    PD2: (pd2, 2, Analog, L8, exticr1),
    PD3: (pd3, 3, Analog, L8, exticr1),
    PD4: (pd4, 4, Analog, L8, exticr2),
    PD5: (pd5, 5, Analog, L8, exticr2),
    PD6: (pd6, 6, Analog, L8, exticr2),
    PD7: (pd7, 7, Analog, L8, exticr2),
    PD8: (pd8, 8, Analog, H8, exticr3),
    PD9: (pd9, 9, Analog, H8, exticr3),
    PD10: (pd10, 10, Analog, H8, exticr3),
    PD11: (pd11, 11, Analog, H8, exticr3),
    PD12: (pd12, 12, Analog, H8, exticr4),
    PD13: (pd13, 13, Analog, H8, exticr4),
    PD14: (pd14, 14, Analog, H8, exticr4),
    PD15: (pd15, 15, Analog, H8, exticr4),
]);

gpio!(GPIOE, gpioe, PEx, 'E', 4, [
    PE0: (pe0, 0, Analog, L8, exticr1),
    PE1: (pe1, 1, Analog, L8, exticr1),
    PE2: (pe2, 2, Analog, L8, exticr1),
    PE3: (pe3, 3, Analog, L8, exticr1),
    PE4: (pe4, 4, Analog, L8, exticr2),
    PE5: (pe5, 5, Analog, L8, exticr2),
    PE6: (pe6, 6, Analog, L8, exticr2),
    PE7: (pe7, 7, Analog, L8, exticr2),
    PE8: (pe8, 8, Analog, H8, exticr3),
    PE9: (pe9, 9, Analog, H8, exticr3),
    PE10: (pe10, 10, Analog, H8, exticr3),
    PE11: (pe11, 11, Analog, H8, exticr3),
    PE12: (pe12, 12, Analog, H8, exticr4),
    PE13: (pe13, 13, Analog, H8, exticr4),
    PE14: (pe14, 14, Analog, H8, exticr4),
    PE15: (pe15, 15, Analog, H8, exticr4),
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
gpio!(GPIOF, gpiof, PFx, 'F', 5, [
    PF0: (pf0, 0, Analog, L8, exticr1),
    PF1: (pf1, 1, Analog, L8, exticr1),
    PF2: (pf2, 2, Analog, L8, exticr1),
    PF3: (pf3, 3, Analog, L8, exticr1),
    PF4: (pf4, 4, Analog, L8, exticr2),
    PF5: (pf5, 5, Analog, L8, exticr2),
    PF6: (pf6, 6, Analog, L8, exticr2),
    PF7: (pf7, 7, Analog, L8, exticr2),
    PF8: (pf8, 8, Analog, H8, exticr3),
    PF9: (pf9, 9, Analog, H8, exticr3),
    PF10: (pf10, 10, Analog, H8, exticr3),
    PF11: (pf11, 11, Analog, H8, exticr3),
    PF12: (pf12, 12, Analog, H8, exticr4),
    PF13: (pf13, 13, Analog, H8, exticr4),
    PF14: (pf14, 14, Analog, H8, exticr4),
    PF15: (pf15, 15, Analog, H8, exticr4),
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
gpio!(GPIOG, gpiog, PGx, 'G', 6,
    { unsafe { (*crate::pac::PWR::ptr()).cr2.modify(|_,w| w.iosv().set_bit()); } },
[
    PG0: (pg0, 0, Analog, L8, exticr1),
    PG1: (pg1, 1, Analog, L8, exticr1),
    PG2: (pg2, 2, Analog, L8, exticr1),
    PG3: (pg3, 3, Analog, L8, exticr1),
    PG4: (pg4, 4, Analog, L8, exticr2),
    PG5: (pg5, 5, Analog, L8, exticr2),
    PG6: (pg6, 6, Analog, L8, exticr2),
    PG7: (pg7, 7, Analog, L8, exticr2),
    PG8: (pg8, 8, Analog, H8, exticr3),
    PG9: (pg9, 9, Analog, H8, exticr3),
    PG10: (pg10, 10, Analog, H8, exticr3),
    PG11: (pg11, 11, Analog, H8, exticr3),
    PG12: (pg12, 12, Analog, H8, exticr4),
    PG13: (pg13, 13, Analog, H8, exticr4),
    PG14: (pg14, 14, Analog, H8, exticr4),
    PG15: (pg15, 15, Analog, H8, exticr4),
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
            _ => crate::pac::GPIOA::ptr(),
        }
    }
}
