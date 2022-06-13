//! # General Purpose Input / Output
//!
//! To use the GPIO pins, you first need to configure the GPIO port (GPIOA, GPIOB, ...) that you
//! are interested in. This is done using the [`GpioExt::split`] function.
//!
//! ```
//! let dp = pac::Peripherals::take().unwrap();
//! let rcc = dp.RCC.constrain();
//!
//! let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
//! ```
//!
//! The resulting [Parts](gpioa::Parts) struct contains one field for each pin, as well as some
//! shared registers. Every pin type is a specialized version of generic [pin](Pin) struct.
//!
//! To use a pin, first use the relevant `into_...` method of the [pin](Pin).
//!
//! ```rust
//! let pa0 = gpioa.pa0.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
//! ```
//!
//! And finally, you can use the functions from the [InputPin] or [OutputPin] traits in
//! `embedded_hal`
//!
//! For a complete example, see [examples/toggle.rs]
//!
//! ## Pin Configuration
//!
//! ### Mode
//!
//! Each GPIO pin can be set to various modes by corresponding `into_...` method:
//!
//! - **Input**: The output buffer is disabled and the schmitt trigger input is activated
//! - **Output**: Both the output buffer and the schmitt trigger input is enabled
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it to ground in drain
//!     mode. Can be used as an input in the `open` configuration
//! - **Alternate**: Pin mode required when the pin is driven by other peripherals. The schmitt
//! trigger input is activated. The Output buffer is automatically enabled and disabled by
//! peripherals. Output behavior is same as the output mode
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it to ground in drain
//!     mode
//! - **Analog**: Pin mode required for ADC, DAC, OPAMP, and COMP peripherals. It is also suitable
//! for minimize energy consumption as the output buffer and the schmitt trigger input is disabled
//!
//! ## Changing modes
//! The simplest way to change the pin mode is to use the `into_<mode>` functions. These return a
//! new struct with the correct mode that you can use the input or output functions on.
//!
//! ### Output Speed
//!
//! Output speed (slew rate) for each pin is selectable from low, medium, and high by calling
//! [`set_speed`](Pin::set_speed) method. Refer to the device datasheet for specifications for each
//!  speed.
//!
//! ### Internal Resistor
//!
//! Weak internal pull-up and pull-down resistors for each pin is configurable by calling
//! [`set_internal_resistor`](Pin::set_internal_resistor) method. `into_..._input` methods are also
//! available for convenience.
//!
//! [InputPin]: embedded_hal::digital::v2::InputPin
//! [OutputPin]: embedded_hal::digital::v2::OutputPin
//! [examples/toggle.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.0/examples/toggle.rs
//!
//! If you need a more temporary mode change, and can not use the `into_<mode>` functions for
//! ownership reasons, you can use the closure based `with_<mode>` functions to temporarily change the pin type, do
//! some output or input, and then have it change back once done.
//!
//! ### Dynamic Mode Change
//! The above mode change methods guarantee that you can only call input functions when the pin is
//! in input mode, and output when in output modes, but can lead to some issues. Therefore, there
//! is also a mode where the state is kept track of at runtime, allowing you to change the mode
//! often, and without problems with ownership, or references, at the cost of some performance and
//! the risk of runtime errors.
//!
//! To make a pin dynamic, use the `into_dynamic` function, and then use the `make_<mode>` functions to
//! change the mode

use core::marker::PhantomData;

use crate::rcc::AHB2;

mod convert;
pub use convert::PinMode;
mod partially_erased;
pub use partially_erased::PartiallyErasedPin;
mod erased;
pub use erased::ErasedPin;
mod exti;
pub use exti::ExtiPin;
mod dynamic;
pub use dynamic::{Dynamic, DynamicPin};
mod hal_02;

pub use embedded_hal::digital::v2::PinState;

use core::fmt;

/// A filler pin type
#[derive(Debug)]
pub struct NoPin;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, ahb: &mut AHB2) -> Self::Parts;
}

/// Id, port and mode for any pin
pub trait PinExt {
    /// Current pin mode
    type Mode;
    /// Pin number
    fn pin_id(&self) -> u8;
    /// Port number starting from 0
    fn port_id(&self) -> u8;
}

/// Some alternate mode (type state)
pub struct Alternate<const A: u8, Otype = PushPull>(PhantomData<Otype>);

/// Input mode (type state)
pub struct Input;

/// Pull setting for an input.
#[derive(Debug, Eq, PartialEq)]
pub enum Pull {
    /// Floating
    None = 0,
    /// Pulled up
    Up = 1,
    /// Pulled down
    Down = 2,
}

/// Open drain input or output (type state)
pub struct OpenDrain;

/// Output mode (type state)
pub struct Output<MODE = PushPull> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;

/// Analog mode (type state)
pub struct Analog;

/// JTAG/SWD mote (type state)
pub type Debugger = Alternate<0, PushPull>;

mod marker {
    /// Marker trait that show if `ExtiPin` can be implemented
    pub trait Interruptable {}
    /// Marker trait for readable pin modes
    pub trait Readable {}
    /// Marker trait for slew rate configurable pin modes
    pub trait OutputSpeed {}
    /// Marker trait for active pin modes
    pub trait Active {}
    /// Marker trait for pins with alternate function `A` mapping
    pub trait IntoAf<const A: u8>: super::HL {}
}

impl<MODE> marker::Interruptable for Output<MODE> {}
impl marker::Interruptable for Input {}
impl marker::Readable for Input {}
impl marker::Readable for Output<OpenDrain> {}
impl marker::Active for Input {}
impl<Otype> marker::OutputSpeed for Output<Otype> {}
impl<const A: u8, Otype> marker::OutputSpeed for Alternate<A, Otype> {}
impl<Otype> marker::Active for Output<Otype> {}
impl<const A: u8, Otype> marker::Active for Alternate<A, Otype> {}

/// GPIO Pin speed selection
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Speed {
    /// Low speed
    Low = 0,
    /// Medium speed
    Medium = 1,
    /// High speed
    High = 2,
    /// Very high speed
    VeryHigh = 3,
}

/// GPIO interrupt trigger edge selection
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Edge {
    /// Rising edge of voltage
    Rising,
    /// Falling edge of voltage
    Falling,
    /// Rising and falling edge of voltage
    RisingFalling,
}

/// Opaque MODER register
pub struct MODER<const P: char>(());

/// Opaque OTYPER register
pub struct OTYPER<const P: char>(());

/// Opaque OSPEEDR register
pub struct OSPEEDR<const P: char>(());

/// Opaque PUPDR register
pub struct PUPDR<const P: char>(());

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

macro_rules! af {
    ($($i:literal: $AFi:ident),+) => {
        $(
            #[doc = concat!("Alternate function ", $i, " (type state)" )]
            pub type $AFi<Otype = PushPull> = Alternate<$i, Otype>;
        )+
    };
}

af!(
    0: AF0,
    1: AF1,
    2: AF2,
    3: AF3,
    4: AF4,
    5: AF5,
    6: AF6,
    7: AF7,
    8: AF8,
    9: AF9,
    10: AF10,
    11: AF11,
    12: AF12,
    13: AF13,
    14: AF14,
    15: AF15
);

/// Generic pin type
///
/// - `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
/// - `P` is port name: `A` for GPIOA, `B` for GPIOB, etc.
/// - `N` is pin number: from `0` to `15`.
pub struct Pin<const P: char, const N: u8, MODE = Analog> {
    _mode: PhantomData<MODE>,
}
impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }
}

impl<const P: char, const N: u8, MODE> fmt::Debug for Pin<P, N, MODE> {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_fmt(format_args!(
            "P{}{}<{}>",
            P,
            N,
            crate::stripped_type_name::<MODE>()
        ))
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

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: marker::OutputSpeed,
{
    /// Set pin speed
    // TODO: add OSPEEDR<P>
    pub fn set_speed(&mut self, speed: Speed) {
        let offset = 2 * { N };

        unsafe {
            (*Gpio::<P>::ptr())
                .ospeedr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset)));
        }
    }

    /// Set pin speed
    // TODO: add OSPEEDR<P>
    pub fn speed(mut self, speed: Speed) -> Self {
        self.set_speed(speed);
        self
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: marker::Active,
{
    /// Set the internal pull-up and pull-down resistor
    pub fn set_internal_resistor(&mut self, _pupdr: &mut PUPDR<P>, resistor: Pull) {
        let offset = 2 * { N };
        let value = resistor as u32;
        unsafe {
            (*Gpio::<P>::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)))
        };
    }

    /// Set the internal pull-up and pull-down resistor
    pub fn internal_resistor(mut self, pupdr: &mut PUPDR<P>, resistor: Pull) -> Self {
        self.set_internal_resistor(pupdr, resistor);
        self
    }

    /// Enables / disables the internal pull up
    pub fn internal_pull_up(self, pupdr: &mut PUPDR<P>, on: bool) -> Self {
        if on {
            self.internal_resistor(pupdr, Pull::Up)
        } else {
            self.internal_resistor(pupdr, Pull::None)
        }
    }

    /// Enables / disables the internal pull down
    pub fn internal_pull_down(self, pupdr: &mut PUPDR<P>, on: bool) -> Self {
        if on {
            self.internal_resistor(pupdr, Pull::Down)
        } else {
            self.internal_resistor(pupdr, Pull::None)
        }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// Erases the pin number from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase_number(self) -> PartiallyErasedPin<P, MODE> {
        PartiallyErasedPin::new(N)
    }

    /// Erases the pin number and the port from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase(self) -> ErasedPin<MODE> {
        ErasedPin::new(P as u8 - b'A', N)
    }
}

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
    /// Drives the pin high
    #[inline(always)]
    pub fn set_high(&mut self) {
        self._set_high()
    }

    /// Drives the pin low
    #[inline(always)]
    pub fn set_low(&mut self) {
        self._set_low()
    }

    /// Is the pin in drive high or low mode?
    #[inline(always)]
    pub fn get_state(&self) -> PinState {
        if self.is_set_low() {
            PinState::Low
        } else {
            PinState::High
        }
    }

    /// Drives the pin high or low depending on the provided value
    #[inline(always)]
    pub fn set_state(&mut self, state: PinState) {
        match state {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        }
    }

    /// Is the pin in drive high mode?
    #[inline(always)]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    /// Is the pin in drive low mode?
    #[inline(always)]
    pub fn is_set_low(&self) -> bool {
        self._is_set_low()
    }

    /// Toggle pin output
    #[inline(always)]
    pub fn toggle(&mut self) {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: marker::Readable,
{
    /// Is the input pin high?
    #[inline(always)]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    /// Is the input pin low?
    #[inline(always)]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PEPin:ident, $port_id:expr, $PXn:ident, $({ $pwrenable:expr },)? [
        $($PXi:ident: ($pxi:ident, $i:expr, [$($A:literal),*] $(, $MODE:ty)?),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use crate::pac::$GPIOX;
            use crate::rcc::{Enable, Reset, AHB2};
            use super::{Afr, MODER, OTYPER, OSPEEDR, PUPDR};

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

            impl super::GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, ahb: &mut AHB2) -> Parts {
                    <$GPIOX>::enable(ahb);
                    <$GPIOX>::reset(ahb);
                    $($pwrenable)?

                    Parts {
                        afrh: Afr(()),
                        afrl: Afr(()),
                        moder: MODER(()),
                        otyper: OTYPER(()),
                        ospeedr: OSPEEDR(()),
                        pupdr: PUPDR(()),
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }
            }

            #[doc="Common type for "]
            #[doc=stringify!($GPIOX)]
            #[doc=" related pins"]
            pub type $PXn<MODE> = super::PartiallyErasedPin<$port_id, MODE>;

            $(
                #[doc=stringify!($PXi)]
                #[doc=" pin"]
                pub type $PXi<MODE = super::Analog> = super::Pin<$port_id, $i, MODE>;

                $(
                    impl<MODE> super::marker::IntoAf<$A> for $PXi<MODE> { }
                )*
            )+

        }

        pub use $gpiox::{ $($PXi,)+ };
    }
}

#[cfg(feature = "gpio-l41x")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 7, 12, 14, 15]),
    PA1: (pa1, 1, [1, 4, 5, 7, 14, 15]),
    PA2: (pa2, 2, [1, 7, 8, 10, 14, 15]),
    PA3: (pa3, 3, [1, 7, 8, 10, 14, 15]),
    PA4: (pa4, 4, [5, 7, 14, 15]),
    PA5: (pa5, 5, [1, 2, 5, 14, 15]),
    PA6: (pa6, 6, [1, 5, 6, 7, 8, 10, 14, 15]),
    PA7: (pa7, 7, [1, 4, 5, 10, 15]),
    PA8: (pa8, 8, [0, 1, 7, 14, 15]),
    PA9: (pa9, 9, [1, 4, 7, 14, 15]),
    PA10: (pa10, 10, [1, 4, 7, 10, 15]),
    PA11: (pa11, 11, [1, 2, 5, 6, 7, 10, 12, 15]),
    PA12: (pa12, 12, [1, 5, 7, 10, 15]),
    PA13: (pa13, 13, [0, 1, 10, 15], super::Debugger),
    PA14: (pa14, 14, [0, 1, 4, 15], super::Debugger),
    PA15: (pa15, 15, [0, 1, 2, 3, 5, 7, 9, 15], super::Debugger),
]);

#[cfg(feature = "gpio-l41x")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [0, 1, 5, 7, 10, 12, 15]),
    PB1: (pb1, 1, [0, 1, 7, 8, 10, 14, 15]),
    PB2: (pb2, 2, [0, 1, 4, 15]),
    PB3: (pb3, 3, [0, 1, 5, 7, 15], super::Debugger),
    PB4: (pb4, 4, [0, 4, 5, 7, 9, 15], super::Debugger),
    PB5: (pb5, 5, [0, 1, 4, 5, 7, 9, 14, 15]),
    PB6: (pb6, 6, [0, 1, 4, 7, 9, 14, 15]),
    PB7: (pb7, 7, [0, 1, 4, 7, 9, 15]),
    PB8: (pb8, 8, [4, 14, 15]),
    PB9: (pb9, 9, [1, 4, 5, 15]),
    PB10: (pb10, 10, [1, 4, 5, 7, 8, 9, 10, 12, 15]),
    PB11: (pb11, 11, [1, 4, 7, 8, 10, 15]),
    PB12: (pb12, 12, [1, 4, 5, 7, 8, 9, 14, 15]),
    PB13: (pb13, 13, [1, 4, 5, 7, 8, 9, 14, 15]),
    PB14: (pb14, 14, [1, 4, 5, 7, 9, 14, 15]),
    PB15: (pb15, 15, [0, 1, 5, 9, 14, 15]),
]);

#[cfg(feature = "gpio-l41x")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [0, 1, 4, 8, 14, 15]),
    PC1: (pc1, 1, [0, 1, 4, 8, 15]),
    PC2: (pc2, 2, [1, 5, 15]),
    PC3: (pc3, 3, [1, 5, 14, 15]),
    PC4: (pc4, 4, [7, 15]),
    PC5: (pc5, 5, [7, 15]),
    PC6: (pc6, 6, [9, 15]),
    PC7: (pc7, 7, [9, 15]),
    PC8: (pc8, 8, [9, 15]),
    PC9: (pc9, 9, [9, 10, 15]),
    PC10: (pc10, 10, [0, 7, 9, 15]),
    PC11: (pc11, 11, [7, 9, 15]),
    PC12: (pc12, 12, [0, 7, 9, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-l41x")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD2: (pd2, 2, [0, 7, 9, 15]),
]);

#[cfg(feature = "gpio-l41x")]
gpio!(GPIOH, gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
    PH3: (ph3, 3, [15]),
]);

#[cfg(feature = "gpio-l43x")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 7, 12, 13, 14, 15]),
    PA1: (pa1, 1, [1, 4, 5, 7, 11, 14, 15]),
    PA2: (pa2, 2, [1, 7, 8, 10, 11, 12, 14, 15]),
    PA3: (pa3, 3, [1, 7, 8, 10, 11, 13, 14, 15]),
    PA4: (pa4, 4, [5, 6, 7, 13, 14, 15]),
    PA5: (pa5, 5, [1, 2, 5, 14, 15]),
    PA6: (pa6, 6, [1, 5, 6, 7, 8, 10, 11, 12, 14, 15]),
    PA7: (pa7, 7, [1, 4, 5, 10, 11, 12, 15]),
    PA8: (pa8, 8, [0, 1, 7, 11, 12, 13, 14, 15]),
    PA9: (pa9, 9, [1, 4, 7, 11, 13, 14, 15]),
    PA10: (pa10, 10, [1, 4, 7, 10, 11, 13, 15]),
    PA11: (pa11, 11, [1, 2, 5, 6, 7, 9, 10, 12, 15]),
    PA12: (pa12, 12, [1, 5, 7, 9, 10, 15]),
    PA13: (pa13, 13, [0, 1, 10, 12, 13, 15], super::Debugger),
    PA14: (pa14, 14, [0, 1, 4, 12, 13, 15], super::Debugger),
    PA15: (pa15, 15, [0, 1, 2, 3, 5, 6, 7, 9, 11, 12, 15], super::Debugger),
]);

#[cfg(feature = "gpio-l43x")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [1, 5, 7, 10, 11, 12, 13, 15]),
    PB1: (pb1, 1, [1, 7, 8, 10, 11, 14, 15]),
    PB2: (pb2, 2, [0, 1, 4, 11, 15]),
    PB3: (pb3, 3, [0, 1, 5, 6, 7, 11, 13, 15], super::Debugger),
    PB4: (pb4, 4, [0, 4, 5, 6, 7, 9, 11, 13, 15], super::Debugger),
    PB5: (pb5, 5, [1, 4, 5, 6, 7, 9, 11, 12, 13, 14, 15]),
    PB6: (pb6, 6, [1, 4, 7, 9, 13, 14, 15]),
    PB7: (pb7, 7, [1, 4, 7, 9, 11, 15]),
    PB8: (pb8, 8, [4, 9, 11, 12, 13, 14, 15]),
    PB9: (pb9, 9, [1, 4, 5, 9, 11, 12, 13, 15]),
    PB10: (pb10, 10, [1, 4, 5, 7, 8, 9, 10, 11, 12, 13, 15]),
    PB11: (pb11, 11, [1, 4, 7, 8, 10, 11, 12, 15]),
    PB12: (pb12, 12, [1, 3, 4, 5, 7, 8, 9, 11, 12, 13, 14, 15]),
    PB13: (pb13, 13, [1, 4, 5, 7, 8, 9, 11, 12, 13, 14, 15]),
    PB14: (pb14, 14, [1, 4, 5, 7, 9, 11, 12, 13, 14, 15]),
    PB15: (pb15, 15, [0, 1, 5, 9, 11, 12, 13, 14, 15]),
]);

#[cfg(feature = "gpio-l43x")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 4, 8, 11, 14, 15]),
    PC1: (pc1, 1, [1, 4, 8, 11, 15]),
    PC2: (pc2, 2, [1, 5, 11, 15]),
    PC3: (pc3, 3, [1, 5, 11, 13, 14, 15]),
    PC4: (pc4, 4, [7, 11, 15]),
    PC5: (pc5, 5, [7, 11, 15]),
    PC6: (pc6, 6, [9, 11, 12, 15]),
    PC7: (pc7, 7, [9, 11, 12, 15]),
    PC8: (pc8, 8, [9, 11, 12, 15]),
    PC9: (pc9, 9, [9, 10, 11, 12, 15]),
    PC10: (pc10, 10, [6, 7, 9, 11, 12, 15]),
    PC11: (pc11, 11, [6, 7, 9, 11, 12, 15]),
    PC12: (pc12, 12, [6, 7, 9, 11, 12, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-l43x")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [5, 9, 15]),
    PD1: (pd1, 1, [5, 9, 15]),
    PD2: (pd2, 2, [7, 9, 11, 12, 15]),
    PD3: (pd3, 3, [5, 7, 10, 15]),
    PD4: (pd4, 4, [5, 7, 10, 15]),
    PD5: (pd5, 5, [7, 10, 15]),
    PD6: (pd6, 6, [7, 10, 13, 15]),
    PD7: (pd7, 7, [7, 10, 15]),
    PD8: (pd8, 8, [7, 11, 15]),
    PD9: (pd9, 9, [7, 11, 15]),
    PD10: (pd10, 10, [7, 9, 11, 15]),
    PD11: (pd11, 11, [7, 9, 11, 14, 15]),
    PD12: (pd12, 12, [7, 9, 11, 14, 15]),
    PD13: (pd13, 13, [9, 11, 14, 15]),
    PD14: (pd14, 14, [11, 15]),
    PD15: (pd15, 15, [11, 15]),
]);

#[cfg(feature = "gpio-l43x")]
gpio!(GPIOE, gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [11, 14, 15]),
    PE1: (pe1, 1, [11, 15]),
    PE2: (pe2, 2, [0, 9, 11, 13, 15]),
    PE3: (pe3, 3, [0, 9, 11, 13, 15]),
    PE4: (pe4, 4, [0, 9, 13, 15]),
    PE5: (pe5, 5, [0, 9, 13, 15]),
    PE6: (pe6, 6, [0, 13, 15]),
    PE7: (pe7, 7, [1, 13, 15]),
    PE8: (pe8, 8, [1, 13, 15]),
    PE9: (pe9, 9, [1, 13, 15]),
    PE10: (pe10, 10, [1, 9, 10, 13, 15]),
    PE11: (pe11, 11, [1, 9, 10, 15]),
    PE12: (pe12, 12, [1, 5, 9, 10, 15]),
    PE13: (pe13, 13, [1, 5, 9, 10, 15]),
    PE14: (pe14, 14, [1, 2, 3, 5, 10, 15]),
    PE15: (pe15, 15, [1, 3, 5, 10, 15]),
]);

#[cfg(feature = "gpio-l43x")]
gpio!(GPIOH, gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
    PH3: (ph3, 3, [15]),
]);

#[cfg(feature = "gpio-l45x")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 7, 8, 12, 13, 14, 15]),
    PA1: (pa1, 1, [1, 4, 5, 7, 8, 14, 15]),
    PA2: (pa2, 2, [1, 7, 8, 10, 12, 14, 15]),
    PA3: (pa3, 3, [1, 7, 8, 10, 13, 14, 15]),
    PA4: (pa4, 4, [5, 6, 7, 13, 14, 15]),
    PA5: (pa5, 5, [1, 2, 5, 6, 14, 15]),
    PA6: (pa6, 6, [1, 2, 5, 6, 7, 8, 10, 12, 14, 15]),
    PA7: (pa7, 7, [1, 2, 4, 5, 6, 10, 12, 15]),
    PA8: (pa8, 8, [0, 1, 6, 7, 13, 14, 15]),
    PA9: (pa9, 9, [1, 4, 6, 7, 13, 14, 15]),
    PA10: (pa10, 10, [1, 4, 7, 10, 13, 15]),
    PA11: (pa11, 11, [1, 2, 5, 6, 7, 9, 10, 12, 15]),
    PA12: (pa12, 12, [1, 5, 7, 9, 10, 15]),
    PA13: (pa13, 13, [0, 1, 10, 13, 15], super::Debugger),
    PA14: (pa14, 14, [0, 1, 4, 5, 13, 15], super::Debugger),
    PA15: (pa15, 15, [0, 1, 2, 3, 5, 6, 7, 8, 9, 15], super::Debugger),
]);

#[cfg(feature = "gpio-l45x")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [1, 2, 5, 6, 7, 10, 12, 13, 15]),
    PB1: (pb1, 1, [1, 2, 6, 7, 8, 10, 14, 15]),
    PB2: (pb2, 2, [0, 1, 4, 6, 15]),
    PB3: (pb3, 3, [0, 1, 5, 6, 7, 13, 15], super::Debugger),
    PB4: (pb4, 4, [0, 2, 4, 5, 6, 7, 9, 13, 15], super::Debugger),
    PB5: (pb5, 5, [1, 2, 3, 4, 5, 6, 7, 9, 12, 13, 14, 15]),
    PB6: (pb6, 6, [1, 4, 5, 7, 8, 9, 13, 14, 15]),
    PB7: (pb7, 7, [1, 4, 5, 7, 8, 9, 15]),
    PB8: (pb8, 8, [4, 9, 12, 13, 14, 15]),
    PB9: (pb9, 9, [1, 4, 5, 9, 12, 13, 15]),
    PB10: (pb10, 10, [1, 3, 4, 5, 7, 8, 9, 10, 12, 13, 15]),
    PB11: (pb11, 11, [1, 3, 4, 7, 8, 10, 12, 15]),
    PB12: (pb12, 12, [1, 3, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15]),
    PB13: (pb13, 13, [1, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15]),
    PB14: (pb14, 14, [1, 4, 5, 6, 7, 9, 13, 14, 15]),
    PB15: (pb15, 15, [0, 1, 5, 6, 9, 13, 14, 15]),
]);

#[cfg(feature = "gpio-l45x")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 2, 4, 8, 14, 15]),
    PC1: (pc1, 1, [0, 1, 2, 4, 8, 15]),
    PC2: (pc2, 2, [1, 5, 6, 15]),
    PC3: (pc3, 3, [1, 5, 13, 14, 15]),
    PC4: (pc4, 4, [7, 15]),
    PC5: (pc5, 5, [7, 15]),
    PC6: (pc6, 6, [2, 6, 9, 12, 15]),
    PC7: (pc7, 7, [2, 6, 9, 12, 15]),
    PC8: (pc8, 8, [2, 9, 12, 15]),
    PC9: (pc9, 9, [2, 9, 10, 12, 15]),
    PC10: (pc10, 10, [0, 6, 7, 8, 9, 12, 15]),
    PC11: (pc11, 11, [6, 7, 8, 9, 12, 15]),
    PC12: (pc12, 12, [0, 6, 7, 9, 12, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-l45x")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [5, 9, 15]),
    PD1: (pd1, 1, [5, 9, 15]),
    PD2: (pd2, 2, [0, 2, 7, 9, 12, 15]),
    PD3: (pd3, 3, [5, 6, 7, 10, 15]),
    PD4: (pd4, 4, [5, 6, 7, 10, 15]),
    PD5: (pd5, 5, [7, 10, 15]),
    PD6: (pd6, 6, [6, 7, 10, 13, 15]),
    PD7: (pd7, 7, [6, 7, 10, 15]),
    PD8: (pd8, 8, [7, 15]),
    PD9: (pd9, 9, [7, 15]),
    PD10: (pd10, 10, [7, 9, 15]),
    PD11: (pd11, 11, [4, 7, 9, 14, 15]),
    PD12: (pd12, 12, [4, 7, 9, 14, 15]),
    PD13: (pd13, 13, [4, 9, 14, 15]),
    PD14: (pd14, 14, [15]),
    PD15: (pd15, 15, [15]),
]);

#[cfg(feature = "gpio-l45x")]
gpio!(GPIOE, gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [14, 15]),
    PE1: (pe1, 1, [15]),
    PE2: (pe2, 2, [0, 2, 9, 13, 15]),
    PE3: (pe3, 3, [0, 2, 9, 13, 15]),
    PE4: (pe4, 4, [0, 2, 6, 9, 13, 15]),
    PE5: (pe5, 5, [0, 2, 6, 9, 13, 15]),
    PE6: (pe6, 6, [0, 2, 13, 15]),
    PE7: (pe7, 7, [1, 6, 13, 15]),
    PE8: (pe8, 8, [1, 6, 13, 15]),
    PE9: (pe9, 9, [1, 6, 13, 15]),
    PE10: (pe10, 10, [1, 9, 10, 13, 15]),
    PE11: (pe11, 11, [1, 9, 10, 15]),
    PE12: (pe12, 12, [1, 5, 9, 10, 15]),
    PE13: (pe13, 13, [1, 5, 9, 10, 15]),
    PE14: (pe14, 14, [1, 2, 3, 5, 10, 15]),
    PE15: (pe15, 15, [1, 3, 5, 10, 15]),
]);

#[cfg(feature = "gpio-l45x")]
gpio!(GPIOH, gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
    PH3: (ph3, 3, [15]),
]);

#[cfg(feature = "gpio-l47x")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 2, 3, 7, 8, 13, 14, 15]),
    PA1: (pa1, 1, [1, 2, 7, 8, 11, 14, 15]),
    PA2: (pa2, 2, [1, 2, 7, 11, 13, 14, 15]),
    PA3: (pa3, 3, [1, 2, 7, 11, 14, 15]),
    PA4: (pa4, 4, [5, 6, 7, 13, 14, 15]),
    PA5: (pa5, 5, [1, 2, 3, 5, 14, 15]),
    PA6: (pa6, 6, [1, 2, 3, 5, 7, 10, 11, 12, 13, 14, 15]),
    PA7: (pa7, 7, [1, 2, 3, 5, 10, 11, 14, 15]),
    PA8: (pa8, 8, [0, 1, 7, 10, 11, 14, 15]),
    PA9: (pa9, 9, [1, 7, 11, 14, 15]),
    PA10: (pa10, 10, [1, 7, 10, 11, 14, 15]),
    PA11: (pa11, 11, [1, 2, 7, 9, 10, 12, 15]),
    PA12: (pa12, 12, [1, 7, 9, 10, 15]),
    PA13: (pa13, 13, [0, 1, 10, 15], super::Debugger),
    PA14: (pa14, 14, [0, 15], super::Debugger),
    PA15: (pa15, 15, [0, 1, 2, 5, 6, 8, 9, 11, 13, 15], super::Debugger),
]);

#[cfg(feature = "gpio-l47x")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [1, 2, 3, 7, 10, 11, 12, 15]),
    PB1: (pb1, 1, [1, 2, 3, 6, 7, 10, 11, 14, 15]),
    PB2: (pb2, 2, [0, 1, 4, 6, 15]),
    PB3: (pb3, 3, [0, 1, 5, 6, 7, 11, 13, 15], super::Debugger),
    PB4: (pb4, 4, [0, 2, 5, 6, 7, 8, 9, 11, 13, 14, 15], super::Debugger),
    PB5: (pb5, 5, [1, 2, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15]),
    PB6: (pb6, 6, [1, 2, 3, 4, 6, 7, 9, 12, 13, 14, 15]),
    PB7: (pb7, 7, [1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 15]),
    PB8: (pb8, 8, [2, 4, 6, 9, 11, 12, 13, 14, 15]),
    PB9: (pb9, 9, [1, 2, 4, 5, 6, 9, 11, 12, 13, 14, 15]),
    PB10: (pb10, 10, [1, 4, 5, 6, 7, 8, 10, 11, 12, 13, 15]),
    PB11: (pb11, 11, [1, 4, 6, 7, 8, 10, 11, 12, 15]),
    PB12: (pb12, 12, [1, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15]),
    PB13: (pb13, 13, [1, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15]),
    PB14: (pb14, 14, [1, 3, 4, 5, 6, 7, 9, 11, 12, 13, 14, 15]),
    PB15: (pb15, 15, [0, 1, 3, 5, 6, 9, 11, 12, 13, 14, 15]),
]);

#[cfg(feature = "gpio-l47x")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 4, 6, 8, 11, 14, 15]),
    PC1: (pc1, 1, [1, 4, 6, 8, 11, 15]),
    PC2: (pc2, 2, [1, 5, 6, 11, 15]),
    PC3: (pc3, 3, [1, 5, 11, 13, 14, 15]),
    PC4: (pc4, 4, [7, 11, 15]),
    PC5: (pc5, 5, [7, 11, 15]),
    PC6: (pc6, 6, [2, 3, 6, 9, 11, 12, 13, 15]),
    PC7: (pc7, 7, [2, 3, 6, 9, 11, 12, 13, 15]),
    PC8: (pc8, 8, [2, 3, 9, 11, 12, 15]),
    PC9: (pc9, 9, [1, 2, 3, 9, 10, 11, 12, 13, 14, 15]),
    PC10: (pc10, 10, [6, 7, 8, 9, 11, 12, 13, 15]),
    PC11: (pc11, 11, [6, 7, 8, 9, 11, 12, 13, 15]),
    PC12: (pc12, 12, [6, 7, 8, 9, 11, 12, 13, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-l47x")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [5, 6, 9, 12, 15]),
    PD1: (pd1, 1, [5, 6, 9, 12, 15]),
    PD2: (pd2, 2, [2, 7, 8, 9, 11, 12, 15]),
    PD3: (pd3, 3, [5, 6, 7, 12, 15]),
    PD4: (pd4, 4, [5, 6, 7, 12, 15]),
    PD5: (pd5, 5, [7, 12, 15]),
    PD6: (pd6, 6, [6, 7, 12, 13, 15]),
    PD7: (pd7, 7, [6, 7, 12, 15]),
    PD8: (pd8, 8, [7, 11, 12, 15]),
    PD9: (pd9, 9, [7, 11, 12, 13, 15]),
    PD10: (pd10, 10, [7, 9, 11, 12, 13, 15]),
    PD11: (pd11, 11, [7, 9, 11, 12, 13, 14, 15]),
    PD12: (pd12, 12, [2, 7, 9, 11, 12, 13, 14, 15]),
    PD13: (pd13, 13, [2, 9, 11, 12, 14, 15]),
    PD14: (pd14, 14, [2, 11, 12, 15]),
    PD15: (pd15, 15, [2, 11, 12, 15]),
]);

#[cfg(feature = "gpio-l47x")]
gpio!(GPIOE, gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [2, 11, 12, 14, 15]),
    PE1: (pe1, 1, [11, 12, 14, 15]),
    PE2: (pe2, 2, [0, 2, 9, 11, 12, 13, 15]),
    PE3: (pe3, 3, [0, 2, 9, 11, 12, 13, 15]),
    PE4: (pe4, 4, [0, 2, 6, 9, 12, 13, 15]),
    PE5: (pe5, 5, [0, 2, 6, 9, 12, 13, 15]),
    PE6: (pe6, 6, [0, 2, 12, 13, 15]),
    PE7: (pe7, 7, [1, 6, 12, 13, 15]),
    PE8: (pe8, 8, [1, 6, 12, 13, 15]),
    PE9: (pe9, 9, [1, 6, 12, 13, 15]),
    PE10: (pe10, 10, [1, 6, 9, 10, 12, 13, 15]),
    PE11: (pe11, 11, [1, 6, 9, 10, 12, 15]),
    PE12: (pe12, 12, [1, 5, 6, 9, 10, 12, 15]),
    PE13: (pe13, 13, [1, 5, 6, 9, 10, 12, 15]),
    PE14: (pe14, 14, [1, 2, 3, 5, 10, 12, 15]),
    PE15: (pe15, 15, [1, 3, 5, 10, 12, 15]),
]);

#[cfg(not(feature = "stm32l471"))]
#[cfg(feature = "gpio-l47x")]
gpio!(GPIOF, gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4, 12, 15]),
    PF1: (pf1, 1, [4, 12, 15]),
    PF2: (pf2, 2, [4, 12, 15]),
    PF3: (pf3, 3, [12, 15]),
    PF4: (pf4, 4, [12, 15]),
    PF5: (pf5, 5, [12, 15]),
    PF6: (pf6, 6, [1, 2, 13, 15]),
    PF7: (pf7, 7, [2, 13, 15]),
    PF8: (pf8, 8, [2, 13, 15]),
    PF9: (pf9, 9, [2, 13, 14, 15]),
    PF10: (pf10, 10, [14, 15]),
    PF11: (pf11, 11, [15]),
    PF12: (pf12, 12, [12, 15]),
    PF13: (pf13, 13, [6, 12, 15]),
    PF14: (pf14, 14, [6, 9, 12, 15]),
    PF15: (pf15, 15, [9, 12, 15]),
]);

#[cfg(not(feature = "stm32l471"))]
#[cfg(feature = "gpio-l47x")]
gpio!(GPIOG, gpiog, PG, 'G', PGn,
    { unsafe { (*crate::pac::PWR::ptr()).cr2.modify(|_,w| w.iosv().set_bit()); } },
[
    PG0: (pg0, 0, [9, 12, 15]),
    PG1: (pg1, 1, [9, 12, 15]),
    PG2: (pg2, 2, [5, 12, 13, 15]),
    PG3: (pg3, 3, [5, 12, 13, 15]),
    PG4: (pg4, 4, [5, 12, 13, 15]),
    PG5: (pg5, 5, [5, 8, 12, 13, 15]),
    PG6: (pg6, 6, [4, 8, 15]),
    PG7: (pg7, 7, [4, 8, 12, 15]),
    PG8: (pg8, 8, [4, 8, 15]),
    PG9: (pg9, 9, [6, 7, 12, 13, 14, 15]),
    PG10: (pg10, 10, [1, 6, 7, 12, 13, 14, 15]),
    PG11: (pg11, 11, [1, 6, 7, 13, 14, 15]),
    PG12: (pg12, 12, [1, 6, 7, 12, 13, 15]),
    PG13: (pg13, 13, [4, 7, 12, 15]),
    PG14: (pg14, 14, [4, 12, 15]),
    PG15: (pg15, 15, [1, 4, 15]),
]);

#[cfg(feature = "gpio-l47x")]
gpio!(GPIOH, gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
]);

#[cfg(feature = "gpio-l49x")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 2, 3, 7, 8, 13, 14, 15]),
    PA1: (pa1, 1, [1, 2, 4, 5, 7, 8, 11, 14, 15]),
    PA2: (pa2, 2, [1, 2, 7, 8, 10, 11, 13, 14, 15]),
    PA3: (pa3, 3, [1, 2, 7, 8, 10, 11, 13, 14, 15]),
    PA4: (pa4, 4, [5, 6, 7, 10, 13, 14, 15]),
    PA5: (pa5, 5, [1, 2, 3, 5, 14, 15]),
    PA6: (pa6, 6, [1, 2, 3, 4, 5, 7, 8, 10, 11, 12, 13, 14, 15]),
    PA7: (pa7, 7, [1, 2, 3, 4, 5, 10, 11, 14, 15]),
    PA8: (pa8, 8, [0, 1, 7, 10, 11, 12, 13, 14, 15]),
    PA9: (pa9, 9, [1, 3, 5, 7, 11, 13, 14, 15]),
    PA10: (pa10, 10, [1, 5, 7, 10, 11, 13, 14, 15]),
    PA11: (pa11, 11, [1, 2, 5, 7, 9, 10, 12, 15]),
    PA12: (pa12, 12, [1, 5, 7, 9, 10, 15]),
    PA13: (pa13, 13, [0, 1, 10, 12, 13, 15], super::Debugger),
    PA14: (pa14, 14, [0, 1, 4, 5, 10, 12, 13, 15], super::Debugger),
    PA15: (pa15, 15, [0, 1, 2, 3, 5, 6, 7, 8, 9, 11, 12, 13, 15], super::Debugger),
]);

#[cfg(feature = "gpio-l49x")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [1, 2, 3, 5, 7, 10, 11, 12, 13, 15]),
    PB1: (pb1, 1, [1, 2, 3, 6, 7, 8, 10, 11, 14, 15]),
    PB2: (pb2, 2, [0, 1, 4, 6, 11, 15]),
    PB3: (pb3, 3, [0, 1, 5, 6, 7, 10, 11, 13, 15], super::Debugger),
    PB4: (pb4, 4, [0, 2, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15], super::Debugger),
    PB5: (pb5, 5, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB6: (pb6, 6, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15]),
    PB7: (pb7, 7, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB8: (pb8, 8, [2, 4, 6, 9, 10, 11, 12, 13, 14, 15]),
    PB9: (pb9, 9, [1, 2, 4, 5, 6, 9, 10, 11, 12, 13, 14, 15]),
    PB10: (pb10, 10, [1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15]),
    PB11: (pb11, 11, [1, 3, 4, 6, 7, 8, 10, 11, 12, 15]),
    PB12: (pb12, 12, [1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB13: (pb13, 13, [1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB14: (pb14, 14, [1, 3, 4, 5, 6, 7, 9, 11, 12, 13, 14, 15]),
    PB15: (pb15, 15, [0, 1, 3, 5, 6, 9, 11, 12, 13, 14, 15]),
]);

#[cfg(feature = "gpio-l49x")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 2, 4, 6, 8, 11, 14, 15]),
    PC1: (pc1, 1, [0, 1, 2, 3, 4, 6, 8, 10, 11, 13, 15]),
    PC2: (pc2, 2, [1, 5, 6, 10, 11, 15]),
    PC3: (pc3, 3, [1, 5, 10, 11, 13, 14, 15]),
    PC4: (pc4, 4, [7, 10, 11, 15]),
    PC5: (pc5, 5, [7, 11, 15]),
    PC6: (pc6, 6, [2, 3, 6, 9, 10, 11, 12, 13, 15]),
    PC7: (pc7, 7, [2, 3, 6, 9, 10, 11, 12, 13, 15]),
    PC8: (pc8, 8, [2, 3, 9, 10, 11, 12, 15]),
    PC9: (pc9, 9, [1, 2, 3, 4, 6, 9, 10, 11, 12, 13, 14, 15]),
    PC10: (pc10, 10, [0, 6, 7, 8, 9, 10, 11, 12, 13, 15]),
    PC11: (pc11, 11, [5, 6, 7, 8, 9, 10, 11, 12, 13, 15]),
    PC12: (pc12, 12, [0, 6, 7, 8, 9, 10, 11, 12, 13, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-l49x")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [5, 6, 9, 12, 15]),
    PD1: (pd1, 1, [5, 6, 9, 12, 15]),
    PD2: (pd2, 2, [0, 2, 7, 8, 9, 10, 11, 12, 15]),
    PD3: (pd3, 3, [3, 4, 5, 6, 7, 10, 12, 15]),
    PD4: (pd4, 4, [5, 6, 7, 10, 12, 15]),
    PD5: (pd5, 5, [7, 10, 12, 15]),
    PD6: (pd6, 6, [4, 5, 6, 7, 10, 12, 13, 15]),
    PD7: (pd7, 7, [6, 7, 10, 12, 15]),
    PD8: (pd8, 8, [7, 10, 11, 12, 15]),
    PD9: (pd9, 9, [7, 10, 11, 12, 13, 15]),
    PD10: (pd10, 10, [7, 9, 11, 12, 13, 15]),
    PD11: (pd11, 11, [4, 7, 9, 11, 12, 13, 14, 15]),
    PD12: (pd12, 12, [2, 4, 7, 9, 11, 12, 13, 14, 15]),
    PD13: (pd13, 13, [2, 4, 9, 11, 12, 14, 15]),
    PD14: (pd14, 14, [2, 11, 12, 15]),
    PD15: (pd15, 15, [2, 11, 12, 15]),
]);

#[cfg(feature = "gpio-l49x")]
gpio!(GPIOE, gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [2, 10, 11, 12, 14, 15]),
    PE1: (pe1, 1, [10, 11, 12, 14, 15]),
    PE2: (pe2, 2, [0, 2, 9, 11, 12, 13, 15]),
    PE3: (pe3, 3, [0, 2, 9, 11, 12, 13, 15]),
    PE4: (pe4, 4, [0, 2, 6, 9, 10, 12, 13, 15]),
    PE5: (pe5, 5, [0, 2, 6, 9, 10, 12, 13, 15]),
    PE6: (pe6, 6, [0, 2, 10, 12, 13, 15]),
    PE7: (pe7, 7, [1, 6, 12, 13, 15]),
    PE8: (pe8, 8, [1, 6, 12, 13, 15]),
    PE9: (pe9, 9, [1, 6, 12, 13, 15]),
    PE10: (pe10, 10, [1, 6, 9, 10, 12, 13, 15]),
    PE11: (pe11, 11, [1, 6, 9, 10, 12, 15]),
    PE12: (pe12, 12, [1, 5, 6, 9, 10, 12, 15]),
    PE13: (pe13, 13, [1, 5, 6, 9, 10, 12, 15]),
    PE14: (pe14, 14, [1, 2, 3, 5, 10, 12, 15]),
    PE15: (pe15, 15, [1, 3, 5, 10, 12, 15]),
]);

#[cfg(feature = "gpio-l49x")]
gpio!(GPIOF, gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4, 12, 15]),
    PF1: (pf1, 1, [4, 12, 15]),
    PF2: (pf2, 2, [4, 12, 15]),
    PF3: (pf3, 3, [12, 15]),
    PF4: (pf4, 4, [12, 15]),
    PF5: (pf5, 5, [12, 15]),
    PF6: (pf6, 6, [1, 2, 10, 13, 15]),
    PF7: (pf7, 7, [2, 10, 13, 15]),
    PF8: (pf8, 8, [2, 10, 13, 15]),
    PF9: (pf9, 9, [2, 10, 13, 14, 15]),
    PF10: (pf10, 10, [3, 10, 14, 15]),
    PF11: (pf11, 11, [10, 15]),
    PF12: (pf12, 12, [12, 15]),
    PF13: (pf13, 13, [4, 6, 12, 15]),
    PF14: (pf14, 14, [4, 6, 9, 12, 15]),
    PF15: (pf15, 15, [4, 9, 12, 15]),
]);

#[cfg(feature = "gpio-l49x")]
gpio!(GPIOG, gpiog, PG, 'G', PGn,
    { unsafe { (*crate::pac::PWR::ptr()).cr2.modify(|_,w| w.iosv().set_bit()); } },
[
    PG0: (pg0, 0, [9, 12, 15]),
    PG1: (pg1, 1, [9, 12, 15]),
    PG2: (pg2, 2, [5, 12, 13, 15]),
    PG3: (pg3, 3, [5, 12, 13, 15]),
    PG4: (pg4, 4, [5, 12, 13, 15]),
    PG5: (pg5, 5, [5, 8, 12, 13, 15]),
    PG6: (pg6, 6, [4, 8, 15]),
    PG7: (pg7, 7, [4, 8, 12, 13, 15]),
    PG8: (pg8, 8, [4, 8, 15]),
    PG9: (pg9, 9, [6, 7, 12, 13, 14, 15]),
    PG10: (pg10, 10, [1, 6, 7, 12, 13, 14, 15]),
    PG11: (pg11, 11, [1, 6, 7, 13, 14, 15]),
    PG12: (pg12, 12, [1, 6, 7, 12, 13, 15]),
    PG13: (pg13, 13, [4, 7, 12, 15]),
    PG14: (pg14, 14, [4, 12, 15]),
    PG15: (pg15, 15, [1, 4, 10, 15]),
]);

#[cfg(feature = "gpio-l49x")]
gpio!(GPIOH, gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
    PH2: (ph2, 2, [3, 15]),
    PH3: (ph3, 3, [15]),
    PH4: (ph4, 4, [4, 15]),
    PH5: (ph5, 5, [4, 10, 15]),
    PH6: (ph6, 6, [4, 10, 15]),
    PH7: (ph7, 7, [4, 10, 15]),
    PH8: (ph8, 8, [4, 10, 15]),
    PH9: (ph9, 9, [4, 10, 15]),
    PH10: (ph10, 10, [2, 10, 15]),
    PH11: (ph11, 11, [2, 10, 15]),
    PH12: (ph12, 12, [2, 10, 15]),
    PH13: (ph13, 13, [3, 9, 15]),
    PH14: (ph14, 14, [3, 10, 15]),
    PH15: (ph15, 15, [3, 10, 15]),
]);

#[cfg(feature = "gpio-l49x")]
gpio!(GPIOI, gpioi, PI, 'I', PIn, [
    PI0: (pi0, 0, [2, 5, 10, 15]),
    PI1: (pi1, 1, [5, 10, 15]),
    PI2: (pi2, 2, [3, 5, 10, 15]),
    PI3: (pi3, 3, [3, 5, 10, 15]),
    PI4: (pi4, 4, [3, 10, 15]),
    PI5: (pi5, 5, [3, 10, 15]),
    PI6: (pi6, 6, [3, 10, 15]),
    PI7: (pi7, 7, [3, 10, 15]),
    PI8: (pi8, 8, [10, 15]),
    PI9: (pi9, 9, [9, 15]),
    PI10: (pi10, 10, [15]),
    PI11: (pi11, 11, [15]),
]);

#[cfg(feature = "gpio-l4p")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 2, 3, 7, 8, 13, 14, 15]),
    PA1: (pa1, 1, [1, 2, 4, 5, 7, 8, 10, 12, 14, 15]),
    PA2: (pa2, 2, [1, 2, 7, 8, 10, 13, 14, 15]),
    PA3: (pa3, 3, [1, 2, 3, 7, 8, 10, 13, 14, 15]),
    PA4: (pa4, 4, [3, 5, 6, 7, 10, 11, 13, 14, 15]),
    PA5: (pa5, 5, [1, 2, 3, 4, 5, 11, 14, 15]),
    PA6: (pa6, 6, [2, 4, 5, 7, 8, 10, 12, 13, 14, 15]),
    PA7: (pa7, 7, [1, 2, 3, 4, 5, 10, 14, 15]),
    PA8: (pa8, 8, [0, 1, 3, 7, 10, 11, 13, 14, 15]),
    PA9: (pa9, 9, [1, 3, 5, 7, 11, 13, 14, 15]),
    PA10: (pa10, 10, [1, 3, 5, 7, 10, 11, 13, 14, 15]),
    PA11: (pa11, 11, [1, 5, 7, 9, 10, 11, 12, 15]),
    PA12: (pa12, 12, [1, 5, 7, 9, 10, 11, 15]),
    PA13: (pa13, 13, [0, 1, 10, 13, 15], super::Debugger),
    PA14: (pa14, 14, [0, 1, 4, 5, 10, 13, 15], super::Debugger),
    PA15: (pa15, 15, [0, 1, 2, 3, 5, 6, 7, 8, 9, 11, 13, 15], super::Debugger),
]);

#[cfg(feature = "gpio-l4p")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [1, 2, 3, 5, 7, 10, 11, 12, 13, 15]),
    PB1: (pb1, 1, [1, 2, 3, 6, 7, 8, 10, 11, 14, 15]),
    PB2: (pb2, 2, [1, 4, 6, 10, 15]),
    PB3: (pb3, 3, [0, 1, 3, 5, 6, 7, 10, 12, 13, 15], super::Debugger),
    PB4: (pb4, 4, [0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15], super::Debugger),
    PB5: (pb5, 5, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15]),
    PB6: (pb6, 6, [1, 2, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15]),
    PB7: (pb7, 7, [1, 2, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB8: (pb8, 8, [2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB9: (pb9, 9, [1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15]),
    PB10: (pb10, 10, [1, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15]),
    PB11: (pb11, 11, [1, 3, 4, 7, 8, 10, 11, 12, 15]),
    PB12: (pb12, 12, [3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15]),
    PB13: (pb13, 13, [1, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15]),
    PB14: (pb14, 14, [1, 3, 4, 5, 6, 7, 9, 10, 11, 13, 14, 15]),
    PB15: (pb15, 15, [0, 1, 3, 5, 6, 9, 10, 11, 13, 14, 15]),
]);

#[cfg(feature = "gpio-l4p")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 4, 7, 8, 11, 12, 13, 14, 15]),
    PC1: (pc1, 1, [0, 1, 3, 4, 8, 10, 13, 15]),
    PC2: (pc2, 2, [1, 5, 6, 10, 11, 15]),
    PC3: (pc3, 3, [1, 3, 5, 10, 13, 14, 15]),
    PC4: (pc4, 4, [5, 7, 10, 15]),
    PC5: (pc5, 5, [3, 4, 7, 11, 15]),
    PC6: (pc6, 6, [2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 15]),
    PC7: (pc7, 7, [2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 15]),
    PC8: (pc8, 8, [2, 3, 9, 10, 12, 15]),
    PC9: (pc9, 9, [0, 2, 3, 4, 6, 9, 10, 12, 13, 14, 15]),
    PC10: (pc10, 10, [0, 4, 6, 7, 8, 9, 10, 12, 13, 15]),
    PC11: (pc11, 11, [4, 5, 6, 7, 8, 9, 10, 12, 13, 15]),
    PC12: (pc12, 12, [0, 6, 7, 8, 9, 10, 11, 12, 13, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-l4p")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [5, 9, 11, 12, 15]),
    PD1: (pd1, 1, [5, 9, 11, 12, 15]),
    PD2: (pd2, 2, [0, 2, 7, 8, 9, 10, 12, 15]),
    PD3: (pd3, 3, [3, 4, 5, 6, 7, 10, 11, 12, 15]),
    PD4: (pd4, 4, [5, 6, 7, 8, 10, 12, 15]),
    PD5: (pd5, 5, [7, 10, 12, 15]),
    PD6: (pd6, 6, [3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 15]),
    PD7: (pd7, 7, [6, 7, 8, 10, 12, 15]),
    PD8: (pd8, 8, [7, 10, 11, 12, 15]),
    PD9: (pd9, 9, [7, 10, 11, 12, 13, 15]),
    PD10: (pd10, 10, [7, 9, 11, 12, 13, 15]),
    PD11: (pd11, 11, [4, 7, 9, 11, 12, 13, 14, 15]),
    PD12: (pd12, 12, [2, 4, 7, 9, 11, 12, 13, 14, 15]),
    PD13: (pd13, 13, [2, 4, 9, 12, 14, 15]),
    PD14: (pd14, 14, [2, 11, 12, 15]),
    PD15: (pd15, 15, [2, 11, 12, 15]),
]);

#[cfg(feature = "gpio-l4p")]
gpio!(GPIOE, gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [2, 10, 11, 12, 14, 15]),
    PE1: (pe1, 1, [10, 11, 12, 14, 15]),
    PE2: (pe2, 2, [0, 2, 3, 9, 11, 12, 13, 15]),
    PE3: (pe3, 3, [0, 2, 3, 9, 11, 12, 13, 15]),
    PE4: (pe4, 4, [0, 2, 3, 6, 9, 10, 11, 12, 13, 15]),
    PE5: (pe5, 5, [0, 2, 3, 6, 9, 10, 11, 12, 13, 15]),
    PE6: (pe6, 6, [0, 2, 3, 10, 11, 12, 13, 15]),
    PE7: (pe7, 7, [1, 6, 11, 12, 13, 15]),
    PE8: (pe8, 8, [1, 6, 11, 12, 13, 15]),
    PE9: (pe9, 9, [1, 6, 10, 11, 12, 13, 15]),
    PE10: (pe10, 10, [1, 9, 10, 11, 12, 13, 15]),
    PE11: (pe11, 11, [1, 9, 10, 11, 12, 15]),
    PE12: (pe12, 12, [1, 5, 9, 10, 11, 12, 15]),
    PE13: (pe13, 13, [1, 5, 9, 10, 11, 12, 15]),
    PE14: (pe14, 14, [1, 3, 5, 10, 11, 12, 15]),
    PE15: (pe15, 15, [3, 5, 10, 11, 12, 15]),
]);

#[cfg(feature = "gpio-l4p")]
gpio!(GPIOF, gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4, 5, 12, 15]),
    PF1: (pf1, 1, [4, 5, 12, 15]),
    PF2: (pf2, 2, [4, 5, 12, 15]),
    PF3: (pf3, 3, [5, 12, 15]),
    PF4: (pf4, 4, [5, 12, 15]),
    PF5: (pf5, 5, [5, 12, 15]),
    PF6: (pf6, 6, [1, 2, 10, 13, 15]),
    PF7: (pf7, 7, [2, 10, 13, 15]),
    PF8: (pf8, 8, [2, 10, 13, 15]),
    PF9: (pf9, 9, [2, 10, 13, 14, 15]),
    PF10: (pf10, 10, [3, 4, 6, 10, 13, 14, 15]),
    PF11: (pf11, 11, [3, 9, 10, 15]),
    PF12: (pf12, 12, [5, 11, 12, 15]),
    PF13: (pf13, 13, [4, 11, 12, 15]),
    PF14: (pf14, 14, [4, 9, 11, 12, 15]),
    PF15: (pf15, 15, [4, 9, 11, 12, 15]),
]);

#[cfg(feature = "gpio-l4p")]
gpio!(GPIOG, gpiog, PG, 'G', PGn,
    { unsafe { (*crate::pac::PWR::ptr()).cr2.modify(|_,w| w.iosv().set_bit()); } },
[
    PG0: (pg0, 0, [5, 9, 12, 15]),
    PG1: (pg1, 1, [5, 9, 12, 15]),
    PG2: (pg2, 2, [5, 11, 12, 13, 15]),
    PG3: (pg3, 3, [5, 11, 12, 13, 15]),
    PG4: (pg4, 4, [5, 11, 12, 13, 15]),
    PG5: (pg5, 5, [5, 8, 11, 12, 13, 15]),
    PG6: (pg6, 6, [3, 4, 8, 9, 15]),
    PG7: (pg7, 7, [3, 4, 5, 6, 8, 12, 13, 15]),
    PG8: (pg8, 8, [4, 8, 15]),
    PG9: (pg9, 9, [5, 6, 7, 11, 12, 13, 14, 15]),
    PG10: (pg10, 10, [1, 5, 6, 7, 11, 12, 13, 14, 15]),
    PG11: (pg11, 11, [1, 3, 6, 7, 11, 13, 14, 15]),
    PG12: (pg12, 12, [1, 5, 6, 7, 11, 12, 13, 15]),
    PG13: (pg13, 13, [4, 7, 11, 12, 15]),
    PG14: (pg14, 14, [4, 11, 12, 15]),
    PG15: (pg15, 15, [1, 4, 5, 10, 15]),
]);

#[cfg(feature = "gpio-l4p")]
gpio!(GPIOH, gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
    PH2: (ph2, 2, [3, 15]),
    PH3: (ph3, 3, [15]),
    PH4: (ph4, 4, [4, 5, 10, 15]),
    PH5: (ph5, 5, [4, 10, 15]),
    PH6: (ph6, 6, [4, 5, 10, 15]),
    PH7: (ph7, 7, [4, 5, 10, 15]),
    PH8: (ph8, 8, [4, 5, 10, 15]),
    PH9: (ph9, 9, [4, 5, 10, 15]),
    PH10: (ph10, 10, [2, 5, 10, 15]),
    PH11: (ph11, 11, [2, 5, 10, 15]),
    PH12: (ph12, 12, [2, 5, 10, 15]),
    PH13: (ph13, 13, [3, 9, 15]),
    PH14: (ph14, 14, [3, 10, 15]),
    PH15: (ph15, 15, [3, 5, 10, 15]),
]);

#[cfg(feature = "gpio-l4p")]
gpio!(GPIOI, gpioi, PI, 'I', PIn, [
    PI0: (pi0, 0, [2, 3, 5, 10, 15]),
    PI1: (pi1, 1, [5, 10, 15]),
    PI2: (pi2, 2, [3, 5, 10, 15]),
    PI3: (pi3, 3, [3, 5, 10, 15]),
    PI4: (pi4, 4, [3, 10, 15]),
    PI5: (pi5, 5, [3, 5, 10, 15]),
    PI6: (pi6, 6, [3, 5, 10, 15]),
    PI7: (pi7, 7, [3, 5, 10, 15]),
    PI8: (pi8, 8, [5, 10, 15]),
    PI9: (pi9, 9, [5, 9, 15]),
    PI10: (pi10, 10, [5, 10, 15]),
    PI11: (pi11, 11, [5, 10, 15]),
]);

#[cfg(feature = "gpio-l4rx")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 2, 3, 7, 8, 13, 14, 15]),
    PA1: (pa1, 1, [1, 2, 4, 5, 7, 8, 10, 14, 15]),
    PA2: (pa2, 2, [1, 2, 7, 8, 10, 13, 14, 15]),
    PA3: (pa3, 3, [1, 2, 3, 7, 8, 10, 13, 14, 15]),
    PA4: (pa4, 4, [3, 5, 6, 7, 10, 13, 14, 15]),
    PA5: (pa5, 5, [1, 2, 3, 5, 14, 15]),
    PA6: (pa6, 6, [2, 4, 5, 7, 8, 10, 12, 13, 14, 15]),
    PA7: (pa7, 7, [1, 2, 3, 4, 5, 10, 14, 15]),
    PA8: (pa8, 8, [0, 1, 3, 7, 10, 13, 14, 15]),
    PA9: (pa9, 9, [1, 3, 5, 7, 13, 14, 15]),
    PA10: (pa10, 10, [1, 3, 5, 7, 10, 13, 14, 15]),
    PA11: (pa11, 11, [1, 5, 7, 9, 10, 12, 15]),
    PA12: (pa12, 12, [1, 5, 7, 9, 10, 15]),
    PA13: (pa13, 13, [0, 1, 10, 13, 15], super::Debugger),
    PA14: (pa14, 14, [0, 1, 4, 5, 10, 13, 15], super::Debugger),
    PA15: (pa15, 15, [0, 1, 2, 3, 5, 6, 7, 8, 9, 13, 15], super::Debugger),
]);

#[cfg(feature = "gpio-l4rx")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [1, 2, 3, 5, 7, 10, 12, 13, 15]),
    PB1: (pb1, 1, [1, 2, 3, 6, 7, 8, 10, 14, 15]),
    PB2: (pb2, 2, [0, 1, 4, 6, 10, 11, 15]),
    PB3: (pb3, 3, [0, 1, 5, 6, 7, 10, 13, 15], super::Debugger),
    PB4: (pb4, 4, [0, 2, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15], super::Debugger),
    PB5: (pb5, 5, [1, 2, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15]),
    PB6: (pb6, 6, [1, 2, 4, 5, 6, 7, 9, 10, 12, 13, 14, 15]),
    PB7: (pb7, 7, [1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB8: (pb8, 8, [2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB9: (pb9, 9, [1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 13, 14, 15]),
    PB10: (pb10, 10, [1, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15]),
    PB11: (pb11, 11, [1, 3, 4, 6, 7, 8, 10, 11, 12, 15]),
    PB12: (pb12, 12, [3, 4, 5, 6, 7, 8, 9, 13, 14, 15]),
    PB13: (pb13, 13, [1, 4, 5, 6, 7, 8, 9, 13, 14, 15]),
    PB14: (pb14, 14, [1, 3, 4, 5, 6, 7, 9, 13, 14, 15]),
    PB15: (pb15, 15, [0, 1, 3, 5, 6, 9, 13, 14, 15]),
]);

#[cfg(feature = "gpio-l4rx")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 4, 6, 8, 13, 14, 15]),
    PC1: (pc1, 1, [0, 1, 3, 4, 6, 8, 10, 13, 15]),
    PC2: (pc2, 2, [1, 5, 6, 10, 15]),
    PC3: (pc3, 3, [1, 3, 5, 10, 13, 14, 15]),
    PC4: (pc4, 4, [7, 10, 15]),
    PC5: (pc5, 5, [3, 7, 15]),
    PC6: (pc6, 6, [2, 3, 6, 8, 9, 10, 11, 12, 13, 15]),
    PC7: (pc7, 7, [2, 3, 6, 8, 9, 10, 11, 12, 13, 15]),
    PC8: (pc8, 8, [2, 3, 9, 10, 12, 15]),
    PC9: (pc9, 9, [0, 2, 3, 4, 6, 9, 10, 12, 13, 14, 15]),
    PC10: (pc10, 10, [0, 6, 7, 8, 9, 10, 12, 13, 15]),
    PC11: (pc11, 11, [4, 5, 6, 7, 8, 9, 10, 12, 13, 15]),
    PC12: (pc12, 12, [0, 6, 7, 8, 9, 10, 12, 13, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-l4rx")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [5, 6, 9, 11, 12, 15]),
    PD1: (pd1, 1, [5, 6, 9, 11, 12, 15]),
    PD2: (pd2, 2, [0, 2, 7, 8, 9, 10, 12, 15]),
    PD3: (pd3, 3, [3, 4, 5, 6, 7, 10, 11, 12, 15]),
    PD4: (pd4, 4, [5, 6, 7, 10, 12, 15]),
    PD5: (pd5, 5, [7, 10, 12, 15]),
    PD6: (pd6, 6, [3, 4, 5, 6, 7, 10, 11, 12, 13, 15]),
    PD7: (pd7, 7, [6, 7, 10, 12, 15]),
    PD8: (pd8, 8, [7, 10, 11, 12, 15]),
    PD9: (pd9, 9, [7, 10, 11, 12, 13, 15]),
    PD10: (pd10, 10, [7, 9, 11, 12, 13, 15]),
    PD11: (pd11, 11, [4, 7, 9, 11, 12, 13, 14, 15]),
    PD12: (pd12, 12, [2, 4, 7, 9, 11, 12, 13, 14, 15]),
    PD13: (pd13, 13, [2, 4, 9, 12, 14, 15]),
    PD14: (pd14, 14, [2, 11, 12, 15]),
    PD15: (pd15, 15, [2, 11, 12, 15]),
]);

#[cfg(feature = "gpio-l4rx")]
gpio!(GPIOE, gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [2, 10, 11, 12, 14, 15]),
    PE1: (pe1, 1, [10, 11, 12, 14, 15]),
    PE2: (pe2, 2, [0, 2, 3, 9, 11, 12, 13, 15]),
    PE3: (pe3, 3, [0, 2, 3, 9, 11, 12, 13, 15]),
    PE4: (pe4, 4, [0, 2, 3, 6, 9, 10, 11, 12, 13, 15]),
    PE5: (pe5, 5, [0, 2, 3, 6, 9, 10, 11, 12, 13, 15]),
    PE6: (pe6, 6, [0, 2, 3, 10, 11, 12, 13, 15]),
    PE7: (pe7, 7, [1, 6, 11, 12, 13, 15]),
    PE8: (pe8, 8, [1, 6, 11, 12, 13, 15]),
    PE9: (pe9, 9, [1, 6, 11, 12, 13, 15]),
    PE10: (pe10, 10, [1, 6, 9, 10, 11, 12, 13, 15]),
    PE11: (pe11, 11, [1, 6, 9, 10, 11, 12, 15]),
    PE12: (pe12, 12, [1, 5, 6, 9, 10, 11, 12, 15]),
    PE13: (pe13, 13, [1, 5, 6, 9, 10, 11, 12, 15]),
    PE14: (pe14, 14, [1, 3, 5, 10, 11, 12, 15]),
    PE15: (pe15, 15, [3, 5, 10, 11, 12, 15]),
]);

#[cfg(feature = "gpio-l4rx")]
gpio!(GPIOF, gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4, 5, 12, 15]),
    PF1: (pf1, 1, [4, 5, 12, 15]),
    PF2: (pf2, 2, [4, 5, 12, 15]),
    PF3: (pf3, 3, [5, 12, 15]),
    PF4: (pf4, 4, [5, 12, 15]),
    PF5: (pf5, 5, [12, 15]),
    PF6: (pf6, 6, [1, 2, 10, 13, 15]),
    PF7: (pf7, 7, [2, 10, 13, 15]),
    PF8: (pf8, 8, [2, 10, 13, 15]),
    PF9: (pf9, 9, [2, 10, 13, 14, 15]),
    PF10: (pf10, 10, [3, 6, 10, 13, 14, 15]),
    PF11: (pf11, 11, [9, 10, 11, 15]),
    PF12: (pf12, 12, [5, 11, 12, 15]),
    PF13: (pf13, 13, [4, 6, 11, 12, 15]),
    PF14: (pf14, 14, [4, 6, 9, 11, 12, 15]),
    PF15: (pf15, 15, [4, 9, 11, 12, 15]),
]);

#[cfg(feature = "gpio-l4rx")]
gpio!(GPIOG, gpiog, PG, 'G', PGn,
    { unsafe { (*crate::pac::PWR::ptr()).cr2.modify(|_,w| w.iosv().set_bit()); } },
[
    PG0: (pg0, 0, [5, 9, 12, 15]),
    PG1: (pg1, 1, [5, 9, 12, 15]),
    PG2: (pg2, 2, [5, 12, 13, 15]),
    PG3: (pg3, 3, [5, 12, 13, 15]),
    PG4: (pg4, 4, [5, 12, 13, 15]),
    PG5: (pg5, 5, [5, 8, 12, 13, 15]),
    PG6: (pg6, 6, [3, 4, 8, 9, 11, 15]),
    PG7: (pg7, 7, [3, 4, 5, 6, 8, 12, 13, 15]),
    PG8: (pg8, 8, [4, 8, 15]),
    PG9: (pg9, 9, [5, 6, 7, 12, 13, 14, 15]),
    PG10: (pg10, 10, [1, 5, 6, 7, 12, 13, 14, 15]),
    PG11: (pg11, 11, [1, 3, 6, 7, 13, 14, 15]),
    PG12: (pg12, 12, [1, 5, 6, 7, 12, 13, 15]),
    PG13: (pg13, 13, [4, 7, 11, 12, 15]),
    PG14: (pg14, 14, [4, 11, 12, 15]),
    PG15: (pg15, 15, [1, 4, 5, 10, 15]),
]);

#[cfg(feature = "gpio-l4rx")]
gpio!(GPIOH, gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
    PH2: (ph2, 2, [3, 15]),
    PH3: (ph3, 3, [15]),
    PH4: (ph4, 4, [4, 5, 15]),
    PH5: (ph5, 5, [4, 10, 15]),
    PH6: (ph6, 6, [4, 5, 10, 15]),
    PH7: (ph7, 7, [4, 10, 15]),
    PH8: (ph8, 8, [4, 5, 10, 15]),
    PH9: (ph9, 9, [4, 5, 10, 15]),
    PH10: (ph10, 10, [2, 5, 10, 15]),
    PH11: (ph11, 11, [2, 5, 10, 15]),
    PH12: (ph12, 12, [2, 5, 10, 15]),
    PH13: (ph13, 13, [3, 9, 15]),
    PH14: (ph14, 14, [3, 10, 15]),
    PH15: (ph15, 15, [3, 5, 10, 15]),
]);

#[cfg(feature = "gpio-l4rx")]
gpio!(GPIOI, gpioi, PI, 'I', PIn, [
    PI0: (pi0, 0, [2, 3, 5, 10, 15]),
    PI1: (pi1, 1, [5, 10, 15]),
    PI2: (pi2, 2, [3, 5, 10, 15]),
    PI3: (pi3, 3, [3, 5, 10, 15]),
    PI4: (pi4, 4, [3, 10, 15]),
    PI5: (pi5, 5, [3, 5, 10, 15]),
    PI6: (pi6, 6, [3, 5, 10, 15]),
    PI7: (pi7, 7, [3, 10, 15]),
    PI8: (pi8, 8, [5, 10, 15]),
    PI9: (pi9, 9, [5, 9, 15]),
    PI10: (pi10, 10, [5, 15]),
    PI11: (pi11, 11, [5, 15]),
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
            #[cfg(not(feature = "stm32l471"))] // missing PAC support for Port F
            #[cfg(any(
                feature = "gpio-l47x",
                feature = "gpio-l49x",
                feature = "gpio-l4p",
                feature = "gpio-l4rx",
            ))]
            'F' => crate::pac::GPIOF::ptr() as _,
            #[cfg(not(feature = "stm32l471"))] // missing PAC support for Port G
            #[cfg(any(
                feature = "gpio-l47x",
                feature = "gpio-l49x",
                feature = "gpio-l4p",
                feature = "gpio-l4rx",
            ))]
            'G' => crate::pac::GPIOG::ptr() as _,
            #[cfg(not(feature = "stm32l471"))] // missing PAC support for Port G
            'H' => crate::pac::GPIOH::ptr() as _,
            #[cfg(any(feature = "gpio-l49x", feature = "gpio-l4p", feature = "gpio-l4rx",))]
            'I' => crate::pac::GPIOI::ptr() as _,
            _ => core::ptr::null(),
        }
    }
}

// Make all GPIO peripheral trait extensions sealable.
impl<const P: char, const N: u8, MODE> crate::Sealed for Pin<P, N, MODE> {}
