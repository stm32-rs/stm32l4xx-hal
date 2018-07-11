//! General Purpose Input / Output

// TODO the pins here currently correspond to the LQFP-48 package. There should be Cargo features
// that let you select different microcontroller packages

use core::marker::PhantomData;

use hal::prelude::*;

use rcc::AHB2;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, apb2: &mut AHB2) -> Self::Parts;
}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;
/// Open drain output (type state)
pub struct OpenDrain;

/// Alternate function
pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $gpioy:ident, $iopxenr:ident, $iopxrst:ident, $PXx:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $CR:ident),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::marker::PhantomData;

            use hal::digital::{InputPin, OutputPin, StatefulOutputPin, toggleable};
            use stm32l4::stm32l4x2::{$gpioy, $GPIOX};

            use rcc::AHB2;

            use super::{
                Alternate, Floating, GpioExt, Input,
                // OpenDrain,
                Output,
                // PullDown, PullUp,
                PushPull,
            };

            /// GPIO parts
            pub struct Parts {
                /// Opaque AFRL register
                pub afrl: AFRL,
                /// Opaque AFRH register
                pub afrh: AFRH,
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, ahb2: &mut AHB2) -> Parts {
                    ahb2.enr().modify(|_, w| w.$iopxenr().set_bit());
                    ahb2.rstr().modify(|_, w| w.$iopxrst().set_bit());
                    ahb2.rstr().modify(|_, w| w.$iopxrst().clear_bit());

                    Parts {
                        afrl: AFRL { _0: () },
                        afrh: AFRH { _0: () },
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }
            
            // Opaque CRL register
            pub struct AFRL {
                _0: (),
            }

            impl AFRL {
                // NOTE(allow) we get a warning on GPIOC because it only has 3 high pins
                #[allow(dead_code)]
                pub(crate) fn cr(&mut self) -> &$gpioy::AFRL {
                    unsafe { &(*$GPIOX::ptr()).afrl }
                }
            }

            /// Opaque CRH register
            pub struct AFRH {
                _0: (),
            }

            impl AFRH {
                pub(crate) fn cr(&mut self) -> &$gpioy::AFRH {
                    unsafe { &(*$GPIOX::ptr()).afrh }
                }
            }

            /// Partially erased pin
            pub struct $PXx<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> OutputPin for $PXx<Output<MODE>> {
                fn set_high(&mut self) {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)) }
                }

                fn set_low(&mut self) {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + self.i))) }
                }
            }

            impl <MODE> StatefulOutputPin for $PXx<Output<MODE>> {
                fn is_set_high(&self) -> bool {
                    !self.is_set_low()
                }

                fn is_set_low(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << self.i) == 0 }
                }
            }

            impl <MODE> toggleable::Default for $PXx<Output<MODE>> {}

            $(
                /// Pin
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> $PXi<MODE> {
                    /// Configures the pin to operate as an alternate function push pull output pin
                    pub fn into_alternate_push_pull(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Alternate<PushPull>> {
                        let offset = (4 * $i) % 32;
                        // Alternate function output push pull
                        let cnf = 0b10;
                        // Output mode, max speed 50 MHz
                        let mode = 0b11;
                        let bits = (cnf << 2) | mode;

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << offset)) | (bits << offset))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a floating input pin
                    pub fn into_floating_input(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Input<Floating>> {
                        let offset = (4 * $i) % 32;
                        // Floating input
                        let cnf = 0b01;
                        // Input mode
                        let mode = 0b00;
                        let bits = (cnf << 2) | mode;

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << offset)) | (bits << offset))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    // /// Configures the pin to operate as a pulled down input pin
                    // pub fn into_pull_down_input(
                    //     self,
                    //     moder: &mut MODER,
                    //     pupdr: &mut PUPDR,
                    // ) -> $PXi<Input<PullDown>> {
                    //     let offset = 2 * $i;

                    //     // input mode
                    //     moder
                    //         .moder()
                    //         .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << offset)) });

                    //     // pull-down
                    //     pupdr.pupdr().modify(|r, w| unsafe {
                    //         w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                    //     });

                    //     $PXi { _mode: PhantomData }
                    // }

                    // /// Configures the pin to operate as a pulled up input pin
                    // pub fn into_pull_up_input(
                    //     self,
                    //     moder: &mut MODER,
                    //     pupdr: &mut PUPDR,
                    // ) -> $PXi<Input<PullUp>> {
                    //     let offset = 2 * $i;

                    //     // input mode
                    //     moder
                    //         .moder()
                    //         .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << offset)) });

                    //     // pull-up
                    //     pupdr.pupdr().modify(|r, w| unsafe {
                    //         w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                    //     });

                    //     $PXi { _mode: PhantomData }
                    // }

                    // /// Configures the pin to operate as an open drain output pin
                    // pub fn into_open_drain_output(
                    //     self,
                    //     moder: &mut MODER,
                    //     otyper: &mut OTYPER,
                    // ) -> $PXi<Output<OpenDrain>> {
                    //     let offset = 2 * $i;

                    //     // general purpose output mode
                    //     let mode = 0b01;
                    //     moder.moder().modify(|r, w| unsafe {
                    //         w.bits((r.bits() & !(0b11 << offset)) | (mode << offset))
                    //     });

                    //     // open drain output
                    //     otyper
                    //         .otyper()
                    //         .modify(|r, w| unsafe { w.bits(r.bits() | (0b1 << $i)) });

                    //     $PXi { _mode: PhantomData }
                    // }

                    /// Configures the pin to operate as an push pull output pin
                    pub fn into_push_pull_output(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Output<PushPull>> {
                        let offset = (4 * $i) % 32;
                        // General purpose output push-pull
                        let cnf = 0b00;
                        // Output mode, max speed 50 MHz
                        let mode = 0b11;
                        let bits = (cnf << 2) | mode;

                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << offset)) | (bits << offset))
                            });

                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<Output<MODE>> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> $PXx<Output<MODE>> {
                        $PXx {
                            i: $i,
                            _mode: self._mode,
                        }
                    }
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    fn set_high(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) }
                    }

                    fn set_low(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i))) }
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&self) -> bool {
                        !self.is_set_low()
                    }

                    fn is_set_low(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << $i) == 0 }
                    }
                }

                impl <MODE> toggleable::Default for $PXi<Output<MODE>> {}

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    fn is_high(&self) -> bool {
                        !self.is_low()
                    }

                    fn is_low(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 }
                    }
                }
            )+
        }
    }
}

gpio!(GPIOA, gpioa, gpioa, gpioaen, gpioarst, PAx, [
    PA0: (pa0, 0, Input<Floating>, AFRL),
    PA1: (pa1, 1, Input<Floating>, AFRL),
    PA2: (pa2, 2, Input<Floating>, AFRL),
    PA3: (pa3, 3, Input<Floating>, AFRL),
    PA4: (pa4, 4, Input<Floating>, AFRL),
    PA5: (pa5, 5, Input<Floating>, AFRL),
    PA6: (pa6, 6, Input<Floating>, AFRL),
    PA7: (pa7, 7, Input<Floating>, AFRL),
    PA8: (pa8, 8, Input<Floating>, AFRH),
    PA9: (pa9, 9, Input<Floating>, AFRH),
    PA10: (pa10, 10, Input<Floating>, AFRH),
    PA11: (pa11, 11, Input<Floating>, AFRH),
    PA12: (pa12, 12, Input<Floating>, AFRH),
    PA13: (pa13, 13, Input<Floating>, AFRH),
    PA14: (pa14, 14, Input<Floating>, AFRH),
    PA15: (pa15, 15, Input<Floating>, AFRH),
]);

gpio!(GPIOB, gpiob, gpiob, gpioben, gpiobrst, PBx, [
    PB0: (pb0, 0, Input<Floating>, AFRL),
    PB1: (pb1, 1, Input<Floating>, AFRL),
    PB2: (pb2, 2, Input<Floating>, AFRL),
    PB3: (pb3, 3, Input<Floating>, AFRL),
    PB4: (pb4, 4, Input<Floating>, AFRL),
    PB5: (pb5, 5, Input<Floating>, AFRL),
    PB6: (pb6, 6, Input<Floating>, AFRL),
    PB7: (pb7, 7, Input<Floating>, AFRL),
    PB8: (pb8, 8, Input<Floating>, AFRH),
    PB9: (pb9, 9, Input<Floating>, AFRH),
    PB10: (pb10, 10, Input<Floating>, AFRH),
    PB11: (pb11, 11, Input<Floating>, AFRH),
    PB12: (pb12, 12, Input<Floating>, AFRH),
    PB13: (pb13, 13, Input<Floating>, AFRH),
    PB14: (pb14, 14, Input<Floating>, AFRH),
    PB15: (pb15, 15, Input<Floating>, AFRH),
]);

gpio!(GPIOC, gpioc, gpioc, gpiocen, gpiocrst, PCx, [
    PC13: (pc13, 13, Input<Floating>, AFRH),
    PC14: (pc14, 14, Input<Floating>, AFRH),
    PC15: (pc15, 15, Input<Floating>, AFRH),
]);
