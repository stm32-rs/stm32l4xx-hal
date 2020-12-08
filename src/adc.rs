//! # Analog to Digital converter

use core::convert::Infallible;

use crate::{
    gpio::Analog,
    hal::{
        adc::{Channel, OneShot},
        blocking::delay::DelayUs,
    },
    pac,
    rcc::{AHB2, CCIPR},
};

/// Analog to Digital converter interface
pub struct ADC {
    inner: pac::ADC,
}

impl ADC {
    /// Initialize the ADC
    pub fn new(
        inner: pac::ADC,
        ahb: &mut AHB2,
        ccipr: &mut CCIPR,
        delay: &mut impl DelayUs<u32>,
    ) -> Self {
        // Reset peripheral
        ahb.rstr().modify(|_, w| w.adcrst().set_bit());
        ahb.rstr().modify(|_, w| w.adcrst().clear_bit());

        // Select system clock as ADC clock source
        ccipr.ccipr().modify(|_, w| {
            // This is sound, as `0b11` is a valid value for this field.
            unsafe {
                w.adcsel().bits(0b11);
            }

            w
        });

        // Enable peripheral
        ahb.enr().modify(|_, w| w.adcen().set_bit());

        // Initialize the ADC, according to the STM32L4xx Reference Manual,
        // section 16.4.6.
        inner.cr.write(|w| {
            w.deeppwd().clear_bit(); // exit deep-power-down mode
            w.advregen().set_bit(); // enable internal voltage regulator

            w
        });

        // According to the STM32L4xx Reference Manual, section 16.4.6, we need
        // to wait for T_ADCVREG_STUP after enabling the internal voltage
        // regulator. For the STM32L433, this is 20 us.
        delay.delay_us(20);

        // Calibration procedure according to section 16.4.8.
        inner.cr.modify(|_, w| {
            w.adcal().set_bit(); // start calibration
            w.adcaldif().clear_bit(); // single-ended mode

            w
        });
        while inner.cr.read().adcal().bit_is_set() {}

        Self { inner }
    }

    /// Release the ADC peripheral
    ///
    /// Drops `ADC` and returns the `pac::ADC` that is was wrapping, giving the
    /// user full access to the peripheral.
    pub fn release(self) -> pac::ADC {
        self.inner
    }
}

impl<C> OneShot<ADC, u16, C> for ADC
where
    C: Channel<ADC, ID = u8>,
{
    type Error = Infallible;

    fn read(&mut self, _: &mut C) -> nb::Result<u16, Self::Error> {
        // Enable ADC
        self.inner.isr.write(|w| w.adrdy().set_bit());
        self.inner.cr.modify(|_, w| w.aden().set_bit());
        while self.inner.isr.read().adrdy().bit_is_clear() {}

        // Select channel
        self.inner.sqr1.write(|w| {
            // This is sound, as all `Channel` implementations set valid values.
            unsafe {
                w.sq1().bits(C::channel());
            }

            w
        });

        // Start conversion
        self.inner.isr.modify(|_, w| w.eos().set_bit());
        self.inner.cr.modify(|_, w| w.adstart().set_bit());
        while self.inner.isr.read().eos().bit_is_clear() {}

        // Read ADC value
        let val = self.inner.dr.read().bits() as u16;

        // Disable ADC
        self.inner.cr.modify(|_, w| w.addis().set_bit());

        Ok(val)
    }
}

macro_rules! external_channels {
    (
        $(
            $id:expr,
            $pin:ident;
        )*
    ) => {
        $(
            impl Channel<ADC> for crate::gpio::$pin<Analog> {
                type ID = u8;

                fn channel() -> Self::ID {
                    $id
                }
            }
        )*
    };
}

external_channels!(
    1,  PC0;
    2,  PC1;
    3,  PC2;
    4,  PC3;
    5,  PA0;
    6,  PA1;
    7,  PA2;
    8,  PA3;
    9,  PA4;
    10, PA5;
    11, PA6;
    12, PA7;
    13, PC4;
    14, PC5;
    15, PB0;
    16, PB1;
);
