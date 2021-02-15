//! # Analog to Digital converter

use core::convert::Infallible;
use core::ptr;

use crate::{
    gpio::{self, Analog},
    hal::{
        adc::{Channel as EmbeddedHalChannel, OneShot},
        blocking::delay::DelayUs,
    },
    pac,
    rcc::{AHB2, CCIPR},
    signature::{VrefCal, VtempCal130, VtempCal30, VDDA_CALIB_MV},
};

use pac::{ADC1, ADC_COMMON};

/// Vref internal signal, used for calibration
pub struct Vref;

/// Vbat internal signal, used for monitoring the battery
pub struct Vbat;

/// Core temperature internal signal
pub struct Temperature;

/// Analog to Digital converter interface
pub struct ADC {
    adc: ADC1,
    common: ADC_COMMON,
    resolution: Resolution,
    sample_time: SampleTime,
    calibrated_vdda: u32,
}

impl ADC {
    /// Initialize the ADC
    pub fn new(
        adc: ADC1,
        common: ADC_COMMON,
        ahb: &mut AHB2,
        ccipr: &mut CCIPR,
        delay: &mut impl DelayUs<u32>,
    ) -> Self {
        // Enable peripheral
        ahb.enr().modify(|_, w| w.adcen().set_bit());

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

        // Initialize the ADC, according to the STM32L4xx Reference Manual,
        // section 16.4.6.
        adc.cr.write(|w| w.deeppwd().clear_bit()); // exit deep-power-down mode
        adc.cr.modify(|_, w| w.advregen().set_bit()); // enable internal voltage regulator

        // According to the STM32L4xx Reference Manual, section 16.4.6, we need
        // to wait for T_ADCVREG_STUP after enabling the internal voltage
        // regulator. For the STM32L433, this is 20 us. We choose 25 us to
        // account for bad clocks.
        delay.delay_us(25);

        // Calibration procedure according to section 16.4.8.
        adc.cr.modify(|_, w| {
            w.adcal().set_bit(); // start calibration
            w.adcaldif().clear_bit(); // single-ended mode

            w
        });

        while adc.cr.read().adcal().bit_is_set() {}

        // We need to wait 4 ADC clock after ADCAL goes low, 1 us is more than enough
        delay.delay_us(1);

        let mut s = Self {
            adc,
            common,
            resolution: Resolution::default(),
            sample_time: SampleTime::default(),
            calibrated_vdda: VDDA_CALIB_MV,
        };

        // Temporarily enable Vref
        let mut vref = s.enable_vref(delay);

        s.calibrate(&mut vref);

        s.common.ccr.modify(|_, w| w.vrefen().clear_bit());

        s
    }

    /// Enable and get the `Vref`
    pub fn enable_vref(&mut self, delay: &mut impl DelayUs<u32>) -> Vref {
        self.common.ccr.modify(|_, w| w.vrefen().set_bit());

        // "Table 24. Embedded internal voltage reference" states that it takes a maximum of 12 us
        // to stabilize the internal voltage reference, we wait a little more.
        delay.delay_us(15);

        Vref {}
    }

    /// Enable and get the `Temperature`
    pub fn enable_temperature(&mut self) -> Temperature {
        self.common.ccr.modify(|_, w| w.ch17sel().set_bit());

        Temperature {}
    }

    /// Enable and get the `Vbat`
    pub fn enable_vbat(&mut self) -> Vbat {
        self.common.ccr.modify(|_, w| w.ch18sel().set_bit());

        Vbat {}
    }

    /// Calculates the system VDDA by sampling the internal VREF channel and comparing
    /// the result with the value stored at the factory. If the chip's VDDA is not stable, run
    /// this before each ADC conversion.
    pub fn calibrate(&mut self, vref: &mut Vref) {
        let vref_cal = VrefCal::get().read();
        let old_sample_time = self.sample_time;

        // "Table 24. Embedded internal voltage reference" states that the sample time needs to be
        // at a minimum 4 us. With 640.5 ADC cycles we have a minimum of 8 us at 80 MHz, leaving
        // some headroom.
        self.set_sample_time(SampleTime::Cycles640_5);

        // This can't actually fail, it's just in a result to satisfy hal trait
        let vref_samp = self.read(vref).unwrap();

        self.set_sample_time(old_sample_time);

        self.calibrated_vdda = (VDDA_CALIB_MV * u32::from(vref_cal)) / u32::from(vref_samp);
    }

    /// Set the ADC resolution
    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.resolution = resolution;
    }

    /// Set the sample time
    pub fn set_sample_time(&mut self, sample_time: SampleTime) {
        self.sample_time = sample_time;
    }

    /// Release the ADC peripheral
    ///
    /// Drops `ADC` and returns the `(pac::ADC, pad::ADC_COMMON)` that is was wrapping, giving the
    /// user full access to the peripheral.
    pub fn release(self) -> (ADC1, ADC_COMMON) {
        (self.adc, self.common)
    }

    /// Convert a measurement to millivolts
    pub fn to_millivolts(&self, sample: u16) -> u16 {
        ((u32::from(sample) * self.calibrated_vdda) / self.resolution.to_max_count()) as u16
    }

    /// Convert a raw sample from the `Temperature` to deg C
    pub fn to_degrees_centigrade(sample: u16) -> f32 {
        (130.0 - 30.0) / (VtempCal130::get().read() as f32 - VtempCal30::get().read() as f32)
            * (sample as f32 - VtempCal30::get().read() as f32)
            + 30.0
    }
}

impl<C> OneShot<ADC, u16, C> for ADC
where
    C: Channel,
{
    type Error = Infallible;

    fn read(&mut self, channel: &mut C) -> nb::Result<u16, Self::Error> {
        // Make sure bits are off
        while self.adc.cr.read().addis().bit_is_set() {}

        // Enable ADC
        self.adc.isr.write(|w| w.adrdy().set_bit());
        self.adc.cr.modify(|_, w| w.aden().set_bit());
        while self.adc.isr.read().adrdy().bit_is_clear() {}

        // Configure ADC
        self.adc.cfgr.write(|w| {
            // This is sound, as all `Resolution` values are valid for this
            // field.
            unsafe { w.res().bits(self.resolution as u8) }
        });

        // Configure channel
        channel.set_sample_time(&self.adc, self.sample_time);

        // Select channel
        self.adc.sqr1.write(|w| {
            // This is sound, as all `Channel` implementations set valid values.
            unsafe {
                w.sq1().bits(C::channel());
            }

            w
        });

        // Start conversion
        self.adc
            .isr
            .modify(|_, w| w.eos().set_bit().eoc().set_bit());
        self.adc.cr.modify(|_, w| w.adstart().set_bit());
        while self.adc.isr.read().eos().bit_is_clear() {}

        // Read ADC value first time and discard it, as per errata sheet.
        // The errata states that if we do conversions slower than 1 kHz, the
        // first read ADC value can be corrupted, so we discard it and measure again.
        let _ = unsafe { ptr::read_volatile(&self.adc.dr.read().bits()) };

        self.adc
            .isr
            .modify(|_, w| w.eos().set_bit().eoc().set_bit());
        self.adc.cr.modify(|_, w| w.adstart().set_bit());
        while self.adc.isr.read().eos().bit_is_clear() {}

        // Read ADC value
        let val = self.adc.dr.read().bits() as u16;

        // Disable ADC
        self.adc.cr.modify(|_, w| w.addis().set_bit());

        Ok(val)
    }
}

/// ADC resolution setting
///
/// The default setting is 12 bits.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Resolution {
    /// 12-bit resolution
    Bits12 = 0b00,

    /// 10-bit resolution
    Bits10 = 0b01,

    /// 8-bit resolution
    Bits8 = 0b10,

    /// 6-bit resolution
    Bits6 = 0b11,
}

impl Default for Resolution {
    fn default() -> Self {
        Self::Bits12
    }
}

impl Resolution {
    fn to_max_count(&self) -> u32 {
        match self {
            Resolution::Bits12 => (1 << 12) - 1,
            Resolution::Bits10 => (1 << 10) - 1,
            Resolution::Bits8 => (1 << 8) - 1,
            Resolution::Bits6 => (1 << 6) - 1,
        }
    }
}

/// ADC sample time
///
/// The default setting is 2.5 ADC clock cycles.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum SampleTime {
    /// 2.5 ADC clock cycles
    Cycles2_5 = 0b000,

    /// 6.5 ADC clock cycles
    Cycles6_5 = 0b001,

    /// 12.5 ADC clock cycles
    Cycles12_5 = 0b010,

    /// 24.5 ADC clock cycles
    Cycles24_5 = 0b011,

    /// 47.5 ADC clock cycles
    Cycles47_5 = 0b100,

    /// 92.5 ADC clock cycles
    Cycles92_5 = 0b101,

    /// 247.5 ADC clock cycles
    Cycles247_5 = 0b110,

    /// 640.5 ADC clock cycles
    Cycles640_5 = 0b111,
}

impl Default for SampleTime {
    fn default() -> Self {
        Self::Cycles2_5
    }
}

/// Implemented for all types that represent ADC channels
pub trait Channel: EmbeddedHalChannel<ADC, ID = u8> {
    fn set_sample_time(&mut self, adc: &ADC1, sample_time: SampleTime);
}

macro_rules! adc_pins {
    (
        $(
            $id:expr,
            $pin:ty,
            $smpr:ident,
            $smp:ident;
        )*
    ) => {
        $(
            impl EmbeddedHalChannel<ADC> for $pin {
                type ID = u8;

                fn channel() -> Self::ID {
                    $id
                }
            }

            impl Channel for $pin {
                fn set_sample_time(&mut self,
                    adc: &ADC1,
                    sample_time: SampleTime,
                ) {
                    adc.$smpr.modify(|_, w| {
                        // This is sound, as all `SampleTime` values are valid
                        // for this field.
                        unsafe {
                            w.$smp().bits(sample_time as u8)
                        }
                    })
                }
            }
        )*
    };
}

adc_pins!(
    0,  Vref,              smpr1, smp0;
    1,  gpio::PC0<Analog>, smpr1, smp1;
    2,  gpio::PC1<Analog>, smpr1, smp2;
    3,  gpio::PC2<Analog>, smpr1, smp3;
    4,  gpio::PC3<Analog>, smpr1, smp4;
    5,  gpio::PA0<Analog>, smpr1, smp5;
    6,  gpio::PA1<Analog>, smpr1, smp6;
    7,  gpio::PA2<Analog>, smpr1, smp7;
    8,  gpio::PA3<Analog>, smpr1, smp8;
    9,  gpio::PA4<Analog>, smpr1, smp9;
    10, gpio::PA5<Analog>, smpr2, smp10;
    11, gpio::PA6<Analog>, smpr2, smp11;
    12, gpio::PA7<Analog>, smpr2, smp12;
    13, gpio::PC4<Analog>, smpr2, smp13;
    14, gpio::PC5<Analog>, smpr2, smp14;
    15, gpio::PB0<Analog>, smpr2, smp15;
    16, gpio::PB1<Analog>, smpr2, smp16;
    17, Temperature,       smpr2, smp17;
    18, Vbat,              smpr2, smp18;
);
