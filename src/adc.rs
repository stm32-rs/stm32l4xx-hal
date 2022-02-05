//! # Analog to Digital converter

use core::{
    convert::Infallible,
    ops::DerefMut,
    sync::atomic::{self, Ordering},
};

use crate::{
    dma::{dma1, Event as DMAEvent, RxDma, Transfer, TransferPayload, W},
    gpio::{self, Analog},
    hal::{
        adc::{Channel as EmbeddedHalChannel, OneShot},
        blocking::delay::DelayUs,
    },
    pac,
    rcc::{Enable, Reset, AHB2, CCIPR},
    signature::{VrefCal, VtempCalHigh, VtempCalLow, VDDA_CALIB_MV},
};

use pac::{ADC1, ADC_COMMON};
use stable_deref_trait::StableDeref;

/// Vref internal signal, used for calibration
pub struct Vref;

/// Vbat internal signal, used for monitoring the battery
pub struct Vbat;

/// Core temperature internal signal
pub struct Temperature;

/// Opamp1 output connected to ADC1
pub struct Opamp1Out;

/// Opamp1 output connected to ADC1
pub struct Opamp2Out;

/// Analog to Digital converter interface
pub struct ADC {
    pub(crate) adc: ADC1,
    common: ADC_COMMON,
    resolution: Resolution,
    sample_time: SampleTime,
    calibrated_vdda: u32,
}

#[derive(Copy, Clone, PartialEq)]
pub enum DmaMode {
    Disabled = 0,
    Oneshot = 1,
    // FIXME: Figure out how to get circular DMA to function properly (requires circbuffer?)
    // Circular = 2,
    ExtTrigger = 3,
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub enum Sequence {
    One = 0,
    Two = 1,
    Three = 2,
    Four = 3,
    Five = 4,
    Six = 5,
    Seven = 6,
    Eight = 7,
    Nine = 8,
    Ten = 9,
    Eleven = 10,
    Twelve = 11,
    Thirteen = 12,
    Fourteen = 13,
    Fifteen = 14,
    Sixteen = 15,
}

impl From<u8> for Sequence {
    fn from(bits: u8) -> Self {
        match bits {
            0 => Sequence::One,
            1 => Sequence::Two,
            2 => Sequence::Three,
            3 => Sequence::Four,
            4 => Sequence::Five,
            5 => Sequence::Six,
            6 => Sequence::Seven,
            7 => Sequence::Eight,
            8 => Sequence::Nine,
            9 => Sequence::Ten,
            10 => Sequence::Eleven,
            11 => Sequence::Twelve,
            12 => Sequence::Thirteen,
            13 => Sequence::Fourteen,
            14 => Sequence::Fifteen,
            15 => Sequence::Sixteen,
            _ => unimplemented!(),
        }
    }
}

impl Into<u8> for Sequence {
    fn into(self) -> u8 {
        match self {
            Sequence::One => 0,
            Sequence::Two => 1,
            Sequence::Three => 2,
            Sequence::Four => 3,
            Sequence::Five => 4,
            Sequence::Six => 5,
            Sequence::Seven => 6,
            Sequence::Eight => 7,
            Sequence::Nine => 8,
            Sequence::Ten => 9,
            Sequence::Eleven => 10,
            Sequence::Twelve => 11,
            Sequence::Thirteen => 12,
            Sequence::Fourteen => 13,
            Sequence::Fifteen => 14,
            Sequence::Sixteen => 15,
        }
    }
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub enum Jsequence {
    One = 0,
    Two = 1,
    Three = 2,
    Four = 3,
}

impl From<u8> for Jsequence {
    fn from(bits: u8) -> Self {
        match bits {
            0 => Jsequence::One,
            1 => Jsequence::Two,
            2 => Jsequence::Three,
            3 => Jsequence::Four,
            _ => unimplemented!(),
        }
    }
}

impl Into<u8> for Jsequence {
    fn into(self) -> u8 {
        match self {
            Jsequence::One => 0,
            Jsequence::Two => 1,
            Jsequence::Three => 2,
            Jsequence::Four => 3,
        }
    }
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub enum Event {
    EndOfRegularSequence,
    EndOfRegularConversion,
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
        ADC1::enable(ahb);

        // Reset peripheral
        ADC1::reset(ahb);

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
    pub fn enable_temperature(&mut self, delay: &mut impl DelayUs<u32>) -> Temperature {
        self.common.ccr.modify(|_, w| w.ch17sel().set_bit());

        // FIXME: This note from the reference manual is currently not possible
        // rm0351 section 18.4.32 pg580 (L47/L48/L49/L4A models)
        // Note:
        // The sensor has a startup time after waking from power-down mode before it can output VTS
        // at the correct level. The ADC also has a startup time after power-on, so to minimize the
        // delay, the ADEN and CH17SEL bits should be set at the same time.
        //
        // https://github.com/STMicroelectronics/STM32CubeL4/blob/master/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h#L1363
        // 120us is used in the ST HAL code
        delay.delay_us(150);

        Temperature {}
    }

    /// Enable and get the `Vbat`
    pub fn enable_vbat(&mut self) -> Vbat {
        self.common.ccr.modify(|_, w| w.ch18sel().set_bit());

        Vbat {}
    }

    /// Get the `Opamp1Out`
    pub fn enable_opamp1_out(&mut self) -> Opamp1Out {
        Opamp1Out {}
    }

    /// Get the `Opamp2Out`
    pub fn enable_opamp2_out(&mut self) -> Opamp2Out {
        Opamp2Out {}
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

        // Safety: DIV by 0 is possible if vref_samp is 0
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

    /// Get the max value for the current resolution
    pub fn get_max_value(&self) -> u16 {
        match self.resolution {
            Resolution::Bits12 => 4095,
            Resolution::Bits10 => 1023,
            Resolution::Bits8 => 255,
            Resolution::Bits6 => 63,
        }
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
        ((u32::from(sample) * self.calibrated_vdda) / self.resolution.tomax_count()) as u16
    }

    /// Convert a raw sample from the `Temperature` to deg C
    pub fn to_degrees_centigrade(&self, sample: u16) -> f32 {
        let sample = (u32::from(sample) * self.calibrated_vdda) / VDDA_CALIB_MV;
        (VtempCalHigh::TEMP_DEGREES - VtempCalLow::TEMP_DEGREES) as f32
            // as signed because RM0351 doesn't specify against this being an
            // inverse relation (which would result in a negative differential)
            / (VtempCalHigh::get().read() as i32 - VtempCalLow::get().read() as i32) as f32
            // this can definitely be negative so must be done as a signed value
            * (sample as i32 - VtempCalLow::get().read() as i32) as f32
            // while it would make sense for this to be `VtempCalLow::TEMP_DEGREES` (which is 30*C),
            // the RM specifically uses 30*C so this will too
            + 30.0
    }

    // DMA channels:
    //  ADC1: DMA2_3 with C2S 0000
    //  ADC2: DMA2_4 with C2S 0000
    //  ADC1: DMA1_1 with C1S 0000 (implemented)
    //  ADC2: DMA1_2 with C1S 0000

    pub fn get_data(&self) -> u16 {
        // Sound, as bits 31:16 are reserved, read-only and 0 in ADC_DR
        self.adc.dr.read().bits() as u16
    }

    /// Configure the channel for a specific step in the sequence.
    ///
    /// Automatically sets the sequence length to the farthes sequence
    /// index that has been used so far. Use [`ADC::reset_sequence`] to
    /// reset the sequence length.
    pub fn configure_sequence<C>(
        &mut self,
        channel: &mut C,
        sequence: Sequence,
        sample_time: SampleTime,
    ) where
        C: Channel,
    {
        let channel_bits = C::channel();
        channel.set_sample_time(&self.adc, sample_time);

        unsafe {
            // This is sound as channel() always returns a valid channel number
            match sequence {
                Sequence::One => self.adc.sqr1.modify(|_, w| w.sq1().bits(channel_bits)),
                Sequence::Two => self.adc.sqr1.modify(|_, w| w.sq2().bits(channel_bits)),
                Sequence::Three => self.adc.sqr1.modify(|_, w| w.sq3().bits(channel_bits)),
                Sequence::Four => self.adc.sqr1.modify(|_, w| w.sq4().bits(channel_bits)),
                Sequence::Five => self.adc.sqr2.modify(|_, w| w.sq5().bits(channel_bits)),
                Sequence::Six => self.adc.sqr2.modify(|_, w| w.sq6().bits(channel_bits)),
                Sequence::Seven => self.adc.sqr2.modify(|_, w| w.sq7().bits(channel_bits)),
                Sequence::Eight => self.adc.sqr2.modify(|_, w| w.sq8().bits(channel_bits)),
                Sequence::Nine => self.adc.sqr2.modify(|_, w| w.sq9().bits(channel_bits)),
                Sequence::Ten => self.adc.sqr3.modify(|_, w| w.sq10().bits(channel_bits)),
                Sequence::Eleven => self.adc.sqr3.modify(|_, w| w.sq11().bits(channel_bits)),
                Sequence::Twelve => self.adc.sqr3.modify(|_, w| w.sq12().bits(channel_bits)),
                Sequence::Thirteen => self.adc.sqr3.modify(|_, w| w.sq13().bits(channel_bits)),
                Sequence::Fourteen => self.adc.sqr3.modify(|_, w| w.sq14().bits(channel_bits)),
                Sequence::Fifteen => self.adc.sqr4.modify(|_, w| w.sq15().bits(channel_bits)),
                Sequence::Sixteen => self.adc.sqr4.modify(|_, w| w.sq16().bits(channel_bits)),
            }
        }

        // This will only ever extend the sequence, not shrink it.
        let current_seql = self.get_sequence_length();
        let next_seql: u8 = sequence.into();
        if next_seql >= current_seql {
            // Note: sequence length of 0 = 1 conversion
            self.set_sequence_length(sequence.into());
        }
    }

    /// Configure the channel for a specific step in the sequence.
    ///
    /// Automatically sets the sequence length to the farthes sequence
    /// index that has been used so far. Use [`ADC::reset_jsequence`] to
    /// reset the sequence length.
    pub fn configure_jsequence<C>(
        &mut self,
        channel: &mut C,
        sequence: Jsequence,
        sample_time: SampleTime,
    ) where
        C: Channel,
    {
        let channel_bits = C::channel();
        channel.set_sample_time(&self.adc, sample_time);

        unsafe {
            // This is sound as channel() always returns a valid channel number
            match sequence {
                Jsequence::One => self.adc.jsqr.modify(|_, w| w.jsq1().bits(channel_bits)),
                Jsequence::Two => self.adc.jsqr.modify(|_, w| w.jsq2().bits(channel_bits)),
                Jsequence::Three => self.adc.jsqr.modify(|_, w| w.jsq3().bits(channel_bits)),
                Jsequence::Four => self.adc.jsqr.modify(|_, w| w.jsq4().bits(channel_bits)),
            }
        }

        // This will only ever extend the sequence, not shrink it.
        let current_seql = self.get_jsequence_length();
        let next_seql: u8 = sequence.into();
        if next_seql >= current_seql {
            // Note: sequence length of 0 = 1 conversion
            self.set_jsequence_length(sequence.into());
        }
    }

    /// Get the configured sequence length (= `actual sequence length - 1`)
    pub(crate) fn get_jsequence_length(&self) -> u8 {
        self.adc.jsqr.read().jl().bits()
    }

    /// Private: length must be `actual sequence length - 1`, so not API-friendly.
    /// Use [`ADC::reset_sequence`] and [`ADC::configure_sequence`] instead
    fn set_jsequence_length(&mut self, length: u8) {
        self.adc.jsqr.modify(|_, w| unsafe { w.jl().bits(length) });
    }

    /// Get the configured sequence length (= `actual sequence length - 1`)
    pub(crate) fn get_sequence_length(&self) -> u8 {
        self.adc.sqr1.read().l().bits()
    }

    /// Private: length must be `actual sequence length - 1`, so not API-friendly.
    /// Use [`ADC::reset_sequence`] and [`ADC::configure_sequence`] instead
    fn set_sequence_length(&mut self, length: u8) {
        self.adc.sqr1.modify(|_, w| unsafe { w.l().bits(length) });
    }

    /// Reset the sequence length to 1
    ///
    /// Does *not* erase previously configured sequence settings, only
    /// changes the sequence length
    pub fn reset_sequence(&mut self) {
        self.adc.sqr1.modify(|_, w| unsafe { w.l().bits(0b0000) })
    }

    pub fn has_completed_conversion(&self) -> bool {
        self.adc.isr.read().eoc().bit_is_set()
    }

    pub fn has_completed_sequence(&self) -> bool {
        self.adc.isr.read().eos().bit_is_set()
    }

    pub fn clear_end_flags(&mut self) {
        // EOS and EOC are reset by setting them (See reference manual section 16.6.1)
        self.adc
            .isr
            .modify(|_, w| w.eos().set_bit().eoc().set_bit());
    }

    pub fn start_conversion(&mut self) {
        self.enable();
        self.clear_end_flags();
        self.adc.cr.modify(|_, w| w.adstart().set_bit());
    }

    pub fn is_converting(&self) -> bool {
        self.adc.cr.read().adstart().bit_is_set()
    }

    pub fn listen(&mut self, event: Event) {
        self.adc.ier.modify(|_, w| match event {
            Event::EndOfRegularSequence => w.eosie().set_bit(),
            Event::EndOfRegularConversion => w.eocie().set_bit(),
        });
    }

    pub fn unlisten(&mut self, event: Event) {
        self.adc.ier.modify(|_, w| match event {
            Event::EndOfRegularSequence => w.eosie().clear_bit(),
            Event::EndOfRegularConversion => w.eocie().clear_bit(),
        });
    }

    pub fn enable(&mut self) {
        if !self.is_enabled() {
            // Make sure bits are off
            while self.adc.cr.read().addis().bit_is_set() {}

            // Clear ADRDY by setting it (See Reference Manual section 1.16.1)
            self.adc.isr.modify(|_, w| w.adrdy().set_bit());
            self.adc.cr.modify(|_, w| w.aden().set_bit());
            while self.adc.isr.read().adrdy().bit_is_clear() {}

            // Configure ADC
            self.adc.cfgr.modify(|_, w| {
                // This is sound, as all `Resolution` values are valid for this
                // field.
                unsafe { w.res().bits(self.resolution as u8) }
            });
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.adc.cr.read().aden().bit_is_set()
    }

    pub fn disable(&mut self) {
        self.adc.cr.modify(|_, w| w.addis().set_bit());
    }

    /// channels for external triggers
    /// TIM1_CH1     0b0000
    /// TIM1_CH2     0b0001
    /// TIM1_CH3     0b0010
    /// TIM2_CH2     0b0011
    /// TIM3_TRGO    0b0100  (not on all chips available)
    /// EXTI line 11 0b0110
    /// TIM8_TRGO    0b0111
    /// TIM8_TRGO2   0b1000
    /// TIM1_TRGO    0b1001
    /// TIM1_TRGO2   0b1010
    /// TIM2_TRGO    0b1011
    /// TIM4_TRGO    0b1100
    /// TIM6_TRGO    0b1101
    /// TIM15_TRGO   0b1110
    /// TIM3_CH4     0b1111
    /// these bits are only allowed to be set, when ADC is disabled
    /// set edge of injected trigger by timers
    /// None              0b00
    /// Positive edge     0b01
    /// Negative edge     0b10
    /// both edges        0b11
    pub fn set_external_trigger(&mut self, ch: u8, edge: u8) {
        if self.adc.cr.read().adstart().bit_is_clear() {
            self.adc.cfgr.modify(|_, w| unsafe { w.extsel().bits(ch) });

            self.adc.cfgr.modify(|_, w| unsafe { w.exten().bits(edge) }); // 1 as u8
        }
    }

    /// channels for injection Transmitter
    /// TIM1_TRGO       0b0000
    /// TIM1_CH4        0b0001
    /// TIM2_TRGO       0b0010
    /// TIM2_CH1        0b0011
    /// TIM3_CH4        0b0100  (not on all chips available)
    /// EXTI line 15    0b0110
    /// TIM1_TRGO2      0b1000
    /// TIM6_TRGO       0b1110
    /// TIM15_TRGO      0b1111
    /// set edge of injected trigger by timers
    /// None              0b00
    /// Positive edge     0b01
    /// Negative edge     0b10
    /// both edges        0b11
    pub fn set_inject_channel(&mut self, ch: u8, edge: u8) {
        self.adc.jsqr.modify(|_, w| unsafe { w.jextsel().bits(ch) });

        self.adc
            .jsqr
            .modify(|_, w| unsafe { w.jexten().bits(edge) }); // 1 as u8
    }

    pub fn start_injected(&mut self) {
        if !self.is_injected_mod_enabled() {
            while self.adc.cr.read().addis().bit_is_set() {}
            self.adc.ier.modify(|_, w| w.jeocie().set_bit()); // end of sequence interupt enable
            self.adc.cr.modify(|_, w| w.aden().set_bit());
            self.adc.cr.modify(|_, w| w.jadstart().set_bit());
            // ADSTART bit is cleared to 0 bevor using this function
            while self.adc.isr.read().adrdy().bit_is_clear() {}
        }
    }

    pub fn start_injected_sequence(&mut self) {
        if !self.is_injected_mod_enabled() {
            while self.adc.cr.read().addis().bit_is_set() {}
            self.adc.ier.modify(|_, w| w.jeosie().set_bit()); // end of sequence interupt enable

            self.adc.cr.modify(|_, w| w.aden().set_bit());
            while self.adc.isr.read().adrdy().bit_is_clear() {}
            self.adc.cr.modify(|_, w| w.jadstart().set_bit());
            // ADSTART bit is cleared to 0 bevor using this function

            // clear end of sequencd conversion interrupt flag
        }
    }

    pub fn is_injected_mod_enabled(&self) -> bool {
        self.adc.cr.read().jadstart().bit_is_set()
    }

    pub fn stop_injected(&mut self) {
        // ?????? or is it reset after each conversion?
        self.adc.cr.modify(|_, w| w.jadstp().set_bit());
        // self.adc.cr.modify(|_, w| w.jadstp().set_bit());
        // ADSTART bit is cleared to 0 bevor using this function
        // disable EOS interrupt
        // maybe self.rb.cr.adstp().set_bit() must be performed before interrupt is disabled + wait abortion
        self.adc.ier.modify(|_, w| w.jeocie().clear_bit()); // end of sequence interupt disable
    }

    pub fn stop_injected_sequence(&mut self) {
        // ?????? or is it reset after each conversion?
        self.adc.cr.modify(|_, w| w.jadstp().set_bit());
        // self.adc.cr.modify(|_, w| w.jadstp().set_bit());
        // ADSTART bit is cleared to 0 bevor using this function
        // disable EOS interrupt
        // maybe self.rb.cr.adstp().set_bit() must be performed before interrupt is disabled + wait abortion
        self.adc.ier.modify(|_, w| w.jeosie().clear_bit()); // end of sequence interupt disable
    }

    pub fn get_injected_jdr(&mut self, jdrx: u8) -> u16 {
        match jdrx {
            1 => self.adc.jdr1.read().jdata1().bits() as u16,
            2 => self.adc.jdr2.read().jdata2().bits() as u16,
            3 => self.adc.jdr3.read().jdata3().bits() as u16,
            4 => self.adc.jdr4.read().jdata4().bits() as u16,
            _ => 0xffff_u16,
        }
    }

    /// Reset the jsequence length to 1
    ///
    /// Does *not* erase previously configured sequence settings, only
    /// changes the sequence length
    pub fn reset_jsequence(&mut self) {
        self.adc.jsqr.modify(|_, w| unsafe { w.jl().bits(0b00) })
    }

    /// clear jeos interrupt flag
    pub fn set_jeos(&mut self) {
        self.adc.isr.modify(|_, w| w.jeos().set_bit());
    }

    /// Oversampling of adc according to datasheet of stm32g0, when oversampling is enabled
    /// 000: 2x
    /// 001: 4x
    /// 010: 8x
    /// 011: 16x
    /// 100: 32x
    /// 101: 64x
    /// 110: 128x
    /// 111: 256x
    pub fn set_oversampling_ratio(&mut self, multyply: u8) {
        self.adc
            .cfgr2
            .modify(|_, w| unsafe { w.ovsr().bits(multyply) });
    }

    /// Oversampling shif configurations
    /// 0000: No shift
    /// 0001: Shift 1-bit
    /// 0010: Shift 2-bits
    /// 0011: Shift 3-bits
    /// 0100: Shift 4-bits
    /// 0101: Shift 5-bits
    /// 0110: Shift 6-bits
    /// 0111: Shift 7-bits
    /// 1000: Shift 8-bits
    /// Other codes reserved
    pub fn set_oversampling_shift(&mut self, nrbits: u8) {
        self.adc
            .cfgr2
            .modify(|_, w| unsafe { w.ovss().bits(nrbits) });
    }

    pub fn oversampling_enable(&mut self) {
        self.adc.cfgr2.modify(|_, w| w.rovse().set_bit());
    }

    pub fn inject_oversampling_enable(&mut self) {
        self.adc.cfgr2.modify(|_, w| w.jovse().set_bit());
    }
}

impl<C> OneShot<ADC, u16, C> for ADC
where
    C: Channel,
{
    type Error = Infallible;

    fn read(&mut self, channel: &mut C) -> nb::Result<u16, Self::Error> {
        self.configure_sequence(channel, Sequence::One, self.sample_time);

        self.start_conversion();
        while !self.has_completed_sequence() {}

        // Read ADC value first time and discard it, as per errata sheet.
        // The errata states that if we do conversions slower than 1 kHz, the
        // first read ADC value can be corrupted, so we discard it and measure again.
        let _ = self.get_data();

        self.start_conversion();
        while !self.has_completed_sequence() {}

        // Read ADC value
        let val = self.get_data();

        // Disable ADC
        self.disable();

        Ok(val)
    }
}

impl TransferPayload for RxDma<ADC, dma1::C1> {
    fn start(&mut self) {
        self.channel.start();
    }

    fn stop(&mut self) {
        self.channel.stop();
    }
}

impl RxDma<ADC, dma1::C1> {
    pub fn split(mut self) -> (ADC, dma1::C1) {
        self.stop();
        (self.payload, self.channel)
    }
}

impl<BUFFER, const N: usize> Transfer<W, BUFFER, RxDma<ADC, dma1::C1>>
where
    BUFFER: Sized + StableDeref<Target = [u16; N]> + DerefMut + 'static,
{
    pub fn from_adc_dma(
        dma: RxDma<ADC, dma1::C1>,
        buffer: BUFFER,
        dma_mode: DmaMode,
        transfer_complete_interrupt: bool,
    ) -> Self {
        let (adc, channel) = dma.split();
        Transfer::from_adc(adc, channel, buffer, dma_mode, transfer_complete_interrupt)
    }

    /// Initiate a new DMA transfer from an ADC.
    ///
    /// `dma_mode` indicates the desired mode for DMA.
    ///
    /// If `transfer_complete_interrupt` is true, the transfer
    /// complete interrupt (= `DMA1_CH1`) will be enabled
    pub fn from_adc(
        mut adc: ADC,
        mut channel: dma1::C1,
        buffer: BUFFER,
        dma_mode: DmaMode,
        transfer_complete_interrupt: bool,
    ) -> Self {
        assert!(dma_mode != DmaMode::Disabled);

        let (enable, circular, exttrig) = match dma_mode {
            DmaMode::Disabled => (false, false, false),
            DmaMode::Oneshot => (true, false, false),
            DmaMode::ExtTrigger => (true, false, true),
        };

        adc.adc
            .cfgr
            .modify(|_, w| w.dmaen().bit(enable).dmacfg().bit(circular));

        channel.set_peripheral_address(&adc.adc.dr as *const _ as u32, false);

        // SAFETY: since the length of BUFFER is known to be `N`, we are allowed
        // to perform N transfers into said buffer
        channel.set_memory_address(buffer.as_ptr() as u32, true);
        channel.set_transfer_length(N as u16); // N is buffer legnth

        channel.cselr().modify(|_, w| w.c1s().bits(0b0000));

        channel.ccr().modify(|_, w| unsafe {
            w.mem2mem()
                // clear memory to memory mode
                .clear_bit()
                // dma priority: 00: Low, 01: Medium, 10: High, 11: Very high
                .pl()
                .bits(0b01)
                // dma transfer size: 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                .msize()
                .bits(0b01)
                // dma periferal transer size: 00: 8-bits, 01: 16-bits, 10: 32-bits, 11: Reserved
                .psize()
                .bits(0b01)
                // Peripheral -> Mem
                .dir()
                .clear_bit()
                // memory increment mode
                .minc()
                .clear_bit()
                // peripheral increment mode
                .pinc()
                .clear_bit()
                // set circual mode as defined in DMAmode
                .circ()
                .bit(circular)
        });

        // read ADC sequence length
        let sequence_length = adc.adc.sqr1.read().l().bits();
        // buffer increment bit in memory, if dma with several sequential reads are performed
        if sequence_length > 0 && exttrig {
            // memory increment mode
            channel.ccr().modify(|_, w| w.minc().set_bit());
        }

        if transfer_complete_interrupt {
            channel.listen(DMAEvent::TransferComplete);
        }

        atomic::compiler_fence(Ordering::Release);

        channel.start();

        adc.start_conversion();

        Transfer::w(
            buffer,
            RxDma {
                channel,
                payload: adc,
            },
        )
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
    fn tomax_count(&self) -> u32 {
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

// smprx : sample time main register
// smpx  : smple tim channel value register

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
    8,  Opamp1Out,  smpr1, smp8;
    15, Opamp2Out,  smpr2, smp15;
);
