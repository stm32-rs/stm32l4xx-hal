use crate::gpio::gpioa::{PA0, PA1};
use crate::gpio::{Analog, Floating, Input};
use crate::rcc::AHB2;
use core::marker::PhantomData;
use embedded_hal::adc::{Channel, OneShot};

#[cfg(feature = "stm32l4x2")]
use stm32l4::stm32l4x2::{
    rcc::CCIPR,
    ADC
};
#[cfg(feature = "stm32l4x6")]
use stm32l4::stm32l4x6::{
    rcc::CCIPR,
    ADC
};

#[cfg(feature = "stm32l4x2",feature = "stm32l4x6")]
pub struct Adc<ADC> {
    adc: ADC,
}

#[cfg(feature = "stm32l4x2",feature = "stm32l4x6")]
impl Adc<crate::device::ADC> {
    pub fn adc1(adc: crate::device::ADC, ahb2: &mut AHB2) -> Self {
        // Select system clock as clock for ADC
        let rcc = unsafe { &*crate::device::RCC::ptr() };
        rcc.ccipr.modify(|r, w| unsafe { w.adcsel().bits(0b11) });
        //        common.ccr.modify(|r, w| unsafe { w.ckmode().bits(0b11) });
        //        common.ccr.modify(|r, w| unsafe { w.presc().bits(0b1011) });
        ahb2.enr().modify(|r, w| w.adcen().set_bit());
        //        common.ccr.modify(|r, w| w.vrefen().set_bit());

        // Disable deep power down and start ADC voltage regulator
        adc.cr.modify(|r, w| w.deeppwd().clear_bit());
        adc.cr.modify(|r, w| w.advregen().set_bit());
        cortex_m::asm::delay(8_000_000);

        // Calibrate
        adc.cr.modify(|r, w| w.adcaldif().clear_bit());
        adc.cr.modify(|r, w| w.adcal().set_bit());

        while adc.cr.read().adcal().bit_is_set() {}

        Self { adc }
    }

    pub fn power_up(&mut self) {
        self.adc.isr.modify(|_, w| w.adrdy().set_bit());
        self.adc.cr.modify(|_, w| w.aden().set_bit());
        while self.adc.isr.read().adrdy().bit_is_clear() {}
    }

    pub fn power_down(&mut self) {
        self.adc.cr.modify(|_, w| w.addis().set_bit());
        self.adc.isr.modify(|_, w| w.adrdy().set_bit());
        while self.adc.cr.read().aden().bit_is_set() {}
    }
}

#[cfg(feature = "stm32l4x2",feature = "stm32l4x6")]
impl Channel<Adc<crate::device::ADC>> for PA0<Analog> {
    type ID = u8;

    fn channel() -> u8 {
        5
    }
}

#[cfg(feature = "stm32l4x2",feature = "stm32l4x6")]
impl Channel<Adc<crate::device::ADC>> for PA1<Analog> {
    type ID = u8;

    fn channel() -> u8 {
        6
    }
}

#[cfg(feature = "stm32l4x2",feature = "stm32l4x6")]
impl<WORD, PIN> OneShot<Adc<crate::device::ADC>, WORD, PIN> for Adc<crate::stm32::ADC>
where
    WORD: From<u16>,
    PIN: Channel<Adc<crate::device::ADC>, ID = u8>,
{
    type Error = ();

    fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        self.power_up();
        self.adc.cfgr.modify(|r, w| unsafe { w.exten().bits(0b00) });
        self.adc
            .cfgr
            .modify(|r, w| unsafe { w.align().clear_bit().res().bits(0b00).cont().clear_bit() });
        self.adc
            .sqr1
            .modify(|r, w| unsafe { w.sq1().bits(PIN::channel()) });
        self.adc
            .cfgr2
            .modify(|r, w| unsafe { w.rovse().set_bit().ovsr().bits(0b011) });
        self.adc.isr.modify(|r, w| w.eoc().set_bit());
        self.adc.cr.modify(|r, w| w.adstart().set_bit());

        //        cortex_m::asm::delay(80_000_000);
        while self.adc.isr.read().eoc().bit_is_clear() {}

        let val = self.adc.dr.read().regular_data().bits().into();
        self.power_down();
        Ok(val)
    }
}