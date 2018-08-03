/// RTC peripheral abstraction

use datetime::*;
use rcc::{BDCR, Clocks};
use stm32l4::stm32l4x2::{RTC, rtc};

pub struct Time {
    seconds: Seconds,
}

impl Time {
    /// Gets the current time from the RTC 
    pub fn get_time(/* Some RTC object */) -> Self {
        unimplemented!()
    }

    /// Set the RTC time
    pub fn set_time(_time: Time) { // TODO return result? 

    }
}

/// RTC Abstraction
pub struct Rtc {

}

impl Rtc {
    pub fn init(self, bdcr: &mut BDCR){
        bdcr.enr().modify(|_, w| unsafe {
            w.rtcsel()
                /* 
                    00: No clock
                    01: LSE oscillator clock used as RTC clock
                    10: LSI oscillator clock used as RTC clock
                    11: HSE oscillator clock divided by 32 used as RTC clock 
                */
                .bits(0b10)
                .rtcen()
                .set_bit()
        });

        // BDCR
    }

    fn enable_write_protection(self, enable: bool){
        // TODO
        // RTC.wpr.write(|w| {

        // });
    }
}