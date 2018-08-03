/// RTC peripheral abstraction

use datetime::*;
use rcc::{BDCR, Clocks};
use stm32l4::stm32l4x2::{RTC, rtc};

pub struct Time {
    seconds: Seconds,
    minutes: Minutes,
    hours: Hours,
    daylight_savings: bool
}

/// RTC Abstraction
pub struct Rtc {
    rtc: RTC
}

impl Rtc {
    pub fn init(&self, rtc: RTC, bdcr: &mut BDCR) -> Self {
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

        self.write_protection(&rtc, false);
        {
            self.init_mode(&rtc, true);
            {
                rtc.cr.modify(|_, w| unsafe {
                    w.fmt()
                        .clear_bit() // 24hr
                        .osel()
                        /* 
                            00: Output disabled
                            01: Alarm A output enabled
                            10: Alarm B output enabled
                            11: Wakeup output enabled 
                        */
                        .bits(0b00)
                        .pol()
                        .clear_bit() // pol high
                });
                
                rtc.prer.modify(|_, w| unsafe {
                    w.prediv_s()
                        .bits(255)
                        .prediv_a()
                        .bits(127)
                });
            }
            self.init_mode(&rtc, false);

            rtc.or.modify(|_, w| {
                w.rtc_alarm_type()
                    .clear_bit()
                    .rtc_out_rmp()
                    .clear_bit()
            });
            
        }
        self.write_protection(&rtc, true);

        Self {
            rtc: rtc
        }
    }

    fn init_mode(&self, rtc: &RTC, enabled: bool) {
        if enabled {
            rtc.isr.write(|w| unsafe { w.bits(0xFFFFFFFF) }); // Sets init mode
            while rtc.isr.read().initf().bit_is_clear() {} // wait to return to init state
        } else {
            rtc.isr.write(|w| { w.init().clear_bit() }); // Exits init mode
        }
        
    }

    pub fn set_time(&self, time: Time){
        self.write_protection(&self.rtc, false);
        {
            self.init_mode(&self.rtc, true);
            {
                self.rtc.tr.write(|w| {
                    w.pm()
                        .clear_bit()
                        //TODO set time!
                })
            }
            self.init_mode(&self.rtc, false);
        }
        self.write_protection(&self.rtc, true);
    }

    fn write_protection(&self, rtc: &RTC, enable: bool){
        if enable {
            rtc.wpr.write(|w| unsafe {
            w.bits(0xFF)
            });
        } else {
            rtc.wpr.write(|w| unsafe {
                w.bits(0xCA)
            });

            rtc.wpr.write(|w| unsafe {
                w.bits(0x53)
            });
        }
    }
}

fn byte_to_bcd2(byte: u8) -> u8{
    let mut bcd_high: u8 = 0;
    let mut value = byte;
    
    while byte >= 10 {
        bcd_high += 1;
        value -= 10;
    }

    return  ((bcd_high << 4) | value) as u8;
}