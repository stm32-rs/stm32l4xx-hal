//! RTC peripheral abstraction

use crate::datetime::*;
use crate::pwr;
use crate::rcc::{APB1R1, BDCR};
use crate::stm32::{RTC};

/// RTC Abstraction
pub struct Rtc {
    rtc: RTC,
    rtc_config: RtcConfig,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum RtcClockSource {
    /// 00: No clock
    NoClock = 0b00,
    /// 01: LSE oscillator clock used as RTC clock
    LSE = 0b01,
    /// 10: LSI oscillator clock used as RTC clock
    LSI = 0b10,
    /// 11: HSE oscillator clock divided by 32 used as RTC clock
    HSE = 0b11,
}
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RtcConfig {
    /// RTC clock source
    clock_config: RtcClockSource,
    /// Asynchronous prescaler factor
    /// This is the asynchronous division factor:
    /// ck_apre frequency = RTCCLK frequency/(PREDIV_A+1)
    /// ck_apre drives the subsecond register
    async_prescaler: u8,
    /// Synchronous prescaler factor
    /// This is the synchronous division factor:
    /// ck_spre frequency = ck_apre frequency/(PREDIV_S+1)
    /// ck_spre must be 1Hz
    sync_prescaler: u16,
}

impl Default for RtcConfig {
    /// LSI with prescalers assuming 32.768 kHz.
    /// Raw sub-seconds in 1/256.
    fn default() -> Self {
        RtcConfig {
            clock_config: RtcClockSource::LSI,
            async_prescaler: 127,
            sync_prescaler: 255,
        }
    }
}

impl RtcConfig {
    /// Sets the clock source of RTC config
    pub fn clock_config(mut self, cfg: RtcClockSource) -> Self {
        self.clock_config = cfg;
        self
    }

    /// Set the asynchronous prescaler of RTC config
    pub fn async_prescaler(mut self, prescaler: u8) -> Self {
        self.async_prescaler = prescaler;
        self
    }

    /// Set the synchronous prescaler of RTC config
    pub fn sync_prescaler(mut self, prescaler: u16) -> Self {
        self.sync_prescaler = prescaler;
        self
    }
}

impl Rtc {
    pub fn rtc(
        rtc: RTC,
        apb1r1: &mut APB1R1,
        bdcr: &mut BDCR,
        pwrcr1: &mut pwr::CR1,
        rtc_config: RtcConfig,
    ) -> Self {
        // assert_eq!(clocks.lsi(), true); // make sure LSI is enabled
        // enable peripheral clock for communication
        apb1r1.enr().modify(|_, w| w.rtcapben().set_bit());
        pwrcr1.reg().read(); // read to allow the pwr clock to enable

        let mut rtc_struct = Self { rtc, rtc_config };
        rtc_struct.set_config(bdcr, pwrcr1, rtc_config);

        rtc_struct
    }

    /// Get date and time touple
    pub fn get_date_time(&self) -> (Date, Time) {
        let time;
        let date;

        let sync_p = self.rtc_config.sync_prescaler as u32;
        let micros =
            1_000_000u32 / (sync_p + 1) * (sync_p - self.rtc.ssr.read().ss().bits() as u32);
        let timer = self.rtc.tr.read();
        let cr = self.rtc.cr.read();

        // Reading either RTC_SSR or RTC_TR locks the values in the higher-order
        // calendar shadow registers until RTC_DR is read.
        let dater = self.rtc.dr.read();

        time = Time::new(
            bcd2_to_byte((timer.ht().bits(), timer.hu().bits())).into(),
            bcd2_to_byte((timer.mnt().bits(), timer.mnu().bits())).into(),
            bcd2_to_byte((timer.st().bits(), timer.su().bits())).into(),
            micros.into(),
            cr.bkp().bit(),
        );

        date = Date::new(
            dater.wdu().bits().into(),
            bcd2_to_byte((dater.dt().bits(), dater.du().bits())).into(),
            bcd2_to_byte((dater.mt().bit() as u8, dater.mu().bits())).into(),
            (bcd2_to_byte((dater.yt().bits(), dater.yu().bits())) as u16 + 1970_u16).into(),
        );

        (date, time)
    }

    /// Set Date and Time
    pub fn set_date_time(&self, date: Date, time: Time) {
        write_protection(&self.rtc, false);
        {
            init_mode(&self.rtc, true);
            {
                set_time_raw(&self.rtc, time);
                set_date_raw(&self.rtc, date);
            }
            init_mode(&self.rtc, false);
        }
        write_protection(&self.rtc, true);
    }

    /// Set Time
    /// Note: If setting both time and date, use set_date_time(...) to avoid errors.
    pub fn set_time(&self, time: Time) {
        write_protection(&self.rtc, false);
        {
            init_mode(&self.rtc, true);
            {
                set_time_raw(&self.rtc, time);
            }
            init_mode(&self.rtc, false);
        }
        write_protection(&self.rtc, true);
    }

    /// Set Date
    /// Note: If setting both time and date, use set_date_time(...) to avoid errors.
    pub fn set_date(&self, date: Date) {
        write_protection(&self.rtc, false);
        {
            init_mode(&self.rtc, true);
            {
                set_date_raw(&self.rtc, date);
            }
            init_mode(&self.rtc, false);
        }
        write_protection(&self.rtc, true);
    }

    pub fn get_config(&self) -> RtcConfig {
        self.rtc_config
    }

    /// Applies the RTC config
    /// It this changes the RTC clock source the time will be reset
    pub fn set_config(
        &mut self,
        bdcr: &mut BDCR,
        pwrcr1: &mut pwr::CR1,
        rtc_config: RtcConfig,
    ) {
        // Unlock the backup domain
        pwrcr1.reg().modify(|_, w| w.dbp().set_bit());
        while pwrcr1.reg().read().dbp().bit_is_clear() {}

        let reg = bdcr.enr().read();
        assert!(
            !reg.lsecsson().bit(),
            "RTC is not compatible with LSE, yet."
        );

        if reg.rtcen().bit() != true || reg.rtcsel().bits() != rtc_config.clock_config as u8 {
            bdcr.enr().modify(|_, w| w.bdrst().set_bit());
    
            bdcr.enr().modify(|_, w| unsafe {
                //Reset
                w.bdrst().clear_bit();
                //Select RTC source
                w.rtcsel()
                    .bits(rtc_config.clock_config as u8)
                    .rtcen()
                    .set_bit();
    
                //Restore bcdr
                w.lscosel()
                    .bit(reg.lscosel().bit())
                    .lscoen()
                    .bit(reg.lscoen().bit());
    
                w.lseon()
                    .bit(reg.lseon().bit())
                    .lsedrv()
                    .bits(reg.lsedrv().bits())
                    .lsebyp()
                    .bit(reg.lsebyp().bit())
            });
        }


        write_protection(&self.rtc, false);
        {
            init_mode(&self.rtc, true);
            {
                self.rtc.cr.modify(|_, w| unsafe {
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

                self.rtc.prer.modify(|_, w| unsafe {
                    w.prediv_s()
                        .bits(rtc_config.sync_prescaler)
                        .prediv_a()
                        .bits(rtc_config.async_prescaler)
                });
            }
            init_mode(&self.rtc, false);

            // TODO configuration for output pins
            self.rtc
                .or
                .modify(|_, w| w.rtc_alarm_type().clear_bit().rtc_out_rmp().clear_bit());
        }
        write_protection(&self.rtc, true);

        self.rtc_config = rtc_config;
    }
}

fn write_protection(rtc: &RTC, enable: bool) {
    if enable {
        rtc.wpr.write(|w| unsafe { w.bits(0xFF) });
    } else {
        rtc.wpr.write(|w| unsafe { w.bits(0xCA) });

        rtc.wpr.write(|w| unsafe { w.bits(0x53) });
    }
}

fn init_mode(rtc: &RTC, enabled: bool) {
    if enabled {
        let isr = rtc.isr.read();
        if isr.initf().bit_is_clear() {
            // are we already in init mode?
            rtc.isr.write(|w| w.init().set_bit());
            // rtc.isr.write(|w| unsafe { w.bits(0xFFFFFFFF) }); // Sets init mode
            while rtc.isr.read().initf().bit_is_clear() {} // wait to return to init state
        }
    } else {
        rtc.isr.write(|w| w.init().clear_bit()); // Exits init mode
    }
}

/// Raw set time
/// Expects init mode enabled and write protection disabled
fn set_time_raw (rtc: &RTC, time: Time){
    let (ht, hu) = byte_to_bcd2(time.hours as u8);
    let (mnt, mnu) = byte_to_bcd2(time.minutes as u8);
    let (st, su) = byte_to_bcd2(time.seconds as u8);

    rtc.tr.write(|w| unsafe {
        w.ht()
            .bits(ht)
            .hu()
            .bits(hu)
            .mnt()
            .bits(mnt)
            .mnu()
            .bits(mnu)
            .st()
            .bits(st)
            .su()
            .bits(su)
            .pm()
            .clear_bit()
    });

    rtc
        .cr
        .modify(|_, w| w.bkp().bit(time.daylight_savings));
}

/// Raw set date
/// Expects init mode enabled and write protection disabled
fn set_date_raw (rtc: &RTC, date: Date){
    let (dt, du) = byte_to_bcd2(date.date as u8);
    let (mt, mu) = byte_to_bcd2(date.month as u8);
    let yr = date.year as u16;
    let yr_offset = (yr - 1970_u16) as u8;
    let (yt, yu) = byte_to_bcd2(yr_offset);

    rtc.dr.write(|w| unsafe {
        w.dt()
            .bits(dt)
            .du()
            .bits(du)
            .mt()
            .bit(mt > 0)
            .mu()
            .bits(mu)
            .yt()
            .bits(yt)
            .yu()
            .bits(yu)
            .wdu()
            .bits(date.day as u8)
    });
}

fn byte_to_bcd2(byte: u8) -> (u8, u8) {
    let mut bcd_high: u8 = 0;
    let mut value = byte;

    while value >= 10 {
        bcd_high += 1;
        value -= 10;
    }

    (bcd_high, ((bcd_high << 4) | value) as u8)
}

fn bcd2_to_byte(bcd: (u8, u8)) -> u8 {
    let value = bcd.1 | bcd.0 << 4;

    let tmp = ((value & 0xF0) >> 0x4) * 10;

    tmp + (value & 0x0F)
}
