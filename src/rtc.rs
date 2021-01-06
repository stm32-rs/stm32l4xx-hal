//! RTC peripheral abstraction

use void::Void;

use crate::{
    datetime::*,
    hal::timer::{self, Cancel as _},
    pwr,
    rcc::{APB1R1, BDCR},
    stm32::{EXTI, RTC},
};

#[cfg(any(feature = "rt"))]
/// This provides a default handler for RTC inputs that clears the EXTI line and
/// wakeup flag. If you don't need additional functionality, run this in the main body of your program, eg:
/// `make_rtc_interrupt_handler!(RTC_WKUP);`
#[macro_export]
macro_rules! make_wakeup_interrupt_handler {
    ($line:ident) => {
        #[interrupt]
        fn $line() {
            free(|cs| {
                unsafe {
                    // Reset pending bit for interrupt line
                    (*pac::EXTI::ptr()).pr1.modify(|_, w| w.pr20().bit(true));

                    // Clear the wakeup timer flag, after disabling write protections.
                    (*pac::RTC::ptr()).wpr.write(|w| w.bits(0xCA));
                    (*pac::RTC::ptr()).wpr.write(|w| w.bits(0x53));
                    (*pac::RTC::ptr()).cr.modify(|_, w| w.wute().clear_bit());

                    (*pac::RTC::ptr()).isr.modify(|_, w| w.wutf().clear_bit());

                    (*pac::RTC::ptr()).cr.modify(|_, w| w.wute().set_bit());
                    (*pac::RTC::ptr()).wpr.write(|w| w.bits(0xFF));
                }
            });
        }
    };
}

/// RTC error type
#[derive(Debug)]
pub enum Error {
    /// Invalid input error
    InvalidInputData,
}

/// See ref man, section 27.6.3, or AN4769, section 2.4.2.
/// To be used with WakeupPrescaler
#[derive(Clone, Copy, Debug)]
enum WakeupDivision {
    Sixteen,
    Eight,
    Four,
    Two,
}

/// See AN4759, table 13.
#[derive(Clone, Copy, Debug)]
enum ClockConfig {
    One(WakeupDivision),
    Two,
    Three,
}

/// Interrupt event
pub enum Event {
    WakeupTimer,
    AlarmA,
    AlarmB,
    Timestamp,
}

pub enum Alarm {
    AlarmA,
    AlarmB,
}

impl From<Alarm> for Event {
    fn from(a: Alarm) -> Self {
        match a {
            Alarm::AlarmA => Event::AlarmA,
            Alarm::AlarmB => Event::AlarmB,
        }
    }
}

/// RTC Abstraction
pub struct Rtc {
    rtc: RTC,
    config: RtcConfig,
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
    pub fn new(
        rtc: RTC,
        apb1r1: &mut APB1R1,
        bdcr: &mut BDCR,
        pwrcr1: &mut pwr::CR1,
        config: RtcConfig,
    ) -> Self {
        // assert_eq!(clocks.lsi(), true); // make sure LSI is enabled
        // enable peripheral clock for communication
        apb1r1.enr().modify(|_, w| w.rtcapben().set_bit());
        pwrcr1.reg().read(); // read to allow the pwr clock to enable

        let mut rtc_struct = Self { rtc, config };
        rtc_struct.set_config(bdcr, pwrcr1, config);

        rtc_struct
    }

    /// Helper fn, to do the important bits of setting the interval, with
    /// the registers already unlocked.
    fn set_wakeup_interval_inner(&mut self, sleep_time: f32) {
        // Program the value into the wakeup timer
        // Set WUT[15:0] in RTC_WUTR register. For RTC3 the user must also program
        // WUTOCLR bits.
        // See ref man Section 2.4.2: Maximum and minimum RTC wakeup period.
        // todo check ref man register table

        // See notes reffed below about WUCKSEL. We choose one of 3 "modes" described in AN4759 based
        // on sleep time. If in the overlap area, choose the lower (more precise) mode.
        // These all assume a 1hz `ck_spre`.
        let lfe_freq = match self.config.clock_config {
            RtcClockSource::LSE => 32_768.,
            RtcClockSource::LSI => 40_000.,
            RtcClockSource::HSE => 250_000., // Assuming 8Mhz HSE, which may not be the case
            RtcClockSource::NoClock => 1.,   // todo: Return an `Err` ?
        };

        // sleep_time = (1/lfe_freq) * div * (wutr + 1)
        // res = 1/lfe_freq * div
        // sleep_time = res * WUTR = 1/lfe_freq * div * (wutr + 1)
        // wutr = sleep_time * lfe_freq / div - 1

        let clock_cfg;
        let wutr;

        if sleep_time >= 0.00012207 && sleep_time < 32. {
            let division;
            let div;
            if sleep_time < 4. {
                division = WakeupDivision::Two; // Resolution: 61.035µs
                div = 2.;
            } else if sleep_time < 8. {
                division = WakeupDivision::Four; // Resolution: 122.08µs
                div = 4.;
            } else if sleep_time < 16. {
                division = WakeupDivision::Eight; // Resolution: 244.141
                div = 8.;
            } else {
                division = WakeupDivision::Sixteen; // Resolution: 488.281
                div = 16.;
            }
            clock_cfg = ClockConfig::One(division);
            wutr = sleep_time * lfe_freq / div - 1.
        } else if sleep_time < 65_536. {
            // 32s to 18 hours (This mode goes 1s to 18 hours; we use Config1 for the overlap)
            clock_cfg = ClockConfig::Two;
            wutr = sleep_time; // This works out conveniently!
        } else if sleep_time < 131_072. {
            // 18 to 36 hours
            clock_cfg = ClockConfig::Three;
            wutr = sleep_time - 65_537.;
        } else {
            panic!("Wakeup period must be between 0122.07µs and 36 hours.")
        }

        self.rtc
            .wutr
            .modify(|_, w| unsafe { w.wut().bits(wutr as u16) });

        // Select the desired clock source. Program WUCKSEL[2:0] bits in RTC_CR register.
        // See ref man Section 2.4.2: Maximum and minimum RTC wakeup period.
        // todo: Check register docs and see what to set here.

        // See AN4759, Table 13. RM, 27.3.6

        // When ck_spre frequency is 1Hz, this allows to achieve a wakeup time from 1 s to
        // around 36 hours with one-second resolution. This large programmable time range is
        // divided in 2 parts:
        // – from 1s to 18 hours when WUCKSEL [2:1] = 10
        // – and from around 18h to 36h when WUCKSEL[2:1] = 11. In this last case 216 is
        // added to the 16-bit counter current value.When the initialization sequence is
        // complete (see Programming the wakeup timer on page 781), the timer starts
        // counting down.When the wakeup function is enabled, the down-counting remains
        // active in low-power modes. In addition, when it reaches 0, the WUTF flag is set in
        // the RTC_ISR register, and the wakeup counter is automatically reloaded with its
        // reload value (RTC_WUTR register value).
        let word = match clock_cfg {
            ClockConfig::One(division) => match division {
                WakeupDivision::Sixteen => 0b000,
                WakeupDivision::Eight => 0b001,
                WakeupDivision::Four => 0b010,
                WakeupDivision::Two => 0b011,
            },
            // for 2 and 3, what does `x` mean in the docs? Best guess is it doesn't matter.
            ClockConfig::Two => 0b100,   // eg 1s to 18h.
            ClockConfig::Three => 0b110, // eg 18h to 36h
        };

        // 000: RTC/16 clock is selected
        // 001: RTC/8 clock is selected
        // 010: RTC/4 clock is selected
        // 011: RTC/2 clock is selected
        // 10x: ck_spre (usually 1 Hz) clock is selected
        // 11x: ck_spre (usually 1 Hz) clock is selected and 216 is added to the WUT counter value

        self.rtc.cr.modify(|_, w| unsafe { w.wcksel().bits(word) });
    }

    /// Setup periodic auto-wakeup interrupts. See ST AN4759, Table 11, and more broadly,
    /// section 2.4.1. See also reference manual, section 27.5.
    /// In addition to running this function, set up the interrupt handling function by
    /// adding the line `make_rtc_interrupt_handler!(RTC_WKUP);` somewhere in the body
    /// of your program.
    /// `sleep_time` is in ms.
    pub fn set_wakeup(&mut self, exti: &mut EXTI, sleep_time: f32) {
        // Configure and enable the EXTI line corresponding to the Wakeup timer even in
        // interrupt mode and select the rising edge sensitivity.
        // Sleep time is in seconds

        exti.imr1.modify(|_, w| w.mr20().unmasked());
        exti.rtsr1.modify(|_, w| w.tr20().bit(true));
        exti.ftsr1.modify(|_, w| w.tr20().bit(false));

        // Disable the RTC registers Write protection.
        // Write 0xCA and then 0x53 into the RTC_WPR register. RTC registers can then be modified.
        self.rtc.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.rtc.wpr.write(|w| unsafe { w.bits(0x53) });

        // Disable the wakeup timer. Clear WUTE bit in RTC_CR register
        self.rtc.cr.modify(|_, w| w.wute().clear_bit());

        // Ensure access to Wakeup auto-reload counter and bits WUCKSEL[2:0] is allowed.
        // Poll WUTWF until it is set in RTC_ISR (RTC2)/RTC_ICSR (RTC3) (May not be avail on F3)
        while self.rtc.isr.read().wutwf().bit_is_clear() {}

        self.set_wakeup_interval_inner(sleep_time);
        // Re-enable the wakeup timer. Set WUTE bit in RTC_CR register.
        // The wakeup timer restarts counting down.
        self.rtc.cr.modify(|_, w| w.wute().set_bit());

        // Enable the wakeup timer interrupt.
        self.rtc.cr.modify(|_, w| w.wutie().set_bit());
        self.rtc.cr.modify(|_, w| w.wutie().set_bit());

        // Clear the  wakeup flag.
        self.rtc.isr.modify(|_, w| w.wutf().clear_bit());

        // Enable the RTC registers Write protection. Write 0xFF into the
        // RTC_WPR register. RTC registers can no more be modified.
        self.rtc.wpr.write(|w| unsafe { w.bits(0xFF) });
    }

    /// Change the sleep time for the auto wakeup, after it's been set up.
    /// Sleep time is in MS. Major DRY from `set_wakeup`.
    pub fn set_wakeup_interval(&mut self, sleep_time: f32) {
        // `sleep_time` is in seconds.
        // See comments in `set_auto_wakeup` for what these writes do.
        self.rtc.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.rtc.wpr.write(|w| unsafe { w.bits(0x53) });

        self.rtc.cr.modify(|_, w| w.wute().clear_bit());
        while self.rtc.isr.read().wutwf().bit_is_clear() {}

        self.set_wakeup_interval_inner(sleep_time);

        self.rtc.cr.modify(|_, w| w.wute().set_bit());
        self.rtc.wpr.write(|w| unsafe { w.bits(0xFF) });
    }

    /// Get date and time touple
    pub fn get_date_time(&self) -> (Date, Time) {
        let time;
        let date;

        let sync_p = self.config.sync_prescaler as u32;
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
    pub fn set_date_time(&mut self, date: Date, time: Time) {
        self.write(true, |rtc| {
            set_time_raw(rtc, time);
            set_date_raw(rtc, date);
        })
    }

    /// Set Time
    /// Note: If setting both time and date, use set_date_time(...) to avoid errors.
    pub fn set_time(&mut self, time: Time) {
        self.write(true, |rtc| {
            set_time_raw(rtc, time);
        })
    }

    /// Set Date
    /// Note: If setting both time and date, use set_date_time(...) to avoid errors.
    pub fn set_date(&mut self, date: Date) {
        self.write(true, |rtc| {
            set_date_raw(rtc, date);
        })
    }

    pub fn get_config(&self) -> RtcConfig {
        self.config
    }

    /// Sets the time at which an alarm will be triggered
    /// This also clears the alarm flag if it is set
    pub fn set_alarm(&mut self, alarm: Alarm, date: Date, time: Time) {
        let (dt, du) = byte_to_bcd2(date.date as u8);
        let (ht, hu) = byte_to_bcd2(time.hours as u8);
        let (mnt, mnu) = byte_to_bcd2(time.minutes as u8);
        let (st, su) = byte_to_bcd2(time.seconds as u8);

        self.write(false, |rtc| match alarm {
            Alarm::AlarmA => {
                rtc.cr.modify(|_, w| w.alrae().clear_bit());

                // Wait until we're allowed to update the alarm b configuration
                while rtc.isr.read().alrawf().bit_is_clear() {}

                rtc.alrmar.modify(|_, w| unsafe {
                    w.dt()
                        .bits(dt)
                        .du()
                        .bits(du)
                        .ht()
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
                        .wdsel()
                        .clear_bit()
                });
                rtc.cr.modify(|_, w| w.alrae().set_bit());
            }
            Alarm::AlarmB => {
                rtc.cr.modify(|_, w| w.alrbe().clear_bit());

                // Wait until we're allowed to update the alarm b configuration
                while rtc.isr.read().alrbwf().bit_is_clear() {}

                rtc.alrmbr.modify(|_, w| unsafe {
                    w.dt()
                        .bits(dt)
                        .du()
                        .bits(du)
                        .ht()
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
                        .wdsel()
                        .clear_bit()
                });
                rtc.cr.modify(|_, w| w.alrbe().set_bit());
            }
        });
        self.check_interrupt(alarm.into(), true);
    }

    /// Starts listening for an interrupt event
    pub fn listen(&mut self, exti: &mut EXTI, event: Event) {
        self.write(false, |rtc| match event {
            Event::WakeupTimer => {
                exti.rtsr1.modify(|_, w| w.tr20().set_bit());
                exti.imr1.modify(|_, w| w.mr20().set_bit());
                rtc.cr.modify(|_, w| w.wutie().set_bit())
            }
            Event::AlarmA => {
                // Workaround until tr17() is implemented ()
                exti.rtsr1.modify(|_, w| w.tr18().set_bit());
                exti.imr1.modify(|_, w| w.mr18().set_bit());
                rtc.cr.modify(|_, w| w.alraie().set_bit())
            }
            Event::AlarmB => {
                exti.rtsr1.modify(|_, w| w.tr18().set_bit());
                exti.imr1.modify(|_, w| w.mr18().set_bit());
                rtc.cr.modify(|_, w| w.alrbie().set_bit())
            }
            Event::Timestamp => {
                exti.rtsr1.modify(|_, w| w.tr19().set_bit());
                exti.imr1.modify(|_, w| w.mr19().set_bit());
                rtc.cr.modify(|_, w| w.tsie().set_bit())
            }
        })
    }

    /// Stops listening for an interrupt event
    pub fn unlisten(&mut self, exti: &mut EXTI, event: Event) {
        self.write(false, |rtc| match event {
            Event::WakeupTimer => {
                exti.rtsr1.modify(|_, w| w.tr20().clear_bit());
                exti.imr1.modify(|_, w| w.mr20().clear_bit());
                rtc.cr.modify(|_, w| w.wutie().clear_bit())
            }
            Event::AlarmA => {
                // Workaround until tr17() is implemented ()
                exti.rtsr1.modify(|_, w| w.tr18().clear_bit());
                exti.imr1.modify(|_, w| w.mr18().clear_bit());
                rtc.cr.modify(|_, w| w.alraie().clear_bit())
            }
            Event::AlarmB => {
                exti.rtsr1.modify(|_, w| w.tr18().clear_bit());
                exti.imr1.modify(|_, w| w.mr18().clear_bit());
                rtc.cr.modify(|_, w| w.alrbie().clear_bit())
            }
            Event::Timestamp => {
                exti.rtsr1.modify(|_, w| w.tr19().clear_bit());
                exti.imr1.modify(|_, w| w.mr19().clear_bit());
                rtc.cr.modify(|_, w| w.tsie().clear_bit())
            }
        })
    }

    /// Checks for an interrupt event
    pub fn check_interrupt(&mut self, event: Event, clear: bool) -> bool {
        let result = match event {
            Event::WakeupTimer => self.rtc.isr.read().wutf().bit_is_set(),
            Event::AlarmA => self.rtc.isr.read().alraf().bit_is_set(),
            Event::AlarmB => self.rtc.isr.read().alrbf().bit_is_set(),
            Event::Timestamp => self.rtc.isr.read().tsf().bit_is_set(),
        };
        if clear {
            self.write(false, |rtc| match event {
                Event::WakeupTimer => {
                    rtc.isr.modify(|_, w| w.wutf().clear_bit());
                    unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << 20)) };
                }
                Event::AlarmA => {
                    rtc.isr.modify(|_, w| w.alraf().clear_bit());
                    unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << 18)) };
                }
                Event::AlarmB => {
                    rtc.isr.modify(|_, w| w.alrbf().clear_bit());
                    unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << 18)) };
                }
                Event::Timestamp => {
                    rtc.isr.modify(|_, w| w.tsf().clear_bit());
                    unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << 19)) };
                }
            })
        }

        result
    }

    /// Applies the RTC config
    /// It this changes the RTC clock source the time will be reset
    pub fn set_config(&mut self, bdcr: &mut BDCR, pwrcr1: &mut pwr::CR1, rtc_config: RtcConfig) {
        // Unlock the backup domain
        pwrcr1.reg().modify(|_, w| w.dbp().set_bit());
        while pwrcr1.reg().read().dbp().bit_is_clear() {}

        let reg = bdcr.enr().read();
        assert!(
            !reg.lsecsson().bit(),
            "RTC is not compatible with LSE CSS, yet."
        );

        if !reg.rtcen().bit() || reg.rtcsel().bits() != rtc_config.clock_config as u8 {
            bdcr.enr().modify(|_, w| w.bdrst().set_bit());

            bdcr.enr().modify(|_, w| unsafe {
                // Reset
                w.bdrst().clear_bit();
                // Select RTC source
                w.rtcsel()
                    .bits(rtc_config.clock_config as u8)
                    .rtcen()
                    .set_bit();

                // Restore bcdr
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

        self.write(true, |rtc| {
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
                    .bits(rtc_config.sync_prescaler)
                    .prediv_a()
                    .bits(rtc_config.async_prescaler)
            });

            // TODO configuration for output pins
            rtc.or
                .modify(|_, w| w.rtc_alarm_type().clear_bit().rtc_out_rmp().clear_bit());
        });

        self.config = rtc_config;
    }

    /// Access the wakeup timer
    pub fn wakeup_timer(&mut self) -> WakeupTimer {
        WakeupTimer { rtc: self }
    }

    fn write<F, R>(&mut self, init_mode: bool, f: F) -> R
    where
        F: FnOnce(&RTC) -> R,
    {
        // Disable write protection.
        // This is safe, as we're only writin the correct and expected values.
        self.rtc.wpr.write(|w| unsafe { w.key().bits(0xca) });
        self.rtc.wpr.write(|w| unsafe { w.key().bits(0x53) });

        if init_mode && self.rtc.isr.read().initf().bit_is_clear() {
            // are we already in init mode?
            self.rtc.isr.modify(|_, w| w.init().set_bit());
            while self.rtc.isr.read().initf().bit_is_clear() {} // wait to return to init state
        }

        let result = f(&self.rtc);

        if init_mode {
            self.rtc.isr.modify(|_, w| w.init().clear_bit()); // Exits init mode
        }

        // Re-enable write protection.
        // This is safe, as the field accepts the full range of 8-bit values.
        self.rtc.wpr.write(|w| unsafe { w.key().bits(0xff) });

        result
    }
}

/// The RTC wakeup timer
///
/// This timer can be used in two ways:
/// 1. Continually call `wait` until it returns `Ok(())`.
/// 2. Set up the RTC interrupt.
///
/// If you use an interrupt, you should still call `wait` once, after the
/// interrupt fired. This should return `Ok(())` immediately. Doing this will
/// reset the timer flag. If you don't do this, the interrupt will not fire
/// again, if you go to sleep.
///
/// You don't need to call `wait`, if you call `cancel`, as that also resets the
/// flag. Restarting the timer by calling `start` will also reset the flag.
pub struct WakeupTimer<'r> {
    rtc: &'r mut Rtc,
}

impl timer::Periodic for WakeupTimer<'_> {}

impl timer::CountDown for WakeupTimer<'_> {
    type Time = u32;

    /// Starts the wakeup timer
    ///
    /// The `delay` argument specifies the timer delay in seconds. Up to 17 bits
    /// of delay are supported, giving us a range of over 36 hours.
    ///
    /// # Panics
    ///
    /// The `delay` argument must be in the range `1 <= delay <= 2^17`.
    /// Panics, if `delay` is outside of that range.
    fn start<T>(&mut self, delay: T)
    where
        T: Into<Self::Time>,
    {
        // self.set_wakeup(unsafe { &mut (*pac::EXTI::ptr(), delay as u16)});
        // return;

        let delay = delay.into();
        assert!(1 <= delay && delay <= 1 << 17);

        let delay = delay - 1;

        // Can't panic, as the error type is `Void`.
        self.cancel().unwrap();

        self.rtc.write(false, |rtc| {
            // Set the wakeup delay
            rtc.wutr.write(|w|
                // Write the lower 16 bits of `delay`. The 17th bit is taken
                // care of via WUCKSEL in CR (see below).
                // This is safe, as the field accepts a full 16 bit value.
                unsafe { w.wut().bits(delay as u16) });

            rtc.cr.modify(|_, w| {
                if delay & 0x1_00_00 != 0 {
                    unsafe {
                        w.wcksel().bits(0b110);
                    }
                } else {
                    unsafe {
                        w.wcksel().bits(0b100);
                    }
                }

                // Enable wakeup timer
                w.wute().set_bit()
            });
        });

        // Let's wait for WUTWF to clear. Otherwise we might run into a race
        // condition, if the user calls this method again really quickly.
        while self.rtc.rtc.isr.read().wutwf().bit_is_set() {}
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.rtc.check_interrupt(Event::WakeupTimer, true) {
            return Ok(());
        }

        Err(nb::Error::WouldBlock)
    }
}

impl timer::Cancel for WakeupTimer<'_> {
    type Error = Void;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        self.rtc.write(false, |rtc| {
            // Disable the wakeup timer
            rtc.cr.modify(|_, w| w.wute().clear_bit());

            // Wait until we're allowed to update the wakeup timer configuration
            while rtc.isr.read().wutwf().bit_is_clear() {}

            // Clear wakeup timer flag
            rtc.isr.modify(|_, w| w.wutf().clear_bit());

            // According to the reference manual, section 26.7.4, the WUTF flag
            // must be cleared at least 1.5 RTCCLK periods "before WUTF is set
            // to 1 again". If that's true, we're on the safe side, because we
            // use ck_spre as the clock for this timer, which we've scaled to 1
            // Hz.
            //
            // I have the sneaking suspicion though that this is a typo, and the
            // quote in the previous paragraph actually tries to refer to WUTE
            // instead of WUTF. In that case, this might be a bug, so if you're
            // seeing something weird, adding a busy loop of some length here
            // would be a good start of your investigation.
        });

        Ok(())
    }
}

/// Raw set time
/// Expects init mode enabled and write protection disabled
fn set_time_raw(rtc: &RTC, time: Time) {
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

    rtc.cr.modify(|_, w| w.bkp().bit(time.daylight_savings));
}

/// Raw set date
/// Expects init mode enabled and write protection disabled
fn set_date_raw(rtc: &RTC, date: Date) {
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
