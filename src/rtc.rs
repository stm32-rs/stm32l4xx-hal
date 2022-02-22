//! RTC peripheral abstraction

/// refer to AN4759 to compare features of RTC2 and RTC3
#[cfg(not(any(
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l4p5",
    feature = "stm32l4q5"
)))]
pub mod rtc2;
#[cfg(not(any(
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l4p5",
    feature = "stm32l4q5"
)))]
pub use rtc2 as rtc_registers;

/// refer to AN4759 to compare features of RTC2 and RTC3
#[cfg(any(
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l4p5",
    feature = "stm32l4q5"
))]
pub mod rtc3;
#[cfg(any(
    feature = "stm32l412",
    feature = "stm32l422",
    feature = "stm32l4p5",
    feature = "stm32l4q5"
))]
pub use rtc3 as rtc_registers;

use fugit::ExtU32;
use void::Void;

use crate::{
    datetime::*,
    hal::timer::{self, Cancel as _},
    pwr,
    rcc::{APB1R1, BDCR},
    stm32::{EXTI, RTC},
};

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
#[repr(u8)]
pub enum RtcWakeupClockSource {
    /// RTC/16 clock is selected
    RtcClkDiv16 = 0b000,
    /// RTC/8 clock is selected
    RtcClkDiv8 = 0b001,
    /// RTC/4 clock is selected
    RtcClkDiv4 = 0b010,
    /// RTC/2 clock is selected
    RtcClkDiv2 = 0b011,
    /// ck_spre (usually 1 Hz) clock is selected. Handling of the 2 ** 16 bit is done if values
    /// larger than 2 ** 16 are passed to the timer start function.
    CkSpre = 0b100,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RtcConfig {
    /// RTC clock source
    clock_config: RtcClockSource,
    /// Wakeup clock source
    wakeup_clock_config: RtcWakeupClockSource,
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
            wakeup_clock_config: RtcWakeupClockSource::CkSpre,
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

    /// Set the Clock Source for the Wakeup Timer
    pub fn wakeup_clock_config(mut self, cfg: RtcWakeupClockSource) -> Self {
        self.wakeup_clock_config = cfg;
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
            (bcd2_to_byte((timer.ht().bits(), timer.hu().bits())) as u32).hours(),
            (bcd2_to_byte((timer.mnt().bits(), timer.mnu().bits())) as u32).minutes(),
            (bcd2_to_byte((timer.st().bits(), timer.su().bits())) as u32).secs(),
            micros.micros(),
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
        self.rtc_config
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
                rtc.cr.modify(|_, w| w.alrae().clear_bit()); // Disable Alarm A
                rtc_registers::clear_alarm_a_flag(rtc);
                while !rtc_registers::is_alarm_a_accessible(rtc) {}

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
                // binary mode alarm not implemented (RTC3 only)
                // subsecond alarm not implemented
                // would need a conversion method between `time.micros` and RTC ticks
                // write the SS value and mask to `rtc.alrmassr`

                // enable alarm and reenable interrupt if it was enabled
                rtc.cr.modify(|_, w| w.alrae().set_bit());
            }
            Alarm::AlarmB => {
                rtc.cr.modify(|_, w| w.alrbe().clear_bit());

                rtc_registers::clear_alarm_b_flag(rtc);
                while !rtc_registers::is_alarm_b_accessible(rtc) {}

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
                // binary mode alarm not implemented (RTC3 only)
                // subsecond alarm not implemented
                // would need a conversion method between `time.micros` and RTC ticks
                // write the SS value and mask to `rtc.alrmbssr`

                // enable alarm and reenable interrupt if it was enabled
                rtc.cr.modify(|_, w| w.alrbe().set_bit());
            }
        });
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
            Event::WakeupTimer => rtc_registers::is_wakeup_timer_flag_set(&self.rtc),
            Event::AlarmA => rtc_registers::is_alarm_a_flag_set(&self.rtc),
            Event::AlarmB => rtc_registers::is_alarm_b_flag_set(&self.rtc),
            Event::Timestamp => rtc_registers::is_timestamp_flag_set(&self.rtc),
        };
        if clear {
            self.write(false, |rtc| match event {
                Event::WakeupTimer => {
                    rtc_registers::clear_wakeup_timer_flag(rtc);
                    unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << 20)) };
                }
                Event::AlarmA => {
                    rtc_registers::clear_alarm_a_flag(rtc);
                    unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << 18)) };
                }
                Event::AlarmB => {
                    rtc_registers::clear_alarm_b_flag(rtc);
                    unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << 18)) };
                }
                Event::Timestamp => {
                    rtc_registers::clear_timestamp_flag(rtc);
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
            rtc_registers::reset_gpio(rtc);
        });

        self.rtc_config = rtc_config;
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

        if init_mode && !rtc_registers::is_init_mode(&self.rtc) {
            rtc_registers::enter_init_mode(&self.rtc);
            // wait till init state entered
            // ~2 RTCCLK cycles
            while !rtc_registers::is_init_mode(&self.rtc) {}
        }

        let result = f(&self.rtc);
        if init_mode {
            rtc_registers::exit_init_mode(&self.rtc);
        }

        // Re-enable write protection.
        // This is safe, as the field accepts the full range of 8-bit values.
        self.rtc.wpr.write(|w| unsafe { w.key().bits(0xff) });

        result
    }

    pub const BACKUP_REGISTER_COUNT: usize = rtc_registers::BACKUP_REGISTER_COUNT;

    /// Read content of the backup register.
    ///
    /// The registers retain their values during wakes from standby mode or system resets. They also
    /// retain their value when Vdd is switched off as long as V_BAT is powered.
    pub fn read_backup_register(&self, register: usize) -> Option<u32> {
        rtc_registers::read_backup_register(&self.rtc, register)
    }

    /// Set content of the backup register.
    ///
    /// The registers retain their values during wakes from standby mode or system resets. They also
    /// retain their value when Vdd is switched off as long as V_BAT is powered.
    pub fn write_backup_register(&self, register: usize, value: u32) {
        rtc_registers::write_backup_register(&self.rtc, register, value)
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
    /// The `delay` argument specifies the timer delay. If the wakeup_clock_config is set to
    /// CkSpre, the value is in seconds and up to 17 bits
    /// of delay are supported, giving us a range of over 36 hours.
    /// Otherwise, the timeunit depends on the RTCCLK and the configured wakeup_clock_config value.
    ///
    /// # Panics
    ///
    /// The `delay` argument must be in the range `1 <= delay <= 2^17`.
    /// Panics, if `delay` is outside of that range.
    fn start<T>(&mut self, delay: T)
    where
        T: Into<Self::Time>,
    {
        let delay = delay.into();
        assert!(1 <= delay);

        if self.rtc.rtc_config.wakeup_clock_config == RtcWakeupClockSource::CkSpre {
            assert!(delay <= 1 << 17);
        } else {
            assert!(delay <= 1 << 16);
        }

        // Determine the value for the wucksel register
        let wucksel = self.rtc.rtc_config.wakeup_clock_config as u8;
        let wucksel = wucksel
            | if self.rtc.rtc_config.wakeup_clock_config == RtcWakeupClockSource::CkSpre
                && delay & 0x1_00_00 != 0
            {
                0b010
            } else {
                0b000
            };

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
                // Write WUCKSEL depending on value determined previously.
                unsafe {
                    w.wucksel().bits(wucksel);
                }
                // Enable wakeup timer
                w.wute().set_bit()
            });
        });

        // Let's wait for WUTWF to clear. Otherwise we might run into a race
        // condition, if the user calls this method again really quickly.
        while rtc_registers::is_wakeup_timer_write_flag_set(&self.rtc.rtc) {}
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
            while !rtc_registers::is_wakeup_timer_write_flag_set(rtc) {}
            rtc_registers::clear_wakeup_timer_flag(rtc);

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
