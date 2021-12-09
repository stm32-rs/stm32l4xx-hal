use crate::pac::RTC;

pub fn reset_gpio(rtc: &RTC) {
    rtc.cr.modify(|_, w| {
        w.out2en()
            .clear_bit()
            .tampalrm_type()
            .clear_bit()
            .tampalrm_pu()
            .clear_bit()
    });
}

/// true if initf bit indicates RTC peripheral is in init mode
pub fn is_init_mode(rtc: &RTC) -> bool {
    rtc.icsr.read().initf().bit_is_set()
}

/// to update calendar date/time, time format, and prescaler configuration, RTC must be in init mode
pub fn enter_init_mode(rtc: &RTC) {
    rtc.icsr.modify(|_, w| w.init().set_bit());
}

/// counting will restart in 4 RTCCLK cycles
pub fn exit_init_mode(rtc: &RTC) {
    rtc.icsr.modify(|_, w| w.init().clear_bit()); // Exits init mode
}

/// has wakeup timer expired?
pub fn is_wakeup_timer_flag_set(rtc: &RTC) -> bool {
    rtc.sr.read().wutf().bit_is_set()
}

/// are WUT settings modifiable
pub fn is_wakeup_timer_write_flag_set(rtc: &RTC) -> bool {
    rtc.icsr.read().wutwf().bit_is_set()
}

/// clear the wakeup timer flag
pub fn clear_wakeup_timer_flag(rtc: &RTC) {
    rtc.scr.write(|w| w.cwutf().set_bit());
}

/// has alarm A been triggered
pub fn is_alarm_a_flag_set(rtc: &RTC) -> bool {
    rtc.sr.read().alraf().bit_is_set()
}

/// clear the alarm A flag
pub fn clear_alarm_a_flag(rtc: &RTC) {
    rtc.scr.write(|w| w.calraf().set_bit());
}

/// has alarm B been triggered?
pub fn is_alarm_b_flag_set(rtc: &RTC) -> bool {
    rtc.sr.read().alrbf().bit_is_set()
}

/// clear the alarm B flag
pub fn clear_alarm_b_flag(rtc: &RTC) {
    rtc.scr.write(|w| w.calrbf().set_bit());
}

/// has timestamp event triggered
pub fn is_timestamp_flag_set(rtc: &RTC) -> bool {
    rtc.sr.read().tsf().bit_is_set()
}

/// clear the timestamp event flag
pub fn clear_timestamp_flag(rtc: &RTC) {
    rtc.scr.write(|w| w.ctsf().set_bit());
}

pub fn is_alarm_a_accessible(_rtc: &RTC) -> bool {
    // RTC type 3 has no wait after disabling the alarm (AN4759 - Rev 7, Table 8)
    true
}

pub fn is_alarm_b_accessible(_rtc: &RTC) -> bool {
    // RTC type 3 has no wait after disabling the alarm (AN4759 - Rev 7, Table 8)
    true
}
