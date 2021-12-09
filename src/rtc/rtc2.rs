pub fn reset_gpio(rtc: &RTC) {
    rtc.or
        .modify(|_, w| w.rtc_alarm_type().clear_bit().rtc_out_rmp().clear_bit());
}

/// true if initf bit indicates RTC peripheral is in init mode
pub fn is_init_mode(rtc: &RTC) -> bool {
    rtc.isr.read().initf().bit_is_set()
}

/// to update calendar date/time, time format, and prescaler configuration, RTC must be in init mode
pub fn enter_init_mode(rtc: &RTC) {
    rtc.isr.modify(|_, w| w.init().set_bit());
}

/// counting will restart in 4 RTCCLK cycles
pub fn exit_init_mode(rtc: &RTC) {
    rtc.isr.modify(|_, w| w.init().clear_bit()); // Exits init mode
}

/// has wakeup timer expired?
pub fn is_wakeup_timer_flag_set(rtc: &RTC) -> bool {
    rtc.isr.read().wutf().bit_is_set()
}

pub fn is_wakeup_timer_write_flag_set(rtc: &RTC) -> bool {
    rtc.isr.read().wutwf().bit_is_set()
}

/// clear the wakeup timer flag
pub fn clear_wakeup_timer_flag(rtc: &RTC) {
    rtc.isr.modify(|_, w| w.wutf().clear_bit());
}

/// has alarm A been triggered
pub fn is_alarm_a_flag_set(rtc: &RTC) -> bool {
    rtc.isr.read().alraf().bit_is_set()
}

/// clear the alarm A flag
pub fn clear_alarm_a_flag(rtc: &RTC) {
    rtc.isr.modify(|_, w| w.alraf().clear_bit());
}

/// has alarm B been triggered?
pub fn is_alarm_b_flag_set(rtc: &RTC) -> bool {
    rtc.isr.read().alrbf().bit_is_set()
}

/// clear the alarm B flag
pub fn clear_alarm_b_flag(rtc: &RTC) {
    rtc.isr.modify(|_, w| w.alrbf().clear_bit());
}

/// has timestamp event triggered
pub fn is_timestamp_flag_set(rtc: &RTC) -> bool {
    rtc.isr.read().tsf().bit_is_set()
}

/// clear the timestamp event flag
pub fn clear_timestamp_flag(rtc: &RTC) {
    rtc.isr.modify(|_, w| w.tsf().clear_bit());
}

pub fn is_alarm_a_accessible(rtc: &RTC) -> bool {
    rtc.isr.read().alrawf().bit_is_clear()
}

pub fn is_alarm_b_accessible(rtc: &RTC) -> bool {
    rtc.isr.read().alrbwf().bit_is_clear()
}
