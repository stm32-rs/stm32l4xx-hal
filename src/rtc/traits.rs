use crate::pac::RTC;

/// ISR register doesn't exist for RTC3 type
/// this trait abstracts those differences
pub(crate) trait RtcIsr {
    /// true if initf bit indicates RTC peripheral is in init mode
    fn is_init_mode(rtc: &RTC) -> bool;
    /// to update calendar date/time, time format, and prescaler configuration, RTC must be in init mode
    fn enter_init_mode(rtc: &RTC);
    /// counting will restart in 4 RTCCLK cycles
    fn exit_init_mode(rtc: &RTC);
    /// has wakeup timer expired?
    fn wakeup_timer_flag(rtc: &RTC) -> bool;
    /// is wakeup timer writable
    fn wakeup_timer_write_flag(rtc: &RTC) -> bool;
    /// clear the wakeup timer flag
    fn clear_wakeup_timer_flag(rtc: &RTC);
    /// has alarm A been triggered
    fn alarm_a_flag(rtc: &RTC) -> bool;
    /// clear the alarm A flag
    fn clear_alarm_a_flag(rtc: &RTC);
    /// has alarm B been triggered?
    fn alarm_b_flag(rtc: &RTC) -> bool;
    /// clear the alarm B flag
    fn clear_alarm_b_flag(rtc: &RTC);
    /// has timestamp event triggered
    fn timestamp_flag(rtc: &RTC) -> bool;
    /// clear the timestamp event flag
    fn clear_timestamp_flag(rtc: &RTC);
    /// After disabling the alarm, may have to wait before setting the new alarm value
    fn is_alarm_a_accessible(rtc: &RTC) -> bool;
    /// After disabling the alarm, may have to wait before setting the new alarm value
    fn is_alarm_b_accessible(rtc: &RTC) -> bool;
}

pub(crate) trait RtcGpio {
    /// clear any links to gpio state
    fn reset_gpio(rtc: &RTC);
}
