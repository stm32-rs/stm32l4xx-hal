use crate::pac::RTC;

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
    rtc.isr.read().alrawf().bit_is_set()
}

pub fn is_alarm_b_accessible(rtc: &RTC) -> bool {
    rtc.isr.read().alrbwf().bit_is_set()
}

// AN7459
// L4 series except L41/2 has 20 backup registers
// L41/2, L4P/Q and L4R/S have 32 backup registers
#[cfg(not(any(
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    feature = "stm32l4r7",
    feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9"
)))]
pub const BACKUP_REGISTER_COUNT: usize = 20;
#[cfg(any(
    feature = "stm32l4r5",
    feature = "stm32l4s5",
    feature = "stm32l4r7",
    feature = "stm32l4s7",
    feature = "stm32l4r9",
    feature = "stm32l4s9"
))]
pub const BACKUP_REGISTER_COUNT: usize = 32;

/// Read content of the backup register.
///
/// The registers retain their values during wakes from standby mode or system resets. They also
/// retain their value when Vdd is switched off as long as V_BAT is powered.
pub fn read_backup_register(rtc: &RTC, register: usize) -> Option<u32> {
    if register < BACKUP_REGISTER_COUNT {
        Some(rtc.bkpr[register].read().bits())
    } else {
        None
    }
}

/// Set content of the backup register.
///
/// The registers retain their values during wakes from standby mode or system resets. They also
/// retain their value when Vdd is switched off as long as V_BAT is powered.
pub fn write_backup_register(rtc: &RTC, register: usize, value: u32) {
    if register < BACKUP_REGISTER_COUNT {
        unsafe { rtc.bkpr[register].write(|w| w.bits(value)) }
    }
}
