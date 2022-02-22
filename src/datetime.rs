//! Date and timer units & helper functions

pub use fugit::{
    HoursDurationU32 as Hour, MicrosDurationU32 as Micros, MinutesDurationU32 as Minute,
    SecsDurationU32 as Second,
};

/// Day (1-7)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Day(pub u32);

/// Date (1-31)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DateInMonth(pub u32);

/// Week (1-52)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Week(pub u32);

/// Month (1-12)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Month(pub u32);

/// Year
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Year(pub u32);

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    fn day(self) -> Day;
    /// Seconds
    fn date(self) -> DateInMonth;
    /// Month
    fn month(self) -> Month;
    /// Year
    fn year(self) -> Year;
}

impl U32Ext for u32 {
    fn day(self) -> Day {
        Day(self)
    }

    fn date(self) -> DateInMonth {
        DateInMonth(self)
    }

    fn month(self) -> Month {
        Month(self)
    }

    fn year(self) -> Year {
        Year(self)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Time {
    pub hours: u32,
    pub minutes: u32,
    pub seconds: u32,
    pub micros: u32,
    pub daylight_savings: bool,
}

impl Time {
    pub fn new(
        hours: Hour,
        minutes: Minute,
        seconds: Second,
        micros: Micros,
        daylight_savings: bool,
    ) -> Self {
        Self {
            hours: hours.ticks(),
            minutes: minutes.ticks(),
            seconds: seconds.ticks(),
            micros: micros.ticks(),
            daylight_savings,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Date {
    pub day: u32,
    pub date: u32,
    pub month: u32,
    pub year: u32,
}

impl Date {
    pub fn new(day: Day, date: DateInMonth, month: Month, year: Year) -> Self {
        Self {
            day: day.0,
            date: date.0,
            month: month.0,
            year: year.0,
        }
    }
}

macro_rules! impl_from_struct {
    ($(
        $type:ident: [ $($to:ident),+ ],
    )+) => {
        $(
            $(
                impl From <$type> for $to {
                    fn from(inner: $type) -> $to {
                        inner.0 as $to
                    }
                }
            )+
        )+
    }
}

macro_rules! impl_to_struct {
    ($(
        $type:ident: [ $($to:ident),+ ],
    )+) => {
        $(
            $(
                impl From <$type> for $to {
                    fn from(inner: $type) -> $to {
                        $to(inner as u32)
                    }
                }
            )+
        )+
    }
}

impl_from_struct!(
    Day: [u32, u16, u8],
    DateInMonth: [u32, u16, u8],
    Month: [u32, u16, u8],
    Year: [u32, u16, u8],
);

impl_to_struct!(
    u32: [Day, DateInMonth, Month, Year],
    u16: [Day, DateInMonth, Month, Year],
    u8: [Day, DateInMonth, Month, Year],
);
