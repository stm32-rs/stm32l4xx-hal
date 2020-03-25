//! Date and timer units & helper functions

/// SubSeconds
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SubSecondFraction(pub f32);

/// Seconds
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Second(pub u32);

/// Minutes
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Minute(pub u32);

/// Hours
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hour(pub u32);

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
    /// Seconds
    fn seconds(self) -> Second;
    /// Minutes
    fn minutes(self) -> Minute;
    /// Hours
    fn hours(self) -> Hour;
    /// Day
    fn day(self) -> Day;
    /// Seconds
    fn date(self) -> DateInMonth;
    /// Month
    fn month(self) -> Month;
    /// Year
    fn year(self) -> Year;
}

pub trait F32Ext{
    /// Seconds
    fn sub_second_fraction(self) -> SubSecondFraction;
}

impl U32Ext for u32 {
    fn seconds(self) -> Second {
        Second(self)
    }

    fn minutes(self) -> Minute {
        Minute(self)
    }

    fn hours(self) -> Hour {
        Hour(self)
    }

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

impl F32Ext for f32 {
    fn sub_second_fraction(self) -> SubSecondFraction {
        SubSecondFraction(self)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Time {
    pub hours: u32,
    pub minutes: u32,
    pub seconds: u32,
    pub sub_second_fraction: f32,
    pub daylight_savings: bool
}

impl Time {
    pub fn new(hours: Hour, minutes: Minute, seconds: Second, sub_second_fraction: SubSecondFraction, daylight_savings: bool) -> Self {
        Self {
            hours: hours.0,
            minutes: minutes.0,
            seconds: seconds.0,
            sub_second_fraction : sub_second_fraction.0,
            daylight_savings
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
            year: year.0
        }
    }
}

impl Into<Second> for Minute {
    fn into(self) -> Second {
        Second(self.0 * 60)
    }
}

impl Into<Second> for Hour {
    fn into(self) -> Second {
        Second(self.0 * 3600)
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
    Hour: [u32, u16, u8],
    Second: [u32, u16, u8],
    Minute: [u32, u16, u8],
    Day: [u32, u16, u8],
    DateInMonth: [u32, u16, u8],
    Month: [u32, u16, u8],
    Year: [u32, u16, u8],
);

impl_to_struct!(
    u32: [Hour, Minute, Second, Day, DateInMonth, Month, Year],
    u16: [Hour, Minute, Second, Day, DateInMonth, Month, Year],
    u8: [Hour, Minute, Second, Day, DateInMonth, Month, Year],
);

impl From<f32> for SubSecondFraction{
    fn from(sub: f32) -> Self {
        SubSecondFraction(sub)
    }
}

impl From<SubSecondFraction> for f32 {
    fn from(sub : SubSecondFraction) -> f32 {
        sub.0
    }
}