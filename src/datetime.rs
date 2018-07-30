/// Date and timer units & helper functions

/// Seconds
#[derive(Clone, Copy)]
pub struct Seconds(pub u32);

/// Minutes
#[derive(Clone, Copy)]
pub struct Minutes(pub u32);

/// Hours
#[derive(Clone, Copy)]
pub struct Hours(pub u32);

/// Day (1-7)
#[derive(Clone, Copy)]
pub struct Day(pub u32);

/// Date (1-31)
#[derive(Clone, Copy)]
pub struct Date(pub u32);

/// Week (1-52)
#[derive(Clone, Copy)]
pub struct Week(pub u32);

/// Month (1-12)
#[derive(Clone, Copy)]
pub struct Month(pub u32);

/// Year
#[derive(Clone, Copy)]
pub struct Year(pub u32);

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Seconds
    fn seconds(self) -> Seconds;
    /// Minutes
    fn minutes(self) -> Minutes;
    /// Hours
    fn hours(self) -> Hours;
    /// Day
    fn day(self) -> Day;
    /// Seconds
    fn date(self) -> Date;
    /// Month
    fn month(self) -> Month;
    /// Year
    fn year(self) -> Year;
}

impl U32Ext for u32 {
    fn seconds(self) -> Seconds {
        Seconds(self)
    }

    fn minutes(self) -> Minutes {
        Minutes(self)
    }

    fn hours(self) -> Hours {
        Hours(self)
    }

    fn day(self) -> Day {
        Day(self)
    }

    fn date(self) -> Date {
        Date(self)
    }

    fn month(self) -> Month {
        Month(self)
    }

    fn year(self) -> Year {
        Year(self)
    }
}

impl Into<Seconds> for Minutes {
    fn into(self) -> Seconds {
        Seconds(self.0 * 60)
    }
}

impl Into<Seconds> for Hours {
    fn into(self) -> Seconds {
        Seconds(self.0 * 3600)
    }
}
