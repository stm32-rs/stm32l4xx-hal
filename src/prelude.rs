//! Prelude - Include traits for hal

pub use rcc::RccExt as _RccExtHal;
pub use flash::FlashExt as _FlashExtHal;
pub use gpio::GpioExt as _GpioExtHal;
pub use hal::prelude::*;