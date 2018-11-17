//! Prelude - Include traits for hal

pub use crate::rcc::RccExt as _RccExtHal;
pub use crate::flash::FlashExt as _FlashExtHal;
pub use crate::gpio::GpioExt as _GpioExtHal;
pub use crate::hal::prelude::*; // embedded hal traits
pub use crate::time::U32Ext as _stm32f30x_hal_time_U32Ext;
pub use crate::datetime::U32Ext as _stm32f30x_hal_datetime;
pub use crate::dma::DmaExt as _DmaExtHal;
pub use crate::pwr::PwrExt as _PwrExtHal;