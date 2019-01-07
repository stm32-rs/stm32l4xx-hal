//! Prelude - Include traits for hal

pub use crate::hal::prelude::*; // embedded hal traits

pub use crate::rcc::RccExt as _stm32l4_hal_RccExt;
pub use crate::flash::FlashExt as _stm32l4_hal_FlashExt;
pub use crate::gpio::GpioExt as _stm32l4_hal_GpioExt;
pub use crate::time::U32Ext as _stm32l4_hal_time_U32Ext;
pub use crate::datetime::U32Ext as _stm32l4_hal_datetime_U32Ext;
pub use crate::dma::DmaExt as _stm32l4_hal_DmaExt;
pub use crate::pwr::PwrExt as _stm32l4_hal_PwrExt;
pub use crate::rng::RngExt as _stm32l4_hal_RngExt;
