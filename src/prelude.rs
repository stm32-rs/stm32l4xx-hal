//! Prelude - Include traits for hal

pub use rcc::RccExt as _RccExtHal;
pub use flash::FlashExt as _FlashExtHal;
pub use gpio::GpioExt as _GpioExtHal;
pub use hal::prelude::*; // embedded hal traits
pub use time::U32Ext as _stm32f30x_hal_time_U32Ext;
pub use datetime::U32Ext as _stm32f30x_hal_datetime;
pub use dma::DmaExt as _DmaExtHal;