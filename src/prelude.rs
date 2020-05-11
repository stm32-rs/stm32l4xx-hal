//! Prelude - Include traits for hal

pub use crate::hal::digital::v2::*;
pub use crate::hal::prelude::*; // embedded hal traits // for some reason v2 is not exported in the ehal prelude

pub use crate::crc::CrcExt as _stm32l4_hal_CrcExt;
pub use crate::datetime::U32Ext as _stm32l4_hal_datetime_U32Ext;
pub use crate::dma::DmaExt as _stm32l4_hal_DmaExt;
pub use crate::flash::FlashExt as _stm32l4_hal_FlashExt;
pub use crate::gpio::ExtiPin as _stm32l4_hal_ExtiPin;
pub use crate::gpio::GpioExt as _stm32l4_hal_GpioExt;
pub use crate::pwm::PwmExt as _stm32l4_hal_PwmExt;
pub use crate::pwr::PwrExt as _stm32l4_hal_PwrExt;
pub use crate::rcc::RccExt as _stm32l4_hal_RccExt;
pub use crate::rng::RngExt as _stm32l4_hal_RngExt;
pub use crate::time::U32Ext as _stm32l4_hal_time_U32Ext;
