#[doc(hidden)]
mod private {
    pub trait Sealed {}
}

/// I2C SCL pin. This trait is sealed and cannot be implemented.
pub trait SclPin<I2C>: private::Sealed {}
/// I2C SDA pin. This trait is sealed and cannot be implemented.
pub trait SdaPin<I2C>: private::Sealed {}

/// USART Marks pins as being as being TX pins for the given USART instance
pub trait TxPin<Instance>: private::Sealed {}
/// USART Marks pins as being as being RX pins for the given USART instance
pub trait RxPin<Instance>: private::Sealed {}
/// USART Marks pins as being as being RTS pins for the given USART instance
pub trait RtsDePin<Instance>: private::Sealed {}
/// USART Marks pins as being as being CTS pins for the given USART instance
pub trait CtsPin<Instance>: private::Sealed {}

/// PWM Marks a pin that can be used as pwm channel 1 for a given timer
pub trait PwmCh1<TIM>: private::Sealed {}
/// PWM Marks a pin that can be used as pwm channel 2 for a given timer
pub trait PwmCh2<TIM>: private::Sealed {}
/// PWM Marks a pin that can be used as pwm channel 3 for a given timer
pub trait PwmCh3<TIM>: private::Sealed {}
/// PWM Marks a pin that can be used as pwm channel 4 for a given timer
pub trait PwmCh4<TIM>: private::Sealed {}

/// SPI SCK pin. This trait is sealed and cannot be implemented.
pub trait SckPin<SPI>: private::Sealed {}
/// SPI MISO pin. This trait is sealed and cannot be implemented.
pub trait MisoPin<SPI>: private::Sealed {}
/// SPI MOSI pin. This trait is sealed and cannot be implemented.
pub trait MosiPin<SPI>: private::Sealed {}

/// QSPI Marks a pin that can be used as clock for QSPI
pub trait ClkPin<QSPI>: private::Sealed {}
/// QSPI Marks a pin that can be used as IO0 for QSPI
pub trait IO0Pin<QSPI>: private::Sealed {}
/// QSPI Marks a pin that can be used as IO1 for QSPI
pub trait IO1Pin<QSPI>: private::Sealed {}
/// QSPI Marks a pin that can be used as IO2 for QSPI
pub trait IO2Pin<QSPI>: private::Sealed {}
/// QSPI Marks a pin that can be used as IO3 for QSPI
pub trait IO3Pin<QSPI>: private::Sealed {}
/// QSPI Marks a pin that can be used as NcsPin for QSPI
pub trait NcsPin<QSPI>: private::Sealed {}

/// TSC Marks a pin that can be used for the touch sensing controller
pub trait TscPin<const GROUP: usize, const IO: usize>: private::Sealed {}

/// CAN Marks a pin that can be used as a can tx pin
pub trait CanTxPin<CAN>: private::Sealed {}
/// CAN Marks a pin that can be used as a can rx pin
pub trait CanRxPin<CAN>: private::Sealed {}

// use gpio::Alternate, AF0..15, PA0..15, PB...
use super::gpio::*;
#[allow(unused)]
use crate::pac::{
    CAN1, I2C1, I2C2, I2C3, QUADSPI, SPI1, TIM1, TIM15, TIM2, TIM3, USART1, USART2, USART3,
};

// Implements one of the traits above for a given Pin-AF-combination. For
// example, `afpin!(PA0, AF1, SclPin<I2C2>);` allows PA0 in AF1 configuration
// to be used as SclPin for I2C2.
macro_rules! afpin {
    ($pin:ident, $af:ty, $trait:ty) => {
        impl<OTYPE> private::Sealed for $pin<Alternate<$af, OTYPE>> {}
        impl<OTYPE> $trait for $pin<Alternate<$af, OTYPE>> {}
    };
}

// Helper macros for the af_row_af0af7! macro. This is required because we do not
// want to match over all possible (256) trait/no-trait combinations in a single
// row. So we do it step by step for every single column.
macro_rules! AF0 {
    // No trait to implement for this pin on AF0
    ($pin:ident, _ | $($tail:tt)*) => {
        // Just forward the rest of this row ($tail) to AF1
        AF1!($pin, $($tail)*);
    };
    // Implement a trait for this pin on AF0
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        // Implement the trait
        afpin!($pin, AF0, $trait);
        // Forward the rest of this row to AF1
        AF1!($pin, $($tail)*);
    };
}
macro_rules! AF1 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF2!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF1, $trait);
        AF2!($pin, $($tail)*);
    };
}
macro_rules! AF2 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF3!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF2, $trait);
        AF3!($pin, $($tail)*);
    };
}
macro_rules! AF3 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF4!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF3, $trait);
        AF4!($pin, $($tail)*);
    };
}
macro_rules! AF4 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF5!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF4, $trait);
        AF5!($pin, $($tail)*);
    };
}
macro_rules! AF5 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF6!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF5, $trait);
        AF6!($pin, $($tail)*);
    };
}
macro_rules! AF6 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF7!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF6, $trait);
        AF7!($pin, $($tail)*);
    };
}
macro_rules! AF7 {
    ($pin:ident, _) => {};
    ($pin:ident, $trait:ty) => {
        afpin!($pin, AF7, $trait);
    };
}

// Helper macro for the af_table_af0af7! macro which handles a single row. This
// just delegates the work to the AF0! .. AF7! macros.
macro_rules! af_row_af0af7 {
    ($pin:ident | $($tail:tt)*) => {
        AF0!($pin, $($tail)*);
    }
    /* The code below is a first approach in which a whole row is handled
     * in this macro itself by recursion. But we know that the AFx are handled
     * one after another so we can directly forward it to the correct AFx-macro
     * which avoids unsuccessful matchings here.
     *
    // Initialization with no AF0 trait, e.g. PAx | _ | CtsPin<...> | ...
    ($pin:ident| _| $($tail:tt)*) => {
        af_row_af0af7!($pin, AF1, $($tail)*);
    };
    // Initialization _with_ AF0 trait, e.g. PAx | RtsDePin<...> | RxPin<...> | _ | ...
    ($pin:ident| $trait:ty| $($tail:tt)*) => {
        afpin!($pin, AF0, $trait);
        af_row_af0af7!($pin, AF1, $($tail)*);
    };
    ($pin:ident, AF1, _ | $($tail:tt)*) => {
        af_row_af0af7!($pin, AF2, $($tail)*);
    };
    ($pin:ident, AF1, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF1, $trait);
        af_row_af0af7!($pin, AF2, $($tail)*);
    };
    ($pin:ident, AF2, _ | $($tail:tt)*) => {
        af_row_af0af7!($pin, AF3, $($tail)*);
    };
    ($pin:ident, AF2, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF2, $trait);
        af_row_af0af7!($pin, AF3, $($tail)*);
    };
    ($pin:ident, AF3, _ | $($tail:tt)*) => {
        af_row_af0af7!($pin, AF4, $($tail)*);
    };
    ($pin:ident, AF3, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF3, $trait);
        af_row_af0af7!($pin, AF4, $($tail)*);
    };
    ($pin:ident, AF4, _ | $($tail:tt)*) => {
        af_row_af0af7!($pin, AF5, $($tail)*);
    };
    ($pin:ident, AF4, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF4, $trait);
        af_row_af0af7!($pin, AF5, $($tail)*);
    };
    ($pin:ident, AF5, _ | $($tail:tt)*) => {
        af_row_af0af7!($pin, AF6, $($tail)*);
    };
    ($pin:ident, AF5, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF5, $trait);
        af_row_af0af7!($pin, AF6, $($tail)*);
    };
    ($pin:ident, AF6, _ | $($tail:tt)*) => {
        af_row_af0af7!($pin, AF7, $($tail)*);
    };
    ($pin:ident, AF6, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF6, $trait);
        af_row_af0af7!($pin, AF7, $($tail)*);
    };
    ($pin:ident, AF7, _) => {
    };
    ($pin:ident, AF7, $trait:ty) => {
        afpin!($pin, AF2, $trait);
    };
    */
}

//af_row_af0af7!(PA0|  _    |   _    |   _    |   _    |   _    |   _    |   _    |CtsPin<USART2>);

// Implements the traits required for a whole alternate functions table as it
// is found in the datasheets of the different MCUs
macro_rules! af_table_af0af7 {
    () => {};
    ([$($row:tt)*] $($tail:tt)*) => {
        af_row_af0af7!($($row)*);
        af_table_af0af7!($($tail)*);
    };
}

// See AF0! macro
macro_rules! AF8 {
    // No trait to implement for this pin on AF0
    ($pin:ident, _ | $($tail:tt)*) => {
        // Just forward the rest of this row ($tail) to AF1
        AF9!($pin, $($tail)*);
    };
    // Implement a trait for this pin on AF0
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        // Implement the trait
        afpin!($pin, AF8, $trait);
        // Forward the rest of this row to AF1
        AF9!($pin, $($tail)*);
    };
}
macro_rules! AF9 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF10!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF9, $trait);
        AF10!($pin, $($tail)*);
    };
}
macro_rules! AF10 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF11!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF10, $trait);
        AF11!($pin, $($tail)*);
    };
}
macro_rules! AF11 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF12!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF11, $trait);
        AF12!($pin, $($tail)*);
    };
}
macro_rules! AF12 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF13!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF12, $trait);
        AF13!($pin, $($tail)*);
    };
}
macro_rules! AF13 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF14!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF13, $trait);
        AF14!($pin, $($tail)*);
    };
}
macro_rules! AF14 {
    ($pin:ident, _ | $($tail:tt)*) => {
        AF15!($pin, $($tail)*);
    };
    ($pin:ident, $trait:ty | $($tail:tt)*) => {
        afpin!($pin, AF14, $trait);
        AF15!($pin, $($tail)*);
    };
}
macro_rules! AF15 {
    ($pin:ident, _) => {};
    ($pin:ident, $trait:ty) => {
        afpin!($pin, AF15, $trait);
    };
}

// See af_row_af0af7! macro
macro_rules! af_row_af8af15 {
    ($pin:ident | $($tail:tt)*) => {
        AF8!($pin, $($tail)*);
    }
}

// See af_table_af0af7! macro
macro_rules! af_table_af8af15 {
    () => {};
    ([$($row:tt)*] $($tail:tt)*) => {
        af_row_af8af15!($($row)*);
        af_table_af8af15!($($tail)*);
    };
}

#[cfg(feature = "stm32l452")]
mod stm32l452;

// Fallback feature gates
#[cfg(feature = "stm32l4x1")]
mod stm32l4x1;

/* TODO
#[cfg(all(feature = "stm32l4x2", not(any(
    feature = "stm32l452",
))))]
mod stm32l4x2;

#[cfg(feature = "stm32l4x3")]

#[cfg(feature = "stm32l4x5")]

#[cfg(feature = "stm32l4x6")]
*/
