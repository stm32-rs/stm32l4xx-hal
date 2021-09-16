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

// use gpio::Alternate, AF0..15, PA0..15, PB...
use super::gpio::*;
#[allow(unused)]
use crate::pac::{
    I2C1, I2C2, I2C3, QUADSPI, SPI1, TIM1, TIM15, TIM2, TIM3, USART1, USART2, USART3,
};

macro_rules! afpin {
    ($pin:ident, $af:ty, $trait:ty) => {
        impl<OTYPE> private::Sealed for $pin<Alternate<$af, OTYPE>> {}
        impl<OTYPE> $trait for $pin<Alternate<$af, OTYPE>> {}
    };
}

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

macro_rules! af_row_af0af7 {
    ($pin:ident | $($tail:tt)*) => {
        AF0!($pin, $($tail)*);
    }
    /* The code below is a first approach in which a whole row is handled
     * in this macro itself by recursion. But we known that the AFx are handled
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

macro_rules! af_table_af0af7 {
    () => {};
    ([$($row:tt)*] $($tail:tt)*) => {
        af_row_af0af7!($($row)*);
        af_table_af0af7!($($tail)*);
    };
}

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

macro_rules! af_row_af8af15 {
    ($pin:ident | $($tail:tt)*) => {
        AF8!($pin, $($tail)*);
    }
}

macro_rules! af_table_af8af15 {
    () => {};
    ([$($row:tt)*] $($tail:tt)*) => {
        af_row_af8af15!($($row)*);
        af_table_af8af15!($($tail)*);
    };
}

// stm32l452xx datasheet DS11912 Rev7 page 76
af_table_af0af7! {
//      AF0      AF1          AF2          AF3          AF4        AF5      AF6          AF7
[ PA0  | _ |PwmCh1<TIM2>|      _     |      _      |     _      |      _      |   _    |  CtsPin<USART2>]
[ PA1  | _ |PwmCh2<TIM2>|      _     |      _      |     _      |SckPin<SPI1> |   _    |RtsDePin<USART2>]
[ PA2  | _ |PwmCh3<TIM2>|      _     |      _      |     _      |      _      |   _    |  TxPin<USART2> ]
[ PA3  | _ |PwmCh4<TIM2>|      _     |      _      |     _      |      _      |   _    |  RxPin<USART2> ]
[ PA4  | _ |     _      |      _     |      _      |     _      |      _      |   _    |       _        ]
[ PA5  | _ |PwmCh1<TIM2>|      _     |      _      |     _      |SckPin<SPI1> |   _    |       _        ]
[ PA6  | _ |     _      |PwmCh1<TIM3>|      _      |     _      |MisoPin<SPI1>|   _    |  CtsPin<USART3>]
[ PA7  | _ |     _      |PwmCh2<TIM3>|      _      |SclPin<I2C3>|MosiPin<SPI1>|   _    |       _        ]
[ PA8  | _ |PwmCh1<TIM1>|      _     |      _      |     _      |      _      |   _    |       _        ]
[ PA9  | _ |PwmCh2<TIM1>|      _     |      _      |SclPin<I2C1>|      _      |   _    |  TxPin<USART1> ]
[ PA10 | _ |PwmCh3<TIM1>|      _     |      _      |SdaPin<I2C1>|      _      |   _    |  RxPin<USART1> ]
[ PA11 | _ |PwmCh4<TIM1>|      _     |      _      |     _      |MisoPin<SPI1>|   _    |  CtsPin<USART1>]
[ PA12 | _ |     _      |      _     |      _      |     _      |MosiPin<SPI1>|   _    |RtsDePin<USART1>]
[ PA13 | _ |     _      |      _     |      _      |     _      |      _      |   _    |       _        ]
[ PA14 | _ |     _      |      _     |      _      |     _      |      _      |   _    |       _        ]
[ PA15 | _ |PwmCh1<TIM2>|      _     |RxPin<USART2>|     _      |      _      |   _    |RtsDePin<USART3>]
}

// stm32l452xx datasheet page 82, only qspi
af_table_af8af15! {
//      AF8
[ PA0  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA1  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA2  | _ | _ |NcsPin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PA3  | _ | _ |ClkPin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PA4  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA5  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA6  | _ | _ |IO3Pin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PA7  | _ | _ |IO2Pin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PA8  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA9  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA10 | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA11 | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA12 | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA13 | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA14 | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PA15 | _ | _ |       _       | _ | _ | _ | _ | _ ]
}

// stm32l452xx datasheet page 83, only qspi and tsc
af_table_af8af15! {
//      AF8
[ PB0  | _ |      _     |IO1Pin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PB1  | _ |      _     |IO0Pin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PB2  | _ |      _     |       _       | _ | _ | _ | _ | _ ]
[ PB3  | _ |      _     |       _       | _ | _ | _ | _ | _ ]
[ PB4  | _ |TscPin<2, 1>|       _       | _ | _ | _ | _ | _ ]
[ PB5  | _ |TscPin<2, 2>|       _       | _ | _ | _ | _ | _ ]
[ PB6  | _ |TscPin<2, 3>|       _       | _ | _ | _ | _ | _ ]
[ PB7  | _ |TscPin<2, 4>|       _       | _ | _ | _ | _ | _ ]
[ PB8  | _ |      _     |       _       | _ | _ | _ | _ | _ ]
[ PB9  | _ |      _     |       _       | _ | _ | _ | _ | _ ]
[ PB10 | _ |      _     |ClkPin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PB11 | _ |      _     |NcsPin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PB12 | _ |TscPin<1, 1>|       _       | _ | _ | _ | _ | _ ]
[ PB13 | _ |TscPin<1, 2>|       _       | _ | _ | _ | _ | _ ]
[ PB14 | _ |TscPin<1, 3>|       _       | _ | _ | _ | _ | _ ]
[ PB15 | _ |TscPin<1, 4>|       _       | _ | _ | _ | _ | _ ]
}

// stm32l452xx datasheet page 86, only qspi
af_table_af8af15! {
//      AF8
[ PE0  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE1  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE2  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE3  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE4  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE5  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE6  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE7  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE8  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE9  | _ | _ |       _       | _ | _ | _ | _ | _ ]
[ PE10 | _ | _ |ClkPin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PE11 | _ | _ |NcsPin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PE12 | _ | _ |IO0Pin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PE13 | _ | _ |IO1Pin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PE14 | _ | _ |IO2Pin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PE15 | _ | _ |IO3Pin<QUADSPI>| _ | _ | _ | _ | _ ]
}
