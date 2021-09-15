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

/// USART Marks pins as being TX Half Duplex pins for the given USART instance
pub trait TxHalfDuplexPin<Instance>: private::Sealed {}

/// USART Marks pins as being as being RX pins for the given USART instance
pub trait RxPin<Instance>: private::Sealed {}

/// USART Marks pins as being as being RTS pins for the given USART instance
pub trait RtsDePin<Instance>: private::Sealed {}

/// USART Marks pins as being as being CTS pins for the given USART instance
pub trait CtsPin<Instance>: private::Sealed {}

#[allow(unused)]
use super::gpio::{
    Alternate, AF0, AF1, AF10, AF11, AF12, AF13, AF14, AF15, AF2, AF3, AF4, AF5, AF6, AF7, AF8, AF9, 
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
};
#[allow(unused)]
use crate::pac::{I2C1, I2C2, I2C3, USART1, USART2, USART3};

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

af_table_af0af7! {
//      AF0 AF1 AF2      AF3          AF4        AF5      AF6          AF7
[ PA0  | _ | _ | _ |      _      |     _      |   _    |   _    |  CtsPin<USART2>]
[ PA1  | _ | _ | _ |      _      |     _      |   _    |   _    |RtsDePin<USART2>]
[ PA2  | _ | _ | _ |      _      |     _      |   _    |   _    |  TxPin<USART2> ]
[ PA3  | _ | _ | _ |      _      |     _      |   _    |   _    |  RxPin<USART2> ]
[ PA4  | _ | _ | _ |      _      |     _      |   _    |   _    |       _        ]
[ PA5  | _ | _ | _ |      _      |     _      |   _    |   _    |       _        ]
[ PA6  | _ | _ | _ |      _      |     _      |   _    |   _    |  CtsPin<USART3>]
[ PA7  | _ | _ | _ |      _      |SclPin<I2C3>|   _    |   _    |       _        ]
[ PA8  | _ | _ | _ |      _      |     _      |   _    |   _    |       _        ]
[ PA9  | _ | _ | _ |      _      |SclPin<I2C1>|   _    |   _    |  TxPin<USART1> ]
[ PA10 | _ | _ | _ |      _      |SdaPin<I2C1>|   _    |   _    |  RxPin<USART1> ]
[ PA11 | _ | _ | _ |      _      |     _      |   _    |   _    |  CtsPin<USART1>]
[ PA12 | _ | _ | _ |      _      |     _      |   _    |   _    |RtsDePin<USART1>]
[ PA13 | _ | _ | _ |      _      |     _      |   _    |   _    |       _        ]
[ PA14 | _ | _ | _ |      _      |     _      |   _    |   _    |       _        ]
[ PA15 | _ | _ | _ |RxPin<USART2>|     _      |   _    |   _    |RtsDePin<USART3>]
}
