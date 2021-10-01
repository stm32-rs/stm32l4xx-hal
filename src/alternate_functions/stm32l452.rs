// TODO Not yet complete

use super::*; // Traits: SckPin, SclPin, ...
use crate::gpio::*; // Alternate Functions and Pins: AFx, PAx, PBx, ...
use crate::pac::*; // Peripherals: I2C1, I2C2, ...

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
[ PA0  | _ |       _      |       _       | _ | _ | _ | _ | _ ]
[ PA1  | _ |       _      |       _       | _ | _ | _ | _ | _ ]
[ PA2  | _ |       _      |NcsPin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PA3  | _ |       _      |ClkPin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PA4  | _ |       _      |       _       | _ | _ | _ | _ | _ ]
[ PA5  | _ |       _      |       _       | _ | _ | _ | _ | _ ]
[ PA6  | _ |       _      |IO3Pin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PA7  | _ |       _      |IO2Pin<QUADSPI>| _ | _ | _ | _ | _ ]
[ PA8  | _ |       _      |       _       | _ | _ | _ | _ | _ ]
[ PA9  | _ |       _      |       _       | _ | _ | _ | _ | _ ]
[ PA10 | _ |       _      |       _       | _ | _ | _ | _ | _ ]
[ PA11 | _ |CanRxPin<CAN1>|       _       | _ | _ | _ | _ | _ ]
[ PA12 | _ |CanTxPin<CAN1>|       _       | _ | _ | _ | _ | _ ]
[ PA13 | _ |       _      |       _       | _ | _ | _ | _ | _ ]
[ PA14 | _ |       _      |       _       | _ | _ | _ | _ | _ ]
[ PA15 | _ |       _      |       _       | _ | _ | _ | _ | _ ]
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
