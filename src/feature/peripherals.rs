macro_rules! if_adc2 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l471",
            feature = "stm32l475",
            feature = "stm32l476",
            feature = "stm32l486",
            feature = "stm32l496",
            feature = "stm32l4a6",
            feature = "stm32l4p5",
            feature = "stm32l4q5",
            feature = "stm32l4r5",
            feature = "stm32l4s5",
            feature = "stm32l4r7",
            feature = "stm32l4s7",
            feature = "stm32l4r9",
            feature = "stm32l4s9",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l471",
            feature = "stm32l475",
            feature = "stm32l476",
            feature = "stm32l486",
            feature = "stm32l496",
            feature = "stm32l4a6",
            feature = "stm32l4p5",
            feature = "stm32l4q5",
            feature = "stm32l4r5",
            feature = "stm32l4s5",
            feature = "stm32l4r7",
            feature = "stm32l4s7",
            feature = "stm32l4r9",
            feature = "stm32l4s9",
        )))]
        $ex
    };
}
pub(crate) use if_adc2;

macro_rules! if_adc3 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l471",
            feature = "stm32l475",
            feature = "stm32l476",
            feature = "stm32l486",
            feature = "stm32l496",
            feature = "stm32l4a6",
            feature = "stm32l4p5",
            feature = "stm32l4q5",
            feature = "stm32l4r5",
            feature = "stm32l4s5",
            feature = "stm32l4r7",
            feature = "stm32l4s7",
            feature = "stm32l4r9",
            feature = "stm32l4s9",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l471",
            feature = "stm32l475",
            feature = "stm32l476",
            feature = "stm32l486",
            feature = "stm32l496",
            feature = "stm32l4a6",
            feature = "stm32l4p5",
            feature = "stm32l4q5",
            feature = "stm32l4r5",
            feature = "stm32l4s5",
            feature = "stm32l4r7",
            feature = "stm32l4s7",
            feature = "stm32l4r9",
            feature = "stm32l4s9",
        )))]
        $ex
    };
}
pub(crate) use if_adc3;

macro_rules! if_dac1 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_dac1;

macro_rules! if_comp1 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_comp1;

macro_rules! if_comp2 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_comp2;

macro_rules! if_dfsdm1 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_dfsdm1;

macro_rules! if_lcd {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l433",
            feature = "stm32l443",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l433",
            feature = "stm32l443",
        )))]
        $ex
    };
}
pub(crate) use if_lcd;

macro_rules! if_aes {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l422",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l422",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_aes;

macro_rules! if_tim3 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_tim3;

macro_rules! if_tim7 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
        )))]
        $ex
    };
}
pub(crate) use if_tim7;

macro_rules! if_i2c3 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l431",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l431",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_i2c3;

macro_rules! if_i2c4 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_i2c4;

macro_rules! if_usart3 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l431",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l431",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_usart3;

macro_rules! if_uart4 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_uart4;

macro_rules! if_spi2 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l431",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l431",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_spi2;

macro_rules! if_spi3 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_spi3;

macro_rules! if_sai {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_sai;

macro_rules! if_swpmi1 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
        )))]
        $ex
    };
}
pub(crate) use if_swpmi1;

macro_rules! if_sdmmc {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_sdmmc;

macro_rules! if_usb_fs {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l412",
            feature = "stm32l422",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_usb_fs;

macro_rules! if_can1 {
    (present: $ex:expr) => {
        #[cfg(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        ))]
        $ex
    };
    (absent: $ex:expr) => {
        #[cfg(not(any(
            feature = "stm32l431",
            feature = "stm32l432",
            feature = "stm32l433",
            feature = "stm32l442",
            feature = "stm32l443",
            feature = "stm32l451",
            feature = "stm32l452",
            feature = "stm32l462",
        )))]
        $ex
    };
}
pub(crate) use if_can1;

