use crate::{features::*, FeatureGate};

/// list of peripherals to be gated and whether they are present for the selected features
/// they can then be checked in the library using
///
/// ```Rust
/// #[cfg(condition = peripheral_<name>)]
/// ```
pub(crate) const PERIPHERAL_FEATURES: &[FeatureGate] = &[
    FeatureGate {
        name: "adc2",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5,
    },
    FeatureGate {
        name: "adc3",
        state: IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6,
    },
    FeatureGate {
        name: "dac1",
        state: IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "comp1",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "comp2",
        state: IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "dfsdm1",
        state: IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "lcd",
        state: IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6,
    },
    FeatureGate {
        name: "aes",
        state: IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "tim3",
        state: IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "tim7",
        state: IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443,
    },
    FeatureGate {
        name: "i2c2",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "i2c2",
        state: !(IS_FEATURE_ENABLED_L432 || IS_FEATURE_ENABLED_L442),
    },
    FeatureGate {
        name: "i2c4",
        state: IS_FEATURE_ENABLED_L451
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "usart3",
        state: !(IS_FEATURE_ENABLED_L432 || IS_FEATURE_ENABLED_L442),
    },
    FeatureGate {
        name: "uart4",
        state: //IS_FEATURE_ENABLED_L451 -- missing PAC support
            //|| IS_FEATURE_ENABLED_L452 -- missing PAC support
            //|| IS_FEATURE_ENABLED_L462 -- missing PAC support
            //|| IS_FEATURE_ENABLED_L471 -- missing PAC support
             IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "uart5",
        state: //IS_FEATURE_ENABLED_L471 -- missing PAC support
             IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "spi2",
        state: !(IS_FEATURE_ENABLED_L432 || IS_FEATURE_ENABLED_L442),
    },
    FeatureGate {
        name: "spi3",
        state: !(IS_FEATURE_ENABLED_L412 || IS_FEATURE_ENABLED_L422),
    },
    FeatureGate {
        name: "sai1",
        state: !(IS_FEATURE_ENABLED_L412 || IS_FEATURE_ENABLED_L422),
    },
    FeatureGate {
        name: "swpmi1",
        state: IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6,
    },
    FeatureGate {
        name: "sdmmc",
        state: !(IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L442),
    },
    FeatureGate {
        name: "usb_device_fs",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462,
    },
    FeatureGate {
        name: "usb_otg_fs",
        state: IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "can1",
        state: !(IS_FEATURE_ENABLED_L412 || IS_FEATURE_ENABLED_L422),
    },
    FeatureGate {
        name: "rtc_type3",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5,
    },
    FeatureGate {
        name: "gpiog",
        state: //IS_FEATURE_ENABLED_L471 -- missing PAC support
                IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
    FeatureGate {
        name: "gpiof",
        state: //IS_FEATURE_ENABLED_L471 -- missing PAC support
                IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6
            || IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
];
