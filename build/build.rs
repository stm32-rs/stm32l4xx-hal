use crate::features::*;

pub(crate) mod features;

pub(crate) fn feature_validate() -> bool {
    const DEVICE_FEATURES: &[bool] = &[
        IS_FEATURE_ENABLED_L412,
        IS_FEATURE_ENABLED_L422,
        IS_FEATURE_ENABLED_L431,
        IS_FEATURE_ENABLED_L432,
        IS_FEATURE_ENABLED_L433,
        IS_FEATURE_ENABLED_L442,
        IS_FEATURE_ENABLED_L442,
        IS_FEATURE_ENABLED_L443,
        IS_FEATURE_ENABLED_L451,
        IS_FEATURE_ENABLED_L452,
        IS_FEATURE_ENABLED_L462,
        IS_FEATURE_ENABLED_L471,
        IS_FEATURE_ENABLED_L475,
        IS_FEATURE_ENABLED_L476,
        IS_FEATURE_ENABLED_L486,
        IS_FEATURE_ENABLED_L496,
        IS_FEATURE_ENABLED_L4A6,
        //IS_FEATURE_ENABLED_L4P5,
        //IS_FEATURE_ENABLED_L4Q5,
        //IS_FEATURE_ENABLED_L4R5,
        //IS_FEATURE_ENABLED_L4S5,
        //IS_FEATURE_ENABLED_L4R7,
        //IS_FEATURE_ENABLED_L4S7,
        IS_FEATURE_ENABLED_L4R9,
        IS_FEATURE_ENABLED_L4S9,
    ];
    DEVICE_FEATURES.iter().filter(|&&b| b).count() == 1
}

fn main() {
    if !feature_validate() {
        panic!(
            "
This crate requires exactly one of the following features to be enabled:
    stm32l431, stm32l451, stm32l471
    stm32l412, stm32l422, stm32l432, stm32l442, stm32l452, stm32l462
    stm32l433, stm32l443
    stm32l475, 
    stm32l476, stm32l486, stm32l496, stm32l4a6
    stm32l4r9, stm32l4s9
"
        );
    }
}
