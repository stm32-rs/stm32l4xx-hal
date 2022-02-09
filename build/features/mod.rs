pub(crate) mod family;
pub(crate) mod peripherals;

/// check that a valid combination of externally visible features has been enabled by dependant crates
/// this check asserts:
/// * exactly one of the device selection features is enabled (e.g. stm32l432)
pub(crate) fn validate_selected_features() -> bool {
    let check_selected_device = DEVICE_FEATURES.iter().filter(|&&b| b).count() == 1;

    check_selected_device
}

pub(crate) fn generate_internal_features() {
    for gate in peripherals::PERIPHERAL_FEATURES {
        if gate.state {
            println!(r#"cargo:rustc-cfg=condition="peripheral_{}""#, gate.name);
        }
    }
    for gate in family::DEVICE_FAMILY {
        if gate.state {
            println!(r#"cargo:rustc-cfg=condition="family_{}""#, gate.name);
        }
    }
}

pub(crate) const IS_FEATURE_ENABLED_L412: bool = cfg!(feature = "stm32l412");
pub(crate) const IS_FEATURE_ENABLED_L422: bool = cfg!(feature = "stm32l422");
pub(crate) const IS_FEATURE_ENABLED_L431: bool = cfg!(feature = "stm32l431");
pub(crate) const IS_FEATURE_ENABLED_L432: bool = cfg!(feature = "stm32l432");
pub(crate) const IS_FEATURE_ENABLED_L433: bool = cfg!(feature = "stm32l433");
pub(crate) const IS_FEATURE_ENABLED_L442: bool = cfg!(feature = "stm32l442");
pub(crate) const IS_FEATURE_ENABLED_L443: bool = cfg!(feature = "stm32l443");
pub(crate) const IS_FEATURE_ENABLED_L451: bool = cfg!(feature = "stm32l451");
pub(crate) const IS_FEATURE_ENABLED_L452: bool = cfg!(feature = "stm32l452");
pub(crate) const IS_FEATURE_ENABLED_L462: bool = cfg!(feature = "stm32l462");
pub(crate) const IS_FEATURE_ENABLED_L471: bool = cfg!(feature = "stm32l471");
pub(crate) const IS_FEATURE_ENABLED_L475: bool = cfg!(feature = "stm32l475");
pub(crate) const IS_FEATURE_ENABLED_L476: bool = cfg!(feature = "stm32l476");
pub(crate) const IS_FEATURE_ENABLED_L485: bool = cfg!(feature = "stm32l485");
pub(crate) const IS_FEATURE_ENABLED_L486: bool = cfg!(feature = "stm32l486");
pub(crate) const IS_FEATURE_ENABLED_L496: bool = cfg!(feature = "stm32l496");
pub(crate) const IS_FEATURE_ENABLED_L4A6: bool = cfg!(feature = "stm32l4a6");
pub(crate) const IS_FEATURE_ENABLED_L4P5: bool = cfg!(feature = "stm32l4p5");
pub(crate) const IS_FEATURE_ENABLED_L4Q5: bool = cfg!(feature = "stm32l4q5");
pub(crate) const IS_FEATURE_ENABLED_L4R5: bool = cfg!(feature = "stm32l4r5");
pub(crate) const IS_FEATURE_ENABLED_L4S5: bool = cfg!(feature = "stm32l4s5");
pub(crate) const IS_FEATURE_ENABLED_L4R7: bool = cfg!(feature = "stm32l4r7");
pub(crate) const IS_FEATURE_ENABLED_L4S7: bool = cfg!(feature = "stm32l4s7");
pub(crate) const IS_FEATURE_ENABLED_L4R9: bool = cfg!(feature = "stm32l4r9");
pub(crate) const IS_FEATURE_ENABLED_L4S9: bool = cfg!(feature = "stm32l4s9");

const DEVICE_FEATURES: &[bool] = &[
    IS_FEATURE_ENABLED_L412,
    IS_FEATURE_ENABLED_L422,
    IS_FEATURE_ENABLED_L431,
    IS_FEATURE_ENABLED_L432,
    IS_FEATURE_ENABLED_L433,
    IS_FEATURE_ENABLED_L442,
    IS_FEATURE_ENABLED_L443,
    IS_FEATURE_ENABLED_L451,
    IS_FEATURE_ENABLED_L452,
    IS_FEATURE_ENABLED_L462,
    IS_FEATURE_ENABLED_L471,
    IS_FEATURE_ENABLED_L475,
    IS_FEATURE_ENABLED_L476,
    IS_FEATURE_ENABLED_L485,
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
