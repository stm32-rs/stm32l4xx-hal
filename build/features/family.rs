use crate::{features::*, FeatureGate};

/// list of peripherals to be gated and whether they are present for the selected features
/// see [crate::features::generate_internal_features()] for how to reference these
pub(crate) const DEVICE_FAMILY: &[FeatureGate] = &[
    FeatureGate {
        name: "L4x1",
        state: IS_FEATURE_ENABLED_L431 || IS_FEATURE_ENABLED_L451 || IS_FEATURE_ENABLED_L471,
    },
    FeatureGate {
        name: "L4x2",
        state: IS_FEATURE_ENABLED_L412
            || IS_FEATURE_ENABLED_L422
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L452
            || IS_FEATURE_ENABLED_L462,
    },
    FeatureGate {
        name: "L4x3",
        state: IS_FEATURE_ENABLED_L433 || IS_FEATURE_ENABLED_L443,
    },
    FeatureGate {
        name: "L4x5",
        state: IS_FEATURE_ENABLED_L475 || IS_FEATURE_ENABLED_L485,
    },
    FeatureGate {
        name: "L4x6",
        state: IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L486
            || IS_FEATURE_ENABLED_L496
            || IS_FEATURE_ENABLED_L4A6,
    },
    FeatureGate {
        name: "L4+x5",
        state: IS_FEATURE_ENABLED_L4P5
            || IS_FEATURE_ENABLED_L4Q5
            || IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5,
    },
    FeatureGate {
        name: "L4+x7",
        state: IS_FEATURE_ENABLED_L4S7 || IS_FEATURE_ENABLED_L4R7,
    },
    FeatureGate {
        name: "L4+x9",
        state: IS_FEATURE_ENABLED_L4S9 || IS_FEATURE_ENABLED_L4R9,
    },
    FeatureGate {
        name: "L41_42",
        state: IS_FEATURE_ENABLED_L412 || IS_FEATURE_ENABLED_L422,
    },
    FeatureGate {
        name: "L43_44",
        state: IS_FEATURE_ENABLED_L431
            || IS_FEATURE_ENABLED_L432
            || IS_FEATURE_ENABLED_L433
            || IS_FEATURE_ENABLED_L442
            || IS_FEATURE_ENABLED_L443,
    },
    FeatureGate {
        name: "L45_46",
        state: IS_FEATURE_ENABLED_L451 || IS_FEATURE_ENABLED_L452 || IS_FEATURE_ENABLED_L462,
    },
    FeatureGate {
        name: "L47_48",
        state: IS_FEATURE_ENABLED_L471
            || IS_FEATURE_ENABLED_L475
            || IS_FEATURE_ENABLED_L476
            || IS_FEATURE_ENABLED_L485
            || IS_FEATURE_ENABLED_L486,
    },
    FeatureGate {
        name: "L4P_4Q",
        state: IS_FEATURE_ENABLED_L4P5 || IS_FEATURE_ENABLED_L4Q5,
    },
    FeatureGate {
        name: "L4R_4S",
        state: IS_FEATURE_ENABLED_L4R5
            || IS_FEATURE_ENABLED_L4S5
            || IS_FEATURE_ENABLED_L4R7
            || IS_FEATURE_ENABLED_L4S7
            || IS_FEATURE_ENABLED_L4R9
            || IS_FEATURE_ENABLED_L4S9,
    },
];
