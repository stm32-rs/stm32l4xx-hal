pub(crate) mod features;

fn main() {
    assert!(
        features::validate_selected_features(),
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
    features::generate_internal_features();
}

pub(crate) struct FeatureGate<'a> {
    pub name: &'a str,
    pub state: bool,
}
