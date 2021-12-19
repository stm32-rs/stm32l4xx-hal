//! tool used to generate the feature macros for stm32l4xx-hal
//! the macros are a whole lot of copy paste, change which MCUs are enabled. This is much easier to handle as a mini-program

use std::{fs, path::Path};

fn main() {
    let out_dir = "out";
    fs::create_dir(out_dir).ok();
    let dest_path = Path::new(&out_dir).join("peripherals.rs");
    write_peripherals(&dest_path);
}

const L412: &str = "stm32l412";
const L422: &str = "stm32l422";
const L431: &str = "stm32l431";
const L432: &str = "stm32l432";
const L433: &str = "stm32l433";
const L442: &str = "stm32l442";
const L443: &str = "stm32l443";
const L451: &str = "stm32l451";
const L452: &str = "stm32l452";
const L462: &str = "stm32l462";
const L471: &str = "stm32l471";
const L475: &str = "stm32l475";
const L476: &str = "stm32l476";
const L486: &str = "stm32l486";
const L496: &str = "stm32l496";
const L4A6: &str = "stm32l4a6";
// L4+
const L4P5: &str = "stm32l4p5";
const L4Q5: &str = "stm32l4q5";
const L4R5: &str = "stm32l4r5";
const L4S5: &str = "stm32l4s5";
const L4R7: &str = "stm32l4r7";
const L4S7: &str = "stm32l4s7";
const L4R9: &str = "stm32l4r9";
const L4S9: &str = "stm32l4s9";

const HAS_ADC2: [&str; 16] = [
    L412, L422, L471, L475, L476, L486, L496, L4A6, L4P5, L4Q5, L4R5, L4S5, L4R7, L4S7, L4R9, L4S9,
];

const HAS_ADC3: [&str; 14] = [
    L471, L475, L476, L486, L496, L4A6, L4P5, L4Q5, L4R5, L4S5, L4R7, L4S7, L4R9, L4S9,
];

const HAS_DAC1: [&str; 8] = [L431, L432, L433, L442, L443, L451, L452, L462];

const HAS_COMP1: [&str; 8] = [L431, L432, L433, L442, L443, L451, L452, L462];

const HAS_COMP2: [&str; 8] = [L431, L432, L433, L442, L443, L451, L452, L462];

const HAS_DFSDM1: [&str; 3] = [L451, L452, L462];

const HAS_LCD: [&str; 2] = [L433, L443];

const HAS_AES: [&str; 4] = [L422, L442, L443, L462];

const HAS_TIM3: [&str; 3] = [L451, L452, L462];

const HAS_TIM7: [&str; 5] = [L431, L432, L433, L442, L443];

const HAS_I2C2: [&str; 8] = [L412, L422, L431, L433, L443, L451, L452, L462];

const HAS_I2C4: [&str; 3] = [L451, L452, L462];

const HAS_USART3: [&str; 8] = [L412, L422, L431, L433, L443, L451, L452, L462];

const HAS_UART4: [&str; 3] = [L451, L452, L462];

const HAS_SPI2: [&str; 8] = [L412, L422, L431, L433, L443, L451, L452, L462];

const HAS_SPI3: [&str; 8] = [L431, L432, L433, L442, L443, L451, L452, L462];

const HAS_SAI: [&str; 8] = [L431, L432, L433, L442, L443, L451, L452, L462];

const HAS_SWPMI1: [&str; 5] = [L431, L432, L433, L442, L443];

const HAS_SDMMC: [&str; 6] = [L431, L433, L443, L451, L452, L462];

const HAS_USB_FS: [&str; 8] = [L412, L422, L432, L433, L442, L443, L452, L462];

const HAS_CAN1: [&str; 8] = [L431, L432, L433, L442, L443, L451, L452, L462];

fn write_peripherals(dest_file: &Path) {
    let mut out = String::new();
    write_macro(&mut out, "if_adc2", &HAS_ADC2);
    write_macro(&mut out, "if_adc3", &HAS_ADC3);
    write_macro(&mut out, "if_dac1", &HAS_DAC1);
    write_macro(&mut out, "if_comp1", &HAS_COMP1);
    write_macro(&mut out, "if_comp2", &HAS_COMP2);
    write_macro(&mut out, "if_dfsdm1", &HAS_DFSDM1);
    write_macro(&mut out, "if_lcd", &HAS_LCD);
    write_macro(&mut out, "if_aes", &HAS_AES);
    write_macro(&mut out, "if_tim3", &HAS_TIM3);
    write_macro(&mut out, "if_tim7", &HAS_TIM7);
    write_macro(&mut out, "if_i2c3", &HAS_I2C2);
    write_macro(&mut out, "if_i2c4", &HAS_I2C4);
    write_macro(&mut out, "if_usart3", &HAS_USART3);
    write_macro(&mut out, "if_uart4", &HAS_UART4);
    write_macro(&mut out, "if_spi2", &HAS_SPI2);
    write_macro(&mut out, "if_spi3", &HAS_SPI3);
    write_macro(&mut out, "if_sai", &HAS_SAI);
    write_macro(&mut out, "if_swpmi1", &HAS_SWPMI1);
    write_macro(&mut out, "if_sdmmc", &HAS_SDMMC);
    write_macro(&mut out, "if_usb_fs", &HAS_USB_FS);
    write_macro(&mut out, "if_can1", &HAS_CAN1);

    fs::write(dest_file, out).unwrap();
}

fn write_macro(append_to: &mut String, name: &str, features: &[&str]) {
    let feature_list = features.iter().fold(String::new(), |mut curr, &s| {
        curr.push_str(&format!("feature = {:?},\n", s));
        curr
    });

    append_to.push_str(&format!("macro_rules! {} {{\n", name));

    for frag_spac in ["expr", "item"] {
        // present:
        append_to.push_str(&format!("(present: $ex:{}) => {{\n", frag_spac));
        append_to.push_str(&format!("#[cfg(any(\n{}))]\n", feature_list));
        if frag_spac == "expr" {
            // wrap expressions in a block to avoid multi-line issues
            append_to.push_str(&format!("{{ $ex }}\n"));
        } else {
            append_to.push_str(&format!("$ex\n"));
        }
        append_to.push_str(&format!("}};\n"));
        // absent:
        append_to.push_str(&format!("(absent: $ex:{}) => {{\n", frag_spac));
        append_to.push_str(&format!("#[cfg(not(any(\n{})))]\n", feature_list));
        if frag_spac == "expr" {
            // wrap expressions in a block to avoid multi-line issues
            append_to.push_str(&format!("{{ $ex }}\n"));
        } else {
            append_to.push_str(&format!("$ex\n"));
        }
        append_to.push_str(&format!("}};\n"));
    }
    // footer
    append_to.push_str("}\n");
    append_to.push_str(&format!("pub(crate) use {};\n\n", name));
}
