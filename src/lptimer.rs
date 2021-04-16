//! Timers

use crate::rcc::{APB1R1, APB1R2, CCIPR};

use crate::stm32::{LPTIM1, LPTIM2};

/// Hardware timers
pub struct LowPowerTimer<LPTIM> {
    lptim: LPTIM,
}

/// Clock sources
pub enum ClockSource {
    PCLK = 0b00,
    LSI = 0b01,
    HSI16 = 0b10,
    LSE = 0b11,
}

pub enum PreScaler {
    U1 = 0b000,
    U2 = 0b001,
    U4 = 0b010,
    U8 = 0b011,
    U16 = 0b100,
    U32 = 0b101,
    U64 = 0b110,
    U128 = 0b111,
}

pub enum CountMode {
    Internal = 0b0,
    External = 0b1,
}

/// All currently supported interrupt events
pub enum Event {
    CompareMatch,
    AutoReloadMatch,
}

pub struct LowPowerTimerConfig {
    clock_source: ClockSource,
    prescaler: PreScaler,
    count_mode: CountMode,
    compare_value: u16,
    arr_value: u16,
}

impl Default for LowPowerTimerConfig {
    fn default() -> Self {
        Self {
            clock_source: ClockSource::LSE,
            prescaler: PreScaler::U128,
            count_mode: CountMode::Internal,
            compare_value: 0x0,
            arr_value: 0xFFFF,
        }
    }
}

impl LowPowerTimerConfig {
    pub fn clock_source(mut self, clock_source: ClockSource) -> Self {
        self.clock_source = clock_source;
        self
    }

    pub fn prescaler(mut self, prescaler: PreScaler) -> Self {
        self.prescaler = prescaler;
        self
    }

    pub fn count_mode(mut self, count_mode: CountMode) -> Self {
        self.count_mode = count_mode;
        self
    }

    pub fn compare_value(mut self, compare_value: u16) -> Self {
        self.compare_value = compare_value;
        self
    }
}

macro_rules! hal {
    ($timer_type: ident, $lptimX: ident, $apb1rX: ident, $timXen: ident, $timXrst: ident, $timXsel: ident) => {
        impl LowPowerTimer<$timer_type> {
            pub fn $lptimX(
                apb1rn: &mut $apb1rX,
                ccipr: &mut CCIPR,
                lptim: $timer_type,
                config: LowPowerTimerConfig,
            ) -> Self {
                let LowPowerTimerConfig {
                    clock_source,
                    count_mode,
                    prescaler,
                    compare_value,
                    arr_value,
                } = config;

                // ARR value must be strictly greater than CMP value
                assert!(arr_value > compare_value);

                apb1rn.enr().modify(|_, w| w.$timXen().set_bit());
                apb1rn.rstr().modify(|_, w| w.$timXrst().set_bit());
                apb1rn.rstr().modify(|_, w| w.$timXrst().clear_bit());

                // This operation is sound as `ClockSource as u8` only produces valid values
                ccipr
                    .ccipr()
                    .modify(|_, w| unsafe { w.$timXsel().bits(clock_source as u8) });

                lptim.cfgr.modify(|_, w| unsafe {
                    w.enc()
                        .bit(false)
                        .countmode()
                        .bit(count_mode as u8 > 0)
                        // This operation is sound as `PreScaler as u8` only produces valid values
                        .presc()
                        .bits(prescaler as u8)
                        .cksel()
                        .bit(false)
                });

                let mut instance = LowPowerTimer { lptim };

                instance.enable();

                // Write compare, arr, and continous mode start register _after_ enabling lptim
                instance.lptim.cr.modify(|_, w| w.cntstrt().bit(true));

                // This operation is sound as arr_value is a u16, and there are 16 writeable bits
                instance
                    .lptim
                    .arr
                    .write(|w| unsafe { w.bits(arr_value as u32) });

                instance.set_compare_match(compare_value);

                instance
            }

            fn set_enable(&mut self, enabled: bool) {
                self.lptim.cr.modify(|_, w| w.enable().bit(enabled));
            }

            fn enable(&mut self) {
                self.set_enable(true);
            }

            fn disable(&mut self) {
                self.set_enable(true);
            }

            pub fn listen(&mut self, event: Event) {
                // LPTIM_IER may only be modified when LPTIM is disabled
                self.disable();
                self.lptim.ier.modify(|_, w| match event {
                    Event::CompareMatch => w.cmpmie().set_bit(),
                    Event::AutoReloadMatch => w.arrmie().set_bit(),
                });
                self.enable();
            }

            pub fn unlisten(&mut self, event: Event) {
                // LPTIM_IER may only be modified when LPTIM is disabled
                self.disable();
                self.lptim.ier.modify(|_, w| match event {
                    Event::CompareMatch => w.cmpmie().clear_bit(),
                    Event::AutoReloadMatch => w.arrmie().clear_bit(),
                });
                self.enable();
            }

            pub fn is_event_triggered(&self, event: Event, clear_interrupt: bool) -> bool {
                let reg_val = self.lptim.isr.read();
                let bit_is_set = match event {
                    Event::CompareMatch => reg_val.cmpm().bit_is_set(),
                    Event::AutoReloadMatch => reg_val.arrm().bit_is_set(),
                };
                if bit_is_set && clear_interrupt {
                    self.lptim.icr.write(|w| match event {
                        Event::CompareMatch => w.cmpmcf().set_bit(),
                        Event::AutoReloadMatch => w.arrmcf().set_bit(),
                    });
                }
                bit_is_set
            }

            pub fn set_compare_match(&mut self, value: u16) {
                // This operation is sound as compare_value is a u16, and there are 16 writeable bits
                // Additionally, the LPTIM peripheral will always be in the enabled state when this code is called
                self.lptim.cmp.write(|w| unsafe { w.bits(value as u32) });
            }

            pub fn get_counter(&mut self) -> u16 {
                self.lptim.cnt.read().bits() as u16
            }

            pub fn get_arr(&mut self) -> u16 {
                self.lptim.arr.read().bits() as u16
            }
        }
    };
}

hal!(LPTIM1, lptim1, APB1R1, lptim1en, lptim1rst, lptim1sel);
hal!(LPTIM2, lptim2, APB1R2, lptim2en, lptim2rst, lptim2sel);
