use crate::gpio::gpioa;
use crate::gpio::{Analog};
use crate::rcc::{AHB2,CCIPR};
use embedded_hal::adc::{Channel, OneShot};
use crate::pac::ADC1;
use core::ptr;

const VREFCAL: *const u16 = 0x1FFF_75AA as *const u16;
const VTEMPCAL30: *const u16 = 0x1FFF_75A8 as *const u16;
const VTEMPCAL110: *const u16 = 0x1FFF_75CA as *const u16;
const VDD_CALIB: u16 = 3000;


/// Inspiration has been drawn from three different ADC libs
/// https://github.com/stm32-rs/stm32l1xx-hal/blob/master/src/adc.rs
/// https://github.com/stm32-rs/stm32f0xx-hal/blob/master/src/adc.rs
/// https://github.com/stm32-rs/stm32f4xx-hal/blob/master/src/adc.rs

/// ADC Result Alignment
#[derive(Copy, Clone, PartialEq)]
// #[repr(bool)]
pub enum Align {
    /// Right aligned results (least significant bits)
    ///
    /// Results in all Resolutions returning values from 0-(2^bits-1) in
    /// steps of 1.
    Right,
    /// Left aligned results (most significant bits)
    ///
    /// Results in all Resolutions returning a value in the range 0-65535.
    /// Depending on the Resolution the result will step by larger or smaller
    /// amounts.
    Left,
}


#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum RegularOversampling{
    On = 0b1,
    Off = 0b0
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum InjectedOversampling{
    On = 0b1,
    Off = 0b0
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u16)]
pub enum OversamplingRatio{
   /// 2x ADC Oversampling ratio
   X2 = 0b000,
   /// 4x ADC Oversampling ratio
   X4 = 0b001,
   /// 8x ADC Oversampling ratio
   X8 = 0b010,
   /// 16x ADC Oversampling ratio
   X16 = 0b011,
   /// 32x ADC Oversampling ratio
   X32 = 0b100,
   /// 64x ADC Oversampling ratio
   X64 = 0b101,
   /// 128x ADC Oversampling ratio
   X128 = 0b110,
   /// 256x ADC Oversampling ratio
   X256 = 0b111,
}


///
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u16)]
pub enum OversamplingShift{
    /// 0 bit Oversampling shift. Same as divide by 0
   S0 = 0b0000,
   /// 1 bits Oversampling shift. Same as divide by 2
   S1 = 0b0001,
   /// 2 bits Oversampling shift. Same as divide by 4
   S2 = 0b0010,
   /// 3 bits Oversampling shift. Same as divide by 8
   S3 = 0b0011,
   /// 4 bits Oversampling shift. Same as divide by 16
   S4 = 0b0100,
   /// 5 bits Oversampling shift. Same as divide by 32
   S5 = 0b0101,
   /// 6 bits Oversampling shift. Same as divide by 64
   S6 = 0b0110,
   /// 7 bits Oversampling shift. Same as divide by 128
   S7 = 0b0111,
   /// 8 bits Oversampling shift. Same as divide by 256
   S8 = 0b1000,
}



/// ADC Sampling Resolution
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum Resolution {
    /// 12 bit Resolution
    B12 = 0b00,
    /// 10 bit Resolution
    B10 = 0b01,
    /// 8 bit Resolution
    B8 = 0b10,
    /// 6 bit Resolution
    B6 = 0b11,
}

/// ADC Sampling time
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u16)]
pub enum SampleTime {
    /// 2.5 ADC clock cycles
    T2_5 = 0b000,
    /// 6.5 ADC clock cycles
    T6_5 = 0b001,
    /// 12.5 ADC clock cycles
    T12_5 = 0b010,
    /// 24.5 ADC clock cycles
    T24_5 = 0b011,
    /// 47.5 ADC clock cycles
    T47_5 = 0b100,
    /// 92.5 ADC clock cycles
    T92_5 = 0b101,
    /// 247.5 ADC clock cycles
    T247_5 = 0b110,
    /// 640.5 ADC clock cycles
    T640_5 = 0b111,
}

#[derive(Copy, Clone, PartialEq)]
pub struct Config {
    pub align: Align,
    pub resolution: Resolution,
    pub sample_time: SampleTime,
    pub reg_oversampl : RegularOversampling,
    pub inj_oversampl : InjectedOversampling,
    pub oversampl_ratio : OversamplingRatio,
    pub oversampl_shift : OversamplingShift,
}

impl Config {
    pub fn align(mut self, align: Align) -> Self {
        self.align = align;
        self
    }

    pub fn resolution(mut self, resolution : Resolution) -> Self {
        self.resolution = resolution;
        self
    }

    pub fn sample_time(mut self, sample_time: SampleTime) -> Self {
        self.sample_time = sample_time;
        self
    }
    pub fn reg_oversampl(mut self, reg_oversampl: RegularOversampling) -> Self{
        self.reg_oversampl = reg_oversampl;
        self
    }
    pub fn inj_oversampl(mut self, inj_oversampl: InjectedOversampling) -> Self{
        self.inj_oversampl = inj_oversampl;
        self
    }
    pub fn oversampl_ratio(mut self, oversampl_ratio: OversamplingRatio) -> Self{
        self.oversampl_ratio = oversampl_ratio;
        self
    }
    pub fn oversampl_shift(mut self, oversampl_shift: OversamplingShift) -> Self{
        self.oversampl_shift = oversampl_shift;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        Config {
            resolution: Resolution::B12,
            align : Align::Right,
            sample_time: SampleTime::T640_5,
            reg_oversampl : RegularOversampling::Off,
            inj_oversampl : InjectedOversampling::Off,
            oversampl_ratio : OversamplingRatio::X16,
            oversampl_shift : OversamplingShift::S4,
        }
    }
}
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum ClockMode {
    /// Makes use of the ADC select from RCC (Asynchronous clock mode),
    /// generated at product level (refer to Section 6: Reset and clock control (RCC))
    CkAdc = 0b00,
    /// HCLK/1 (Synchronous clock mode). 
    /// This configuration must be enabled only if the AHB
    /// clock prescaler is set to 1 (HPRE[3:0] = 0xxx in RCC_CFGR register) and if the system clock
    HclkDiv1 = 0b01,
    /// 8 bit Resolution
    HclkDiv2 = 0b10,
    /// 6 bit Resolution
    HclkDiv4 = 0b11,
}


#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum AdcClockSelect {
    /// No clock selected
    NoClock = 0b00,
    /// PLLSAI1 “R” clock (PLLADC1CLK) selected as ADCs clock
    Pllsai1 = 0b01,
    /// PLLSAI2 “R” clock (PLLADC2CLK) selected as ADCs clock
    Pllsai2 = 0b10,
    /// System clock selected as ADCs clock
    SysClk = 0b11,
}

#[derive(Copy, Clone, PartialEq)]
pub struct CommonConfig {
    pub clock_mode : ClockMode,
    pub adc_clk_sel : AdcClockSelect,
}

impl CommonConfig {
    pub fn clock_mode(mut self, clock_mode: ClockMode) -> Self {
        self.clock_mode = clock_mode;
        self
    }

    pub fn adc_clk_sel(mut self, adc_clk_sel: AdcClockSelect) -> Self {
        self.adc_clk_sel = adc_clk_sel;
        self
    }
}

impl Default for CommonConfig {
    fn default() -> CommonConfig {
        CommonConfig {
            clock_mode : ClockMode::HclkDiv1,
            adc_clk_sel : AdcClockSelect::Pllsai1,
        }
    }
}

/// Global setup for ADC
pub fn adc_global_setup(config : CommonConfig, ahb2 : &mut AHB2, ccipr : &mut CCIPR) {

    // Setup Rcc clock select
    ccipr.ccipr().modify(|_, w|unsafe { w.adcsel().bits(config.adc_clk_sel as u8) } );  
    
    // Enable clock for ADC
    ahb2.enr().modify(|_r, w| w.adcen().set_bit());
    
    // Set ADC common clock
    // The software is allowed to write these bits only when the ADCs are disabled
    //  (ADCAL=0, JADSTART=0, ADSTART=0, ADSTP=0, ADDIS=0 and ADEN=0) in all adc_cr registers.    
    let adc_common = unsafe { &*crate::device::ADC_COMMON::ptr() };
    if !any_adc_active() {
        adc_common.ccr.modify(|_, w| unsafe {w.ckmode().bits(config.clock_mode as u8)} );
    }

    // Not possible yet, due to missing register
    //        adc_common.ccr.modify(|_, w| unsafe { w.presc().bits(0b1011) });
}

/// Checks if any ADCs are enabled or running
fn any_adc_active() -> bool{
    let adc1 = unsafe { &*crate::device::ADC1::ptr() };
    adc1.cr.read().adcal().bit_is_set()       |
        adc1.cr.read().adstp().bit_is_set()     |
        adc1.cr.read().jadstart().bit_is_set()  |
        adc1.cr.read().adstart().bit_is_set()   |
        adc1.cr.read().addis().bit_is_set()     |
        adc1.cr.read().aden().bit_is_set()      
}


pub struct Adc<ADC1> {
    adc: ADC1,
    config : Config,
}

#[cfg(feature = "stm32l4x6")]
impl Adc<ADC1> {
    /// Sets up the ADC
    pub fn adc1(adc: ADC1, config : Config, ahb2 : &mut AHB2, ccipr : &mut CCIPR) -> Self {
        


        //Check these two registers for 0
        if ahb2.enr().read().adcen().bit_is_clear(){
            adc_global_setup(CommonConfig::default(), ahb2, ccipr);
        }

        // Disable deep power down and start ADC voltage regulator
        adc.cr.modify(|_, w| w.deeppwd().clear_bit());
        adc.cr.modify(|_, w| w.advregen().set_bit());
        cortex_m::asm::delay(8_000_000); //Delay at 80MHz for L475
        if adc.cr.read().advregen().bit_is_clear(){
            panic!("ADC Vreg not enabled corectly")
        }

        // Calibrate
        adc.cfgr.modify(|_, w| w.dmaen().clear_bit());
        adc.cr.modify(|_, w| w.adcaldif().clear_bit());
        adc.cr.modify(|_, w| w.adcal().set_bit());
        while adc.cr.read().adcal().bit_is_set() {}

        let temp_c = Config::default();
        let mut adc = Self{
            adc,
            config: temp_c,
        };
        adc.apply_config(config);
        adc
    }

    fn power_up(&mut self) {
        self.adc.isr.modify(|_, w| w.adrdy().set_bit());
        self.adc.cr.modify(|_, w| w.aden().set_bit());
        while self.adc.isr.read().adrdy().bit_is_clear() {}
    }

    fn power_down(&mut self) {
        self.adc.cr.modify(|_, w| w.addis().set_bit());
        self.adc.isr.modify(|_, w| w.adrdy().set_bit());
        while self.adc.cr.read().aden().bit_is_set() {}
    }

    /// Apply a configuration to the ADC
    pub fn apply_config(&mut self, config : Config){
        self.set_align(config.align);
        self.set_injected_oversampling(config.inj_oversampl);
        self.set_oversampling_ratio(config.oversampl_ratio);
        self.set_oversampling_shift(config.oversampl_shift);
        self.set_regular_oversampling(config.reg_oversampl);
        self.set_resolution(config.resolution);
        self.config = config;
    }

    /// Resets the ADC config to default, returning the existing config as
    /// a stored config.
    pub fn default_config(&mut self) -> Config {
        let cfg = self.get_config();
        if !(cfg == Config::default()){
            self.apply_config(Config::default());
        }
        cfg
    }

    /// Returns a copy of the current configuration
    pub fn get_config(&mut self) -> Config{
        self.config
    }



    pub fn set_align(&mut self, align: Align) {
        self.adc.cfgr.modify(|_, w| w.align().bit(align == Align::Left))
    }

    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.adc.cfgr.modify(|_, w| unsafe {w.res().bits(resolution as u8)})
    }

    pub fn set_regular_oversampling(&mut self, reg_oversampl : RegularOversampling){
        self.adc.cfgr2.modify(|_, w| w.rovse().bit(reg_oversampl == RegularOversampling::On))
    }

    pub fn set_injected_oversampling(&mut self, inj_oversampl : InjectedOversampling){
        self.adc.cfgr2.modify(|_, w| w.jovse().bit(inj_oversampl == InjectedOversampling::On))
    }

    pub fn set_oversampling_ratio(&mut self, oversampl_ratio : OversamplingRatio){
        self.adc.cfgr2.modify(|_, w| unsafe{ w.ovsr().bits(oversampl_ratio as u8)})
    }

    pub fn set_oversampling_shift(&mut self, oversampl_shift : OversamplingShift){
        self.adc.cfgr2.modify(|_, w| unsafe{ w.ovss().bits(oversampl_shift as u8)})
    }

}



#[cfg(feature = "stm32l4x6")]
impl<WORD, PIN> OneShot<Adc<ADC1>, WORD, PIN> for Adc<ADC1>
where
    WORD: From<u16>,
    PIN: AdcChannel<ADC1> + Channel<Adc<ADC1>, ID = u8>,
{
    type Error = ();

    fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        
        self.power_up();

        //Save ADC state
        //Note: Sampling time for pin is not restored
        let cont_mode = self.adc.cfgr.read().cont().bit_is_set();
        let external_trig = self.adc.cfgr.read().exten().bits();
        let seq_1 = self.adc.sqr1.read().sq1().bits();
        let seq_len = self.adc.sqr1.read().l().bits();

        pin.setup(&mut self.adc, self.config.sample_time);

        self.adc
            .cfgr
            .modify(|_, w| w.cont().clear_bit());
        self.adc
            .cfgr
            .modify(|_, w| unsafe { w.exten().bits(0b00) });
        self.adc
            .sqr1
            .modify(|_, w| unsafe { w.sq1().bits(PIN::channel()) });
        self.adc
            .sqr1
            .modify(|_, w| unsafe { w.l().bits(0b0000)});

        self.adc.isr.modify(|_, w| w.eoc().set_bit());
        self.adc.cr.modify(|_, w| w.adstart().set_bit());

        while self.adc.isr.read().eoc().bit_is_clear() {}

        let val = self.adc.dr.read().rdata().bits().into();

        //Restore state
        self.adc
            .cfgr
            .modify(|_, w| w.cont().bit(cont_mode));
        self.adc
            .cfgr
            .modify(|_, w| unsafe { w.exten().bits(external_trig) });
        self.adc
            .sqr1
            .modify(|_, w| unsafe { w.sq1().bits(seq_1) });
        self.adc
            .sqr1
            .modify(|_, w| unsafe { w.l().bits(seq_len)});

        self.power_down();
        Ok(val)
    }
}



macro_rules! int_adc {
    ($($Chan:ident: ($chan:expr, $en:ident)),+ $(,)*) => {
        $(
            pub struct $Chan;

            impl $Chan {
                pub fn new() -> Self {
                    Self {}
                }


                pub fn enable(&mut self) {
                    let adc_common = unsafe { &*crate::device::ADC_COMMON::ptr() };
                    adc_common.ccr.modify(|_, w| w.$en().set_bit());
                }

                pub fn disable(&mut self) {
                    let adc_common = unsafe { &*crate::device::ADC_COMMON::ptr() };
                    adc_common.ccr.modify(|_, w| w.$en().clear_bit());
                }

                pub fn is_enabled(&mut self) -> bool {
                    let adc_common = unsafe { &*crate::device::ADC_COMMON::ptr() };
                    adc_common.ccr.read().$en().bit_is_set()
                }
            }
        )+
    };
}
int_adc! {
    VTemp: (17, ch17sel),
    VRef: (0, vrefen),
    VBat: (18, ch18sel)
}


pub trait AdcChannel<T> {
    fn setup(&mut self, adc: &mut T, sample_time : SampleTime);
}

macro_rules! adc_pins {
    ($($Adc:ty: ($Chan:ty: ($pin:ty, $chan:expr, $smprx:ident))),+ $(,)*) => {
        $(
            impl Channel<Adc<$Adc>> for $pin {
                type ID = u8;

                fn channel() -> u8 { $chan }
            }

            impl AdcChannel <$Adc> for $pin {
                fn setup(&mut self, adc: &mut $Adc, sample_time : SampleTime) {
                    adc.$smprx.modify(|r, w| unsafe {
                        const OFFSET: u8 = 3 * $chan % 10;
                        let mut bits = r.bits() as u32;
                        bits &= !(0xfff << OFFSET);
                        bits |= (sample_time as u32) << OFFSET;
                        w.bits(bits)
                    });
                    // adc.rb.sqr5.write(|w| unsafe { w.sq1().bits($chan) });
                }
            }
        )+
    };
}

adc_pins! {
    ADC1: (Channel5: (gpioa::PA0<Analog>, 5_u8, smpr1)),
    ADC1: (Channel6: (gpioa::PA1<Analog>, 6_u8, smpr1)),
    
    ADC1: (VTemp: (VTemp, 17_u8, smpr2)),
    ADC1: (VBat: (VBat, 18_u8, smpr2)),
}

//Special case Setup for VRef
impl Channel<Adc<ADC1>> for VRef {
    type ID = u8;

    fn channel() -> u8 { 0 }
}
impl AdcChannel <ADC1> for VRef {
    fn setup(&mut self, _adc: &mut ADC1, _sample_time : SampleTime) {
        
    }
}


impl VTemp{
    fn convert_temp(vtemp: u16, vdda: u16) -> i16 {
        let vtemp30_cal = i32::from(unsafe { ptr::read(VTEMPCAL30) }) * 100;
        let vtemp110_cal = i32::from(unsafe { ptr::read(VTEMPCAL110) }) * 100;

        let mut temperature = i32::from(vtemp) * 100;
        temperature = (temperature * (i32::from(vdda) / i32::from(VDD_CALIB))) - vtemp30_cal;
        temperature *= (110 - 30) * 100;
        temperature /= vtemp110_cal - vtemp30_cal;
        temperature += 3000;
        temperature as i16

    }

    pub fn get_cal() -> (i32, i32){
        let vtemp30_cal = i32::from(unsafe { ptr::read(VTEMPCAL30) }) * 100;
        let vtemp110_cal = i32::from(unsafe { ptr::read(VTEMPCAL110) }) * 100;
        (vtemp30_cal, vtemp110_cal)
    }

    /// Read the value of the internal temperature sensor and return the
    /// result in 100ths of a degree centigrade.
    ///
    /// Given a delay reference it will attempt to restrict to the
    /// minimum delay needed to ensure a 10 us t<sub>START</sub> value.
    /// Otherwise it will approximate the required delay using ADC reads.
    //Presumes ADC is setup for reciving a single measurement
    pub fn read(adc: &mut Adc<ADC1>) -> i16 {
        let mut vtemp = Self::new();
        let vtemp_preenable = vtemp.is_enabled();

        if !vtemp_preenable {
            vtemp.enable();
            //Delay set for worst case senario for STM32L475
            cortex_m::asm::delay(40);

        }

        let vdda = VRef::read_vdda(adc);

        let prev_cfg = adc.get_config();
        
        //Sample time dependant on electrical caracteristics for temp sensor here STM32L475
        adc.apply_config(Config::default().sample_time(SampleTime::T47_5));

        let vtemp_val = adc.read(&mut vtemp).unwrap();

        if !vtemp_preenable {
            vtemp.disable();
        }

        adc.apply_config(prev_cfg);

        Self::convert_temp(vtemp_val, vdda)
    }
}


impl VRef {
    /// Reads the value of VDDA in milli-volts
    pub fn read_vdda(adc: &mut Adc<ADC1>) -> u16 {
        let vrefint_cal = u32::from(unsafe { ptr::read(VREFCAL) });
        let mut vref = Self::new();

        let prev_cfg = adc.default_config();
        // let prev_cfg = adc.get_config();
        // adc.apply_config(Config::default().sample_time(SampleTime::T640_5));

        let vref_preenable = vref.is_enabled();

        if !vref_preenable {
            vref.enable();
        }

        let vref_val: u32 = adc.read(&mut vref).unwrap();
       
        if !vref_preenable{
            vref.disable();
        }

        adc.apply_config(prev_cfg);

        (u32::from(VDD_CALIB) * vrefint_cal / vref_val) as u16
    }
}
