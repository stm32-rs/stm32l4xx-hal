//! Reset and Clock Control

use crate::stm32::{rcc, RCC};
use cast::u32;

use crate::flash::ACR;
use crate::pwr::Pwr;
use crate::time::Hertz;

mod enable;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MsiFreq {
    #[doc = "range 0 around 100 kHz"]
    RANGE100K = 0,
    #[doc = "range 1 around 200 kHz"]
    RANGE200K = 1,
    #[doc = "range 2 around 400 kHz"]
    RANGE400K = 2,
    #[doc = "range 3 around 800 kHz"]
    RANGE800K = 3,
    #[doc = "range 4 around 1 MHz"]
    RANGE1M = 4,
    #[doc = "range 5 around 2 MHz"]
    RANGE2M = 5,
    #[doc = "range 6 around 4 MHz"]
    RANGE4M = 6,
    #[doc = "range 7 around 8 MHz"]
    RANGE8M = 7,
    #[doc = "range 8 around 16 MHz"]
    RANGE16M = 8,
    #[doc = "range 9 around 24 MHz"]
    RANGE24M = 9,
    #[doc = "range 10 around 32 MHz"]
    RANGE32M = 10,
    #[doc = "range 11 around 48 MHz"]
    RANGE48M = 11,
}

impl MsiFreq {
    fn to_hertz(self) -> Hertz {
        Hertz(match self {
            Self::RANGE100K => 100_000,
            Self::RANGE200K => 200_000,
            Self::RANGE400K => 400_000,
            Self::RANGE800K => 800_000,
            Self::RANGE1M => 1_000_000,
            Self::RANGE2M => 2_000_000,
            Self::RANGE4M => 4_000_000,
            Self::RANGE8M => 8_000_000,
            Self::RANGE16M => 16_000_000,
            Self::RANGE24M => 24_000_000,
            Self::RANGE32M => 32_000_000,
            Self::RANGE48M => 48_000_000,
        })
    }
}

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb1: AHB1::new(),
            ahb2: AHB2::new(),
            ahb3: AHB3::new(),
            apb1r1: APB1R1::new(),
            apb1r2: APB1R2::new(),
            apb2: APB2::new(),
            bdcr: BDCR { _0: () },
            csr: CSR { _0: () },
            crrcr: CRRCR { _0: () },
            ccipr: CCIPR { _0: () },
            cfgr: CFGR {
                hse: None,
                lse: None,
                msi: None,
                hsi48: false,
                lsi: false,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
                sai1clk: None,
                sai2clk: None,
                pll_source: None,
                pll_config: None,
                pllsai1_config: None,
                pllsai2_config: None,
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus (AHB1) registers
    pub ahb1: AHB1,
    /// AMBA High-performance Bus (AHB2) registers
    pub ahb2: AHB2,
    /// AMBA High-performance Bus (AHB3) registers
    pub ahb3: AHB3,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1r1: APB1R1,
    /// Advanced Peripheral Bus 1 (APB2) registers
    pub apb1r2: APB1R2,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    /// Clock configuration register
    pub cfgr: CFGR,
    /// Backup domain control register
    pub bdcr: BDCR,
    /// Control/Status Register
    pub csr: CSR,
    /// Clock recovery RC register
    pub crrcr: CRRCR,
    /// Peripherals independent clock configuration register
    pub ccipr: CCIPR,
}

/// CSR Control/Status Register
pub struct CSR {
    _0: (),
}

impl CSR {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn csr(&mut self) -> &rcc::CSR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).csr }
    }
}

/// Clock recovery RC register
pub struct CRRCR {
    _0: (),
}

impl CRRCR {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn crrcr(&mut self) -> &rcc::CRRCR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).crrcr }
    }

    /// Checks if the 48 MHz HSI is enabled
    pub fn is_hsi48_on(&mut self) -> bool {
        self.crrcr().read().hsi48on().bit()
    }

    /// Checks if the 48 MHz HSI is ready
    pub fn is_hsi48_ready(&mut self) -> bool {
        self.crrcr().read().hsi48rdy().bit()
    }
}

/// Peripherals independent clock configuration register
pub struct CCIPR {
    _0: (),
}

impl CCIPR {
    #[allow(dead_code)]
    pub(crate) fn ccipr(&mut self) -> &rcc::CCIPR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ccipr }
    }
}

/// BDCR Backup domain control register registers
pub struct BDCR {
    _0: (),
}

impl BDCR {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn enr(&mut self) -> &rcc::BDCR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).bdcr }
    }
}

macro_rules! bus_struct {
    ($($busX:ident => ($EN:ident, $en:ident, $SMEN:ident, $smen:ident, $RST:ident, $rst:ident, $doc:literal),)+) => {
        $(
            #[doc = $doc]
            pub struct $busX {
                _0: (),
            }

            impl $busX {
                pub(crate) fn new() -> Self {
                    Self { _0: () }
                }

                #[allow(unused)]
                pub(crate) fn enr(&self) -> &rcc::$EN {
                    // NOTE(unsafe) this proxy grants exclusive access to this register
                    unsafe { &(*RCC::ptr()).$en }
                }

                #[allow(unused)]
                pub(crate) fn smenr(&self) -> &rcc::$SMEN {
                    // NOTE(unsafe) this proxy grants exclusive access to this register
                    unsafe { &(*RCC::ptr()).$smen }
                }

                #[allow(unused)]
                pub(crate) fn rstr(&self) -> &rcc::$RST {
                    // NOTE(unsafe) this proxy grants exclusive access to this register
                    unsafe { &(*RCC::ptr()).$rst }
                }
            }
        )+
    };
}

bus_struct! {
    AHB1 => (AHB1ENR, ahb1enr, AHB1SMENR, ahb1smenr, AHB1RSTR, ahb1rstr, "Advanced High-performance Bus 1 (AHB1) registers"),
    AHB2 => (AHB2ENR, ahb2enr, AHB2SMENR, ahb2smenr, AHB2RSTR, ahb2rstr, "Advanced High-performance Bus 2 (AHB2) registers"),
    AHB3 => (AHB3ENR, ahb3enr, AHB3SMENR, ahb3smenr, AHB3RSTR, ahb3rstr, "Advanced High-performance Bus 3 (AHB3) registers"),
    APB1R1 => (APB1ENR1, apb1enr1, APB1SMENR1, apb1smenr1, APB1RSTR1, apb1rstr1, "Advanced Peripheral Bus 1 (APB1) registers"),
    APB1R2 => (APB1ENR2, apb1enr2, APB1SMENR2, apb1smenr2, APB1RSTR2, apb1rstr2, "Advanced Peripheral Bus 1 (APB1) registers"),
    APB2 => (APB2ENR, apb2enr, APB2SMENR, apb2smenr, APB2RSTR, apb2rstr, "Advanced Peripheral Bus 2 (APB2) registers"),
}

/// Bus associated to peripheral
pub trait RccBus: crate::Sealed {
    /// Bus type;
    type Bus;
}

/// Enable/disable peripheral
pub trait Enable: RccBus {
    /// Enables peripheral
    fn enable(bus: &mut Self::Bus);

    /// Disables peripheral
    fn disable(bus: &mut Self::Bus);

    /// Check if peripheral enabled
    fn is_enabled() -> bool;

    /// Check if peripheral disabled
    fn is_disabled() -> bool;

    /// # Safety
    ///
    /// Enables peripheral. Takes access to RCC internally
    unsafe fn enable_unchecked();

    /// # Safety
    ///
    /// Disables peripheral. Takes access to RCC internally
    unsafe fn disable_unchecked();
}

/// Enable/disable peripheral in sleep mode
pub trait SMEnable: RccBus {
    /// Enables peripheral
    fn enable_in_sleep_mode(bus: &mut Self::Bus);

    /// Disables peripheral
    fn disable_in_sleep_mode(bus: &mut Self::Bus);

    /// Check if peripheral enabled
    fn is_enabled_in_sleep_mode() -> bool;

    /// Check if peripheral disabled
    fn is_disabled_in_sleep_mode() -> bool;

    /// # Safety
    ///
    /// Enables peripheral. Takes access to RCC internally
    unsafe fn enable_in_sleep_mode_unchecked();

    /// # Safety
    ///
    /// Disables peripheral. Takes access to RCC internally
    unsafe fn disable_in_sleep_mode_unchecked();
}

/// Reset peripheral
pub trait Reset: RccBus {
    /// Resets peripheral
    fn reset(bus: &mut Self::Bus);

    /// # Safety
    ///
    /// Resets peripheral. Takes access to RCC internally
    unsafe fn reset_unchecked();
}

#[derive(Debug, PartialEq)]
/// HSE Configuration
struct HseConfig {
    /// Clock speed of HSE
    speed: u32,
    /// If the clock driving circuitry is bypassed i.e. using an oscillator, not a crystal or
    /// resonator
    bypass: CrystalBypass,
    /// Clock Security System enable/disable
    css: ClockSecuritySystem,
}

#[derive(Debug, PartialEq)]
/// LSE Configuration
struct LseConfig {
    /// If the clock driving circuitry is bypassed i.e. using an oscillator, not a crystal or
    /// resonator
    bypass: CrystalBypass,
    /// Clock Security System enable/disable
    css: ClockSecuritySystem,
}

/// Crystal bypass selector
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CrystalBypass {
    /// If the clock driving circuitry is bypassed i.e. using an oscillator
    Enable,
    /// If the clock driving circuitry is not bypassed i.e. using a crystal or resonator
    Disable,
}

/// Clock Security System (CSS) selector
///
/// When this is enabled on HSE it will fire of the NMI interrupt on failure and for the LSE the
/// MCU will be woken if in Standby and then the LSECSS interrupt will fire. See datasheet on how
/// to recover for CSS failures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ClockSecuritySystem {
    /// Enable the clock security system to detect clock failures
    Enable,
    /// Leave the clock security system disabled
    Disable,
}

const HSI: u32 = 16_000_000; // Hz

/// Clock configuration
pub struct CFGR {
    hse: Option<HseConfig>,
    lse: Option<LseConfig>,
    msi: Option<MsiFreq>,
    hsi48: bool,
    lsi: bool,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    sai1clk: Option<u32>,
    sai2clk: Option<u32>,
    pll_source: Option<PllSource>,
    pll_config: Option<PllConfig>,
    pllsai1_config: Option<PllConfig>,
    pllsai2_config: Option<PllConfig>,
}

impl CFGR {
    /// Add an HSE to the system
    pub fn hse<F>(mut self, freq: F, bypass: CrystalBypass, css: ClockSecuritySystem) -> Self
    where
        F: Into<Hertz>,
    {
        self.hse = Some(HseConfig {
            speed: freq.into().0,
            bypass,
            css,
        });

        self
    }

    /// Add an 32.768 kHz LSE to the system
    pub fn lse(mut self, bypass: CrystalBypass, css: ClockSecuritySystem) -> Self {
        self.lse = Some(LseConfig { bypass, css });

        self
    }

    /// Sets a frequency for the AHB bus
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Enable the 48 MHz USB, RNG, SDMMC HSI clock source. Not available on all stm32l4x6 series
    pub fn hsi48(mut self, on: bool) -> Self {
        self.hsi48 = on;
        self
    }

    /// Enables the MSI with the specified speed
    pub fn msi(mut self, range: MsiFreq) -> Self {
        self.msi = Some(range);
        self
    }

    /// Sets LSI clock on (the default) or off
    pub fn lsi(mut self, on: bool) -> Self {
        self.lsi = on;
        self
    }

    /// Sets a frequency for the APB1 bus
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB2 bus
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    /// Sets the system (core) frequency
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    /// Sets the system (core) frequency with some pll configuration
    pub fn sysclk_with_pll<F>(mut self, freq: F, cfg: PllConfig) -> Self
    where
        F: Into<Hertz>,
    {
        self.pll_config = Some(cfg);
        self.sysclk = Some(freq.into().0);
        self
    }

    #[cfg(not(any(
        // pllsai1cfgr.pllsai1pdiv missing in PAC
        feature = "stm32l433",
        feature = "stm32l443",
        feature = "stm32l475",
    )))]
    /// Sets the SAI1 frequency with some pll configuration
    pub fn sai1clk_with_pll<F>(mut self, freq: F, cfg: PllConfig) -> Self
    where
        F: Into<Hertz>,
    {
        self.pllsai1_config = Some(cfg);
        self.sai1clk = Some(freq.into().0);
        self
    }

    #[cfg(any(
        // feature = "stm32l475", // pllsai2cfgr.pllsai2pdiv missing in PAC
        feature = "stm32l476",
        feature = "stm32l486",
        feature = "stm32l496",
        feature = "stm32l4a6",
        feature = "stm32l4r9",
        feature = "stm32l4s9",
    ))]
    /// Sets the SAI2 frequency with some pll configuration
    pub fn sai2clk_with_pll<F>(mut self, freq: F, cfg: PllConfig) -> Self
    where
        F: Into<Hertz>,
    {
        self.pllsai2_config = Some(cfg);
        self.sai2clk = Some(freq.into().0);
        self
    }

    /// Sets the PLL source
    pub fn pll_source(mut self, source: PllSource) -> Self {
        self.pll_source = Some(source);
        self
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(&self, acr: &mut ACR, pwr: &mut Pwr) -> Clocks {
        let rcc = unsafe { &*RCC::ptr() };

        // Switch to MSI to prevent problems with PLL configuration.
        if rcc.cr.read().msion().bit_is_clear() {
            // Turn on MSI and configure it to 4MHz.
            rcc.cr.modify(|_, w| {
                w.msirgsel().set_bit(); // MSI Range is provided by MSIRANGE[3:0].
                w.msirange().range4m();
                w.msipllen().clear_bit();
                w.msion().set_bit()
            });

            // Wait until MSI is running
            while rcc.cr.read().msirdy().bit_is_clear() {}
        }
        if rcc.cfgr.read().sws().bits() != 0 {
            // Set MSI as a clock source, reset prescalers.
            rcc.cfgr.reset();
            // Wait for clock switch status bits to change.
            while rcc.cfgr.read().sws().bits() != 0 {}
        }

        //
        // 1. Setup clocks
        //

        // Turn on the internal 32 kHz LSI oscillator
        let lsi_used = match (self.lsi, &self.lse) {
            (true, _)
            | (
                _,
                &Some(LseConfig {
                    bypass: _,
                    css: ClockSecuritySystem::Enable,
                }),
            ) => {
                rcc.csr.modify(|_, w| w.lsion().set_bit());

                // Wait until LSI is running
                while rcc.csr.read().lsirdy().bit_is_clear() {}

                true
            }
            _ => false,
        };

        if let Some(lse_cfg) = &self.lse {
            // 1. Unlock the backup domain
            pwr.cr1.reg().modify(|_, w| w.dbp().set_bit());
            while pwr.cr1.reg().read().dbp().bit_is_clear() {}

            // 2. Setup the LSE
            rcc.bdcr.modify(|_, w| {
                w.lseon().set_bit(); // Enable LSE

                if lse_cfg.bypass == CrystalBypass::Enable {
                    w.lsebyp().set_bit();
                } else {
                    unsafe {
                        w.lsedrv().bits(0b11);
                    } // Max drive strength, TODO: should probably be settable
                }

                w
            });

            // Wait until LSE is running
            while rcc.bdcr.read().lserdy().bit_is_clear() {}

            // Setup CSS
            if lse_cfg.css == ClockSecuritySystem::Enable {
                // Enable CSS and interrupt
                rcc.bdcr.modify(|_, w| w.lsecsson().set_bit());
                rcc.cier.modify(|_, w| w.lsecssie().set_bit());
            }
        }

        // If HSE is available, set it up
        if let Some(hse_cfg) = &self.hse {
            rcc.cr.write(|w| {
                w.hseon().set_bit();

                if hse_cfg.bypass == CrystalBypass::Enable {
                    w.hsebyp().set_bit();
                }

                w
            });

            while rcc.cr.read().hserdy().bit_is_clear() {}

            // Setup CSS
            if hse_cfg.css == ClockSecuritySystem::Enable {
                // Enable CSS
                rcc.cr.modify(|_, w| w.csson().set_bit());
            }
        }

        if let Some(msi) = self.msi {
            unsafe {
                rcc.cr.modify(|_, w| {
                    w.msirange()
                        .bits(msi as u8)
                        .msirgsel()
                        .set_bit()
                        .msion()
                        .set_bit();

                    // If LSE is enabled, enable calibration of MSI
                    if self.lse.is_some() {
                        w.msipllen().set_bit();
                    }

                    w
                })
            };

            // Wait until MSI is running
            while rcc.cr.read().msirdy().bit_is_clear() {}
        }

        // Turn on USB, RNG Clock using the HSI48 CLK source
        if self.hsi48 {
            // p. 180 in ref-manual
            rcc.crrcr.modify(|_, w| w.hsi48on().set_bit());

            // Wait until HSI48 is running
            while rcc.crrcr.read().hsi48rdy().bit_is_clear() {}
        }

        // Select MSI as clock source for usb48, rng ...
        if let Some(MsiFreq::RANGE48M) = self.msi {
            unsafe { rcc.ccipr.modify(|_, w| w.clk48sel().bits(0b11)) };
        }

        //
        // 2. Setup PLL
        //

        // Select PLL source
        let (clock_speed, pll_source) = if let Some(source) = self.pll_source {
            match source {
                PllSource::HSE => {
                    if let Some(hse) = &self.hse {
                        (hse.speed, source)
                    } else {
                        panic!("HSE selected as PLL source, but not enabled");
                    }
                }
                PllSource::HSI16 => (HSI, source),
                PllSource::MSI => {
                    if let Some(msi) = self.msi {
                        (msi.to_hertz().0, source)
                    } else {
                        panic!("MSI selected as PLL source, but not enabled");
                    }
                }
            }
        } else {
            // No specific PLL source selected, do educated guess

            // 1. HSE
            if let Some(hse) = &self.hse {
                (hse.speed, PllSource::HSE)
            }
            // 2. MSI
            else if let Some(msi) = self.msi {
                (msi.to_hertz().0, PllSource::MSI)
            }
            // 3. HSI as fallback
            else {
                (HSI, PllSource::HSI16)
            }
        };

        // Check if HSI should be started
        if pll_source == PllSource::HSI16 || (self.msi.is_none() && self.hse.is_none()) {
            rcc.cr.write(|w| w.hsion().set_bit());
            while rcc.cr.read().hsirdy().bit_is_clear() {}
        }

        let pllconf = if self.pll_config.is_none() {
            if let Some(sysclk) = self.sysclk {
                // Calculate PLL multiplier and create a best effort pll config, just multiply n
                let plln = (2 * sysclk) / clock_speed;

                Some(PllConfig::new(
                    1,
                    plln as u8,
                    None,
                    None,
                    Some(PllDivider::Div2),
                ))
            } else {
                None
            }
        } else {
            self.pll_config
        };

        // Check that all M values are equal
        let p = [pllconf, self.pllsai1_config, self.pllsai2_config];
        let _ = p.iter().flatten().reduce(|first, pc| {
            assert_eq!(pc.m, first.m);
            pc
        });

        let sysclk = match (self.sysclk, self.msi) {
            (Some(sysclk), _) => sysclk,
            (None, Some(msi)) => msi.to_hertz().0,
            (None, None) => MsiFreq::RANGE4M.to_hertz().0,
        };

        assert!(sysclk <= 80_000_000);

        let (hpre_bits, hpre_div) = self
            .hclk
            .map(|hclk| match sysclk / hclk {
                // From p 194 in RM0394
                0 => unreachable!(),
                1 => (0b0000, 1),
                2 => (0b1000, 2),
                3..=5 => (0b1001, 4),
                6..=11 => (0b1010, 8),
                12..=39 => (0b1011, 16),
                40..=95 => (0b1100, 64),
                96..=191 => (0b1101, 128),
                192..=383 => (0b1110, 256),
                _ => (0b1111, 512),
            })
            .unwrap_or((0b0000, 1));

        let hclk = sysclk / hpre_div;

        assert!(hclk <= sysclk);

        let (ppre1_bits, ppre1) = self
            .pclk1
            .map(|pclk1| match hclk / pclk1 {
                // From p 194 in RM0394
                0 => unreachable!(),
                1 => (0b000, 1),
                2 => (0b100, 2),
                3..=5 => (0b101, 4),
                6..=11 => (0b110, 8),
                _ => (0b111, 16),
            })
            .unwrap_or((0b000, 1));

        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 <= sysclk);

        let (ppre2_bits, ppre2) = self
            .pclk2
            .map(|pclk2| match hclk / pclk2 {
                // From p 194 in RM0394
                0 => unreachable!(),
                1 => (0b000, 1),
                2 => (0b100, 2),
                3..=5 => (0b101, 4),
                6..=11 => (0b110, 8),
                _ => (0b111, 16),
            })
            .unwrap_or((0b000, 1));

        let pclk2 = hclk / u32(ppre2);

        assert!(pclk2 <= sysclk);

        // adjust flash wait states
        unsafe {
            acr.acr().write(|w| {
                w.latency().bits(if hclk <= 16_000_000 {
                    0b000
                } else if hclk <= 32_000_000 {
                    0b001
                } else if hclk <= 48_000_000 {
                    0b010
                } else if hclk <= 64_000_000 {
                    0b011
                } else {
                    0b100
                })
            })
        }

        let sysclk_src_bits;
        let mut msi = self.msi;
        if let Some(pllconf) = pllconf {
            pllconf.check(clock_speed).unwrap();

            // use PLL as source
            sysclk_src_bits = 0b11;
            rcc.cr.modify(|_, w| w.pllon().clear_bit());
            while rcc.cr.read().pllrdy().bit_is_set() {}

            let pllsrc_bits = pll_source.to_pllsrc();

            rcc.pllcfgr.modify(|_, w| unsafe {
                w.pllsrc()
                    .bits(pllsrc_bits)
                    .pllm()
                    .bits(pllconf.m)
                    .pllr()
                    .bits(pllconf.r.unwrap().to_bits())
                    .plln()
                    .bits(pllconf.n)
            });

            rcc.cr.modify(|_, w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_clear() {}

            rcc.pllcfgr.modify(|_, w| w.pllren().set_bit());

            // SW: PLL selected as system clock
            rcc.cfgr.modify(|_, w| unsafe {
                w.ppre2()
                    .bits(ppre2_bits)
                    .ppre1()
                    .bits(ppre1_bits)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .bits(sysclk_src_bits)
            });
        } else {
            // use MSI as fallback source for sysclk
            sysclk_src_bits = 0b00;
            if msi.is_none() {
                msi = Some(MsiFreq::RANGE4M);
            }

            // SW: MSI selected as system clock
            rcc.cfgr.write(|w| unsafe {
                w.ppre2()
                    .bits(ppre2_bits)
                    .ppre1()
                    .bits(ppre1_bits)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .bits(sysclk_src_bits)
            });
        }

        while rcc.cfgr.read().sws().bits() != sysclk_src_bits {}

        // SAI PLL

        #[cfg(not(any(
            // pllsai1cfgr.pllsai1pdiv missing in PAC
            feature = "stm32l433",
            feature = "stm32l443",
            feature = "stm32l475"
        )))]
        if let Some(pllconf) = self.pllsai1_config {
            pllconf.check(clock_speed).unwrap();

            rcc.cr.modify(|_, w| w.pllsai1on().clear_bit());
            while rcc.cr.read().pllsai1rdy().bit_is_set() {}

            rcc.pllsai1cfgr.modify(|_, w| unsafe {
                w.pllsai1pdiv()
                    .bits(pllconf.p.unwrap())
                    .pllsai1n()
                    .bits(pllconf.n)
            });

            rcc.cr.modify(|_, w| w.pllsai1on().set_bit());

            while rcc.cr.read().pllsai1rdy().bit_is_clear() {}

            rcc.pllsai1cfgr.modify(|_, w| w.pllsai1pen().set_bit());
        }

        #[cfg(any(
            // feature = "stm32l475", // pllsai2cfgr.pllsai2pdiv missing in PAC
            feature = "stm32l476",
            feature = "stm32l486",
            feature = "stm32l496",
            feature = "stm32l4a6",
            feature = "stm32l4r9",
            feature = "stm32l4s9",
        ))]
        if let Some(pllconf) = self.pllsai2_config {
            pllconf.check(clock_speed).unwrap();

            rcc.cr.modify(|_, w| w.pllsai2on().clear_bit());
            while rcc.cr.read().pllsai2rdy().bit_is_set() {}

            rcc.pllsai2cfgr.modify(|_, w| unsafe {
                w.pllsai2pdiv()
                    .bits(pllconf.p.unwrap())
                    .pllsai2n()
                    .bits(pllconf.n)
            });

            rcc.cr.modify(|_, w| w.pllsai2on().set_bit());

            while rcc.cr.read().pllsai2rdy().bit_is_clear() {}

            rcc.pllsai2cfgr.modify(|_, w| w.pllsai2pen().set_bit());
        }

        //
        // 3. Shutdown unused clocks that have auto-started
        //

        // MSI always starts on reset
        if msi.is_none() {
            rcc.cr
                .modify(|_, w| w.msion().clear_bit().msipllen().clear_bit())
        }

        //
        // 4. Clock setup done!
        //

        Clocks {
            hclk: Hertz(hclk),
            lsi: lsi_used,
            lse: self.lse.is_some(),
            msi,
            hsi48: self.hsi48,
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1,
            ppre2,
            sysclk: Hertz(sysclk),
            sai1clk: self.sai1clk.map(|clk| Hertz(clk)),
            sai2clk: self.sai2clk.map(|clk| Hertz(clk)),
            pll_source: pllconf.map(|_| pll_source),
        }
    }
}

#[derive(Clone, Copy, Debug)]
/// PLL output divider options
pub enum PllDivider {
    /// Divider PLL output by 2
    Div2 = 0b00,
    /// Divider PLL output by 4
    Div4 = 0b01,
    /// Divider PLL output by 6
    Div6 = 0b10,
    /// Divider PLL output by 8
    Div8 = 0b11,
}

impl PllDivider {
    #[inline(always)]
    fn to_bits(self) -> u8 {
        self as u8
    }

    #[inline(always)]
    fn to_division_factor(self) -> u32 {
        match self {
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div6 => 6,
            Self::Div8 => 8,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum PllConfigError {
    MinOutputDivider,
    MaxOutputDivider,
    MinMultiplier,
    MaxMultiplier,
    MinVcoInputClock,
    MaxVcoInputClock,
    MinVcoOutputClock,
    MaxVcoOutputClock,
    MaxOutputClock,
}

#[derive(Clone, Copy, Debug)]
/// PLL Configuration
pub struct PllConfig {
    // Main PLL division factor
    m: u8,
    // PLL multiplication factor
    n: u8,
    // PLL division factor for PLLR
    p: Option<u8>,
    // PLL division factor for PLLR
    q: Option<PllDivider>,
    // PLL division factor for PLLR
    r: Option<PllDivider>,
}

impl PllConfig {
    /// Create a new PLL config from manual settings
    ///
    /// PLL output = ((SourceClk / input_divider) * multiplier) / output_divider
    pub fn new(
        input_divider: u8,
        multiplier: u8,
        output_divider_p: Option<u8>,
        output_divider_q: Option<PllDivider>,
        output_divider_r: Option<PllDivider>,
    ) -> Self {
        assert!(input_divider > 0);

        PllConfig {
            m: input_divider - 1,
            n: multiplier,
            p: output_divider_p,
            q: output_divider_q,
            r: output_divider_r,
        }
    }

    fn check(&self, clock_speed: u32) -> Result<(), PllConfigError> {
        // Sanity-checks per RM0394, 6.4.4 PLL configuration register (RCC_PLLCFGR)
        let clock_speed = clock_speed / (self.m as u32 + 1);
        let vco = clock_speed * self.n as u32;

        if !(self.n >= 8) {
            return Err(PllConfigError::MinMultiplier);
        }
        if !(self.n <= 86) {
            return Err(PllConfigError::MaxMultiplier);
        }
        if !(clock_speed >= 4_000_000) {
            return Err(PllConfigError::MinVcoInputClock);
        }
        if !(clock_speed <= 16_000_000) {
            return Err(PllConfigError::MaxVcoInputClock);
        }
        if !(vco >= 64_000_000) {
            return Err(PllConfigError::MinVcoOutputClock);
        }
        if !(vco <= 334_000_000) {
            return Err(PllConfigError::MaxVcoOutputClock);
        }

        if let Some(p) = self.p {
            let output_clock = vco / p as u32;
            if !(p >= 2) {
                return Err(PllConfigError::MinOutputDivider);
            }
            if !(p <= 31) {
                return Err(PllConfigError::MaxOutputDivider);
            }
            if !(output_clock <= 80_000_000) {
                return Err(PllConfigError::MaxOutputClock);
            }
        }

        if let Some(q) = self.q {
            let q = q.to_division_factor();
            let output_clock = vco / q as u32;
            if !(q <= 8) {
                return Err(PllConfigError::MaxOutputDivider);
            }
            if !(output_clock <= 80_000_000) {
                return Err(PllConfigError::MaxOutputClock);
            }
        }

        if let Some(r) = self.r {
            let r = r.to_division_factor();
            let output_clock = vco / r;
            if !(r <= 8) {
                return Err(PllConfigError::MaxOutputDivider);
            }
            if !(output_clock <= 80_000_000) {
                return Err(PllConfigError::MaxOutputClock);
            }
        }

        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// PLL Source
pub enum PllSource {
    /// Multi-speed internal clock
    MSI,
    /// High-speed internal clock
    HSI16,
    /// High-speed external clock
    HSE,
}

impl PllSource {
    fn to_pllsrc(self) -> u8 {
        match self {
            Self::MSI => 0b01,
            Self::HSI16 => 0b10,
            Self::HSE => 0b11,
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    hclk: Hertz,
    hsi48: bool,
    msi: Option<MsiFreq>,
    lsi: bool,
    lse: bool,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
    sai1clk: Option<Hertz>,
    sai2clk: Option<Hertz>,
    pll_source: Option<PllSource>,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns status of HSI48
    pub fn hsi48(&self) -> bool {
        self.hsi48
    }

    // Returns the status of the MSI
    pub fn msi(&self) -> Option<MsiFreq> {
        self.msi
    }

    /// Returns status of the LSI
    pub fn lsi(&self) -> bool {
        self.lsi
    }

    // Return the status of the LSE
    pub fn lse(&self) -> bool {
        self.lse
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Get which source is being used for PLL
    pub fn pll_source(&self) -> Option<PllSource> {
        self.pll_source
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre1(&self) -> u8 {
        self.ppre1
    }
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    // Returns the SAI1 frequency
    pub fn sai1clk(&self) -> Option<Hertz> {
        self.sai1clk
    }

    // Returns the SAI1 frequency
    pub fn sai2clk(&self) -> Option<Hertz> {
        self.sai2clk
    }
}
