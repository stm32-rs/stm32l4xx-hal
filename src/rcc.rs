//! Reset and Clock Control

use crate::stm32::{rcc, RCC};
use cast::u32;

use crate::flash::ACR;
use crate::pwr::Pwr;
use crate::time::Hertz;

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
            ahb1: AHB1 { _0: () },
            ahb2: AHB2 { _0: () },
            ahb3: AHB3 { _0: () },
            apb1r1: APB1R1 { _0: () },
            apb1r2: APB1R2 { _0: () },
            apb2: APB2 { _0: () },
            bdcr: BDCR { _0: () },
            csr: CSR { _0: () },
            crrcr: CRRCR { _0: () },
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
                pll_source: None,
                pll_config: None,
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

/// AMBA High-performance Bus 1 (AHB1) registers
pub struct AHB1 {
    _0: (),
}

impl AHB1 {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn enr(&mut self) -> &rcc::AHB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1enr }
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn rstr(&mut self) -> &rcc::AHB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1rstr }
    }
}

/// AMBA High-performance Bus 2 (AHB2) registers
pub struct AHB2 {
    _0: (),
}

impl AHB2 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb2rstr }
    }
}

/// AMBA High-performance Bus (AHB3) registers
pub struct AHB3 {
    _0: (),
}

impl AHB3 {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn enr(&mut self) -> &rcc::AHB3ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3enr }
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn rstr(&mut self) -> &rcc::AHB3RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3rstr }
    }
}

/// Advanced Peripheral Bus 1 (APB1) register 1 registers
pub struct APB1R1 {
    _0: (),
}

impl APB1R1 {
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR1 {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1enr1 }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR1 {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1rstr1 }
    }
}

/// Advanced Peripheral Bus 1 (APB1) register 2 registers
pub struct APB1R2 {
    _0: (),
}

impl APB1R2 {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR2 {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1enr2 }
    }
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR2 {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1rstr2 }
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
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
    pll_source: Option<PllSource>,
    pll_config: Option<PllConfig>,
}

impl CFGR {
    /// Add an HSE to the system
    pub fn hse<F>(mut self, freq: F, bypass: CrystalBypass, css: ClockSecuritySystem) -> Self
    where
        F: Into<Hertz>,
    {
        self.hse = Some(HseConfig {
            speed: freq.into().0,
            bypass: bypass,
            css: css,
        });

        self
    }

    /// Add an 32.768 kHz LSE to the system
    pub fn lse(mut self, bypass: CrystalBypass, css: ClockSecuritySystem) -> Self {
        self.lse = Some(LseConfig {
            bypass: bypass,
            css: css,
        });

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

    /// Sets the PLL source
    pub fn pll_source(mut self, source: PllSource) -> Self {
        self.pll_source = Some(source);
        self
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(&self, acr: &mut ACR, pwr: &mut Pwr) -> Clocks {
        let rcc = unsafe { &*RCC::ptr() };

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

            // 2. Setup the LSE
            rcc.bdcr.modify(|_, w| {
                unsafe { //TODO: fix RTC sel
                    w.lseon().set_bit().rtcsel().bits(0b01); // Enable LSE and use as RTC source
                }

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
                    if let Some(_) = self.lse {
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

                Some(PllConfig::new(1, plln as u8, PllDivider::Div2))
            } else {
                None
            }
        } else {
            self.pll_config
        };

        let sysclk = self.sysclk.unwrap_or(HSI);

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
                w.latency().bits(if sysclk <= 24_000_000 {
                    0b000
                } else if sysclk <= 48_000_000 {
                    0b001
                } else {
                    0b010
                })
            })
        }

        let sysclk_src_bits;
        if let Some(pllconf) = pllconf {
            // Sanity-checks per RM0394, 6.4.4 PLL configuration register (RCC_PLLCFGR)
            let r = pllconf.r.to_division_factor();
            let clock_speed = clock_speed / (pllconf.m as u32 + 1);
            let vco = clock_speed * pllconf.n as u32;
            let output_clock = vco / r;

            assert!(r <= 8); // Allowed max output divider
            assert!(pllconf.n >= 8); // Allowed min multiplier
            assert!(pllconf.n <= 86); // Allowed max multiplier
            assert!(clock_speed >= 4_000_000); // VCO input clock min
            assert!(clock_speed <= 16_000_000); // VCO input clock max
            assert!(vco >= 64_000_000); // VCO output min
            assert!(vco <= 334_000_000); // VCO output max
            assert!(output_clock <= 80_000_000); // Max output clock

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
                    .bits(pllconf.r.to_bits())
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
            // use HSI as source
            sysclk_src_bits = 0b01;

            rcc.cr.write(|w| w.hsion().set_bit());
            while rcc.cr.read().hsirdy().bit_is_clear() {}

            // SW: HSI selected as system clock
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

        //
        // 3. Shutdown unused clocks that have auto-started
        //

        // MSI always starts on reset
        if self.msi.is_none() {
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
            msi: self.msi,
            hsi48: self.hsi48,
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1: ppre1,
            ppre2: ppre2,
            sysclk: Hertz(sysclk),
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
/// PLL Configuration
pub struct PllConfig {
    // Main PLL division factor
    m: u8,
    // Main PLL multiplication factor
    n: u8,
    // Main PLL division factor for PLLCLK (system clock)
    r: PllDivider,
}

impl PllConfig {
    /// Create a new PLL config from manual settings
    ///
    /// PLL output = ((SourceClk / input_divider) * multiplier) / output_divider
    pub fn new(input_divider: u8, multiplier: u8, output_divider: PllDivider) -> Self {
        assert!(input_divider > 0);

        PllConfig {
            m: input_divider - 1,
            n: multiplier,
            r: output_divider,
        }
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

    /// Returns status of HSI48
    pub fn lsi(&self) -> bool {
        self.lsi
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
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
}
