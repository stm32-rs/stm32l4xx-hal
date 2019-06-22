//! Reset and Clock Control

use core::cmp;

use cast::u32;
use crate::stm32::{rcc, RCC};

use crate::flash::ACR;
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
                hclk: None,
                hsi48: false,
                msi: None,
                lsi: false,
                pclk1: None,
                pclk2: None,
                sysclk: None,
                pllcfg: None,
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

    pub fn is_hsi48_on(&mut self) -> bool {
        self.crrcr().read().hsi48on().bit()
    }
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

const HSI: u32 = 16_000_000; // Hz

/// Clock configuration
pub struct CFGR {
    hclk: Option<u32>,
    hsi48: bool,
    msi: Option<MsiFreq>,
    lsi: bool,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    pllcfg: Option<PllConfig>
}

impl CFGR {
    /// Sets a frequency for the AHB bus
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Enable the 48Mh USB, RNG, SDMMC clock source. Not available on all stm32l4x6 series
    pub fn hsi48(mut self, on: bool) -> Self
    {
        self.hsi48 = on;
        self
    }

    pub fn msi(mut self, range: MsiFreq) -> Self
    {
        self.msi = Some(range);
        self
    }

    /// Sets LSI clock on (the default) or off
    pub fn lsi(mut self, on: bool) -> Self
    {
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
        self.pllcfg = Some(cfg);
        self.sysclk = Some(freq.into().0);
        self
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(&self, acr: &mut ACR) -> Clocks {

        let pllconf = if self.pllcfg.is_none() {
            let plln = (2 * self.sysclk.unwrap_or(HSI)) / HSI;
            let plln = cmp::min(cmp::max(plln, 2), 16);
            if plln == 2 {
                None
            } else {
                // create a best effort pll config, just multiply n
                // TODO should we reject this configuration as the clocks stored in RCC could cause timing issues?
                let conf = PllConfig {
                    m: 0b0,
                    r: 0b0,
                    n: plln as u8
                };
                Some(conf)
            }

        } else {
            let conf = self.pllcfg.unwrap();
            Some(conf)
        };

        let sysclk = self.sysclk.unwrap_or(HSI);

        assert!(sysclk <= 80_000_000);

        let (hpre_bits, hpre_div) = self.hclk
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

        let (ppre1_bits, ppre1) = self.pclk1
            .map(|pclk1| match hclk / pclk1 {
                // From p 194 in RM0394
                0 => unreachable!(),
                1 => (0b000, 1),
                2 => (0b100, 2),
                3..=5 => (0b101, 4),
                6..=11 => (0b110, 8),
                _ => (0b111, 16)
            })
            .unwrap_or((0b000, 1));

        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 <= sysclk);

        let (ppre2_bits, ppre2) = self.pclk2
            .map(|pclk2| match hclk / pclk2 {
                // From p 194 in RM0394
                0 => unreachable!(),
                1 => (0b000, 1),
                2 => (0b100, 2),
                3..=5 => (0b101, 4),
                6..=11 => (0b110, 8),
                _ => (0b111, 16)
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

        let rcc = unsafe { &*RCC::ptr() };
        let sysclk_src_bits;
        if let Some(pllconf) = pllconf {
            // use PLL as source
            sysclk_src_bits = 0b11;
            rcc.cr.modify(|_, w| w.pllon().clear_bit());
            while rcc.cr.read().pllrdy().bit_is_set() {}

            let pllsrc_bits = 0b10; // use HSI16 as PLL source
            rcc.cr.write(|w| w.hsion().set_bit());
            while rcc.cr.read().hsirdy().bit_is_clear() {}

            rcc.pllcfgr
            .modify(|_, w| unsafe {
                w.pllsrc()
                    .bits(pllsrc_bits)
                    .pllm().bits(pllconf.m)
                    .pllr().bits(pllconf.r)
                    .plln().bits(pllconf.n)
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

        // Turn on the internal 32khz lsi oscillator
        if self.lsi {
            rcc.csr.modify(|_, w| w.lsion().set_bit());
            // Wait until LSI is running
            while rcc.csr.read().lsirdy().bit_is_clear() {}
        }

        if let Some(msi) = self.msi {
            unsafe { rcc.cr.modify(|_, w| w.msirange().bits(msi as u8).msirgsel().set_bit().msion().set_bit() )};
            // Wait until MSI is running
            while rcc.cr.read().msirdy().bit_is_clear() {}
        }

        {
            // Turn on USB, RNG Clock using the HSI48 CLK source (default)
            if self.hsi48 {
                // p. 180 in ref-manual
                rcc.crrcr.modify(|_, w| w.hsi48on().set_bit());
                // Wait until HSI48 is running
                while rcc.crrcr.read().hsi48rdy().bit_is_clear() {}
            }
        }

        // Select MSI as clock source for usb48, rng ...
        if let Some(MsiFreq::RANGE48M) = self.msi {
            unsafe { rcc.ccipr.modify(|_, w| w.clk48sel().bits(MsiFreq::RANGE48M as u8)) };
        }
        //TODO proper clk48sel and other selects

        Clocks {
            hclk: Hertz(hclk),
            lsi: self.lsi,
            msi: self.msi,
            hsi48: self.hsi48,
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1: ppre1,
            ppre2: ppre2,
            sysclk: Hertz(sysclk),
        }
    }

}

#[derive(Clone, Copy)]
/// Pll Configuration - Calculation = ((SourceClk / m) * n) / r
pub struct PllConfig {
    /// Main PLL Division factor
    pub m: u8,
    /// Main Pll Multiplication factor
    pub n: u8,
    /// Main PLL division factor for PLLCLK (system clock)
    pub r: u8,
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
    pclk1: Hertz,
    pclk2: Hertz,
    // TODO remove `allow`
    #[allow(dead_code)]
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
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
