//! Serial Audio Interface
//!
//! Based on the implementation in the stm32h7xx-hal
//! https://github.com/stm32-rs/stm32h7xx-hal/tree/master/src/sai

use crate::stm32::sai1::CH;

use crate::stm32::SAI1;
#[cfg(any(
    feature = "stm32l496",
    feature = "stm32l476",
    feature = "stm32l475",
    // Missing in PAC
    // feature = "stm32l471"
))]
use crate::stm32::SAI2;

// clocks
use crate::time::Hertz;

const CLEAR_ALL_FLAGS_BITS: u32 = 0b0111_0111;

mod i2s;
use crate::rcc::{Clocks, Enable, RccBus, Reset};
pub use i2s::{
    I2SChanConfig, I2SClockStrobe, I2SCompanding, I2SComplement, I2SDataSize, I2SDir, I2SMode,
    I2SProtocol, I2SSync, I2sUsers, SaiI2sExt, I2S,
};

/// Trait for associating clocks with SAI instances
pub trait GetClkSAI {
    /// Return the kernel clock for the SAI - A
    ///
    /// # Panics
    ///
    /// Panics if the kernel clock is not running
    fn sai_a_ker_ck(clocks: &Clocks) -> Hertz;
    /// Return the kernel clock for the SAI - B
    ///
    /// # Panics
    ///
    /// Panics if the kernel clock is not running
    fn sai_b_ker_ck(clocks: &Clocks) -> Hertz;
}

impl GetClkSAI for SAI1 {
    fn sai_a_ker_ck(clocks: &Clocks) -> Hertz {
        clocks.sai1clk().unwrap()
    }

    fn sai_b_ker_ck(clocks: &Clocks) -> Hertz {
        clocks.sai1clk().unwrap()
    }
}

#[cfg(any(
    feature = "stm32l496",
    feature = "stm32l476",
    feature = "stm32l475",
    // Missing in PAC
    // feature = "stm32l471"
))]
impl GetClkSAI for SAI2 {
    fn sai_a_ker_ck(clocks: &Clocks) -> Hertz {
        clocks.sai2clk().unwrap()
    }

    fn sai_b_ker_ck(clocks: &Clocks) -> Hertz {
        clocks.sai2clk().unwrap()
    }
}

pub trait INTERFACE {}

/// SAI Events
///
/// Each event is a possible interrupt source, if enabled
#[derive(Copy, Clone, PartialEq)]
pub enum Event {
    /// Overdue/Underrun error detection
    Overdue,
    /// Mute detected (Rx only)
    Muted,
    /// Clock not setup per frame sync rules see RM0433 Section 51.4.6: Frame synchronization
    WrongClock,
    /// Data is available / is required in the FIFO
    Data,
    /// Frame synchronization signal is detected earlier than expected
    AnticipatedFrameSync,
    /// Frame synchronization signal is not present at the right time
    LateFrameSync,
}

/// SAI Channels
#[derive(Copy, Clone, PartialEq)]
pub enum SaiChannel {
    ChannelA,
    ChannelB,
}

/// Hardware serial audio interface peripheral
pub struct Sai<SAI, INTERFACE> {
    rb: SAI,
    master_channel: SaiChannel,
    slave_channel: Option<SaiChannel>,
    interface: INTERFACE,
}

macro_rules! sai_hal {
    ($($SAIX:ident: ($saiX:ident, $Rec:ident),)+) => {
        $(
            // Common to all interfaces
            impl<INTERFACE> Sai<$SAIX, INTERFACE> {
                /// Low level RCC initialisation
                fn sai_rcc_init(&mut self, apb2: &mut <$SAIX as RccBus>::Bus)
                {
                    // enable or reset $SAIX
                    <$SAIX>::enable(apb2);
                    <$SAIX>::reset(apb2);
                }

                /// Access to the current master channel
                fn master_channel<F, T>(&self, func: F) -> T
                    where F: FnOnce(&CH) -> T,
                {
                    match self.master_channel {
                        SaiChannel::ChannelA => func(&self.rb.cha),
                        SaiChannel::ChannelB => func(&self.rb.chb),
                    }
                }

                /// Access to the current slave channel, if set
                fn slave_channel<F, T>(&self, func: F) -> Option<T>
                    where F: FnOnce(&CH) -> T,
                {
                    match self.slave_channel {
                        Some(SaiChannel::ChannelA) => Some(func(&self.rb.cha)),
                        Some(SaiChannel::ChannelB) => Some(func(&self.rb.chb)),
                        None => None
                    }
                }

                /// Start listening for `event` on a given `channel`
                pub fn listen(&mut self, channel: SaiChannel, event: Event) {
                    let ch = match channel {
                        SaiChannel::ChannelA => &self.rb.cha,
                        SaiChannel::ChannelB => &self.rb.chb,
                    };
                    match event {
                        Event::Overdue              => ch.im.modify(|_, w| w.ovrudrie().set_bit()),
                        Event::Muted                => ch.im.modify(|_, w| w.mutedetie().set_bit()),
                        Event::WrongClock           => ch.im.modify(|_, w| w.wckcfgie().set_bit()),
                        Event::Data                 => ch.im.modify(|_, w| w.freqie().set_bit()),
                        Event::AnticipatedFrameSync => ch.im.modify(|_, w| w.afsdetie().set_bit()),
                        Event::LateFrameSync        => ch.im.modify(|_, w| w.lfsdetie().set_bit()),
                    }
                }

                /// Stop listening for `event` on a given `channel`
                pub fn unlisten(&mut self, channel: SaiChannel, event: Event) {
                    let ch = match channel {
                        SaiChannel::ChannelA => &self.rb.cha,
                        SaiChannel::ChannelB => &self.rb.chb,
                    };
                    match event {
                        Event::Overdue              => ch.im.modify(|_, w| w.ovrudrie().clear_bit()),
                        Event::Muted                => ch.im.modify(|_, w| w.mutedetie().clear_bit()),
                        Event::WrongClock           => ch.im.modify(|_, w| w.wckcfgie().clear_bit()),
                        Event::Data                 => ch.im.modify(|_, w| w.freqie().clear_bit()),
                        Event::AnticipatedFrameSync => ch.im.modify(|_, w| w.afsdetie().clear_bit()),
                        Event::LateFrameSync        => ch.im.modify(|_, w| w.lfsdetie().clear_bit()),
                    }
                    let _ = ch.im.read();
                    let _ = ch.im.read(); // Delay 2 peripheral clocks
                }

                /// Clears interrupt flag `event` on the `channel`
                ///
                /// Note: Event::Data is accepted but does nothing as that flag is cleared by reading/writing data
                pub fn clear_irq(&mut self, channel: SaiChannel, event: Event) {
                    let ch = match channel {
                        SaiChannel::ChannelA => &self.rb.cha,
                        SaiChannel::ChannelB => &self.rb.chb,
                    };
                    match event {
                        Event::Overdue              => ch.clrfr.write(|w| w.covrudr().set_bit()),
                        Event::Muted                => ch.clrfr.write(|w| w.cmutedet().set_bit()),
                        Event::WrongClock           => ch.clrfr.write(|w| w.cwckcfg().set_bit()),
                        Event::Data                 => (), // Cleared by reading/writing data
                        Event::AnticipatedFrameSync => ch.clrfr.write(|w| w.cafsdet().set_bit()),
                        Event::LateFrameSync        => ch.clrfr.write(|w| w.clfsdet().set_bit()),
                    }
                    let _ = ch.sr.read();
                    let _ = ch.sr.read(); // Delay 2 peripheral clocks
                }

                /// Clears all interrupts on the `channel`
                pub fn clear_all_irq(&mut self, channel: SaiChannel) {
                    let ch = match channel {
                        SaiChannel::ChannelA => &self.rb.cha,
                        SaiChannel::ChannelB => &self.rb.chb,
                    };
                    unsafe {
                        ch.clrfr.write(|w| w.bits(CLEAR_ALL_FLAGS_BITS));
                    }
                    let _ = ch.sr.read();
                    let _ = ch.sr.read(); // Delay 2 peripheral clocks
                }

                /// Mute `channel`, this is checked at the start of each frame
                /// Meaningful only in Tx mode
                pub fn mute(&mut self, channel: SaiChannel) {
                    match channel {
                        SaiChannel::ChannelA => &self.rb.cha.cr2.modify(|_, w| w.mute().enabled()),
                        SaiChannel::ChannelB => &self.rb.cha.cr2.modify(|_, w| w.mute().enabled()),
                    };
                }

                /// Unmute `channel`, this is checked at the start of each frame
                /// Meaningful only in Tx mode
                pub fn unmute(&mut self, channel: SaiChannel) {
                    match channel {
                        SaiChannel::ChannelA => &self.rb.cha.cr2.modify(|_, w| w.mute().disabled()),
                        SaiChannel::ChannelB => &self.rb.chb.cr2.modify(|_, w| w.mute().disabled()),
                    };
                }

                /*
                GCR missing in PAC

                /// Used to operate the audio block(s) with an external SAI for synchronization
                /// Refer to RM0433 rev 7 section 51.4.4 for valid values
                ///
                /// In short 0-3 maps SAI1-4 with the ones pointing to self being reserved.
                /// e.g. for SAI1 1-3 are valid and 0 is invalid
                pub fn set_sync_input(&mut self, selection: u8) {
                    assert!(selection < 0b1_00);
                    unsafe { &self.rb.gcr.modify(|_, w| w.syncout().bits(selection)) };
                }

                /// Synchronization output for other SAI blocks
                pub fn set_sync_output(&mut self, channel: Option<SaiChannel>) {
                    match channel {
                        Some(SaiChannel::ChannelA) => unsafe { &self.rb.gcr.modify(|_, w| w.syncout().bits(0b01) ) },
                        Some(SaiChannel::ChannelB) => unsafe { &self.rb.gcr.modify(|_, w| w.syncout().bits(0b10) ) },
                        None                       => unsafe { &self.rb.gcr.modify(|_, w| w.syncout().bits(0b00) ) },
                    };
                }
                */

                /// Enable DMA for the SAI peripheral.
                pub fn enable_dma(&mut self, channel: SaiChannel) {
                    match channel {
                        SaiChannel::ChannelA => self.rb.cha.cr1.modify(|_, w| w.dmaen().enabled()),
                        SaiChannel::ChannelB => self.rb.chb.cr1.modify(|_, w| w.dmaen().enabled()),
                    };
                }

                /// Disable DMA for the SAI peripheral.
                pub fn disable_dma(&mut self, channel: SaiChannel) {
                    match channel {
                        SaiChannel::ChannelA => self.rb.cha.cr1.modify(|_, w| w.dmaen().disabled()),
                        SaiChannel::ChannelB => self.rb.chb.cr1.modify(|_, w| w.dmaen().disabled()),
                    };
                }

                /// Releases the SAI peripheral
                pub fn free(self) -> $SAIX {
                    // Refer to RM0433 Rev 7 51.4.15 Disabling the SAI

                    // Master: Clear SAIEN
                    self.master_channel(|ch| {
                        ch.cr1.modify(|_, w| w.saien().disabled())
                    });

                    // Master: Wait for SAI to clear at the end of the
                    // frame
                    while self.master_channel(|ch| {
                        ch.cr1.read().saien().bit_is_set()
                    }) {}

                    // Slave: Clear SAIEN
                    self.slave_channel(|ch| {
                        ch.cr1.modify(|_, w| w.saien().disabled())
                    });

                    // Slave: Wait for SAI to clear
                    while self.slave_channel(|ch| {
                        ch.cr1.read().saien().bit_is_set()
                    }).unwrap_or(false) {}


                    self.rb
                }
            }
        )+
    }
}

sai_hal! {
    SAI1: (sai1, Sai1),
}
#[cfg(any(
    feature = "stm32l496",
    feature = "stm32l476",
    feature = "stm32l475",
    // Missing in PAC
    // feature = "stm32l471"
))]
sai_hal! {
    SAI2: (sai2, Sai2),
}
