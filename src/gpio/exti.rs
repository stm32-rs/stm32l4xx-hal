use super::{marker, Edge, PinExt};
use crate::pac::{Interrupt, EXTI, SYSCFG};
use crate::rcc::{Enable, APB2};

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG, apb2: &mut APB2);
    fn trigger_on_edge(&mut self, exti: &mut EXTI, level: Edge);
    fn enable_interrupt(&mut self, exti: &mut EXTI);
    fn disable_interrupt(&mut self, exti: &mut EXTI);
    fn clear_interrupt_pending_bit(&mut self);
    fn check_interrupt(&self) -> bool;
    fn interrupt(&self) -> Interrupt;
}

impl<PIN> ExtiPin for PIN
where
    PIN: PinExt,
    PIN::Mode: marker::Interruptable,
{
    /// Make corresponding EXTI line sensitive to this pin
    #[inline(always)]
    fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG, apb2: &mut APB2) {
        // SYSCFG clock must be enabled in order to do register writes
        SYSCFG::enable(apb2);

        let i = self.pin_id();
        let port = self.port_id() as u32;
        let offset = 4 * (i % 4);
        match i {
            0..=3 => {
                syscfg.exticr1.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            4..=7 => {
                syscfg.exticr2.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            8..=11 => {
                syscfg.exticr3.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            12..=15 => {
                syscfg.exticr4.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            _ => unreachable!(),
        }
    }

    /// Generate interrupt on rising edge, falling edge or both
    #[inline(always)]
    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
        let i = self.pin_id();
        match edge {
            Edge::Rising => {
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::Falling => {
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::RisingFalling => {
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
            }
        }
    }

    /// Enable external interrupts from this pin.
    #[inline(always)]
    fn enable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr1
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
    }

    /// Disable external interrupts from this pin
    #[inline(always)]
    fn disable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr1
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
    }

    /// Clear the interrupt pending bit for this pin
    #[inline(always)]
    fn clear_interrupt_pending_bit(&mut self) {
        unsafe { (*EXTI::ptr()).pr1.write(|w| w.bits(1 << self.pin_id())) };
    }

    /// Reads the interrupt pending bit for this pin
    #[inline(always)]
    fn check_interrupt(&self) -> bool {
        unsafe { ((*EXTI::ptr()).pr1.read().bits() & (1 << self.pin_id())) != 0 }
    }

    /// Get the `Interrupt` for this pin.
    fn interrupt(&self) -> Interrupt {
        match self.pin_id() {
            0 => Interrupt::EXTI0,
            1 => Interrupt::EXTI1,
            2 => Interrupt::EXTI2,
            3 => Interrupt::EXTI3,
            4 => Interrupt::EXTI4,
            5..=9 => Interrupt::EXTI9_5,
            10..=15 => Interrupt::EXTI15_10,
            _ => unreachable!(),
        }
    }
}
