use crate::hal::blocking::delay::DelayUs;
use crate::rcc::APB1R1; //{Enable, Reset, APB1R1, CCIPR};
use crate::stm32::opamp::*;
use crate::traits::opamp as opamp_trait;

use core::sync::atomic::{compiler_fence, Ordering};

// // s.common.ccr.modify(|_, w| w.vrefen().clear_bit());
// // self.csr.modify(|_, w| w.opaen().clear_bit());
// // .clear_bit()
// // .set_bit()
// // .bit_is_set()
// // .bit_is_clear()
// // let en = self.csr.read().opaen().bit_is_set();
// // let mode = self.csr.read().opamode().bits() as u8;

// // let en = self.csr.read().opaen().bit_is_set();
// // self.csr.modify(|_, w| w.opaen().clear_bit());
// // let mode = self.csr.read().opamode().bits() as u8;

macro_rules! opamps {
    (
        $(
            $op:ident,
            $csr_ty:ty,
            $otr_ty:ty,
            $lpotr_ty:ty,
            $range_ty:ty;
        )*
    ) => {
        $(

            pub struct $op<'a> {
                csr: &'a $csr_ty,
                otr: &'a $otr_ty,
                lpotr: &'a $lpotr_ty,
                range: &'a $range_ty,
            }

            impl<'a> $op<'a> {

                /// Create a new OPAMP instance. the default range is always set to >2.4V so that the device is
                /// not damaged at first call on typical eval-boards from ST
                pub fn new(
                    csr: &'a $csr_ty,
                    otr: &'a $otr_ty,
                    lpotr: &'a $lpotr_ty,
                    // to set default range to > 2.4V
                    range: &'a $range_ty,
                    // rcc pointer and enable the clock for the configuration registers
                    apb1r1: &'a APB1R1,
                ) -> Self {
                    // enable clock for configuration registers of OPAMPS
                    apb1r1.enr().modify(|_, w| w.opampen().set_bit());

                    // set OPA_RANGE = 1 (VDDA > 2.4V) by default, if ADC1 is not enabled
                    // if a lower voltage is applied, use an instance of ADC1 to clear the opa_range bit
                    if range.read().opaen().bit_is_clear() == true {
                        range.modify(|_, w| w.opa_range().set_bit()); // else expect bit is set correct
                    }
                    compiler_fence(Ordering::SeqCst);
                    $op {
                        csr: csr,
                        otr: otr,
                        lpotr: lpotr,
                        range: range,
                    }

                }

                /// opa range specifice the VDDA voltage applied to the device.
                /// Is valied for both OP, if there are two in the device
                /// it is by default set to >2.4V (high)
                /// do not use this function, if you do not know, what you are decoding
                /// you might damage the devices
                /// since the setting applies to all opamps in the device and before
                /// changeing this value, all opamps must be disabled; up to this pointe
                /// only one opamp is supportd in this file, for that only that opamp is disable
                /// before the value is set.
                /// you need to enable the opamp after calling this function separately
                pub fn set_opa_range(&self, high: bool) {
                    self.csr.modify(|_, w| w.opaen().clear_bit());
                    if high {
                        self.range.modify(|_, w| w.opa_range().set_bit());
                    } else {
                        self.range.modify(|_, w| w.opa_range().clear_bit());
                    }
                    compiler_fence(Ordering::SeqCst);
                }
            }

            impl<'a> opamp_trait::ConfigOpamp for $op<'a> {

                /// Set operation mode, External is the defalult mode, where all signals are routed to pins
                /// and must be configured connected to define the function of the opamp
                fn set_opamp_oper_mode(&self, opmode: opamp_trait::OperationMode) -> opamp_trait::Result {

                    match opmode {
                        opamp_trait::OperationMode::External => {
                                // disable OPEAN (before changeing anything else)
                                self.csr.modify(|_, w| w.opaen().clear_bit());
                                // set OPAMODE = 0
                                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b00)});
                                // VP_SEL = 0 (GPIO)
                                self.csr.modify(|_, w| w.vp_sel().clear_bit());
                                // VM_SEL = 0 (GPIO)
                                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
                                Ok(())
                        },
                        opamp_trait::OperationMode::Pga => {
                                // disable OPEAN (before changeing anything else)
                                self.csr.modify(|_, w| w.opaen().clear_bit());
                                // set OPAMODE = 3 // follower mode = pga gain = 1
                                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b11)});
                                // VP_SEL = 0 (GPIO)
                                self.csr.modify(|_, w| w.vp_sel().clear_bit());
                                // VM_SEL = 0 (GPIO)
                                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b10)});
                                Ok(())
                        },
                        opamp_trait::OperationMode::PgaExternalFiltering => {
                                // disable OPEAN (before changeing anything else)
                                self.csr.modify(|_, w| w.opaen().clear_bit());
                                // set OPAMODE = 2 // pga mode = pga, gain = 2..16 (no filtering in follower mode)
                                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                                // VP_SEL = 0 (GPIO)
                                self.csr.modify(|_, w| w.vp_sel().clear_bit());
                                // VM_SEL = 0 (GPIO) for external filtering
                                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
                                Ok(())
                        },
                        _ => Err(opamp_trait::Error::NotImplemented),
                    }
                }

                /// Connect the input pins of the opamp to DAC or internnaly to follower or leakage-input
                fn conect_inputs(&self, vinp: opamp_trait::VINP, vinm: opamp_trait::VINM) -> opamp_trait::Result {
                    match vinp {
                        opamp_trait::VINP::ExternalPin1 => {
                            // VP_SEL = 0 (GPIO), PA0
                            self.csr.modify(|_, w| w.vp_sel().clear_bit());
                        },
                        opamp_trait::VINP::DAC1 => {
                            // VP_SEL = 1 (DAC1)
                            self.csr.modify(|_, w| w.vp_sel().set_bit());
                        },
                        _ => return Err(opamp_trait::Error::NotImplemented),
                    };
                    match vinm {
                        opamp_trait::VINM::ExternalPin1 => {
                            // VM_SEL = 0 (GPIO), PA1 for external filtering
                            self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
                        },
                        opamp_trait::VINM::LeakageInputPin => {
                            // VM_SEL = 01
                            self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b01)});
                        },
                        opamp_trait::VINM::PGA=> {
                            // VM_SEL = 10
                            self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b10)});
                        },
                        _ => return Err(opamp_trait::Error::NotImplemented),
                    };
                    Ok(())
                }

                /// Set the gain in pga mode, the device supports
                /// the values 1, 2, 4, 8, 16 all other values are ignored
                fn set_pga_gain(&self, gain: u16) -> opamp_trait::Result {
                    let opaen_state: bool = self.csr.read().opaen().bit_is_set();
                    // disable OPEAN (before changeing anything else)
                    self.csr.modify(|_, w| w.opaen().clear_bit());

                    match gain {
                        1 => {
                            // set OPAMODE = 3 // follower mode = pga gain = 1
                            self.csr.modify(|_, w| unsafe {w.opamode().bits(0b11)});
                        },
                        2  => {
                            // set OPAMODE = 3
                            self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                            // set PGA_GAIN = 2 // follower mode = pga gain = 1
                            self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b00)});
                        },
                        4  => {
                            // set OPAMODE = 3
                            self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                            // set PGA_GAIN = 2 // follower mode = pga gain = 1
                            self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b01)});
                        },
                        8  => {
                            // set OPAMODE = 3
                            self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                            // set PGA_GAIN = 2 // follower mode = pga gain = 1
                            self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b10)});
                        },
                        16  => {
                            // set OPAMODE = 3
                            self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                            // set PGA_GAIN = 2 // follower mode = pga gain = 1
                            self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b11)});
                        },
                        _   => return Err(opamp_trait::Error::NotImplemented),
                    };

                    if opaen_state == true {
                        // if it has been enabled before, enable it again
                        self.csr.modify(|_, w| w.opaen().set_bit());
                    }
                    Ok(())
                }

                /// the function preserves the enable state
                /// when the function is called and the opamp is enabled, it is disabled
                /// and reenabled after switch the power mode.
                /// short incontinous signals may occrue
                fn set_power_mode(&self, power_mode: opamp_trait::PowerMode) -> opamp_trait::Result {
                    let ena_state = self.csr.read().opaen().bit_is_set();
                    self.enable(false);
                    match power_mode {
                        opamp_trait::PowerMode::Normal => {
                            // set normal mode
                            self.csr.modify(|_, w| w.opalpm().clear_bit());
                        },
                        opamp_trait::PowerMode::LowPower => {
                            // set normal mode
                            self.csr.modify(|_, w| w.opalpm().set_bit());
                        },
                        _ => return Err(opamp_trait::Error::NotImplemented),
                    };
                    if ena_state {
                        self.enable(true);
                    }

                    Ok(())
                }

                /// For the calibration to work, the opamp must be enabled and in
                /// Calibration must be called for low power and normal mode separately if both
                /// are needed
                /// USERTRIM is preserved as it is before calling calibrate()
                /// the driven load must be below 500uA during calibration
                /// and must not be in external filtering mode and PGA-GAIN=1
                fn calibrate(&self, delay: &mut impl DelayUs<u32>) -> opamp_trait::Result {
                    // get USERTRIM bit value
                    let usertrim = self.csr.read().usertrim().bit();
                    // set usertrim bit, so that calibration is possible
                    self.csr.modify(|_, w| w.usertrim().bit(true));
                    // set opamp into callibration mode
                    self.csr.modify(|_, w| w.calon().set_bit());
                    // select PMOS calibration first
                    self.csr.modify(|_, w| w.calsel().set_bit());
                    // read if in LowPower Mode or in normal mode
                    let low_poer_mode = self.csr.read().opalpm().bit_is_set();

                    // set N and P calibration registers to 0
                    if low_poer_mode == true {
                        self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetn().bits(0b00000)});
                        self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetp().bits(0b00000)});
                    } else {
                        self.otr.modify(|_, w| unsafe {w.trimoffsetn().bits(0b00000)});
                        self.otr.modify(|_, w| unsafe {w.trimoffsetp().bits(0b00000)});
                    }

                    // increase calibration reg P till it toggles
                    for i in 0..32 {
                        if low_poer_mode == true {
                            self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetp().bits(i)});
                        } else {
                            self.otr.modify(|_, w| unsafe {w.trimoffsetp().bits(i)});
                        }

                        compiler_fence(Ordering::SeqCst);
                        // wait at least 1ms to new config to settle
                        delay.delay_us(1200);
                        if self.csr.read().calout().bit_is_set() == false {
                            break;
                        }
                    }
                    // if this point is reached, an the flag didn't change over the whole range
                    if self.csr.read().calout().bit_is_set() == true {
                        return Err(opamp_trait::Error::CalibrationError);
                    }

                    // select NMOS calibration first
                    self.csr.modify(|_, w| w.calsel().clear_bit());
                    // increase calibration reg N till it toggles
                    for i in 0..32 {
                        if low_poer_mode == true {
                            self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetn().bits(i)});
                        } else {
                            self.otr.modify(|_, w| unsafe {w.trimoffsetn().bits(i)});
                        }
                        compiler_fence(Ordering::SeqCst);
                        // wait for at least 1ms to settle of Configuration
                        delay.delay_us(1200);
                        if self.csr.read().calout().bit_is_set() == true {
                            break
                        }
                    }
                    if self.csr.read().calout().bit_is_set() == false {
                        return Err(opamp_trait::Error::CalibrationError);
                    }
                    // set opamp into normal mode
                    self.csr.modify(|_, w| w.calon().clear_bit());
                    // restore usertrim bit as it was before caling calibrate()
                    self.csr.modify(|_, w| w.usertrim().bit(usertrim));

                    Ok(())
                }
                /// in calibration mode the calibrated values are used, which were set with calibrate()
                /// default is using factory trimmed values which should be good for room temperatures
                fn set_calibration_mode(&self, usertrim: bool){
                    if usertrim {
                        self.csr.modify(|_, w| w.usertrim().set_bit());
                    } else {
                        self.csr.modify(|_, w| w.usertrim().clear_bit());
                    }
                }

                /// enable the opamp
                fn enable(&self, en: bool) {
                    compiler_fence(Ordering::SeqCst);
                    if en {
                        self.csr.modify(|_, w| w.opaen().set_bit());
                    } else {
                        self.csr.modify(|_, w| w.opaen().clear_bit());
                    }
                }
            }
        )*
    }

}

opamps!(
    OP1, OPAMP1_CSR, OPAMP1_OTR, OPAMP1_LPOTR, OPAMP1_CSR;
    OP2, OPAMP2_CSR, OPAMP2_OTR, OPAMP2_LPOTR, OPAMP1_CSR;
);
