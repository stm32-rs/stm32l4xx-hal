

// use core::{
//     // convert::Infallible,
//     ops::DerefMut,
//     // sync::atomic::{self, Ordering},
// };

use crate::stm32::{opamp::*};
use crate::traits::opamp as opamp_trait;
use crate::rcc::APB1R1; //{Enable, Reset, APB1R1, CCIPR};

// OPAMP1_CSR::opaen
// OPAMP1_OTR
// OPAMP1_LPOTR
// OPAMP2_CSR
// OPAMP2_OTR
// OPAMP2_LPOTR

//opamp1_csr::opaen

#[derive(Clone, Copy)]
pub struct OP1<'a> {
    csr: &'a OPAMP1_CSR,
    otr: &'a OPAMP1_OTR,
    lpotr: &'a OPAMP1_LPOTR,
}

impl<'a> OP1<'a> {
    pub fn new(
        csr: &'a OPAMP1_CSR,
        otr: &'a OPAMP1_OTR,
        lpotr: &'a OPAMP1_LPOTR,
        // rcc pointer and enable the clock for the configuration registers
        // RCC_APB1ENR1  OPAMPEN
        apb1r1: &'a APB1R1,
    ) -> Self {
        // enable clock for configuration registers of OPAMPS
        apb1r1.enr().modify(|_, w| w.opampen().set_bit());
        OP1 {
            csr: csr,
            otr: otr,
            lpotr: lpotr,
        }

    }
}

// s.common.ccr.modify(|_, w| w.vrefen().clear_bit());
// self.csr.modify(|_, w| w.opaen().clear_bit());
// .clear_bit()
// .set_bit()
// .bit_is_set()
// .bit_is_cleared()
// let en = self.csr.read().opaen().bit_is_set();
// let mode = self.csr.read().opamode().bits() as u8;

// let en = self.csr.read().opaen().bit_is_set();
// self.csr.modify(|_, w| w.opaen().clear_bit());
// let mode = self.csr.read().opamode().bits() as u8;

impl<'a> opamp_trait::ConfigOpamp for OP1<'a> {
    fn set_opamp_oper_mode(&self, opmode: opamp_trait::OperationMode) -> opamp_trait::Result {

        match opmode {
            opamp_trait::OperationMode::External => {
                // disable OPEAN (before changeing anything else)
                self.csr.modify(|_, w| w.opaen().clear_bit());
                // set OPA_RANGE = 1 (VDDA > 2.4V)
                self.csr.modify(|_, w| w.opa_range().set_bit());
                // set OPAMODE = 0
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b00)});
                // VP_SEL = 0 (GPIO)
                self.csr.modify(|_, w| w.vp_sel().clear_bit());
                // VM_SEL = 0 (GPIO)
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
                Ok(())
            },
            opamp_trait::OperationMode::PgaADC1 => {
                // disable OPEAN (before changeing anything else)
                self.csr.modify(|_, w| w.opaen().clear_bit());
                // set OPA_RANGE = 1 (VDDA > 2.4V)
                self.csr.modify(|_, w| w.opa_range().set_bit());
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b11)});
                // VP_SEL = 0 (GPIO)
                self.csr.modify(|_, w| w.vp_sel().clear_bit());
                // VM_SEL = 0 (GPIO)
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b10)});
                Ok(())
            },
            opamp_trait::OperationMode::PgaADC1ExternalFiltering => {
                // disable OPEAN (before changeing anything else)
                self.csr.modify(|_, w| w.opaen().clear_bit());
                // set OPA_RANGE = 1 (VDDA > 2.4V)
                self.csr.modify(|_, w| w.opa_range().set_bit());
                // set OPAMODE = 2 // pga mode = pga, gain = 2..16 (no filtering in follower mode)
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // VP_SEL = 0 (GPIO)
                self.csr.modify(|_, w| w.vp_sel().clear_bit());
                // VM_SEL = 0 (GPIO) for external filtering
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
                Ok(())
            },
            // opamp_trait::OperationMode::CalibrationMode => {
            //     // disable OPEAN (before changeing anything else)
            //     self.csr.modify(|_, w| w.opaen().clear_bit());
            //     // set OPA_RANGE = 1 (VDDA > 2.4V)
            //     self.csr.modify(|_, w| w.opa_range().set_bit());
            //     // set OPAMODE = 3 // pga mode = pga gain = 2 (no filtering in follower mode)
            //     self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
            //     // VP_SEL = 0 (GPIO)
            //     self.csr.modify(|_, w| w.vp_sel().clear_bit());
            //     // VM_SEL = 0 (GPIO) for external filtering
            //     self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
            //     ()
            // },
            _ => Err(opamp_trait::Error::NotImplemented),
        }
        // Err(opamp_trait::Error::NotImplemented)
    }

    fn conect_inputs(&self, vinp: opamp_trait::VINP, vinm: opamp_trait::VINM) -> opamp_trait::Result {
        match vinp {
            opamp_trait::VINP::ExternalPin1 => {
                // VP_SEL = 0 (GPIO)
                self.csr.modify(|_, w| w.vp_sel().clear_bit());
            },
            opamp_trait::VINP::DAC1 => {
                // VP_SEL = 0 (GPIO)
                self.csr.modify(|_, w| w.vp_sel().set_bit());
            },
            _ => return Err(opamp_trait::Error::NotImplemented),
        };
        match vinm {
            opamp_trait::VINM::ExternalPin1 => {
                // VM_SEL = 0 (GPIO) for external filtering
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b00)});
            },
            opamp_trait::VINM::LeakageInputPin => {
                // VM_SEL = 0 (GPIO) for external filtering
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b01)});
            },
            opamp_trait::VINM::PGA=> {
                // VM_SEL = 0 (GPIO) for external filtering
                self.csr.modify(|_, w| unsafe {w.vm_sel().bits(0b10)});
            },
            _ => return Err(opamp_trait::Error::NotImplemented),
        };
        Ok(())
    }

    fn set_pga_gain(&self, gain: opamp_trait::PgaGain) -> opamp_trait::Result {
        let opaen_state: bool = self.csr.read().opaen().bit_is_set();
        match gain {
            opamp_trait::PgaGain::PgaG1  => {
                // disable OPEAN (before changeing anything else)
                self.csr.modify(|_, w| w.opaen().clear_bit());
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b11)});
            },
            opamp_trait::PgaGain::PgaG2  => {
                // disable OPEAN (before changeing anything else)
                self.csr.modify(|_, w| w.opaen().clear_bit());
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b00)});
            },
            opamp_trait::PgaGain::PgaG4  => {
                // disable OPEAN (before changeing anything else)
                self.csr.modify(|_, w| w.opaen().clear_bit());
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b01)});
            },
            opamp_trait::PgaGain::PgaG8  => {
                // disable OPEAN (before changeing anything else)
                self.csr.modify(|_, w| w.opaen().clear_bit());
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b10)});
            },
            opamp_trait::PgaGain::PgaG16  => {
                // disable OPEAN (before changeing anything else)
                self.csr.modify(|_, w| w.opaen().clear_bit());
                // set OPAMODE = 3 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.opamode().bits(0b10)});
                // set PGA_GAIN = 2 // follower mode = pga gain = 1
                self.csr.modify(|_, w| unsafe {w.pga_gain().bits(0b11)});
            },
            _ => return Err(opamp_trait::Error::NotImplemented),
        };
        if opaen_state == true {
            // if it has been enabled before, enable it again
            self.csr.modify(|_, w| w.opaen().set_bit());
        }
        Ok(())
    }

    fn set_power_mode(&self, power_mode: opamp_trait::PowerMode) -> opamp_trait::Result {
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
        Ok(())
    }

    fn calibrate(&self) -> opamp_trait::Result {
        // set opamp into callibration mode
        self.csr.modify(|_, w| w.calon().set_bit());
        // select NMOS calibration first
        self.csr.modify(|_, w| w.calsel().clear_bit());
        // read if in LowPower Mode or in normal mode
        let low_poer_mode = self.csr.read().opalpm().bit_is_set();

        // set N calibration registers to 0
        if low_poer_mode == true {
            self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetn().bits(0b00000)});
        } else {
            self.otr.modify(|_, w| unsafe {w.trimoffsetn().bits(0b00000)});
        }

        // increase calibration reg N till it toggles
        while self.csr.read().calout().bit_is_set() == false {
            let t_val: u8;
            if low_poer_mode == true {
                t_val = self.lpotr.read().trimlpoffsetn().bits();
                self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetn().bits(t_val + 1)});
            } else {
                t_val = self.otr.read().trimoffsetn().bits();
                self.otr.modify(|_, w| unsafe {w.trimoffsetn().bits(t_val + 1)});
            }
            if t_val > 32 {
                return Err(opamp_trait::Error::CalibrationError);
            }
            // wait at least 1ms to new config to settle
            // TODO
        }

        // select NMOS calibration first
        self.csr.modify(|_, w| w.calsel().set_bit());

        // set P calibration registers to 0
        if low_poer_mode == true {
            self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetp().bits(0b00000)});
        } else {
            self.otr.modify(|_, w| unsafe {w.trimoffsetp().bits(0b00000)});
        }
        // increase calibration reg P till it toggles
        while self.csr.read().calout().bit_is_set() == false {
            let t_val: u8;
            if low_poer_mode == true {
                t_val = self.lpotr.read().trimlpoffsetp().bits();
                self.lpotr.modify(|_, w| unsafe {w.trimlpoffsetp().bits(t_val + 1)});
            } else {
                t_val = self.otr.read().trimoffsetp().bits();
                self.otr.modify(|_, w| unsafe {w.trimoffsetp().bits(t_val + 1)});
            }
            // wait at least 1ms to new config to settle
            if t_val > 32 {
                return Err(opamp_trait::Error::CalibrationError);
            }
            // wait for at least 1ms to settle of Configuration
            // TODO
        }
        Ok(())
    }

    fn enable(&self, en: bool) {
        if en {
            self.csr.modify(|_, w| w.opaen().set_bit());
        } else {
            self.csr.modify(|_, w| w.opaen().clear_bit());
        }
    }
}
