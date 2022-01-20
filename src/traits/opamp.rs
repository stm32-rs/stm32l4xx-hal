
use crate::hal::{blocking::delay::DelayUs,};

#[derive(Copy, Clone, Debug)]
pub enum OPAMPS {
    OP1,
    OP2,
}

/// Opamp Operation Modes
#[derive(Copy, Clone, Debug)]
pub enum OperationMode {
    // wrong values provided
    External,
    // PGA mode, (configurable gain) connected to an ADC
    PgaADC1,
    // pga mode with external filter connected
    PgaADC1ExternalFiltering,
    // PGA mode, (configurable gain) connected to an ADC
    PgaADC2,
    // pga mode with external filter connected
    PgaADC2ExternalFiltering,
    // // Calibration Modes
    // CalibrationMode,
}

/// Opamp Power Mode configuration
#[derive(Copy, Clone, Debug)]
pub enum PowerMode {
    // Normal operational power (full bandwith)
    Normal,
    // low power mode with resticted bandwith
    LowPower,
}

#[derive(Copy, Clone, Debug)]
pub enum VINP {
    // output conected to external pin
    ExternalPin1,
    // output conected to DAC1
    DAC1,
    // output of opamp1
    OPAMP1
}

#[derive(Copy, Clone, Debug)]
pub enum VINM {
    // input connected to Opamp output of other Opamp
    LeakageInputPin,
    // internally connected to PGA gain settings
    PGA,
    // input conected to external pin (also pga mode with filtering)
    ExternalPin1,
    // input conected to external pin (also pga mode with filtering)
    ExternalPin2,
}

#[derive(Copy, Clone, Debug)]
pub enum PgaGain {
    PgaG1,
    PgaG2,
    PgaG4,
    PgaG8,
    PgaG16,
}



/// Opamp configuration error
#[derive(Copy, Clone, Debug)]
pub enum Error {
    // wrong values provided
    ConfigFailure,
    // operaation mode not supportet for this Opamp
    UnsuportedMode,
    // gain out of bound
    PDAGainOutOfBound,
    // not Implemented
    NotImplemented,
    // calibration Error
    CalibrationError,
}



/// A type alias for the result of opamp configruation error.
pub type Result = core::result::Result<(), Error>;

// impl From<> for Error {
//     fn from
// }

pub trait ConfigOpamp {

    fn set_opamp_oper_mode(&self, opmode: OperationMode) -> Result;

    fn conect_inputs(&self, vinp: VINP, vinm: VINM) -> Result;

    fn set_pga_gain(&self, gain: PgaGain) -> Result;

    fn set_power_mode(&self, power_mode: PowerMode) -> Result;

    fn calibrate(&self, delay: &mut impl DelayUs<u32>) -> Result;

    fn set_calibration_mode(&self, usertrim: bool);

    fn enable(&self, en: bool);
}
