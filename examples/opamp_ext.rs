#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

// use cortex_m::asm;
use cortex_m_rt::entry;
// use rtt_target::{rprint, rprintln};
use stm32l4xx_hal::{opamp::*, prelude::*, pac};

use stm32l4xx_hal::traits::opamp::*;
//use stm32l4xx_hal::delay::Delay;
use stm32l4xx_hal::delay::DelayCM;

#[entry]
fn main() -> ! {

    // rtt_target::rtt_init_print!();
    // rprint!("Initializing...");

    let _cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut delay = DelayCM::new(clocks);

    // set IOs to analgo mode, which are used by the Opamp
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    // OPAMP1_VINP
    let mut _pa0 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // OPAMP1_VINM
    let mut _pa1 = gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // OPAMP1_VOUT
    let mut _pa3 = gpioa.pa3.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let ops = dp.OPAMP;
    //ops.opamp1_csr.opaen;
    let op1: OP1 = OP1::new(& ops.opamp1_csr,
                            & ops.opamp1_otr,
                            & ops.opamp1_lpotr,
                            &rcc.apb1r1,);
    // set operation models
    let _ret = op1.set_opamp_oper_mode(OperationMode::External);

    // calibrate the opamp in normal mode, since it has not been switched to low power mode
    op1.calibrate(&mut delay);

    // set user trim mode (as calibration for user has been perfomed above)
    op1.set_calibration_mode(true);
    op1.enable(true);

    loop {
    }
}
