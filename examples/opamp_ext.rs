// #![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

// use cortex_m::asm;
use cortex_m_rt::entry;
use rtt_target::{rprint, rprintln};
use stm32l4xx_hal;
use stm32l4xx_hal::{opamp::*, pac, prelude::*};

use stm32l4xx_hal::traits::opamp::*;

// use embedded_hal::*;
// use embedded_hal::blocking::delay::DelayUs;
use stm32l4xx_hal::*;

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();
    rprint!("Initializing...");

    let _cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut _delay = delay::DelayCM::new(clocks);

    // // set IOs to analgo mode, which are used by the Opamp
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    // OPAMP1_VINP
    let mut _pa0 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // OPAMP1_VINM
    let mut _pa1 = gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // OPAMP1_VOUT
    let mut _pa3 = gpioa.pa3.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    // // set IOs to analgo mode, which are used by the Opamp
    // let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    // // OPAMP1_VINP
    // let mut _pa6 = gpioa.pa6.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // // OPAMP1_VINM
    // let mut _pa7 = gpioa.pa7.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // // OPAMP1_VOUT
    // let mut _pb0 = gpiob.pb0.into_analog(&mut gpiob.moder, &mut gpiob.pupdr);

    let ops = dp.OPAMP;

    //ops.opamp1_csr.opaen;
    let op1: OP1 = OP1::new(
        &ops.opamp1_csr,
        &ops.opamp1_otr,
        &ops.opamp1_lpotr,
        &ops.opamp1_csr,
        &rcc.apb1r1,
    );
    // let op2: OP2 = OP2::new(
    //     &ops.opamp2_csr,
    //     &ops.opamp2_otr,
    //     &ops.opamp2_lpotr,
    //     &ops.opamp1_csr,
    //     &rcc.apb1r1,
    // );

    rprintln!("op1 object created...");
    // set operation models
    let _ret = op1.set_opamp_oper_mode(OperationMode::External).unwrap();
    rprintln!("op1 operation mode set...");
    
   
    op1.enable(true);

    // ===================================================================================
    // Debug Info below

    rprintln!("opamode OP1: {}\n", ops.opamp1_csr.read().opamode().bits() as u8);
    // rprintln!("opaen: {}\n", ops.opamp1_csr.read().opaen().bits() as u8);
    // rprintln!("opa_range: {}\n", ops.opamp1_csr.read().opa_range().bits() as u8);
    // rprintln!("vp_sel: {}\n", ops.opamp1_csr.read().vp_sel().bits() as u8);
    // rprintln!("vm_sel: {}\n", ops.opamp1_csr.read().vm_sel().bits() as u8);
    // rprintln!("opalpm: {}\n", ops.opamp1_csr.read().opalpm().bits() as u8);
    // rprintln!("calout: {}\n", ops.opamp1_csr.read().calout().bits() as u8);
    // rprintln!("calon: {}\n", ops.opamp1_csr.read().calon().bits() as u8);
    // rprintln!("calsel: {}\n", ops.opamp1_csr.read().calsel().bits() as u8);
    // rprintln!("usertrim: {}\n", ops.opamp1_csr.read().usertrim().bits() as u8);
    // rprintln!("pga_gain: {}\n", ops.opamp1_csr.read().pga_gain().bits() as u8);

    
    loop {}
}
