#![no_main]
#![no_std]

use panic_rtt_target as _;

use cortex_m_rt::entry;
use rtt_target::{rprint, rprintln};
use stm32l4xx_hal::{adc::ADC, delay::Delay, opamp::*, pac, prelude::*};

use stm32l4xx_hal::traits::opamp::*;

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();
    rprint!("Initializing...");

    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    // set IOs to analgo mode, which are used by the Opamp
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    // OPAMP1_VINP
    let mut pa0 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let mut delay = Delay::new(cp.SYST, clocks);
    let mut adc = ADC::new(
        dp.ADC1,
        dp.ADC_COMMON,
        &mut rcc.ahb2,
        &mut rcc.ccipr,
        &mut delay,
    );

    let mut opamp1_out = adc.enable_opamp1_out();

    let ops = dp.OPAMP;
    let op1: OP1 = OP1::new(
        &ops.opamp1_csr,
        &ops.opamp1_otr,
        &ops.opamp1_lpotr,
        &ops.opamp1_csr,
        &rcc.apb1r1,
    );

    op1.set_opamp_oper_mode(OperationMode::Pga);
    // set pga gain
    op1.set_pga_gain(16);
    op1.enable(true);

    rprintln!(" done.");

    loop {
        // let value = adc.read(&mut opamp2_out).unwrap();
        let value_opamp1 = adc.read(&mut opamp1_out).unwrap();
        let value_a0 = adc.read(&mut pa0).unwrap();
        rprintln!("Value: pa0 {} opam1 {}", value_a0, value_opamp1);
    }
}
