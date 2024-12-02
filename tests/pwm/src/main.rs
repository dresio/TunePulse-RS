#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use hal::pac;
use panic_probe as _;

// Import tunepulse modules
use tunepulse_rs::pins::pins::*;
use tunepulse_rs::pwm::pwm::*;

/// PWM Test: This test pulls in the pwm module and uses it to control the duty cycle
/// of all 4 channels of PWM outputs on the TIM2 hardware
#[entry]
fn main() -> ! {
    let FREQUENCY: u16 = 10000;
    // Set up microcontroller peripherals
    let _dp = pac::Peripherals::take().unwrap();

    let clock_cfg = init_clocks();
    let mut timer = init_timer(_dp.TIM2, &clock_cfg, FREQUENCY);

    let mut dr_reset = RESET_PIN.to_pin();
    dr_reset.set_high();

    let mut dr_enable = ENABLE_PIN.to_pin();
    dr_enable.set_high();

    // Configure PWM pins with proper modes (we won't actaully use the reference)
    let mut pwm1 = Pwm::new(PWM_PIN_A1.to_pin());
    let mut pwm2 = Pwm::new(PWM_PIN_A2.to_pin());
    let mut pwm3 = Pwm::new(PWM_PIN_B1.to_pin());
    let mut pwm4 = Pwm::new(PWM_PIN_B2.to_pin());

    pwm1.init(&mut timer);
    pwm2.init(&mut timer);
    pwm3.init(&mut timer);
    pwm4.init(&mut timer);

    pwm1.set_duty_cycle(25, &mut timer);
    pwm2.set_duty_cycle(50, &mut timer);
    pwm3.set_duty_cycle(75, &mut timer);
    pwm4.set_duty_cycle(100, &mut timer);

    loop {}
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
