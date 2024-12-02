#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use hal::pac;
use panic_probe as _;

// Import tunepulse modules
use tunepulse_rs::{
    encoder_position::EncoderPosition, // Encoder position handling};
    math_integer::trigonometry::*,     // Trigonometry functions
    motor_driver::pwm_control::{
        motor_selector::MotorSelector, phase_selector::PhaseSelector, MotorType, PhasePattern,
    }, // Motor control modules
    pins::pins::*,                     // pin definitions
    pwm::pwm::*,                       // hardware pwm control
};

/// PWM Test: This test pulls in the pwm module and uses it to control the duty cycle
/// of all 4 channels of PWM outputs on the TIM2 hardware
#[entry]
fn main() -> ! {
    let FREQUENCY = 10000;

    // Set up microcontroller peripherals
    let _dp = pac::Peripherals::take().unwrap();

    let clock_cfg = init_clocks();
    let mut timer = init_timer(_dp.TIM2, &clock_cfg, FREQUENCY);

    let mut dr_reset = RESET_PIN.to_pin();
    dr_reset.set_high();

    let mut dr_enable = ENABLE_PIN.to_pin();
    dr_enable.set_high();

    // Configure PWM pins with proper modes (we won't actaully use the reference)
    let mut chan1 = Pwm::new(PWM_PIN_A2.to_pin());
    let mut chan2 = Pwm::new(PWM_PIN_A1.to_pin());
    let mut chan3 = Pwm::new(PWM_PIN_B1.to_pin());
    let mut chan4 = Pwm::new(PWM_PIN_B2.to_pin());

    chan1.init(&mut timer);
    chan2.init(&mut timer);
    chan3.init(&mut timer);
    chan4.init(&mut timer);

    // Initialize motor and phase selectors
    let mut motor_sel = MotorSelector::new(MotorType::STEPPER);
    let mut phase_sel = PhaseSelector::new(PhasePattern::ABCD as u8);
    // Initialize encoder position tracking
    let mut encoder_pos = EncoderPosition::new(0, FREQUENCY, 220);

    loop {}
}

// Function to calculate PWM values for the motor based on the given angle
fn tick_motor(
    angle: i16,
    motor_sel: &mut MotorSelector,
    phase_sel: &mut PhaseSelector,
) -> [i16; 4] {
    // Convert the angle to sine and cosine values
    let angle = angle2sincos((angle as i32) << 16);
    // Scale down the sine and cosine values
    let angle = (angle.0 / 5, angle.1 / 5);
    // Set the voltage values in the motor selector
    motor_sel.voltg = angle;
    motor_sel.voltg_sup = 25000;
    // Update the motor selector state
    motor_sel.tick();
    // Update the phase selector channels based on motor PWM channels
    phase_sel.ch_abcd = motor_sel.pwm_channels();
    // Update the phase selector state
    phase_sel.tick();
    // Return the PWM channel values
    phase_sel.pwm_channels()
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
