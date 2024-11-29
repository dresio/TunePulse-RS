#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use embedded_time::{duration::*, rate::*, timer};

use hal::{
    self,
    clocks::Clocks,
    gpio::{Edge, Pin, PinMode, Port, Pull},
    pac,
    timer::{CaptureCompare, CountDir, OutputCompare, Timer, TimerConfig, UpdateReqSrc},
};

// use tunepulse_rs::pwm::pwm::PwmPin;

use defmt_rtt as _;
// global logger
use panic_probe as _;

/// PWM Example: This will control the Green LED to the duty cycle of the pwm signal
#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let _dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();
    let mut timer_cfg = TimerConfig::default();
    timer_cfg.one_pulse_mode = false;
    timer_cfg.update_request_source = UpdateReqSrc::Any;
    timer_cfg.auto_reload_preload = true;
    timer_cfg.alignment = hal::timer::Alignment::Center1;
    timer_cfg.capture_compare_dma = hal::timer::CaptureCompareDma::Update;
    timer_cfg.direction = CountDir::Up;

    clock_cfg.setup().unwrap();

    let mut led_green = Pin::new(Port::B, 14, PinMode::Output);
    let mut button = Pin::new(Port::A, 15, PinMode::Input);
    button.pull(Pull::Up);
    button.enable_interrupt(Edge::Rising);

    led_green.set_high();

    let mut dr_reset = Pin::new(Port::B, 2, PinMode::Output);
    dr_reset.set_high();

    let mut dr_enable = Pin::new(Port::A, 4, PinMode::Output);
    dr_enable.set_high();

    // Configure PWM pins
    Pin::new(Port::A, 1, PinMode::Alt(1)); // Configure PA1 as alternate function (PWM output)
    Pin::new(Port::A, 0, PinMode::Alt(1)); // Configure PA0 as alternate function
    Pin::new(Port::B, 11, PinMode::Alt(1)); // Configure PB11 as alternate function
    Pin::new(Port::B, 10, PinMode::Alt(1)); // Configure PB10 as alternate function

    let mut pwm_timer = Timer::new_tim2(_dp.TIM2, 20000.0, timer_cfg, &clock_cfg);
    pwm_timer.enable_pwm_output(hal::timer::TimChannel::C1, OutputCompare::Pwm1, 0.0);
    pwm_timer.enable_interrupt(hal::timer::TimerInterrupt::Update);

    let value: i16 = 16384;
    let duty = (value.abs() as u32 * pwm_timer.get_max_duty()) >> 15;
    pwm_timer.set_duty(hal::timer::TimChannel::C1, duty);

    pwm_timer.enable(); // Enable the timer

    'runtime_loop: loop {}
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
