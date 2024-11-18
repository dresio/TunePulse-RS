#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use embedded_time::{duration::*, rate::*};

use hal::{
    self,
    clocks::Clocks,
    gpio::{Edge, Pin, PinMode, Port, Pull},
    pac,
    timer::{Timer, TimerConfig},
};

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
    let timer_cfg = TimerConfig::default();

    clock_cfg.setup().unwrap();
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    let mut led_green = Pin::new(Port::B, 14, PinMode::Output);

    let mut button = Pin::new(Port::A, 15, PinMode::Input);
    button.pull(Pull::Up);
    button.enable_interrupt(Edge::Rising);

    led_green.set_high();

    let pwm_timer = Timer::new_tim2(_dp.TIM2, 1_000.0, timer_cfg, &clock_cfg);

    //TODO: Abstract out pwm generation so that it can be used in a more generic way
    // eg. duty cycle and pin input, DMA output of pwm output comparison to pin high/low
    // pwm_timer.enable_pwm_output(channel, compare, duty);

    'runtime_loop: loop {
        led_green.set_high();
        delay.delay_ms(1_000);
        led_green.set_low();
        delay.delay_ms(1_000);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
