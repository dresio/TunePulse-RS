use hal::{
    clocks::Clocks,
    gpio::{Pin, Port},
    pac::TIM2,
    timer::{
        Alignment, CaptureCompareDma, CountDir, OutputCompare, TimChannel, Timer, TimerConfig,
        TimerInterrupt, UpdateReqSrc,
    },
};

/// Initialize the clocks for the microcontroller
pub fn init_clocks() -> hal::clocks::Clocks {
    let clock_cfg = hal::clocks::Clocks::default();
    clock_cfg.setup().unwrap();
    clock_cfg
}

/// Initialize the TIM2 hardware for use in pwm
pub fn init_timer(tim2: TIM2, clock_cfg: &Clocks, freq: u16) -> Timer<TIM2> {
    // Create a new Timer with the specified frequency and configuration
    let mut timer_pwd = Timer::new_tim2(
        tim2,
        freq as f32,
        TimerConfig {
            one_pulse_mode: false,
            update_request_source: UpdateReqSrc::Any,
            auto_reload_preload: true,
            alignment: Alignment::Center1,
            capture_compare_dma: CaptureCompareDma::Update,
            direction: CountDir::Up,
        },
        clock_cfg,
    );
    // Enable update interrupt for the timer
    timer_pwd.enable_interrupt(TimerInterrupt::Update);
    // Start the timer
    timer_pwd.enable();

    // Return the initialized timer
    timer_pwd
}

/// PWM object that can be used to control the duty cycle of a PWM signal on a specific pin through hardware TIM2
///
/// # Arguments
///
/// * `duty_percent` - The duty cycle percentage as a `u8` value. Must be between 0 and 100.
/// * `timer_instance` - A mutable reference to the `Timer` instance used for PWM generation.
///
/// # Panics
///
/// This function will panic if the `duty_percent` is greater than 100.
///
/// # Example
///
/// ```
/// use tunepulse_rs::pins::pins::*;
/// use tunepulse_rs::pwm::pwm::*;
///
/// let _dp = pac::Peripherals::take().unwrap();
/// let clock_cfg = init_clocks();
/// let mut timer = init_timer(_dp.TIM2, &clock_cfg, 20000);
/// let mut pwm = Pwm::new(PWM_PIN_A2.to_pin(), &mut timer);
///
/// pwm.init(&mut timer);
///
/// pwm.set_duty_cycle(50, &mut timer);
/// ```
pub struct Pwm {
    pin: Pin,
}

impl Pwm {
    pub fn new(pin: Pin) -> Self {
        Pwm { pin }
    }

    pub fn init(&self, timer_instance: &mut Timer<TIM2>) {
        timer_instance.enable_pwm_output(self.get_channel(), OutputCompare::Pwm1, 0.0);
    }

    pub fn set_duty_cycle(&mut self, duty_percent: u8, timer_instance: &mut Timer<TIM2>) {
        if duty_percent > 100 {
            // panic!("Duty cycle must be between 0 and 100");
            timer_instance.set_duty(self.get_channel(), timer_instance.get_max_duty());
        } else {
            let final_duty = (duty_percent as u32 * timer_instance.get_max_duty()) / 100;
            timer_instance.set_duty(self.get_channel(), final_duty);
        }
    }

    /// Set the duty cycle of the PWM signal using a 16-bit signed integer value
    /// The duty cycle is calculated as (duty_16bit.abs() * timer_instance.get_max_duty()) >> 15
    pub fn set_16bit_duty_cycle(&mut self, duty_16bit: i16, timer_instance: &mut Timer<TIM2>) {
        let duty: u32 = (duty_16bit.abs() as u32 * timer_instance.get_max_duty()) >> 15;
        timer_instance.set_duty(self.get_channel(), duty);
    }

    fn get_channel(&self) -> TimChannel {
        match (self.pin.port, self.pin.pin) {
            (Port::A, 1) => TimChannel::C1,
            (Port::A, 0) => TimChannel::C2,
            (Port::B, 11) => TimChannel::C3,
            (Port::B, 10) => TimChannel::C4,
            _ => panic!("Invalid pin for PWM"),
        }
    }
}
