use hal::{
    clocks::Clocks,
    pac::TIM2,
    timer::{
        Alignment, CaptureCompareDma, CountDir, OutputCompare, TimChannel, Timer, TimerConfig,
        TimerInterrupt, UpdateReqSrc,
    },
};

use super::pinout;
pub struct TimPWM {
    tim: Timer<TIM2>,
}

impl TimPWM {
    pub fn new(tim2: TIM2, clock_cfg: &Clocks, freq: u16) -> Self {
        // Create a new Timer with the specified frequency and configuration
        let mut timer = Timer::new_tim2(
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
        timer.enable_interrupt(TimerInterrupt::Update);
        // Start the timer
        timer.enable();

        // Return the initialized timer
        TimPWM { tim: timer }
    }

    pub fn get_timer(&mut self) -> &mut Timer<TIM2> {
        &mut self.tim
    }

    pub fn begin(&mut self) {
        // Enable PWM outputs on channels 1 to 4 with initial duty cycle 0.0
        self.tim
            .enable_pwm_output(TimChannel::C1, OutputCompare::Pwm1, 0.0);
        self.tim
            .enable_pwm_output(TimChannel::C2, OutputCompare::Pwm1, 0.0);
        self.tim
            .enable_pwm_output(TimChannel::C3, OutputCompare::Pwm1, 0.0);
        self.tim
            .enable_pwm_output(TimChannel::C4, OutputCompare::Pwm1, 0.0);

        pinout::driver::PWM_A1.init();
        pinout::driver::PWM_A2.init();
        pinout::driver::PWM_B1.init();
        pinout::driver::PWM_B2.init();
    }

    pub fn apply_pwm(&mut self, pwm: [i16; 4]) {
        let period = self.tim.get_max_duty();
        self.tim
            .set_duty(TimChannel::C1, Self::duty2period(pwm[0], period));
        self.tim
            .set_duty(TimChannel::C2, Self::duty2period(pwm[1], period));
        self.tim
            .set_duty(TimChannel::C3, Self::duty2period(pwm[2], period));
        self.tim
            .set_duty(TimChannel::C4, Self::duty2period(pwm[3], period));
    }

    fn duty2period(duty: i16, period: u32) -> u32 {
        // Calculate the duty cycle value based on the input value and maximum period
        if duty > 0 {
            (duty as u32 * period) >> 15
        } else {
            0
        }
    }
}
