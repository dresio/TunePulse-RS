#![no_main] // Do not use the standard main function (entry point defined elsewhere)
#![no_std] // Do not use the standard library (no_std environment)

use defmt_rtt as _; // Use defmt_rtt for logging over RTT (Real-Time Transfer), aliasing it to '_'
use panic_probe as _; // Use panic_probe as the panic handler, aliasing it to '_'

use hal::{ // Import hardware abstraction layer (HAL) modules
    self,
    clocks::Clocks, // Import clock configuration utilities
    gpio::{Edge, Pin, PinMode, Port, Pull}, // Import GPIO-related structs and enums
    pac, // Import peripheral access crate (PAC) for device-specific peripherals
    pac::TIM2, // Import TIM2 peripheral
    timer::*, // Import all timer-related items
};

use tunepulse_rs::{motor_driver::*, math_integer::trigonometry::*}; // Import custom motor driver and trigonometry modules from tunepulse_rs

#[rtic::app(device = pac, peripherals = true)] // Define an RTIC (Real-Time Interrupt-driven Concurrency) application with the given device and peripherals
mod app {
    use pwm_control::{ // Import PWM control modules
        motor_selector::MotorSelector, // Import MotorSelector for motor control
    };

    use super::*; // Bring all items from the parent module into scope

    #[shared] // Define shared resources between tasks (empty in this case)
    struct Shared {}

    #[local] // Define local resources for tasks
    struct Local {
        timer_pwd: Timer<TIM2>, // PWM timer instance
        underflow: bool, // Flag to switch between overflow and underflow of PWM center-aligned timer
        tick_counter: u16, // Counter for PWM ticks
    }

    #[init] // Initialization function
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device; // Get device peripherals

        let clock_cfg = Clocks::default(); // Create default clock configuration
        clock_cfg.setup().unwrap(); // Apply clock configuration

        init_driver_pins(); // Initialize driver pins
        init_button_it(); // Initialize button interrupt

        let mut timer_pwd = init_timer(dp.TIM2, &clock_cfg); // Initialize timer for PWM

        let initial_duties = [8192 / 2, 16384 / 2, 3000, 4000]; // Initial PWM duty cycles
        set_pwm_duties(&mut timer_pwd, &initial_duties); // Set initial PWM duties

        (
            Shared {},
            Local {
                timer_pwd,
                underflow: true,
                tick_counter: 0,
            },
        ) // Return shared and local resources
    }

    fn tick_motor(angle: i16, motor_sel: &mut MotorSelector) -> [i16; 4] {
        let angle = angle2sincos((angle as i32) << 16); // Convert angle to sine and cosine values
        let angle = (angle.0 / 5, angle.1 / 5); // Scale down sine and cosine values
        motor_sel.voltg = angle; // Set voltage vector
        motor_sel.voltg_sup = 25000; // Set supply voltage
        motor_sel.tick(); // Update motor state
        motor_sel.pwm_channels() // Return PWM channel values
    }

    fn init_button_it() {
        let mut sw1_button = Pin::new(Port::A, 10, PinMode::Input); // Initialize SW1 button pin as input
        sw1_button.pull(Pull::Up); // Enable pull-up resistor
        sw1_button.enable_interrupt(Edge::Rising); // Enable interrupt on rising edge
    }

    fn init_driver_pins() {
        let mut dr_reset = Pin::new(Port::B, 2, PinMode::Output); // Initialize driver reset pin as output
        dr_reset.set_high(); // Set driver reset pin high (disable reset)

        let mut dr_en = Pin::new(Port::A, 4, PinMode::Output); // Initialize driver enable pin as output
        dr_en.set_high(); // Set driver enable pin high (enable driver)

        // Configure PWM output pins for alternate function
        Pin::new(Port::A, 1, PinMode::Alt(1)); // Configure PA1 as alternate function (PWM output)
        Pin::new(Port::A, 0, PinMode::Alt(1)); // Configure PA0 as alternate function
        Pin::new(Port::B, 11, PinMode::Alt(1)); // Configure PB11 as alternate function
        Pin::new(Port::B, 10, PinMode::Alt(1)); // Configure PB10 as alternate function
    }

    fn init_timer(tim2: TIM2, clock_cfg: &Clocks) -> Timer<TIM2> {
        let mut timer_pwd = Timer::new_tim2(
            tim2,
            20000.0, // Set timer frequency to 20 kHz (PWM will be 40khz)
            TimerConfig {
                one_pulse_mode: false, // Continuous mode
                update_request_source: UpdateReqSrc::Any, // Update on any event
                auto_reload_preload: true, // Preload auto-reload register
                alignment: Alignment::Center1, // Center-aligned PWM mode 1
                capture_compare_dma: CaptureCompareDma::Update, // Capture/compare DMA on update
                direction: CountDir::Up, // Count up
            },
            clock_cfg,
        );

        // Enable PWM outputs on channels 1 to 4
        timer_pwd.enable_pwm_output(TimChannel::C1, OutputCompare::Pwm1, 0.0); // Channel 1
        timer_pwd.enable_pwm_output(TimChannel::C2, OutputCompare::Pwm1, 0.0); // Channel 2
        timer_pwd.enable_pwm_output(TimChannel::C3, OutputCompare::Pwm1, 0.0); // Channel 3
        timer_pwd.enable_pwm_output(TimChannel::C4, OutputCompare::Pwm1, 0.0); // Channel 4

        timer_pwd.enable_interrupt(TimerInterrupt::Update); // Enable timer update interrupt
        timer_pwd.enable(); // Start the timer

        timer_pwd // Return the timer instance
    }

    fn set_pwm_duties(timer: &mut Timer<TIM2>, duties: &[i16; 4]) {
        let max_duty = timer.get_max_duty(); // Get maximum duty cycle value
        // Set PWM duty cycles for channels 1 to 4
        set_pwm_duty(timer, TimChannel::C1, duties[0], max_duty); // Channel 1
        set_pwm_duty(timer, TimChannel::C2, duties[1], max_duty); // Channel 2
        set_pwm_duty(timer, TimChannel::C3, duties[2], max_duty); // Channel 3
        set_pwm_duty(timer, TimChannel::C4, duties[3], max_duty); // Channel 4
    }

    #[inline(always)]
    fn set_pwm_duty(
        timer: &mut Timer<TIM2>,
        channel: TimChannel,
        value: i16,
        max_period: u32,
    ) {
        let duty: u32 = (value.abs() as u32 * max_period) >> 15; // Calculate duty cycle based on value and max_period
        timer.set_duty(channel, duty); // Set duty cycle for the specified channel
    }

    #[task(binds = TIM2, local = [timer_pwd, underflow, tick_counter])] // Define interrupt handler for TIM2 update event
    fn tim2_period_elapsed(cx: tim2_period_elapsed::Context) {
        let speed = 50;
        cx.local
            .timer_pwd
            .clear_interrupt(TimerInterrupt::Update); // Clear timer interrupt flag

        *cx.local.tick_counter = cx.local.tick_counter.wrapping_add(speed); // Increment tick counter with wrapping

        if *cx.local.underflow {
            pwm_callback(cx.local.tick_counter, cx.local.timer_pwd); // Call PWM update callback
        } else {
            analog_callback(); // Call analog update callback
        }

        *cx.local.underflow = !*cx.local.underflow; // Toggle underflow flag
    }

    fn pwm_callback(tick_counter: &u16, timer_pwd: &mut Timer<TIM2>) {
        static mut MOTOR_SEL: Option<MotorSelector> = None; // Static mutable variable for motor selector

        unsafe {
            if MOTOR_SEL.is_none() {
                MOTOR_SEL = Some(MotorSelector::new(pwm_control::MotorType::STEPPER)); // Initialize motor selector if not already done
            }
            let motor_sel = MOTOR_SEL.as_mut().unwrap(); // Get mutable reference to motor selector
            let pwm_values = tick_motor(*tick_counter as i16, motor_sel); // Compute new PWM values based on tick counter
            set_pwm_duties(timer_pwd, &pwm_values); // Update PWM duty cycles
        }
    }

    fn analog_callback() {
        // Placeholder for analog callback function
        // TODO: Implement ADC reading
        // defmt::println!("ANALOG_Callback called"); // Debug print statement
    }
}

#[defmt::panic_handler] // Define panic handler using defmt
fn panic() -> ! {
    cortex_m::asm::udf() // Trigger undefined instruction exception
}
