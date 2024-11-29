#![no_main]
#![no_std]

// Rust attributes indicating that this is a no_std (no standard library) application without a main function.

// Use the defmt_rtt crate for logging (Real-Time Transfer)
use defmt_rtt as _;
// Use panic_probe crate to handle panics
use panic_probe as _;

// Import necessary modules from the hardware abstraction layer (hal)
use hal::{
    self,
    clocks::Clocks, // For configuring the system clocks
    gpio::{Edge, Pin, PinMode, Port, Pull}, // GPIO handling
    pac, // Peripheral Access Crate (PAC) for device-specific peripherals
    pac::{SPI1, TIM2}, // Specific peripherals used
    spi::{BaudRate, Spi, SpiConfig, SpiMode}, // SPI peripheral configuration
    timer::*, // Timer peripherals
};

// Import custom modules from tunepulse_rs crate
use tunepulse_rs::{
    math_integer::trigonometry::*, // Trigonometry functions
    motor_driver::pwm_control::{
        motor_selector::MotorSelector, phase_selector::PhaseSelector, MotorType, PhasePattern,
    }, // Motor control modules
    encoder_position::EncoderPosition, // Encoder position handling
};

// Additional import for Cortex-M specific functionalities
use cortex_m;

#[rtic::app(device = pac, peripherals = true)]
mod app {
    // Bring all previous imports into scope
    use super::*;
    
    // Shared resources between tasks (empty in this case)
    #[shared]
    struct Shared {}
    
    // Local resources for tasks
    #[local]
    struct Local {
        timer_pwd: Timer<TIM2>, // Timer for PWM
        underflow: bool, // Underflow flag
        tick_counter: i16, // Counter for ticks
        motor_sel: MotorSelector, // Motor selector for PWM control
        phase_sel: PhaseSelector, // Phase selector for PWM control
        encoder_pos: EncoderPosition, // Encoder position tracking
        spi: Spi<SPI1>, // SPI peripheral
        cs_pin: Pin, // Chip Select pin for SPI
    }
    
    // Initialization function
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Define the frequency for the timer
        let FREQUENCY = 10000;
        // Get the device peripherals
        let dp = ctx.device;
    
        // Initialize the system clocks with default configuration
        let clock_cfg = Clocks::default();
        clock_cfg.setup().unwrap();
    
        // Get the system clock frequency
        let sysclk_freq = clock_cfg.sysclk(); // System clock frequency in Hz
        defmt::println!("System clock frequency: {} Hz", sysclk_freq);
    
        // Initialize driver pins and button interrupt
        init_driver_pins();
        init_button_it();
    
        // Initialize the PWM timer with the given frequency
        let mut timer_pwd = init_timer(dp.TIM2, &clock_cfg, FREQUENCY);
    
        // Initialize SPI pins and SPI peripheral
        let cs_pin = init_spi_pins();
        let spi = init_spi(dp.SPI1);
    
        // Initialize motor and phase selectors
        let mut motor_sel = MotorSelector::new(MotorType::STEPPER);
        let mut phase_sel = PhaseSelector::new(PhasePattern::ABCD as u8);
        // Initialize encoder position tracking
        let mut encoder_pos = EncoderPosition::new(0, FREQUENCY, 220);
        
        // Return the shared and local resources
        (
            Shared {},
            Local {
                timer_pwd,
                underflow: true,
                tick_counter: 0,
                motor_sel,
                phase_sel,
                encoder_pos,
                spi,
                cs_pin,
            },
        )
    }
    
    // Initialization functions
    // -------------------------
    
    // Function to initialize driver control pins and configure PWM output pins
    fn init_driver_pins() {
        // Create a new output pin for driver reset on Port B, Pin 2
        let mut dr_reset = Pin::new(Port::B, 2, PinMode::Output);
        // Set the driver reset pin high (inactive)
        dr_reset.set_high();
    
        // Create a new output pin for driver enable on Port A, Pin 4
        let mut dr_en = Pin::new(Port::A, 4, PinMode::Output);
        // Set the driver enable pin high (enabled)
        dr_en.set_high();
    
        // Configure PWM output pins for alternate function (Timer channels)
        Pin::new(Port::A, 1, PinMode::Alt(1)); // TIM2_CH2
        Pin::new(Port::A, 0, PinMode::Alt(1)); // TIM2_CH1
        Pin::new(Port::B, 11, PinMode::Alt(1)); // TIM2_CH4
        Pin::new(Port::B, 10, PinMode::Alt(1)); // TIM2_CH3
    }
    
    // Function to initialize the button interrupt
    fn init_button_it() {
        // Create a new input pin for the button on Port A, Pin 10
        let mut sw1_button = Pin::new(Port::A, 10, PinMode::Input);
        // Enable the internal pull-up resistor
        sw1_button.pull(Pull::Up);
        // Enable interrupt on rising edge
        sw1_button.enable_interrupt(Edge::Rising);
    }
    
    // Function to initialize SPI pins and return the CS (Chip Select) pin
    fn init_spi_pins() -> Pin {
        // Configure SPI1 pins for alternate function mode
        // PA5 (SCK), PA6 (MISO), PA7 (MOSI)
        Pin::new(Port::A, 5, PinMode::Alt(5)); // SPI1_SCK
        Pin::new(Port::A, 6, PinMode::Alt(5)); // SPI1_MISO
        Pin::new(Port::A, 7, PinMode::Alt(5)); // SPI1_MOSI
    
        // Configure CS pin on Port C, Pin 4 as output
        let mut cs_pin = Pin::new(Port::C, 4, PinMode::Output);
        // Set CS high (inactive)
        cs_pin.set_high(); // Set CS high (inactive)
    
        // Return the CS pin
        cs_pin
    }
    
    // Function to initialize the SPI peripheral with specific configuration
    fn init_spi(spi1: SPI1) -> Spi<SPI1> {
        // Create SPI configuration with mode 1
        let spi_cfg = SpiConfig {
            mode: SpiMode::mode1(),
            ..Default::default()
        };
    
        // Initialize SPI1 with the configuration and set BaudRate
        // Adjust BaudRate as needed; Div32 for lower speed if APB clock is high
        Spi::new(spi1, spi_cfg, BaudRate::Div32)
    }
    
    // Function to initialize Timer TIM2 for PWM outputs
    fn init_timer(tim2: TIM2, clock_cfg: &Clocks, freq: u16) -> Timer<TIM2> {
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
    
        // Enable PWM outputs on channels 1 to 4 with initial duty cycle 0.0
        timer_pwd.enable_pwm_output(TimChannel::C1, OutputCompare::Pwm1, 0.0);
        timer_pwd.enable_pwm_output(TimChannel::C2, OutputCompare::Pwm1, 0.0);
        timer_pwd.enable_pwm_output(TimChannel::C3, OutputCompare::Pwm1, 0.0);
        timer_pwd.enable_pwm_output(TimChannel::C4, OutputCompare::Pwm1, 0.0);
    
        // Enable update interrupt for the timer
        timer_pwd.enable_interrupt(TimerInterrupt::Update);
        // Start the timer
        timer_pwd.enable();
    
        // Return the initialized timer
        timer_pwd
    }
    
    // Utility functions
    // -----------------
    
    // Function to calculate PWM values for the motor based on the given angle
    fn tick_motor(angle: i16, motor_sel: &mut MotorSelector, phase_sel: &mut PhaseSelector) -> [i16; 4] {
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
    
    // Function to set PWM duties on all four channels
    fn set_pwm_duties(timer: &mut Timer<TIM2>, duties: &[i16; 4]) {
        // Get the maximum duty cycle value from the timer
        let max_duty = timer.get_max_duty();
        // Set the duty cycle for each channel
        set_pwm_duty(timer, TimChannel::C1, duties[0], max_duty);
        set_pwm_duty(timer, TimChannel::C2, duties[1], max_duty);
        set_pwm_duty(timer, TimChannel::C3, duties[2], max_duty);
        set_pwm_duty(timer, TimChannel::C4, duties[3], max_duty);
    }
    
    // Function to set PWM duty cycle for a single channel
    #[inline(always)]
    fn set_pwm_duty(timer: &mut Timer<TIM2>, channel: TimChannel, value: i16, max_period: u32) {
        // Calculate the duty cycle value based on the input value and maximum period
        let duty: u32 = (value.abs() as u32 * max_period) >> 15;
        // Set the duty cycle for the specified channel
        timer.set_duty(channel, duty);
    }
    
    // Function to read the encoder value over SPI
    fn read_encoder(spi: &mut Spi<SPI1>, cs_pin: &mut Pin) -> u16 {
        // Pull CS low to start SPI communication
        cs_pin.set_low();
    
        // Prepare the buffer with the command word 0x8020 and placeholders for response
        let mut buf = [0x80, 0x20, 0x00, 0x00];
        // Transfer the data over SPI and read the response
        spi.transfer(&mut buf).unwrap();
    
        // Pull CS high to end SPI communication
        cs_pin.set_high();
    
        // Process the received data to extract the encoder value
        let respond = ((buf[2] as u16) << 8) | buf[3] as u16;
        let respond = respond << 1; // Shift left to align the data
    
        // Return the encoder value
        respond
    }
    
    // Interrupt handler and callbacks
    // -------------------------------
    
    // Timer interrupt handler for TIM2
    #[task(binds = TIM2, local = [timer_pwd, underflow, tick_counter, motor_sel, phase_sel, encoder_pos, spi, cs_pin])]
    fn tim2_period_elapsed(cx: tim2_period_elapsed::Context) {
        // Clear the update interrupt flag
        cx.local.timer_pwd.clear_interrupt(TimerInterrupt::Update);
    
        // Increment the tick counter, wrapping around on overflow
        *cx.local.tick_counter = cx.local.tick_counter.wrapping_add(1);
    
        // Alternate between PWM and analog callbacks on underflow flag
        if *cx.local.underflow {
            // Call the PWM callback function
            pwm_callback(
                cx.local.tick_counter,
                cx.local.timer_pwd,
                cx.local.motor_sel,
                cx.local.phase_sel,
                cx.local.encoder_pos,
                cx.local.spi,
                cx.local.cs_pin,
            );
        } else {
            // Call the analog callback function (placeholder)
            analog_callback();
        }
    
        // Toggle the underflow flag
        *cx.local.underflow = !*cx.local.underflow;
    }
    
    // Function to handle PWM updates
    fn pwm_callback(
        tick_counter: &i16,
        timer_pwd: &mut Timer<TIM2>,
        motor_sel: &mut MotorSelector,
        phase_sel: &mut PhaseSelector,
        encoder_pos: &mut EncoderPosition,
        spi: &mut Spi<SPI1>,
        cs_pin: &mut Pin,
    ) {
        // Get the current counter value
        let counter = *tick_counter;
        // Define the speed multiplier
        let speed = 50;
        // Calculate PWM values based on the current angle
        let pwm_values = tick_motor(counter.wrapping_mul(speed), motor_sel, phase_sel);
        // Set the PWM duties on the timer
        set_pwm_duties(timer_pwd, &pwm_values);
    
        // Read the encoder value via SPI
        let encoder_value = read_encoder(spi, cs_pin);
        // Update the encoder position with the new value
        encoder_pos.tick(encoder_value);
        
        // Uncomment the line below to print the position (requires defmt support)
        // defmt::println!("Pos: {}", encoder_pos.position());
    }
    
    // Placeholder function for analog updates (e.g., ADC readings)
    fn analog_callback() {
        // TODO: Implement ADC reading
    }
    
} // End of RTIC app module

// Panic handler using defmt
#[defmt::panic_handler]
fn panic() -> ! {
    // Trigger an undefined instruction to cause a breakpoint
    cortex_m::asm::udf()
}
