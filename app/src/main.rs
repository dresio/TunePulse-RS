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
    clocks::Clocks,                           // For configuring the system clocks
    gpio::{Edge, Pin, PinMode, Port, Pull},   // GPIO handling
    pac,               // Peripheral Access Crate (PAC) for device-specific peripherals
    pac::{SPI1, TIM2}, // Specific peripherals used
    spi::{BaudRate, Spi, SpiConfig, SpiMode}, // SPI peripheral configuration
    timer::*,          // Timer peripherals
};

// Import custom modules from tunepulse_rs crate
use tunepulse_algo::{
    encoder_position::EncoderPosition, // Encoder position handling
    motor_driver::{
        pwm_control::{MotorType, PhasePattern},
        MotorDriver,
    }, // Motor control modules
};

use tunepulse_drivers::*;

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
        timer_pwm: pwm::TimPWM,       // Timer for PWM
        underflow: bool,              // Underflow flag
        tick_counter: i16,            // Counter for ticks
        motor: MotorDriver,           // Motor selector for PWM control
        encoder_pos: EncoderPosition, // Encoder position tracking
        spi: Spi<SPI1>,               // SPI peripheral
        cs_pin: Pin,                  // Chip Select pin for SPI
    }

    // Initialization function
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Define the frequency for the timer
        let freq = 18000;
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
        let mut timer_pwm = pwm::TimPWM::new(dp.TIM2, &clock_cfg, freq);
        timer_pwm.run();

        // Initialize SPI pins and SPI peripheral
        let cs_pin = init_spi_pins();
        let spi = init_spi(dp.SPI1);

        // Initialize motor and phase selectors
        let mut motor = MotorDriver::new(MotorType::STEPPER, PhasePattern::ABCD, freq);

        // Initialize encoder position tracking
        let mut encoder_pos = EncoderPosition::new(0, freq, 240);

        // Return the shared and local resources
        (
            Shared {},
            Local {
                timer_pwm,
                underflow: true,
                tick_counter: 0,
                motor,
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
        let mut dr_reset = pinout::driver::RESET.init();
        // Set the driver reset pin high (inactive)
        dr_reset.set_high();

        // Create a new output pin for driver enable on Port A, Pin 4
        let mut dr_en = pinout::driver::ENABLE.init();
        // Set the driver enable pin high (enabled)
        dr_en.set_high();

        // Configure PWM output pins for alternate function (Timer channels)
        pinout::driver::PWM_A1.init();
        pinout::driver::PWM_A2.init();
        pinout::driver::PWM_B1.init();
        pinout::driver::PWM_B2.init();
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
        pinout::encoder::SPI1_SCK.init();
        pinout::encoder::SPI1_MOSI.init();
        pinout::encoder::SPI1_MISO.init();

        // Configure CS pin on Port C, Pin 4 as output
        let mut cs_pin = pinout::encoder::SPI1_CS.init();
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
    #[task(binds = TIM2, local = [timer_pwm, underflow, tick_counter, motor, encoder_pos, spi, cs_pin])]
    fn tim2_period_elapsed(cx: tim2_period_elapsed::Context) {
        // Clear the update interrupt flag
        cx.local
            .timer_pwm
            .get_timer()
            .clear_interrupt(TimerInterrupt::Update);

        // Increment the tick counter, wrapping around on overflow
        *cx.local.tick_counter = cx.local.tick_counter.wrapping_add(1);

        // Alternate between PWM and analog callbacks on underflow flag
        if *cx.local.underflow {
            // Call the PWM callback
            let mut speed = 25;
            let mut duty = 0.2;

            // Get the current counter value
            let counter = *cx.local.tick_counter;
            // Define the speed multiplier

            let mut pwm: i16 = (i16::MAX as f32 * duty) as i16;

            // Read the encoder value via SPI
            let encoder_value = read_encoder(cx.local.spi, cx.local.cs_pin);
            // Update the encoder position with the new value
            cx.local.encoder_pos.tick(encoder_value);
            let pwm_values = cx.local.motor.tick(
                (counter.wrapping_mul(speed), pwm),
                cx.local.encoder_pos.position(),
            );
            // Set the PWM duties on the timer
            cx.local.timer_pwm.apply_pwm(pwm_values);
        } else {
            // Call the analog callback function (placeholder)
            // TODO: Implement ADC reading
        }

        // Toggle the underflow flag
        *cx.local.underflow = !*cx.local.underflow;
    }
} // End of RTIC app module

// Panic handler using defmt
#[defmt::panic_handler]
fn panic() -> ! {
    // Trigger an undefined instruction to cause a breakpoint
    cortex_m::asm::udf()
}
