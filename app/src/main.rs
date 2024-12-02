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
    pac,               // Peripheral Access Crate (PAC) for device-specific peripherals
    timer::*,          // Timer peripherals
};

use hal::dma;
use hal::dma::DmaInterrupt;
use hal::dma::{Dma, DmaChannel, DmaInput, DmaPeriph};

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

static mut SPI_READ_BUF: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
static mut SPI_WRITE_BUF: [u8; 4] = [0x80, 0x20, 0x00, 0x00];

#[rtic::app(device = pac, peripherals = true)]
mod app {
    // Bring all previous imports into scope
    use super::*;

    // Shared resources between tasks (empty in this case)
    #[shared]
    struct Shared {
        spi1: encoder_spi::Spi1DMA,
    }

    // Local resources for tasks
    #[local]
    struct Local {
        timer_pwm: pwm::TimPWM,       // Timer for PWM
        underflow: bool,              // Underflow flag
        tick_counter: i16,            // Counter for ticks
        motor: MotorDriver,           // Motor selector for PWM control
        encoder_pos: EncoderPosition, // Encoder position tracking
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

        // Initialize the PWM timer with the given frequency
        let mut timer_pwm = pwm::TimPWM::new(dp.TIM2, &clock_cfg, freq);
        timer_pwm.begin();

        // Initialize motor and phase selectors
        let mut motor = MotorDriver::new(MotorType::STEPPER, PhasePattern::ABCD, freq);

        // Initialize encoder position tracking
        let mut encoder_pos = EncoderPosition::new(0, freq, 240);

        let mut spi1 = encoder_spi::Spi1DMA::new(dp.SPI1);
        let _dma = Dma::new(dp.DMA1);
        dma::enable_mux1();
        dma::mux(DmaPeriph::Dma1, DmaChannel::C1, DmaInput::Spi1Tx);
        dma::mux(DmaPeriph::Dma1, DmaChannel::C2, DmaInput::Spi1Rx);

        // Return the shared and local resources
        (
            Shared { spi1 },
            Local {
                timer_pwm,
                underflow: true,
                tick_counter: 0,
                motor,
                encoder_pos,
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
    }

    // Interrupt handler and callbacks
    #[task(binds = TIM2,shared = [spi1], local = [timer_pwm, underflow, tick_counter, motor, encoder_pos])]
    fn tim2_period_elapsed(mut cx: tim2_period_elapsed::Context) {
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

            // Define the speed multiplier
            let mut speed = 25;
            let mut duty = 0.2;

            // Get the current counter value
            let counter = *cx.local.tick_counter;
            
            let mut res: u16 = 0;
            cx.shared.spi1.lock(|spi1| {
                res = spi1.get_angle();
            });
            cx.local.encoder_pos.tick(res);

            let mut pwm: i16 = (i16::MAX as f32 * duty) as i16;
            let pos = cx.local.encoder_pos.position();
            let pwm_values = cx.local.motor.tick((counter.wrapping_mul(speed), pwm), pos);

            // Set the PWM duties on the timer
            cx.local.timer_pwm.apply_pwm(pwm_values);

            // defmt::println!("Data read: {:?}", res);
            // defmt::println!("Encoder position: {:?}", pos);
        } else {
            // Call the analog callback function (placeholder)
            // TODO: Implement ADC reading
            encoder_begin_read::spawn().expect("can not spawn on_spi_transfer");
        }

        // Toggle the underflow flag
        *cx.local.underflow = !*cx.local.underflow;
    }

    #[task(priority = 0, shared = [spi1])]
    async fn encoder_begin_read(mut cx: encoder_begin_read::Context) {
        cx.shared.spi1.lock(|spi1| unsafe {
            spi1.start();
            spi1.get_spi().transfer_dma(
                &SPI_WRITE_BUF,
                &mut SPI_READ_BUF,
                DmaChannel::C1,
                DmaChannel::C2,
                Default::default(),
                Default::default(),
                DmaPeriph::Dma1,
            );
        });
    }

    #[task(binds = DMA1_CH2, shared = [spi1], priority = 1)]
    fn encoder_end_read(mut cx: encoder_end_read::Context) {
        dma::clear_interrupt(
            DmaPeriph::Dma1,
            DmaChannel::C2,
            DmaInterrupt::TransferComplete,
        );
        cx.shared.spi1.lock(|spi1| {
            spi1.get_spi()
                .stop_dma(DmaChannel::C1, Some(DmaChannel::C2), DmaPeriph::Dma1);
            spi1.get_spi()
                .cleanup_dma(DmaPeriph::Dma1, DmaChannel::C1, Some(DmaChannel::C2));
            spi1.end(unsafe { SPI_READ_BUF });
        });
    }
} // End of RTIC app module

// Panic handler using defmt
#[defmt::panic_handler]
fn panic() -> ! {
    // Trigger an undefined instruction to cause a breakpoint
    cortex_m::asm::udf()
}
