//! This minimal example causes an LED to blink on a button press R -> G -> B.

#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry; // The runtime

use hal::{
    self,
    clocks::Clocks,
    gpio::{Edge, Pin, PinMode, Port, Pull},
    pac,
};

use defmt_rtt as _; // global logger
use panic_probe as _;

use tunepulse_drivers::pinout;

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    // Write the clock configuration to the MCU.
    clock_cfg.setup().unwrap();

    // Setup a delay, based on the Cortex-M SysTick.
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    // Initialize the LEDs.
    let mut led_red = pinout::led::RED.init();
    let mut led_green = pinout::led::GRN.init();
    let mut led_blue = pinout::led::BLU.init();

    // Initialize the button.
    let mut button = Pin::new(Port::A, 15, PinMode::Input);
    button.pull(Pull::Up);
    button.enable_interrupt(Edge::Rising);

    // Turn off all LEDs initially.
    led_red.set_high();
    led_green.set_high();
    led_blue.set_high();

    // Print initial message
    defmt::println!("Blink test: press button to switch color");

    // State variables.
    let mut current_color = 0; // 0 -> Red, 1 -> Green, 2 -> Blue
    let mut press_count = 0;  // Total button presses

    loop {
        // Blink the current LED
        match current_color {
            0 => {
                led_red.set_low();
                led_green.set_high();
                led_blue.set_high();
            }
            1 => {
                led_red.set_high();
                led_green.set_low();
                led_blue.set_high();
            }
            2 => {
                led_red.set_high();
                led_green.set_high();
                led_blue.set_low();
            }
            _ => {}
        }

        // Check button state.
        if button.is_low() {
            // Increment press count and display it.
            press_count += 1;
            defmt::println!("Button pressed {} times", press_count);

            // Cycle to the next color.
            current_color = (current_color + 1) % 3;

            // Debounce delay to avoid multiple triggers.
            delay.delay_ms(200);
        }
    }
}

// same panicking *behavior* as panic-probe but doesn't print a panic message
// this prevents the panic message being printed *twice* when defmt::panic is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}