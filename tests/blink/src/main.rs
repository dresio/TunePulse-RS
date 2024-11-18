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

use defmt_rtt as _;
// global logger
use panic_probe as _;

// This marks the entrypoint of our application.

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let _dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();

    // Write the clock configuration to the MCU. If you wish, you can modify `clock_cfg` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/latest/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock_cfg.setup().unwrap();

    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    let mut led_red = Pin::new(Port::B, 13, PinMode::Output);
    let mut led_green = Pin::new(Port::B, 14, PinMode::Output);
    let mut led_blue = Pin::new(Port::B, 15, PinMode::Output);

    let mut button = Pin::new(Port::A, 15, PinMode::Input);
    button.pull(Pull::Up);
    button.enable_interrupt(Edge::Rising);

    led_green.set_high();
    led_blue.set_high();
    led_red.set_high();

    loop {
        led_green.set_low();
        delay.delay_ms(1_000);
        led_green.set_high();

        led_blue.set_low();
        delay.delay_ms(1_000);
        led_blue.set_high();

        led_red.set_low();
        delay.delay_ms(1_000);
        led_red.set_high();

        if button.is_low() {
            defmt::println!("low");
        } else {
            defmt::println!("hight");
        }

        defmt::println!("Hello, world!");
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
