#![no_main]
#![no_std]

// use cortex_m as _;
use cortex_m_rt::entry;
use hal::pac;
use panic_halt as _;

use hal::{
    clocks::Clocks,
    gpio::{Edge, Pin, PinMode, Port, Pull},
};

use rtt_target::{rprint, rtt_init_print, ChannelMode::NoBlockSkip};

use tunepulse_rs::{encoder_position::EncoderPosition, spi::spi};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Starting encoder test");

    let FREQUENCY: u16 = 10000;
    let dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();
    clock_cfg.setup().unwrap();
    let sysclk_freq = clock_cfg.sysclk();

    rprintln!("System clock frequency: {} Hz", sysclk_freq);

    let mut cs_pin = spi::init_spi_pins();
    let mut spi = spi::init_spi(dp.SPI1);

    let mut encoder_pos = EncoderPosition::new(0, FREQUENCY, 240);

    loop {
        let encoder_value = EncoderPosition::read_encoder(&mut spi, &mut cs_pin);
        encoder_pos.tick(encoder_value);

        // Send the encoder value over SWO (Serial Wire Output)
        let encoder_position = encoder_pos.position();
        let mut output = [0u8; 32];

        // Get the encoder position and send it over RTT
        let encoder_position = encoder_pos.position();
        rprint!("encoder_pos:{};", encoder_position); // Send over RTT
    }
}

// #[panic_handler]
// fn panic() -> ! {
//     cortex_m::asm::udf()
// }
