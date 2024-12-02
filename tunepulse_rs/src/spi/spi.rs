#![no_main]
#![no_std]

use hal::{
    self,
    gpio::{Pin, PinMode, Port},
    pac::SPI1,
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
};

/// Initialize the clocks for the microcontroller and return the chip select pin
pub fn init_spi_pins() -> Pin {
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
pub fn init_spi(spi1: SPI1) -> Spi<SPI1> {
    // Create SPI configuration with mode 1
    let spi_cfg = SpiConfig {
        mode: SpiMode::mode1(),
        ..Default::default()
    };

    // Initialize SPI1 with the configuration and set BaudRate
    // Adjust BaudRate as needed; Div32 for lower speed if APB clock is high
    Spi::new(spi1, spi_cfg, BaudRate::Div32)
}
