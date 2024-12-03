use hal::{
    self,
    gpio::Pin,
    pac::SPI1,
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
};

use super::pinout;

pub struct Spi1DMA {
    pub spi: Spi<SPI1>,
    cs_pin: Pin,
    angle: u16,
}

impl Spi1DMA {
    pub fn new(spi_reg: SPI1) -> Self {
        let spi_cfg = SpiConfig {
            mode: SpiMode::mode1(),
            ..Default::default()
        };

        pinout::encoder::SPI1_SCK.init();
        pinout::encoder::SPI1_MISO.init();
        pinout::encoder::SPI1_MOSI.init();
        let mut cs_pin = pinout::encoder::SPI1_CS.init();
        cs_pin.set_high();

        let spi1 = Spi::new(spi_reg, spi_cfg, BaudRate::Div32);

        Spi1DMA {
            spi: spi1,
            cs_pin,
            angle: 0,
        }
    }

    pub fn get_cs(&mut self) -> &mut Pin {
        &mut self.cs_pin
    }

    pub fn get_spi(&mut self) -> &mut Spi<SPI1> {
        &mut self.spi
    }

    pub fn get_angle(&mut self) -> u16 {
        self.angle
    }

    pub fn start(&mut self) {
        self.cs_pin.set_low();
    }

    pub fn end(&mut self, buf: [u8; 4]) -> u16 {
        self.cs_pin.set_high();
        let respond = ((buf[2] as u16) << 8) | buf[3] as u16;
        self.angle = respond << 1;
        self.angle
    }
}
