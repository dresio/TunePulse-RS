use super::PinDef;
use super::{PinMode, Port};

pub const SPI1_SCK: PinDef = PinDef {
    port: Port::A,
    pin: 5,
    mode: PinMode::Alt(5),
};

pub const SPI1_MISO: PinDef = PinDef {
    port: Port::A,
    pin: 6,
    mode: PinMode::Alt(5),
};

pub const SPI1_MOSI: PinDef = PinDef {
    port: Port::A,
    pin: 7,
    mode: PinMode::Alt(5),
};

pub const SPI1_CS: PinDef = PinDef {
    port: Port::C,
    pin: 4,
    mode: PinMode::Output,
};