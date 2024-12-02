use super::PinDef;
use super::{PinMode, Port};

pub const RED: PinDef = PinDef {
    port: Port::B,
    pin: 15,
    mode: PinMode::Output,
};

pub const GRN: PinDef = PinDef {
    port: Port::B,
    pin: 14,
    mode: PinMode::Output,
};

pub const BLU: PinDef = PinDef {
    port: Port::B,
    pin: 13,
    mode: PinMode::Output,
};

