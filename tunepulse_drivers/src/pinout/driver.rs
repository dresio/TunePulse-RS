//! This module defines the pin configurations for the hardware abstraction layer (HAL) GPIO pins used in the project.
use super::PinDef;
use super::{PinMode, Port};

/// Reset pin for the motor driver output
pub const RESET: PinDef = PinDef {
    port: Port::B,
    pin: 2,
    mode: PinMode::Output,
};

/// Enable pin for the motor driver output
pub const ENABLE: PinDef = PinDef {
    port: Port::A,
    pin: 4,
    mode: PinMode::Output,
};

/// PWM pins for the motor driver output labled A1
pub const PWM_A1: PinDef = PinDef {
    port: Port::A,
    pin: 1,
    mode: PinMode::Alt(1),
};

/// PWM pins for the motor driver output labled B1

pub const PWM_B1: PinDef = PinDef {
    port: Port::B,
    pin: 10,
    mode: PinMode::Alt(1),
};

/// PWM pins for the motor driver output labled A2
pub const PWM_A2: PinDef = PinDef {
    port: Port::A,
    pin: 0,
    mode: PinMode::Alt(1),
};

/// PWM pins for the motor driver output labled B2
pub const PWM_B2: PinDef = PinDef {
    port: Port::B,
    pin: 11,
    mode: PinMode::Alt(1),
};

