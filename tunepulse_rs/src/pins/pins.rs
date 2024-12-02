//! This module defines the pin configurations for the hardware abstraction layer (HAL) GPIO pins used in the project.

use hal::gpio::{Pin, PinMode, Port};

/// Represents the definition of a GPIO pin.
pub struct PinDef {
    /// The port to which the pin belongs (e.g., Port::A, Port::B).
    port: Port,
    /// The pin number within the port.
    pin: u8,
    /// The mode of the pin (e.g., Output, Input, Alternate function).
    mode: PinMode,
}

/// Reset pin for the motor driver output
pub const RESET_PIN: PinDef = PinDef {
    port: Port::B,
    pin: 2,
    mode: PinMode::Output,
};

/// Enable pin for the motor driver output
pub const ENABLE_PIN: PinDef = PinDef {
    port: Port::A,
    pin: 4,
    mode: PinMode::Output,
};

/// PWM pins for the motor driver output labled A1
pub const PWM_PIN_A1: PinDef = PinDef {
    port: Port::A,
    pin: 1,
    mode: PinMode::Alt(1),
};

/// PWM pins for the motor driver output labled B1

pub const PWM_PIN_B1: PinDef = PinDef {
    port: Port::B,
    pin: 10,
    mode: PinMode::Alt(1),
};

/// PWM pins for the motor driver output labled A2
pub const PWM_PIN_A2: PinDef = PinDef {
    port: Port::A,
    pin: 0,
    mode: PinMode::Alt(1),
};

/// PWM pins for the motor driver output labled B2
pub const PWM_PIN_B2: PinDef = PinDef {
    port: Port::B,
    pin: 11,
    mode: PinMode::Alt(1),
};

impl PinDef {
    pub fn new(port: Port, pin: u8, mode: PinMode) -> PinDef {
        PinDef {
            port: port,
            pin: pin,
            mode: mode,
        }
    }

    /// Converts the PinDef struct to a Pin struct. Useful for predefined pin configurations.
    /// # Example
    /// ```
    /// let mut dr_reset = RESET_PIN.to_pin();
    /// dr_reset.set_high();
    /// ```
    pub fn to_pin(&self) -> Pin {
        Pin::new(self.port, self.pin, self.mode)
    }
}
