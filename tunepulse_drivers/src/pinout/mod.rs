use hal::gpio::{Pin, PinMode, Port};

pub mod led;
pub mod encoder;
pub mod driver;

/// Represents the definition of a GPIO pin.
pub struct PinDef {
    /// The port to which the pin belongs (e.g., Port::A, Port::B).
    port: Port,
    /// The pin number within the port.
    pin: u8,
    /// The mode of the pin (e.g., Output, Input, Alternate function).
    mode: PinMode,
}

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
    pub fn init(&self) -> Pin {
        Pin::new(self.port, self.pin, self.mode)
    }
}
