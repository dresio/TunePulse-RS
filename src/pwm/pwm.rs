use hal::{
    self,
    gpio::{Pin, PinMode, Port},
};

pub struct pwm;

impl pwm {
    pub enum PwmPin {
        A1,
        B1,
        A2,
        B2,
    }

    impl PwmPin {
        fn to_pin(&self) -> Pin {
            match self {
                PwmPin::A1 => Pin::new(Port::A, 1, PinMode::Alt(1)),
                PwmPin::B1 => Pin::new(Port::B, 11, PinMode::Alt(1)),
                PwmPin::A2 => Pin::new(Port::A, 0, PinMode::Alt(1)),
                PwmPin::B2 => Pin::new(Port::B, 10, PinMode::Alt(1)),
            }
        }
    }
}

