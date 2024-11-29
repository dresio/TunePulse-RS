/// Struct to handle the re-mapping of PWM channels
pub struct PhaseSelector {
    /// Reference to the array containing ABCD channels
    pub ch_abcd: [i16; 4],
    /// Reference to the current mode
    pub mode: u8,
    /// Array to store the current output pattern
    output: [i16; 4],
}

impl PhaseSelector {
    /// Constructor for SelectorInterconnectPwm.
    ///
    /// # Arguments
    /// * `mode` - Reference to the current mode.
    /// * `ch_abcd` - Reference to the input array containing PWM channels.
    pub const fn new(mode: u8) -> PhaseSelector {
        PhaseSelector {
            ch_abcd: [i16::MIN; 4],
            mode,
            output: [i16::MIN; 4],
        }
    }

    /// Updates the output pattern based on the current mode.
    pub fn tick(&mut self) {
        for i in 0..4 {
            let indx = (self.mode as usize >> (i << 1)) & 0b11;
            self.output[i] = self.ch_abcd[indx];
        }
    }

    /// Returns the current PWM channels.
    pub fn pwm_channels(&self) -> [i16; 4] {
        self.output
    }
}

