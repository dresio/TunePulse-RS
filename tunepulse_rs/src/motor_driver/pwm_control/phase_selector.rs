/// Struct to handle the re-mapping of PWM channels
pub struct PhaseSelector {
    /// Reference to the current mode
    pub mode: u8,
}

impl PhaseSelector {
    /// Constructor for SelectorInterconnectPwm.
    ///
    /// # Arguments
    /// * `mode` - Reference to the current mode.
    /// * `ch_abcd` - Reference to the input array containing PWM channels.
    pub const fn new(mode: u8) -> PhaseSelector {
        PhaseSelector { mode }
    }

    /// Updates the output pattern based on the current mode.
    pub fn tick(&mut self, voltages: [i16; 4]) -> [i16; 4] {
        let mut output: [i16; 4] = [i16::MIN; 4];
        for i in 0..4 {
            let indx = (self.mode as usize >> (i << 1)) & 0b11;
            output[i] = voltages[indx];
        }
        output
    }

    #[inline(always)]
    pub fn change_mode(&mut self, mode: u8) {
        self.mode = mode;
    }
}
