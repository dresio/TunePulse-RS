use core::default;

use super::PhasePattern;

/// Struct to handle the re-mapping of PWM channels
pub struct PhaseSelector {
    /// Reference to the current mode
    mode: usize,
    idxs: [usize; 4],
}

impl PhaseSelector {
    /// Constructor for SelectorInterconnectPwm.
    ///
    /// # Arguments
    /// * `mode` - Reference to the current mode.
    /// * `ch_abcd` - Reference to the input array containing PWM channels.
    pub const fn new(mode: PhasePattern) -> PhaseSelector {
        let mode = mode as usize;
        let idxs = [
            (mode >> 0) & 0b11,
            (mode >> 2) & 0b11,
            (mode >> 4) & 0b11,
            (mode >> 6) & 0b11,
        ];
        PhaseSelector { mode, idxs }
    }

    /// Updates the output pattern based on the current mode.
    #[inline(always)]
    pub fn tick(&self, voltages: [i16; 4]) -> [i16; 4] {
        [
            voltages[self.idxs[0]],
            voltages[self.idxs[1]],
            voltages[self.idxs[2]],
            voltages[self.idxs[3]],
        ]
    }

    pub fn change_mode(&mut self, mode: u8) {
        self.mode = mode as usize;
        self.idxs = [
            (self.mode >> 0) & 0b11,
            (self.mode >> 2) & 0b11,
            (self.mode >> 4) & 0b11,
            (self.mode >> 6) & 0b11,
        ];
    }
}
