// Implements the PhaseSelector module, handling the re-mapping of PWM channels
// based on different phase patterns.

// Key Features:
// - Initializes the PhaseSelector with a specific phase pattern.
// - Updates PWM output voltages according to the current phase mode.
// - Allows dynamic changing of the phase mode during runtime.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

use super::PhasePattern; // Imports the PhasePattern enum from the parent module

/// Struct to handle the re-mapping of PWM channels
pub struct PhaseSelector {
    /// Current operating mode represented as an index
    mode: usize,
    /// Indices for PWM channel re-mapping
    idxs: [usize; 4],
}

impl PhaseSelector {
    /// Creates a new PhaseSelector with the specified phase pattern
    pub const fn new(mode: PhasePattern) -> PhaseSelector {
        let mode = mode as usize; // Converts the PhasePattern to a usize value
        let idxs = [
            (mode >> 0) & 0b11, // Extracts the first two bits for the first channel
            (mode >> 2) & 0b11, // Extracts the next two bits for the second channel
            (mode >> 4) & 0b11, // Extracts the following two bits for the third channel
            (mode >> 6) & 0b11, // Extracts the last two bits for the fourth channel
        ];
        PhaseSelector { mode, idxs } // Initializes the PhaseSelector with mode and indices
    }

    /// Updates the PWM voltages based on the current phase mode
    #[inline(always)]
    pub fn tick(&self, voltages: [i16; 4]) -> [i16; 4] {
        [
            voltages[self.idxs[0]], // Sets voltage for the first channel
            voltages[self.idxs[1]], // Sets voltage for the second channel
            voltages[self.idxs[2]], // Sets voltage for the third channel
            voltages[self.idxs[3]], // Sets voltage for the fourth channel
        ]
    }

    /// Changes the current phase mode and updates the channel indices
    pub fn change_mode(&mut self, mode: u8) {
        self.mode = mode as usize; // Updates the mode with the new value
        self.idxs = [
            (self.mode >> 0) & 0b11, // Updates the first channel index based on the new mode
            (self.mode >> 2) & 0b11, // Updates the second channel index based on the new mode
            (self.mode >> 4) & 0b11, // Updates the third channel index based on the new mode
            (self.mode >> 6) & 0b11, // Updates the fourth channel index based on the new mode
        ];
    }
}
