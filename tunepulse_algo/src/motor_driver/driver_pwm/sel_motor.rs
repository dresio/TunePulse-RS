// Implements motor selection and control logic, handling various motor types and voltage calculations.

// Key Features:
// - Handles different motor types including DC, Stepper, and BLDC
// - Calculates coil voltages using mathematical transformations
// - Manages phase voltages with SVPWM algorithm
// - Provides methods to update motor control and change motor modes

// Detailed Operation:
// The MotorSelector struct manages motor control by selecting the appropriate motor type mode
// and calculating the required voltages. It uses mathematical functions to compute coil voltages
// and implements the SVPWM algorithm for three-phase motors. The tick method updates the control
// signals based on the current mode and input voltages, ensuring proper motor operation.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

use super::motor::{bldc, coil}; // Imports the inverse Clarke transform function from the parent module
use super::MotorType; // Imports the MotorType enum from the parent module

/// Disabled voltage constant
const DISBL: i16 = i16::MIN;

/// Struct to handle different types of motor controls
pub struct MotorSelector {
    /// Input voltages for the alpha and beta components
    duty_ab: (i16, i16),
    /// Current motor type mode
    mode: MotorType,
    /// Array to store voltages for four channels
    ch_abcd: [i16; 4],
}

impl MotorSelector {
    /// Creates a new MotorSelector with the specified motor type mode
    pub fn new(mode: MotorType) -> Self {
        MotorSelector {
            mode,            // Sets the motor type mode
            duty_ab: (0, 0), // Initializes alpha and beta voltages to zero
            ch_abcd: [0; 4], // Initializes channel voltages to zero
        }
    }

    /// Handles one-phase motor control by setting a single phase to brake voltage and others to zero
    #[inline(always)]
    fn tick0phase(&mut self) {
        // Set single phase to brake voltage, other - zeros
        self.ch_abcd = [0, 0, 0, 0]
    }

    /// Handles one-phase motor control by calculating coil voltages and setting unused phases to brake voltage
    #[inline(always)]
    fn tick1phase(&mut self) {
        (self.ch_abcd[0], self.ch_abcd[1]) = coil::duty::center(self.duty_ab.0); // Calculates and sets voltages for two channels
                                                                                         // Set unused phase to brake voltage (optional)
        self.ch_abcd[2] = DISBL; // Disables third channel
        self.ch_abcd[3] = DISBL; // Disables fourth channel
    }

    /// Handles two-phase motor control by calculating coil voltages for both phases
    #[inline(always)]
    fn tick2phase(&mut self) {
        // Calculates and sets voltages for first two channels
        (self.ch_abcd[0], self.ch_abcd[1]) = coil::duty::center(self.duty_ab.0);
        (self.ch_abcd[2], self.ch_abcd[3]) = coil::duty::center(self.duty_ab.1);
        // Calculates and sets voltages for last two channels
    }

    /// Controls a 3-phase 3-wire motor using the SVPWM algorithm and sets unused phase to brake voltage
    #[inline(always)]
    fn tick3phase(&mut self) {
        // Calculates and sets voltages for three channels
        (self.ch_abcd[0], self.ch_abcd[1], self.ch_abcd[2]) =
            bldc::duty::ab2abc(self.duty_ab.0, self.duty_ab.1);

        // Set unused phase to brake voltage (optional)
        self.ch_abcd[3] = DISBL; // Disables fourth channel
    }

    /// Updates motor control based on the current mode and input voltages
    pub fn tick(&mut self, voltg_ab: (i16, i16)) -> [i16; 4] {
        self.duty_ab = voltg_ab; // Updates alpha and beta voltages
        match self.mode {
            MotorType::UNDEFINED => self.tick0phase(), // Handles undefined motor type
            MotorType::DC => self.tick1phase(),        // Handles DC motor type
            MotorType::STEP => self.tick2phase(),      // Handles Stepper motor type
            MotorType::BLDC => self.tick3phase(),      // Handles BLDC motor type
        }
        self.ch_abcd // Returns the updated channel voltages
    }

    /// Changes the motor type mode to the specified mode
    #[inline(always)]
    pub fn change_mode(&mut self, mode: MotorType) {
        self.mode = mode // Updates the motor type mode
    }
}
