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

use super::inverse_clarke_transform; // Imports the inverse Clarke transform function from the parent module
use super::MotorType; // Imports the MotorType enum from the parent module

/// Struct to handle different types of motor controls
pub struct MotorSelector {
    /// Input voltages for the alpha and beta components
    voltg_ab: (i16, i16),
    /// Input supply voltage
    voltg_sup: i16,
    /// Current motor type mode
    mode: MotorType,
    /// Array to store voltages for four channels
    ch_abcd: [i16; 4],
}

impl MotorSelector {
    /// Disabled voltage constant
    const DISBL: i16 = i16::MIN;

    /// Creates a new MotorSelector with the specified motor type mode
    pub fn new(mode: MotorType) -> Self {
        MotorSelector {
            mode,             // Sets the motor type mode
            voltg_ab: (0, 0), // Initializes alpha and beta voltages to zero
            voltg_sup: 0,     // Initializes supply voltage to zero
            ch_abcd: [0; 4],  // Initializes channel voltages to zero
        }
    }

    /// Calculates coil voltages based on the reference voltage
    fn math_coil(voltg_ref: i16) -> (i16, i16) {
        if voltg_ref == Self::DISBL || voltg_ref == 0 {
            return (voltg_ref, voltg_ref); // Returns disabled or zero voltage if reference is disabled or zero
        }

        // CENTER ALLIGNED PWM IS REQUIRED
        const MIDPOINT: i16 = i16::MAX / 2; // Defines the midpoint for PWM alignment

        let duty: i16 = voltg_ref / 2; // Calculates duty cycle based on reference voltage
        return (MIDPOINT + duty, MIDPOINT - duty); // Returns adjusted voltages based on duty cycle
    }

    /// Calculates SVPWM voltages based on sine and cosine references and available voltage
    fn math_svpwm(voltg_sin: i16, voltg_cos: i16, voltg_available: i16) -> (i16, i16, i16) {
        let voltg_available: i32 = voltg_available as i32; // Converts available voltage to i32 for calculations

        // Inverse Clarke transform
        let (mut voltg_a, mut voltg_b, mut voltg_c) =
            inverse_clarke_transform(voltg_sin, voltg_cos); // Transforms sine and cosine voltages to three-phase voltages

        // Find the minimum and maximum phase voltages
        let voltg_min: i32 = voltg_a.min(voltg_b).min(voltg_c); // Determines the minimum voltage among phases
        let voltg_max: i32 = voltg_a.max(voltg_b).max(voltg_c); // Determines the maximum voltage among phases

        let voltg_offset: i32; // Initializes voltage offset

        let voltg_full_scale: i32 = voltg_max - voltg_min; // Calculates the full scale voltage range

        // Automatic constraining and bottom clamping if available voltage isn't enough
        if voltg_full_scale > voltg_available {
            // Calculate scaling factor (fixed point based on i32, size of scale: 15bit)
            let voltg_scale = (voltg_available << 15) / voltg_full_scale; // Determines scaling factor to fit available voltage

            // Apply scale to all channels
            voltg_a = (voltg_a * voltg_scale) >> 15; // Scales voltage A
            voltg_b = (voltg_b * voltg_scale) >> 15; // Scales voltage B
            voltg_c = (voltg_c * voltg_scale) >> 15; // Scales voltage C

            // Apply scale to voltg_min to allow correct clamping
            let voltg_min: i32 = ((voltg_min) * voltg_scale) >> 15; // Scales minimum voltage
            voltg_offset = -voltg_min; // Sets voltage offset based on scaled minimum voltage
        } else {
            // Calculate reference voltage to shift all phase voltages
            voltg_offset = (voltg_available - voltg_max - voltg_min) >> 1; // Determines voltage offset for shifting
        }

        // If zero voltage is required - activate maximum brake
        if voltg_full_scale != 0 {
            // Shift all phase voltages by reference voltage
            voltg_a += voltg_offset; // Applies offset to voltage A
            voltg_b += voltg_offset; // Applies offset to voltage B
            voltg_c += voltg_offset; // Applies offset to voltage C
        }

        return (voltg_a as i16, voltg_b as i16, voltg_c as i16); // Returns the final adjusted voltages
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
        (self.ch_abcd[0], self.ch_abcd[1]) = Self::math_coil(self.voltg_ab.0); // Calculates and sets voltages for two channels
                                                                               // Set unused phase to brake voltage (optional)
        self.ch_abcd[2] = Self::DISBL; // Disables third channel
        self.ch_abcd[3] = Self::DISBL; // Disables fourth channel
    }

    /// Handles two-phase motor control by calculating coil voltages for both phases
    #[inline(always)]
    fn tick2phase(&mut self) {
        (self.ch_abcd[0], self.ch_abcd[1]) = Self::math_coil(self.voltg_ab.0); // Calculates and sets voltages for first two channels
        (self.ch_abcd[2], self.ch_abcd[3]) = Self::math_coil(self.voltg_ab.1); // Calculates and sets voltages for last two channels
    }

    /// Controls a 3-phase 3-wire motor using the SVPWM algorithm and sets unused phase to brake voltage
    #[inline(always)]
    fn tick3phase(&mut self) {
        (self.ch_abcd[0], self.ch_abcd[1], self.ch_abcd[2]) =
            Self::math_svpwm(self.voltg_ab.0, self.voltg_ab.1, self.voltg_sup); // Calculates and sets voltages for three channels
                                                                                // Set unused phase to brake voltage (optional)
        self.ch_abcd[3] = Self::DISBL; // Disables fourth channel
    }

    /// Updates motor control based on the current mode and input voltages
    pub fn tick(&mut self, voltg_ab: (i16, i16), voltg_sup: i16) -> [i16; 4] {
        self.voltg_ab = voltg_ab; // Updates alpha and beta voltages
        self.voltg_sup = voltg_sup; // Updates supply voltage
        match self.mode {
            MotorType::UNDEFINED => self.tick0phase(), // Handles undefined motor type
            MotorType::DC => self.tick1phase(),        // Handles DC motor type
            MotorType::STEP => self.tick2phase(),   // Handles Stepper motor type
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
