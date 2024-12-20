// Implements the MotorPulse module, responsible for converting angles to motor pulses,
// managing motor direction, microstepping, and error correction for precise stepper motor control.

// Key Features:
// - Converts desired angles into motor pulse steps.
// - Manages motor rotation direction based on error accumulation.
// - Supports configurable microstepping for enhanced motor precision.
// - Implements error correction to maintain accurate motor positioning.

// Detailed Operation:
// The MotorPulse struct calculates the number of steps required to rotate a stepper motor
// by a specified angle. It uses microstepping to increase precision and accumulates error
// to adjust steps accordingly. The direction of rotation is determined based on the sign
// of the accumulated error. The ustep_calc method computes the microstepping value, ensuring
// stability by capping the microstepping power. The tick method updates the motor parameters
// based on the target angle, adjusting steps and direction while correcting for any accumulated errors.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

/// A struct representing a converter from angle to motor pulses
/// This struct is used to determine the number of pulses needed to rotate a stepper motor
/// by a specified angle. It keeps track of motor direction, microstepping, and error correction.
pub struct Angle2Pulse {
    /// Number of steps needed by the motor driver to reach the target angle
    pub steps: i16,
    /// Motor rotation direction (true for one direction, false for the other)
    pub direction: bool,

    // UTILS
    /// Output microstepping value
    ustep: i32,
    /// The previous angle, used to calculate the delta for each tick
    prev_angle: i16,
    /// Accumulated error, used for more accurate step calculation
    error: i32,
}

impl Angle2Pulse {
    /// Constructs a new `MotorPulse` instance
    pub fn new(usteps_pow: u16) -> Self {
        Angle2Pulse {
            ustep: Self::ustep_calc(usteps_pow), // Calculate microstepping value based on power
            steps: 0,                            // Initialize steps to zero
            direction: false,                    // Set initial direction to false
            prev_angle: 0,                       // Initialize previous angle to zero
            error: 0,                            // Initialize error to zero
        }
    }

    /// Calculates the microstepping value based on the provided power
    fn ustep_calc(usteps_pow: u16) -> i32 {
        // Limit microstepping to a power of 9 or below to ensure stability and prevent overflow
        let usteps = if usteps_pow > 9 { 9 } else { usteps_pow };
        // Calculate the microstepping value by shifting bits to adjust precision
        (u16::MAX >> (2 + usteps)) as i32
    }

    /// Updates the motor control parameters based on the desired angle change
    pub fn tick(&mut self, angle: i16) -> (i16, i16) {
        // Calculate the accumulated error using the difference between the current and previous angle
        self.error += angle.wrapping_sub(self.prev_angle) as i32;

        // Update the previous angle to the current angle
        self.prev_angle = angle;

        // Determine the motor direction based on the sign of the error
        let direction = self.error < 0;

        // Calculate the number of microsteps required to reach the target, with rounding
        let step_shift = (self.error.abs() + (self.ustep >> 1)) / self.ustep;

        // Calculate the angle shift in microsteps to determine position change
        let angle_shift = step_shift * self.ustep;

        // Adjust the integral error to avoid accumulation of rounding errors
        self.error -= angle_shift - (angle_shift << 1) * direction as i32;

        // Update the number of steps needed to reach the target angle
        self.steps = step_shift as i16;

        // Avoid toggling the direction pin unnecessarily if no steps are required
        if step_shift > 0 {
            self.direction = direction;
        }

        // Return the current motor direction and the number of steps needed
        return (self.direction as i16, self.steps);
    }

    /// Sets the microstepping division based on the provided power
    pub fn set_ustep_div(&mut self, ustep_pow: u16) {
        self.ustep = Self::ustep_calc(ustep_pow) // Recalculate microstepping value
    }
}
