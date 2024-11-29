/// A struct representing a converter from angle to motor pulses.
/// This struct is used to determine the number of pulses needed to rotate a stepper motor
/// by a specified angle. It keeps track of motor direction, microstepping, and error correction.
pub struct AngleToPulse {
    /// Number of steps needed by the motor driver to reach the target angle.
    pub steps: u16,
    /// Motor rotation direction (true for one direction, false for the other).
    pub direction: bool,

    // UTILS
    /// Output microstepping value.
    ustep: i32,
    /// The previous angle, used to calculate the delta for each tick.
    prev_angle: i16,
    /// Accumulated error, used for more accurate step calculation.
    error: i32,
}

impl AngleToPulse {
    /// Constructs a new `AngleToPulse` instance.
    ///
    /// # Arguments
    /// * `usteps_pow` - The power of microstepping (e.g., 0 for full-step, 1 for half-step, etc.).
    ///                   Values greater than 9 are capped to 9 to avoid overflow issues.
    ///
    /// # Returns
    /// A new `AngleToPulse` instance initialized with default values.
    pub fn new(usteps_pow: u16) -> Self {
        AngleToPulse {
            ustep: Self::ustep_calc(usteps_pow),
            steps: 0,
            direction: false,
            prev_angle: 0,
            error: 0,
        }
    }

    /// Calculates the microstepping value based on the provided power.
    ///
    /// # Arguments
    /// * `usteps_pow` - The power of microstepping, capped to 9 if greater than 9.
    ///
    /// # Returns
    /// The calculated microstepping value used for controlling motor precision.
    fn ustep_calc(usteps_pow: u16) -> i32 {
        // Limit microstepping to a power of 9 or below to ensure stability and prevent overflow.
        let usteps = if usteps_pow > 9 { 9 } else { usteps_pow };
        (u16::MAX >> (2 + usteps)) as i32
    }

    /// Updates the motor control parameters based on the desired angle change.
    ///
    /// # Arguments
    /// * `angle` - The target angle in degrees (or equivalent units) for the motor to reach.
    ///
    /// # Returns
    /// A tuple consisting of:
    /// - A boolean representing the motor direction (true for negative direction, false for positive).
    /// - The number of steps required to reach the target angle.
    pub fn tick(&mut self, angle: i16) -> (bool, u16) {
        // Calculate the accumulated error using the difference between the current and previous angle.
        self.error += angle.wrapping_sub(self.prev_angle) as i32;

        // Update the previous angle to the current angle.
        self.prev_angle = angle;

        // Determine the motor direction based on the sign of the error.
        let direction = self.error < 0;

        // Calculate the number of microsteps required to reach the target, with rounding.
        let step_shift = (self.error.abs() + (self.ustep >> 1)) / self.ustep;

        // Calculate the angle shift in microsteps to determine position change.
        let angle_shift = step_shift * self.ustep;

        // Adjust the integral error to avoid accumulation of rounding errors.
        self.error -= angle_shift - (angle_shift << 1) * direction as i32;

        // Update the number of steps needed to reach the target angle.
        self.steps = step_shift as u16;

        // Avoid toggling the direction pin unnecessarily if no steps are required.
        if step_shift > 0 {
            self.direction = direction;
        }

        // Return the current motor direction and the number of steps needed.
        return (self.direction, self.steps);
    }
}
