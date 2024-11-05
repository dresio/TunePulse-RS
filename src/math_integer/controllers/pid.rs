/// A Proportional-Integral-Derivative (PID) controller implementation
/// to calculate corrective action for controlling dynamic systems.
///
/// **Note**
/// - Based on integer implementation and works with i16 range
/// - Works with constant dt only
/// - Has integral anti-windup
pub struct PID {
    /// Proportional gain coefficient: -10000% to 10000%.
    /// Controls the reaction to the current error magnitude.
    kp: i32,

    /// Integral gain coefficient: -10000% to 10000%.
    /// Controls the reaction based on the accumulation of past errors.
    ki: i32,

    /// Derivative gain coefficient: -10000% to 10000%.
    /// Controls the reaction to the rate of error change.
    kd: i32,

    /// Feed-forward gain coefficient: -10000% to 10000%.
    /// Adds an anticipated value to the output to help the system respond faster.
    kff: i32,

    /// Accumulator for the integral term
    integral: i32,
    /// Stores the previous error value for derivative and integral calculation
    previous_error: i32,
    /// The PID controller output
    output: i16,
}

impl PID {
    /// Constructor for the PID controller
    ///
    /// # Arguments
    /// * `kp` - Proportional gain coefficient
    /// * `ki` - Integral gain coefficient
    /// * `kd` - Derivative gain coefficient
    /// * `kff` - Feed-forward gain coefficient
    ///
    /// # Returns
    /// A new instance of the PID controller with the given gain coefficients.
    pub fn new(kp: i32, ki: i32, kd: i32, kff: i32) -> Self {
        // Adjusts and fits each gain coefficient within a valid range
        let kp: i32 = Self::fit_coef(kp);
        let ki: i32 = Self::fit_coef(ki);
        let kd: i32 = Self::fit_coef(kd);
        let kff: i32 = Self::fit_coef(kff);

        // Initialize the PID structure
        Self {
            kp,
            ki,
            kd,
            kff,
            integral: 0,       // Initialize the integral accumulator
            previous_error: 0, // Initialize the previous error
            output: 0,         // Initialize the output
        }
    }

    /// Update the PID controller calculations
    ///
    /// # Arguments
    /// * `error` - The difference between the desired and measured values
    /// * `feedfwd` - A feed-forward value used to anticipate the system response
    /// * `limit` - The maximum output limit (positive or negative)
    ///
    /// This method computes the new PID output based on the provided error, feed-forward value,
    /// and output limits, considering the proportional, integral, derivative, and feed-forward components.
    pub fn tick(&mut self, error: i16, feedfwd: i16, limit: i16) {
        // Convert inputs as i32 to allow fixed point math
        let error = error as i32;
        let feedfwd = feedfwd as i32;
        let limit = limit as i32;

        // ######################## PROPORTIONAL TERM #################################
        let p = Self::apply_coef(error, self.kp); // Maximum possible value: ±100 * ±2^15

        // ########################## INTEGRAL TERM ###################################
        // Tustin's method (trapezoidal rule) for integrating the error with smoothing
        self.integral += (error + self.previous_error) >> 1;

        // Clamp integral to avoid with anti-windup
        self.integral = Self::clamp(self.integral, limit); // Maximum accumulation: ±2^15

        // Calculate integral term
        let i = Self::apply_coef(self.integral, self.ki); // Maximum possible value: ±100 * ±2^15

        // ######################### DERIVATIVE TERM ##################################
        // Calculate derivative by finding the difference in error
        let derivative = error - self.previous_error; // Maximum value: ±2 * ±2^15 = ±2^16

        // Calculate derivative term
        let d = Self::apply_coef(derivative, self.kd); // Maximum possible value: ±100 * ±2^16

        // Update previous error for the next calculation
        self.previous_error = error;

        // ######################## FEED-FORWARD TERM #################################
        let ff = Self::apply_coef(feedfwd, self.kff); // Maximum possible value: ±100 * ±2^15

        // ############################## OUTPUT ######################################
        // Calculate the total output by combining all components
        let output = p + i + d + ff; // Maximum possible value: ±500 * 2^15

        // Apply fixed-point math correction to the output
        let output = Self::fixed_point_correction(output);

        // Clamp the final output to ensure it stays within the specified limits
        self.output = Self::clamp(output, limit) as i16;
    }

    /// Retrieve the output value of the PID controller
    /// # Returns
    /// The calculated output as a 16-bit integer value.
    pub fn output(&self) -> i16 {
        self.output
    }

    // Constants controlling fast vs. slow math operations
    const FAST_MATH: bool = true;
    const SLOW_MATH_SCALE: i32 = 2; // Do not change!

    /// Apply a gain coefficient to a value, scaling as needed
    ///
    /// # Arguments
    /// * `value` - The value to be multiplied by the coefficient
    /// * `coef` - The gain coefficient
    ///
    /// # Returns
    /// The scaled value after applying the gain coefficient.
    #[inline(always)]
    fn apply_coef(value: i32, coef: i32) -> i32 {
        if !Self::FAST_MATH {
            (value * coef) >> Self::SLOW_MATH_SCALE
        } else {
            (value * coef) >> 7
        }
    }

    /// Apply a gain coefficient to a value, scaling as needed
    ///
    /// # Arguments
    /// * `value` - The value to be multiplied by the coefficient
    /// * `coef` - The gain coefficient
    ///
    /// # Returns
    /// The scaled value after applying the gain coefficient.
    #[inline(always)]
    fn fixed_point_correction(value: i32) -> i32 {
        if !Self::FAST_MATH {
            value / (100 >> Self::SLOW_MATH_SCALE)
        } else {
            value
        }
    }

    /// Fit the gain coefficient within a valid range
    ///
    /// # Arguments
    /// * `coef` - The coefficient to fit within the specified range
    ///
    /// # Returns
    /// The clamped coefficient within the valid range.
    fn fit_coef(coef: i32) -> i32 {
        let coef = PID::clamp(coef, 10000);
        if !Self::FAST_MATH {
            coef
        } else {
            (coef << 7) / 100
        }
    }

    /// Clamp a value within a specified limit
    ///
    /// # Arguments
    /// * `value` - The value to clamp
    /// * `limit` - The maximum allowed positive or negative value
    ///
    /// # Returns
    /// The clamped value, ensuring it stays within ±`limit`.
    #[inline]
    fn clamp(value: i32, limit: i32) -> i32 {
        if value > limit {
            limit
        } else if value < -limit {
            -limit
        } else {
            value
        }
    }
}
