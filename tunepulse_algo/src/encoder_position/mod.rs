use crate::math_integer::filters::lpf::FilterLPF;

pub mod speed_estimator;
use self::speed_estimator::SpeedEstimator;

/// EncoderPosition manages and calculates the absolute position and speed of the encoder.
pub struct EncoderPosition {
    position: i32,                   // Combined value (rotations + angle)
    filter: FilterLPF,               // Position filter instance
    speed_estimator: SpeedEstimator, // Speed estimator instance
}

impl EncoderPosition {
    /// Creates new encoder handler instance
    pub fn new(freq: u16) -> Self {
        // Init filter with input values as default
        let filter = FilterLPF::new(0, 0);
        // Init speed estimator with input values as default
        let speed_estimator = SpeedEstimator::new(0, freq);
        Self {
            position: 0,
            filter,
            speed_estimator,
        }
    }

    /// Updates the encoder state, including position filtering, zero-cross detection, and speed estimation.
    pub fn tick(&mut self, input_pos: u16) {
        // Update the filter with the new input position and handle overflow
        self.filter.tick(input_pos);

        // Retrieve the previous angle by casting the current position to u16
        let prev_angle: u16 = self.position as u16;

        // Calculate the difference between the current and the previous angle with wrapping
        let dif = self.filter.get_output().wrapping_sub(prev_angle) as i16;

        // Update the current position by adding the difference, ensuring it wraps around correctly
        self.position = self.position.wrapping_add(dif as i32);

        // Estimate the instantaneous speed based on the updated position
        self.speed_estimator.tick(self.position);
    }

    /// Getter for angle
    pub fn angle(&self) -> u16 {
        self.position as u16
    }

    /// Getter for rotations
    pub fn rotations(&self) -> i16 {
        (self.position >> 16) as i16
    }

    /// Getter for position, returns i32 (i16 rotations + u16 angle)
    pub fn position(&self) -> i32 {
        self.position
    }

    /// Getter for speed, returns i32 (i16 RPM + u16 fractional part)
    pub fn speed(&self) -> i32 {
        self.speed_estimator.get_speed()
    }

    /// Getter for speed, returns i32 (i16 RPM + u16 fractional part)
    pub fn set_alpha(&mut self, alpha: u8) {
        self.filter.set_alpha(alpha);
    }

    // Call this if ABZ encoder is used at it hit zero very first time
    pub fn reset(&mut self) {
        self.position = 0;
    }
}
