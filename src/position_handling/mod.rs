mod angle_filter;
use angle_filter::AngleFilter;

mod speed_estimator;
use speed_estimator::SpeedEstimator;

/// EncoderPositionHandler manages and calculates the absolute position and speed of the encoder.
pub struct EncoderPositionHandler {
    position: i32,                   // Combined value (rotations + angle)
    rotations: i16,                  // Full rotations count
    angle: u16,                      // Shaft angle
    pub alpha: u8,                   // Filtering coefficient (0 <..> 255)
    filter: AngleFilter,             // Position filter instance
    speed_estimator: SpeedEstimator, // Speed estimator instance
    prev_sector: i16,                // Previous angle for zero-cross detection
}

impl EncoderPositionHandler {
    /// Creates new encoder handler instance
    pub fn new(raw_angle: u16, freq: u16, alpha: u8) -> Self {
        // Set zero position as beginning
        let init_position = (raw_angle as u32) as i32;
        // Init filter with input values as default
        let filter = AngleFilter::new(raw_angle, alpha);
        // Init speed estimator with input values as default
        let speed_estimator = SpeedEstimator::new(init_position, freq);
        Self {
            position: init_position,
            rotations: 0,
            angle: raw_angle,
            alpha,
            filter,
            speed_estimator,
            prev_sector: 2,
        }
    }

    /// Updates the encoder state, including position filtering, zero-cross detection, and speed estimation.
    pub fn tick(&mut self, input_pos: u16) {
        // Update filter value
        self.filter.tick(input_pos);

        // Update current angle based on filtered value
        self.angle = self.filter.get_output();

        // Adjust rotations based on zero-cross detection
        self.rotations += self.angle_zcd(self.filter.get_output());

        // Calculate position combined value
        self.position = self.angle as i32 + ((self.rotations as u32) << 16) as i32;

        // Calculate instant speed based on position change
        self.speed_estimator.tick(self.position);
    }

    /// Detects zero-crossings and updates the rotation count accordingly.
    fn angle_zcd(&mut self, angle: u16) -> i16 {
        // Extract the 2 most significant bits (sectors) of the current angle
        let angle_current = (angle >> 14) as i16;

        // Calculate the difference between previous and current sectors
        let diff: i16 = self.prev_sector - angle_current;

        // Store the current sector as the previous one for the next call
        self.prev_sector = angle_current;

        // Calculate rotation direction based on sector difference (±3 indicates a zero crossing)
        (diff == 3) as i16 - (diff == -3) as i16 // -1 if neg ZCD, +1 if pos ZCD, 0 if no ZCD
    }

    /// Getter for angle
    pub fn angle(&self) -> u16 {
        self.angle
    }

    /// Getter for rotations
    pub fn rotations(&self) -> i16 {
        self.rotations
    }

    /// Getter for position, returns i32 (i16 rotations + u16 angle)
    pub fn position(&self) -> i32 {
        self.position
    }

    /// Getter for speed, returns i32 (i16 RPM + u16 fractional part)
    pub fn speed(&self) -> i32 {
        self.speed_estimator.get_speed()
    }
}
