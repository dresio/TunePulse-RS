pub mod position_container;
use position_container::AbsPosition;

mod angle_filter;
use angle_filter::AngleFilter;

mod speed_estimator;
use speed_estimator::SpeedEstimator;



/// EncoderPositionHandler manages and calculates the absolute position and speed of the encoder.
pub struct EncoderPositionHandler {
    pub position: AbsPosition,       // Offset for absolute position correction
    pub alpha: u8,                   // Filtering coefficient (0 <..> 255)
    speed_estimator: SpeedEstimator, // Speed estimator instance
    filter: AngleFilter,          // Position filter instance
    prev_sector: i16,                // Previous angle for zero-cross detection
}

impl EncoderPositionHandler {
    pub fn new(raw_angle: u16, freq: u16, alpha: u8) -> Self {
        let position = AbsPosition::new(0, 0);
        let filter = AngleFilter::new(raw_angle, alpha);
        let speed_estimator = SpeedEstimator::new(position.get_position(), freq);

        Self {
            position,
            alpha,
            speed_estimator,
            filter,
            prev_sector: 2,
        }
    }

    /// Updates the encoder state, including position filtering, zero-cross detection, and speed estimation.
    pub fn tick(&mut self, input_pos: u16) -> i32 {
        self.filter.tick(input_pos);
        self.position.angle = self.filter.get_output();
        self.position.rotations += self.angle_zcd(self.filter.get_output());
        self.speed_estimator.tick(self.position.get_position());
        self.position.get_position()
    }

    /// Detects zero-crossings and updates the rotation count accordingly.
    fn angle_zcd(&mut self, angle: u16) -> i16 {
        let angle_current = (angle >> 14) as i16;
        let diff: i16 = self.prev_sector - angle_current;

        self.prev_sector = angle_current;

        if diff == 3 {
            1
        } else if diff == -3 {
            -1
        } else {
            0
        }
        
    }

    pub fn get_speed_inst(&self) -> i32 {
        self.speed_estimator.get_speed()
    }
}