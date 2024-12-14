/// SpeedEstimator estimates the instantaneous speed of the encoder.

/// Size of circular buffer (min 2 max 32)
const SIZE: usize = 8;

pub struct SpeedEstimator {
    freq: u16,            // Sampling frequency
    speed: i32,           // Calculated speed
    pos_buffer: [i32; SIZE], // Circular buffer for position samples
    idx: usize,           // Current index in circular buffer
}

impl SpeedEstimator {
    // Create new speed estimator
    pub fn new(init_position: i32, freq: u16) -> Self {
        Self {
            freq,
            speed: 0,
            pos_buffer: [init_position; SIZE],
            idx: 0,
        }
    }

    // Math call
    pub fn tick(&mut self, new_position: i32) {
        // Calculate position difference over N = SIZE samples
        let difference = new_position - self.pos_buffer[self.idx];

        // Calculate speed based on sampling frequency (corrected to buffer size)
        self.speed = difference.wrapping_mul(self.freq as i32) / SIZE as i32;

        // Update buffer
        self.pos_buffer[self.idx] = new_position;

        // Update index value
        self.idx = (self.idx + 1) % SIZE;
    }

    // Getter for instant speed
    pub fn get_speed(&self) -> i32 {
        self.speed
    }
}
