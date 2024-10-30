/// SpeedEstimator estimates the instantaneous speed of the encoder.
pub struct SpeedEstimator {
    freq: u16,            // Sampling frequency
    speed: i32,           // Calculated speed
    pos_buffer: [i32; 8], // Circular buffer for position samples
    idx: usize,           // Current index in circular buffer
}

impl SpeedEstimator {
    const SIZE: usize = 8; // Additional bits for better filtering (min 0 max 7)
    pub fn new(init_position: i32, freq: u16) -> Self {
        Self {
            freq,
            speed: 0,
            pos_buffer: [init_position; SpeedEstimator::SIZE],
            idx: 0,
        }
    }

    pub fn tick(&mut self, new_position: i32) -> i32 {
        self.speed = ((new_position - self.pos_buffer[self.idx]) * self.freq as i32)
            / SpeedEstimator::SIZE as i32;
        self.pos_buffer[self.idx] = new_position;
        self.idx = (self.idx + 1) % SpeedEstimator::SIZE;
        return self.speed;
    }

    pub fn get_speed(&self) -> i32 {
        self.speed
    }
}
