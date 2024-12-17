/// EncoderPosition manages and calculates the absolute position and speed of the encoder.
pub struct Position {
    position: i32, // Combined value (rotations + angle)
}

impl Position {
    /// Creates new encoder handler instance
    pub fn new() -> Self {
        // Init filter with input values as default
        Self { position: 0 }
    }

    /// Updates the encoder state, including position filtering, zero-cross detection, and speed estimation.
    pub fn tick(&mut self, input_pos: u16) -> &Self {
        // Retrieve the previous angle by casting the current position to u16
        let prev_angle: u16 = self.position as u16;

        // Calculate the difference between the current and the previous angle with wrapping
        let dif = input_pos.wrapping_sub(prev_angle) as i16;

        // Update the current position by adding the difference, ensuring it wraps around correctly
        self.position = self.position.wrapping_add(dif as i32);

        self
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

    // Call this if ABZ encoder is used at it hit zero very first time
    pub fn reset(&mut self) {
        self.position = 0;
    }
}
