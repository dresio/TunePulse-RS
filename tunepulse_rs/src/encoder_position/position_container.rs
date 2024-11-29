
pub struct AbsPosition {
    pub angle: u16,     // Angle component of the position
    pub rotations: i16, // Rotations component of the position
}

impl AbsPosition {
    pub fn new(rotations: i16, angle: u16) -> Self {
        AbsPosition { angle, rotations }
    }

    pub fn get_position(&self) -> i32 {
        self.angle as i32 + ((self.rotations as u32) << 16) as i32
    }

    pub fn set_position(&mut self, position: i32) {
        self.angle = (position & 0xFFFF) as u16;
        self.rotations = (position as u32 >> 16) as i16;
    }

    pub fn get_rotations(&self) -> i16 {
        self.rotations
    }

    pub fn set_rotations(&mut self, rotations: i16) {
        self.rotations = rotations;
    }

    pub fn get_angle(&self) -> u16 {
        self.angle
    }

    pub fn set_angle(&mut self, angle: u16) {
        self.angle = angle;
    }
}