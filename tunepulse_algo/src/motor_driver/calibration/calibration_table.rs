use defmt_rtt as _; // Use the defmt_rtt crate for logging via RTT (Real-Time Transfer)

/// The main driver struct for the motor, holding all the state required for operation and calibration.
pub struct CalibrationTable {
    offset: u16,
    start_idx: usize,
    cal_size: usize, // Number of points collected during calibration cycles
    cal_table: [u16; CalibrationTable::CAL_TABLE_SIZE], // Array for storing sampled encoder data during full calibration
}

// Constants used during calibration
impl CalibrationTable {
    const CAL_TABLE_SIZE: usize = 200;

    /// ## Principle of Operation:
    /// Based on the following:
    /// - A full rotation (u16) has a fixed whole number of poles `Np`
    /// - Poles are evenly distributed around the circle with a fixed step
    /// - The first value is always zero
    /// We perform the following steps:
    /// 1) Create a table with a number of elements, allowing space for the number of poles
    /// 2) Assuming linear distribution, calculate that each `i` element of the table is equal to `u16::MAX * i / Np`
    /// 3) ...
    ///
    pub fn new() -> Self {
        Self {
            offset: 0, // Initialize encoder position to 0
            start_idx: 0,
            cal_size: 0, // Initialize number of points to 0
            cal_table: [0; CalibrationTable::CAL_TABLE_SIZE], // Data array for storing calibration samples, initialized to 0
        }
    }

    /// A function to calculate the ideal value of point C using linear interpolation.
    /// Given points A (ideal, real), B (ideal, real), and C (real),
    /// it calculates the ideal value of point C.
    fn interpolate_value(
        a: (u16, u16), // Point A (ideal, real)
        b: (u16, u16), // Point B (ideal, real)
        c_real: u16,   // Real value of point C
    ) -> u16 {
        // Destructure the input tuples for clarity
        let (a_ideal, a_real) = a;
        let (b_ideal, b_real) = b;

        // Calculate the real range
        let real_range = b_real.wrapping_sub(a_real);

        // Calculate the ideal range
        let ideal_range = b_ideal.wrapping_sub(a_ideal);

        // Calculate the real difference
        let real_diff = c_real.wrapping_sub(a_real);

        // Perform interpolation considering wrapping for the real values
        let ideal_diff = (real_diff as u32 * ideal_range as u32) / real_range as u32;

        // Return the wrapped interpolated ideal value
        (a_ideal as u32 + ideal_diff) as u16
    }

    /// Fills the calibration table at the specified index with the given value.
    pub fn fill_first(&mut self, idx: u16, val: u16) {
        let idx = idx as usize;
        if idx < CalibrationTable::CAL_TABLE_SIZE {
            self.cal_table[idx] = val;
            self.cal_size = self.cal_size.max(idx + 1);
        } else {
            // Optionally handle the case where idx is out of bounds
            // self.num_points = 0;
        }
    }

    /// Fills the calibration table with a second set of values, adjusting for hysteresis.
    pub fn fill_second(&mut self, index: u16, val: u16) {
        let idx = index as usize;
        if idx < CalibrationTable::CAL_TABLE_SIZE {
            // Calculate the difference, accounting for wrapping (e.g., values near 0 and max range)
            // This uses `wrapping_sub` to ensure the subtraction wraps correctly if `val` < `self.cal_table[idx]`
            let dif = val.wrapping_sub(self.cal_table[idx]) as i16;

            // Update the calibration table by adding half of the calculated difference
            // This finds the center of hysteresis while also using wrapping to handle potential overflow
            self.cal_table[idx] = self.cal_table[idx].wrapping_add((dif / 2) as u16);
        } else {
            // Optionally handle the case where idx is out of bounds
            // self.num_points = 0;
        }
    }

    /// Finds the minimum value in the calibration table and sets it as the offset.
    pub fn get_min(&mut self) {
        self.offset = u16::MAX;
        for i in 0..self.cal_size {
            self.offset = self.offset.min(self.cal_table[i]);
        }
        defmt::println!("Offset: {}", self.offset);
    }

    /// Normalizes the calibration table by subtracting the offset from each element.
    /// Additionally, it sets the `start_idx` to the index of the first element that equals zero.
    pub fn normalize(&mut self) {
        for i in 0..(self.cal_size as usize) {
            self.cal_table[i] = self.cal_table[i].wrapping_sub(self.offset);
            if self.cal_table[i] == 0 {
                self.start_idx = i;
            }
        }
        defmt::println!("Zero: {}", self.start_idx);
    }

    /// Retrieves the value at the specified index, accounting for the circular buffer's `start_idx`.
    /// Returns `None` if the index is out of bounds.
    pub fn get(&self, index: usize) -> u16 {
        if index >= self.cal_size as usize {
            return u16::MAX;
        }
        let actual_idx = (self.start_idx as usize + index) % self.cal_size;
        self.cal_table[actual_idx]
    }

    /// Retrieves the value at the specified index without shifting.
    #[inline(always)]
    fn get_val(&self, index: usize) -> u16 {
        if self.cal_size != 0 {
            let expected = (index as i32 * u16::MAX as i32) / self.cal_size as i32;
            let restored = expected - self.cal_table[index] as i32;
            return (restored as u16).wrapping_add(self.offset);
        }
        0
    }

    /// Prints the current calibration table.
    pub fn print(&self) {
        defmt::println!("TABLE: {:?}", self.cal_table);
    }
}
