

use defmt_rtt as _; // Use the defmt_rtt crate for logging via RTT (Real-Time Transfer)

/// The main driver struct for the motor, holding all the state required for operation and calibration.
pub struct CalibrationTable<const N: usize> {
    // state: CalibrationState,
    /// Stores sampled encoder data across a full calibration cycle
    cal_table: [u16; N],
    /// Current number of valid points collected during calibration
    cal_size: usize,
    /// Divider representing how many steps form one electrical period
    el_angle_div: usize,
    /// Index of the first zero-position value after normalization
    offst_idx: usize,
    /// Offset to align calibration data such that "zero" aligns to the correct mechanical angle
    offst_val: u16,
    /// Maximum absolute deviation between ideal and actual calibration values
    max_deviation: u16,
    temp_idx: usize,
}

// Constants and methods used during calibration
impl<const N: usize> CalibrationTable<N> {
    /// ## Principle of Operation:
    /// - A full rotation, represented as a u16 value, is divided evenly into a fixed number of poles.
    /// - Each pole corresponds to a certain electrical angle segment.
    /// - The calibration table records actual measured positions for each theoretical angle.
    /// - By filling the table forward and backward (to remove hysteresis), then finding a minimal point
    ///   aligned with zero electrical angle, we establish an offset and a starting index.
    /// - After normalization (subtracting offset), the table ideally forms a smooth linear progression.
    ///
    /// Steps performed during calibration:
    /// 1. Allocate an array large enough to hold calibration samples for a full rotation.
    /// 2. Fill the table as we rotate the motor to sample positions at defined intervals.
    /// 3. Perform a second pass in the opposite direction to remove hysteresis effects.
    /// 4. Identify the minimum point where `el_angle == 0`, determining offset and start index.
    /// 5. Normalize the table by subtracting the offset, aligning the data so that zero angle aligns properly.
    /// 6. Calculate and record maximum deviations to assess calibration quality.

    /// Creates a new `CalibrationTable` with default values.
    pub fn new() -> Self {
        Self {
            // state: CalibrationState::Reset,
            offst_val: 0, // Default offset is 0 until calibration determines otherwise
            offst_idx: 0, // Default start index is 0
            cal_size: 0,  // Initially, no calibration points are filled
            cal_table: [0; N], // The calibration data array, all initialized to zero
            el_angle_div: 1, // Default electrical angle divider is 1 (no division)
            max_deviation: 0, // Initially, no deviation is recorded
            temp_idx: 0,
        }
    }

    /// Resets the calibration table for a new calibration process.
    /// `el_angle_div` sets how many steps form one electrical period.
    pub fn reset(&mut self, el_angle_div: u16) {
        self.offst_idx = 0; // Reset start index
        self.el_angle_div = el_angle_div as usize; // Convert el_angle_div to usize for indexing
        self.cal_size = 0; // Clear the size (no valid data points yet)
        self.offst_val = u16::MAX; // Initialize offset to max, so we can find the minimum value later
        self.max_deviation = 0;
        self.temp_idx = 0;
    }

    /// Stores the first round of calibration data at the given index.
    /// This collects raw samples as the motor is rotated in one direction.
    pub fn fill_first(&mut self, idx: usize, val: u16) -> bool {
        // Each next index should be +1 to prevoius and should not go out limit
        if idx < N && idx.wrapping_sub(self.temp_idx) <= 1  {
            self.temp_idx = idx;
            self.cal_table[idx] = val; // Store the sample value
            self.cal_size = self.cal_size.max(idx + 1); // Update cal_size to ensure it reflects the maximum filled index
            return true;
        }
        // self.state = CalibrationState::Error;
        defmt::warn!("CAL TABLE: fill_first: Index got error {}", idx);
        return false;
    }

    /// Performs a second pass of filling data in the opposite direction to remove hysteresis.
    /// This method averages the forward and backward measurements to center the hysteresis loop.
    pub fn fill_second(&mut self, idx: usize, val: u16) -> bool {
        // Each next index should not go out limit
        if idx < N {     
            // Calculate the signed difference to handle potential wraparound (values near 0 or max range).
            let dif = val.wrapping_sub(self.cal_table[idx]) as i16;

            // Update the table with the midpoint of the hysteresis range.
            let val = self.cal_table[idx].wrapping_add((dif / 2) as u16);
            self.cal_table[idx] = val;

            // Every `el_angle_div` steps, we check if this is the minimal offset position.
            // If so, update the offset to the lowest encountered value.
            if ((idx % self.el_angle_div) == 0) && (val <= self.offst_val) {
                self.offst_val = val;
                self.offst_idx = idx;
            }

            if (idx + 1 == self.cal_size) {
                self.max_deviation = val;
                return true;
            }
            let diff = self.max_deviation.wrapping_sub(val) as i16;
            self.max_deviation = val;
            if diff > 10 {
                return true;
            }
        }
        defmt::warn!("CAL TABLE: fill_first: Index got error {}", idx);
        return false;
    }

    /// Retrieves a calibration value by an index relative to the `start_idx`.
    /// The resulting index is wrapped around `cal_size` to handle modulo arithmetic over a circular table.
    #[inline(always)]
    pub fn get_val_by_idx(&self, index: usize) -> u16 {
        let actual_idx = (self.offst_idx + index) % self.cal_size;
        self.cal_table[actual_idx] // Return the table value at the computed position
    }

    /// Validates the calibration data by checking the consistency of the table.
    /// Ensures that `cal_size` matches an integral number of poles (el_angle_div) and that
    /// deviations from the ideal linear distribution are acceptable.
    /// # TODO: make it without for loop
    pub fn check(&mut self) -> bool {
        // Check if the total number of samples divides evenly by `el_angle_div`.

        let avg_step = u16::MAX / self.cal_size as u16;
        self.max_deviation = 0; // Reset the maximum deviation counter
        for i in 0..self.cal_size {
            let val = self.cal_table[i].wrapping_sub(self.offst_val); // Shift values so offset becomes zero
            let corrected_idx = ((self.cal_size + i) - self.offst_idx) % self.cal_size;
            let deviation = abs_deviation(val, corrected_idx, self.cal_size);

            self.cal_table[i] = val;
            self.max_deviation = self.max_deviation.max(deviation);

            if deviation >= avg_step {
                defmt::error!(
                    "Step deviation too high [Avg step: {}; Max deviation: {}]",
                    avg_step,
                    self.max_deviation
                );
                return false;
            }
        }
        defmt::info!(
            "CAL TABLE: Success! Ofset val: {}; Offset idx: {}, Max deviation: {};",
            self.offst_val,
            self.offst_idx,
            self.max_deviation
        );
        return true;
    }

    /// Corrects a given position using the calibration table.
    /// Given an actual encoder `position`, it accounts for the offset and searches near the expected index.
    /// Uses a small loop to find the segment where real_pos transitions from positive to negative difference,
    /// then interpolates the ideal position to achieve a corrected angle.
    ///
    /// Returns:
    /// (corrected_angle, mech_el_angle):
    /// - `corrected_angle` is the position adjusted by the calibration table offset.
    /// - `mech_el_angle` is the mechanical angle mapped into an electrical period.
    pub fn correct_pos(&self, position: u16) -> (u16, u16) {
        // Align the position so that zero aligns with the table's zero-offset point.
        let real_pos = position.wrapping_sub(self.offst_val);

        // Estimate a starting index by scaling `real_pos` down.
        // The bit-shift by 16 approximates a division by 65536,
        // since `cal_size` and values are in `u16` range.
        let mut idx = (real_pos.wrapping_sub(self.max_deviation) as usize * self.cal_size) >> 16;

        let mut result: u16 = u16::MAX; // Initialize result to max as a fallback

        // Starting comparison points
        let mut cal_pos1 = self.get_val_by_idx(idx);
        let mut idl_pos1 = get_ideal(idx, self.cal_size);

        // Iterate up to 8 steps to find where we cross from positive diff to negative diff.
        for _ in 0..8 {
            idx = (idx + 1) % self.cal_size;
            let cal_pos2 = self.get_val_by_idx(idx);
            let idl_pos2 = get_ideal(idx, self.cal_size);

            let diff1 = real_pos.wrapping_sub(cal_pos1) as i16;
            let diff2 = real_pos.wrapping_sub(cal_pos2) as i16;

            // Once we find a boundary where diff changes sign (diff1 >= 0, diff2 < 0),
            // we interpolate the exact ideal position within that segment.
            if (diff1 >= 0) && (diff2 < 0) {
                result = interpolate(cal_pos1, idl_pos1, cal_pos2, idl_pos2, real_pos);
                break;
            }

            // Move to the next segment
            cal_pos1 = cal_pos2;
            idl_pos1 = idl_pos2;
        }

        // Re-apply offset to return the corrected angle to the global coordinate system.
        let corrected_angle = result.wrapping_add(self.offst_val);

        // Compute the mechanical angle mapped into one electrical period.
        let mech_el_angle = ((result as usize * self.cal_size) / self.el_angle_div) as u16;

        (corrected_angle, mech_el_angle)
    }
}

/// Computes an ideal value for the given index `i` within a range.
/// This function assumes a linear increase from 0 to `u16::MAX` across `range` points.
#[inline(always)]
fn get_ideal(i: usize, range: usize) -> u16 {
    const CAL_VAL_RANGE: usize = u16::MAX as usize; // Range of `u16` values (0 to 65535)
    ((CAL_VAL_RANGE * i) / range) as u16
}

/// Calculates the maximum absolute deviation from the ideal linear progression.
/// Ideal value for each element `i` is `(u16::MAX * i) / cal_size`.
/// Returns the maximum found deviation.
#[inline(always)]
fn abs_deviation(val: u16, idx: usize, size: usize) -> u16 {
    // Compute the ideal value at index `i`.
    let ideal = get_ideal(idx, size);

    // Compute the deviation as the absolute difference from the measured value.
    let deviation = ((ideal as u16).wrapping_sub(val) as i16).abs() as u16;

    deviation
}

/// Interpolates a position `c1` within a reference segment defined by points (a1, a2) and (b1, b2).
/// `a1` and `b1` form one pair (real and ideal), and `a2` and `b2` form the second pair.
/// Returns the ideal interpolated value corresponding to `c1`.
#[inline(always)]
fn interpolate(
    a1: u16,
    a2: u16, // Ideal and real at point A
    b1: u16,
    b2: u16, // Ideal and real at point B
    c1: u16, // Real value at point C
) -> u16 {
    // Calculate the reference range in the real domain
    let ref_range = b1.wrapping_sub(a1);

    // Calculate the target (ideal) range
    let trgt_range = b2.wrapping_sub(a2);

    // Determine how far along C is within the A-B segment in real values
    let c1_ofst = c1.wrapping_sub(a1);

    // Interpolate this fraction into the ideal domain
    let c2_ofst = (c1_ofst as u32 * trgt_range as u32) / ref_range as u32;

    // Return the interpolated ideal value
    (a2 as u32 + c2_ofst) as u16
}
