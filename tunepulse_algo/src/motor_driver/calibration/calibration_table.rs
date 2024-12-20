// Implements the motor calibration module, handling calibration data collection,
// normalization, and position correction for the motor control system.

// Key Features:
// - Collects and stores encoder data during motor rotation for calibration.
// - Removes hysteresis effects through bidirectional data collection.
// - Determines calibration offset and start index based on minimal deviation.
// - Validates calibration data for consistency and accuracy.
// - Corrects motor position readings using the calibrated data.

// Detailed Operation:
// The calibration module divides a full motor rotation into a fixed number of electrical
// angle segments. It records actual encoder positions for each theoretical angle, performing
// forward and backward passes to eliminate hysteresis. The module identifies the minimum
// deviation point to establish an offset and start index, then normalizes the calibration
// data to ensure a smooth linear progression. It validates the calibration by checking
// deviations against the average step size and corrects motor positions using interpolated
// values from the calibration table.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

use defmt_rtt as _; // Use the defmt_rtt crate for logging via RTT (Real-Time Transfer)

/// The main driver struct for the motor, holding all the state required for operation and calibration.
pub struct CalibrationTable<const N: usize> {
    // state: CalibrationState, // Calibration state management (currently commented out)
    /// Stores sampled encoder data across a full calibration cycle.
    cal_table: [u16; N],

    /// Current number of valid points collected during calibration.
    cal_size: usize,

    /// Divider representing how many steps form one electrical period.
    el_angle_div: usize,

    /// Index of the first zero-position value after normalization.
    offst_idx: usize,

    /// Offset to align calibration data such that "zero" aligns to the correct mechanical angle.
    offst_val: u16,

    /// Maximum absolute deviation between ideal and actual calibration values.
    max_deviation: u16,

    /// Temporary index used during calibration data collection.
    temp_idx: usize,
}

// Constants and methods used during calibration
impl<const N: usize> CalibrationTable<N> {
    /// Creates a new `CalibrationTable` with default values.
    pub fn new() -> Self {
        Self {
            // state: CalibrationState::Reset, // Initialize calibration state to Reset
            offst_val: 0, // Default offset is 0 until calibration determines otherwise
            offst_idx: 0, // Default start index is 0
            cal_size: 0,  // Initially, no calibration points are filled
            cal_table: [0; N], // The calibration data array, all initialized to zero
            el_angle_div: 1, // Default electrical angle divider is 1 (no division)
            max_deviation: 0, // Initially, no deviation is recorded
            temp_idx: 0,  // Initialize temporary index to 0
        }
    }

    /// Resets the calibration table for a new calibration process.
    /// `el_angle_div` sets how many steps form one electrical period.
    pub fn reset(&mut self, el_angle_div: u16) {
        self.offst_idx = 0; // Reset start index
        self.el_angle_div = el_angle_div as usize; // Convert el_angle_div to usize for indexing
        self.cal_size = 0; // Clear the size (no valid data points yet)
        self.offst_val = u16::MAX; // Initialize offset to max, so we can find the minimum value later
        self.max_deviation = 0; // Reset maximum deviation
        self.temp_idx = 0; // Reset temporary index
    }

    /// Stores the first round of calibration data at the given index.
    /// This collects raw samples as the motor is rotated in one direction.
    pub fn fill_first(&mut self, idx: usize, val: u16) -> bool {
        // Check if the index is within bounds and sequential
        if idx < N && idx.wrapping_sub(self.temp_idx) <= 1 {
            self.temp_idx = idx; // Update temporary index
            self.cal_table[idx] = val; // Store the sample value
            self.cal_size = self.cal_size.max(idx + 1); // Update cal_size to reflect the maximum filled index
            return true; // Indicate successful storage
        }
        // Log a warning if the index is out of bounds or not sequential
        defmt::warn!("CAL TABLE: fill_first: Index got error {}", idx);
        return false; // Indicate failure
    }

    /// Performs a second pass of filling data in the opposite direction to remove hysteresis.
    /// This method averages the forward and backward measurements to center the hysteresis loop.
    pub fn fill_second(&mut self, idx: usize, val: u16) -> bool {
        // Check if the index is within bounds
        if idx < N {
            // Calculate the signed difference to handle potential wraparound (values near 0 or max range).
            let dif = val.wrapping_sub(self.cal_table[idx]) as i16;

            // Update the table with the midpoint of the hysteresis range.
            let val = self.cal_table[idx].wrapping_add((dif / 2) as u16);
            self.cal_table[idx] = val; // Store the averaged value

            // Every `el_angle_div` steps, check if this is the minimal offset position.
            if ((idx % self.el_angle_div) == 0) && (val <= self.offst_val) {
                self.offst_val = val; // Update offset to the lowest encountered value
                self.offst_idx = idx; // Update start index
            }

            // If the current index is the last filled index, record the max deviation
            if (idx + 1 == self.cal_size) {
                self.max_deviation = val; // Update maximum deviation
                return true; // Indicate successful completion
            }

            // Calculate the difference between current max deviation and the new value
            let diff = self.max_deviation.wrapping_sub(val) as i16;
            self.max_deviation = val; // Update max deviation to the new value

            // If the deviation exceeds the threshold, indicate successful completion
            if diff > 10 {
                return true;
            }
        }
        // Log a warning if the index is out of bounds
        defmt::warn!("CAL TABLE: fill_second: Index got error {}", idx);
        return false; // Indicate failure
    }

    /// Retrieves a calibration value by an index relative to the `start_idx`.
    /// The resulting index is wrapped around `cal_size` to handle modulo arithmetic over a circular table.
    #[inline(always)]
    pub fn get_val_by_idx(&self, index: usize) -> u16 {
        let actual_idx = (self.offst_idx + index) % self.cal_size; // Compute the actual index
        self.cal_table[actual_idx] // Return the table value at the computed position
    }

    /// Validates the calibration data by checking the consistency of the table.
    /// Ensures that `cal_size` matches an integral number of poles (el_angle_div) and that
    /// deviations from the ideal linear distribution are acceptable.
    /// # TODO: make it without for loop
    pub fn check(&mut self) -> bool {
        // Calculate the average step size based on the total range and number of samples
        let avg_step = u16::MAX / self.cal_size as u16;
        self.max_deviation = 0; // Reset the maximum deviation counter

        // Iterate through each calibration point to compute deviations
        for i in 0..self.cal_size {
            let val = self.cal_table[i].wrapping_sub(self.offst_val); // Shift values so offset becomes zero
            let corrected_idx = ((self.cal_size + i) - self.offst_idx) % self.cal_size; // Compute corrected index
            let deviation = abs_deviation(val, corrected_idx, self.cal_size); // Calculate deviation

            self.cal_table[i] = val; // Update calibration table with normalized value
            self.max_deviation = self.max_deviation.max(deviation); // Update maximum deviation if necessary

            // Check if the deviation exceeds the average step size
            if deviation >= avg_step {
                defmt::error!(
                    "Step deviation too high [Avg step: {}; Max deviation: {}]",
                    avg_step,
                    self.max_deviation
                );
                return false; // Indicate validation failure
            }
        }

        // Log successful calibration validation
        defmt::info!(
            "CAL TABLE: Success! Offset val: {}; Offset idx: {}, Max deviation: {};",
            self.offst_val,
            self.offst_idx,
            self.max_deviation
        );
        return true; // Indicate validation success
    }

    /// Corrects a given position using the calibration table.
    /// Given an actual encoder `position`, it accounts for the offset and searches near the expected index.
    /// Uses a small loop to find the segment where real_pos transitions from positive to negative difference,
    /// then interpolates the ideal position to achieve a corrected angle.
    pub fn correct_pos(&self, position: u16) -> (u16, u16) {
        // Align the position so that zero aligns with the table's zero-offset point.
        let real_pos = position.wrapping_sub(self.offst_val);

        // Estimate a starting index by scaling `real_pos` down.
        let mut idx = (real_pos.wrapping_sub(self.max_deviation) as usize * self.cal_size) >> 16;

        let mut result: u16 = u16::MAX; // Initialize result to max as a fallback

        // Starting comparison points
        let mut cal_pos1 = self.get_val_by_idx(idx); // Retrieve calibration value at current index
        let mut idl_pos1 = get_ideal(idx, self.cal_size); // Retrieve ideal value at current index

        // Iterate up to 8 steps to find where we cross from positive diff to negative diff.
        for _ in 0..8 {
            idx = (idx + 1) % self.cal_size; // Move to the next index
            let cal_pos2 = self.get_val_by_idx(idx); // Retrieve calibration value at new index
            let idl_pos2 = get_ideal(idx, self.cal_size); // Retrieve ideal value at new index

            let diff1 = real_pos.wrapping_sub(cal_pos1) as i16; // Difference at previous index
            let diff2 = real_pos.wrapping_sub(cal_pos2) as i16; // Difference at current index

            // Once we find a boundary where diff changes sign (diff1 >= 0, diff2 < 0),
            // we interpolate the exact ideal position within that segment.
            if (diff1 >= 0) && (diff2 < 0) {
                result = interpolate(cal_pos1, idl_pos1, cal_pos2, idl_pos2, real_pos); // Perform interpolation
                break; // Exit the loop after interpolation
            }

            // Move to the next segment
            cal_pos1 = cal_pos2; // Update previous calibration position
            idl_pos1 = idl_pos2; // Update previous ideal position
        }

        // Re-apply offset to return the corrected angle to the global coordinate system.
        let corrected_angle = result.wrapping_add(self.offst_val); // Adjust corrected angle with offset

        // Compute the mechanical angle mapped into one electrical period.
        let mech_el_angle = ((result as usize * self.cal_size) / self.el_angle_div) as u16; // Calculate mechanical to electrical angle

        (corrected_angle, mech_el_angle) // Return the corrected angles
    }
}

/// Computes an ideal value for the given index `i` within a range.
/// This function assumes a linear increase from 0 to `u16::MAX` across `range` points.
#[inline(always)]
fn get_ideal(i: usize, range: usize) -> u16 {
    const CAL_VAL_RANGE: usize = u16::MAX as usize; // Range of `u16` values (0 to 65535)
    ((CAL_VAL_RANGE * i) / range) as u16 // Calculate ideal value based on linear progression
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

    deviation // Return the calculated deviation
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
