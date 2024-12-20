// Implements a double-buffering system for storing ADC and sensor data,
// ensuring data consistency and minimizing synchronization overhead.
// This module allows safe and efficient data capture from interrupt routines
// while allowing the main loop or other tasks to access fully updated data.

// Key Features:
// - Implements a double-buffering system for ADC and sensor data.
// - Ensures each buffer is fully updated before it becomes available to reading tasks.
// - Minimizes synchronization overhead by using a lock bit only during brief data reads.
// - Allows interrupt routines to safely capture data snapshots without partial updates.
// - Guarantees that fetched data is complete, up-to-date, and consistent.

// Detailed Operation:
// The module uses two `DataInputs` buffers and a set of flags to manage data updates and reads.
// Each buffer can be in one of two states: being updated or ready for reading.
// A bitmask tracks the completion of each data field within a buffer.
// When all mandatory fields in a buffer are filled, the buffer is marked as ready.
// The `get_data()` function acquires a lock on the ready buffer to prevent modifications
// during the read operation, ensuring data consistency without requiring heavy synchronization.
// This approach allows interrupt routines to update data swiftly while the main loop can
// read complete and coherent data sets efficiently.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

use defmt_rtt as _; // Use the defmt_rtt crate for logging via RTT (Real-Time Transfer)

/// Data structure holding various ADC readings and raw angle measurements.
#[derive(Clone, Copy, Default)]
pub struct DataInputs {
    /// Supply ADC reading.
    pub supply_adc: u16,

    /// Temperature ADC reading.
    pub temper_adc: u16,

    /// Current ADC readings (4 channels).
    pub currnt_adc: [u16; 4],

    /// Raw angle measurement.
    pub angle_raw: u16,
}

/// Enum defining bit masks for each data field and a lock bit.
/// Each variant represents a specific field in the `DataInputs` struct.
/// The `LOCK` variant is used to prevent modifications during data reads.
#[repr(u32)]
enum FieldBit {
    /// Mask for the supply ADC field bit.
    SUPPLY = 1 << 0,

    /// Mask for the temperature ADC field bit.
    TEMP = 1 << 1,

    /// Mask for the current ADC array field bit.
    CURRENT = 1 << 2,

    /// Mask for the angle field bit.
    ANGLE = 1 << 3,

    /// Mask for the lock bit at the most significant bit.
    LOCK = 1 << 31,
}

/// Combine all field bits using inverted logic to represent a fully unfilled (all fields pending) state.
const MANDATORY: u32 = FieldBit::SUPPLY as u32
    | FieldBit::TEMP as u32
    | FieldBit::CURRENT as u32
    | FieldBit::ANGLE as u32;

/// Structure for managing two buffers of `DataInputs` and related flags.
/// Utilizes double-buffering to ensure data consistency and minimize synchronization overhead.
pub struct InputsDump<const MANDATORY_FIELDS: u32> {
    /// Two buffers: one being updated, one ready for reading.
    buffers: [DataInputs; 2],

    /// Index of the buffer currently being updated.
    idx2update: usize,

    /// Flags array holding field completion and lock status for each buffer.
    flags: [u32; 2],

    /// Iteration counter to track updates.
    iter: usize,

    /// Previous iteration counter to detect updates.
    prev_iter: usize,
}

impl<const MANDATORY_FIELDS: u32> InputsDump<MANDATORY_FIELDS> {
    /// Creates a new `InputsDump` with both buffers cleared and ready to be filled.
    pub fn new() -> Self {
        Self {
            buffers: [DataInputs::default(), DataInputs::default()], // Initialize both buffers to default
            idx2update: 0,                                           // Start updating buffer 0
            flags: [MANDATORY_FIELDS, 0], // Buffer 0: all fields pending; Buffer 1: ready (no fields pending)
            iter: 0,                      // Initialize iteration counters
            prev_iter: 0,
        }
    }

    /// Checks if the given buffer index is ready (no fields pending).
    #[inline(always)]
    fn is_ready(&self, idx: usize) -> bool {
        self.flags[idx] == 0 // If flags are 0, the buffer is completely filled (ready)
    }

    /// Gets the opposite buffer index (if idx=0 return 1, if idx=1 return 0).
    #[inline(always)]
    fn get_opposite(&self, idx: usize) -> usize {
        1 - idx
    }

    /// Clears a particular field bit in the flags for the specified buffer.
    #[inline(always)]
    fn clear_field_bit(&mut self, idx: usize, bit: FieldBit) {
        self.flags[idx] &= !(bit as u32); // Use NOT mask to clear the bit
    }

    /// Checks if all fields of the current buffer are filled; if both buffers are ready,
    /// reinitializes one for updating.
    fn check_fill(&mut self, idx: usize) {
        // If both buffers are ready (no pending fields),
        // re-initialize the opposite buffer for new data collection
        if self.is_ready(0) && self.is_ready(1) {
            let idx = self.get_opposite(idx); // Get the opposite buffer index
            self.flags[idx] = MANDATORY_FIELDS; // Set the opposite buffer to all fields pending
            self.idx2update = idx; // Switch to updating the opposite buffer
            self.iter = self.iter.wrapping_add(1); // Increment iteration counter
        }
    }

    /// Sets the `supply_adc` field in the currently updating buffer.
    pub fn set_supply_adc(&mut self, value: u16) {
        let idx = self.idx2update; // Get the currently updating buffer index
        self.buffers[idx].supply_adc = value; // Store the supply ADC value
        self.clear_field_bit(idx, FieldBit::SUPPLY); // Mark the supply field as filled
        self.check_fill(idx); // Check if buffer filling is complete or if we need to switch
    }

    /// Sets the `temper_adc` field in the currently updating buffer.
    pub fn set_temper_adc(&mut self, value: u16) {
        let idx = self.idx2update; // Get the currently updating buffer index
        self.buffers[idx].temper_adc = value; // Store the temperature ADC value
        self.clear_field_bit(idx, FieldBit::TEMP); // Mark the temperature field as filled
        self.check_fill(idx); // Check if buffer filling is complete or if we need to switch
    }

    /// Sets the `currnt_adc` field in the currently updating buffer.
    pub fn set_current_adc(&mut self, values: [u16; 4]) {
        let idx = self.idx2update; // Get the currently updating buffer index
        self.buffers[idx].currnt_adc = values; // Store the current ADC array
        self.clear_field_bit(idx, FieldBit::CURRENT); // Mark the current field as filled
        self.check_fill(idx); // Check if buffer filling is complete or if we need to switch
    }

    /// Sets the `angle_raw` field in the currently updating buffer.
    pub fn set_angle_raw(&mut self, value: u16) {
        let idx = self.idx2update; // Get the currently updating buffer index
        self.buffers[idx].angle_raw = value; // Store the angle data
        self.clear_field_bit(idx, FieldBit::ANGLE); // Mark the angle field as filled
        self.check_fill(idx); // Check if buffer filling is complete or if we need to switch
    }

    /// Checks if the data has been updated since the last read.
    #[inline(always)]
    pub fn is_updated(&self) -> bool {
        self.iter != self.prev_iter // Returns true if there has been an update
    }

    /// Gets a fully updated `DataInputs` from the opposite buffer.
    /// This method briefly sets a lock bit to prevent modifications during reading.
    #[inline(always)]
    pub fn get_data(&mut self) -> DataInputs {
        let ready_idx = self.get_opposite(self.idx2update); // Get the opposite buffer which should be ready
        self.flags[ready_idx] |= FieldBit::LOCK as u32; // Set the lock bit on the ready buffer
        let data = self.buffers[ready_idx]; // Copy the data from the locked buffer
        self.prev_iter = self.iter; // Update the previous iteration counter
        self.flags[ready_idx] &= !(FieldBit::LOCK as u32); // Clear the lock bit after reading
        data // Return the copied data
    }
}
