use super::lpf::FilterLPF;

use super::norm_to_value;


/// @file supply_voltage.rs
/// @brief Provides the `SupplyVoltage` struct for managing and filtering supply voltage measurements.
///
/// This module defines the `SupplyVoltage` struct, which handles raw supply voltage readings,
/// applies a low-pass filter to smooth the data, and scales the filtered output to obtain
/// the voltage in millivolts.

/// Manages supply voltage measurements with low-pass filtering.
///
/// This struct processes raw supply voltage readings, applies a low-pass filter to smooth the data,
/// and scales the filtered output to obtain the voltage in millivolts.
///
/// # Parameters
/// - `MAX_VOLTAGE_MV`: The maximum expected supply voltage in millivolts.
pub struct SupplyVoltage {
    filter: FilterLPF,  // Instance of low-pass filter for smoothing voltage measurements.
    max_voltage_mv: i32,  // Maximum voltage in millivolts for scaling.
    voltage_norm: i16,  // Current normalized voltage value.
    voltage_mv: i32,  // Current voltage measurement in millivolts.
}

impl SupplyVoltage {
    /// Constructs a `SupplyVoltage` object with the specified raw voltage and filter constant.
    ///
    /// # Parameters
    /// - `vsup_raw`: The raw supply voltage reading from ADC [supposed 16-bit for positive range].
    /// - `k_filter`: The filter constant for the low-pass filter.
    /// - `max_sup_voltage`: The maximum supply voltage in millivolts.
    pub fn new(k_filter: u8, max_sup_voltage: i32) -> Self {
        SupplyVoltage {
            max_voltage_mv: max_sup_voltage,
            filter: FilterLPF::new(0, k_filter),  // Initialize the filter with raw voltage and filter constant.
            voltage_norm: 0,
            voltage_mv: 0,
        }
    }

    /// Updates the voltage measurement by processing the filter and scaling the output.
    ///
    /// This function should be called periodically to refresh the voltage measurement.
    pub fn tick(&mut self, vsup_adc: u16) {
        self.filter.tick(vsup_adc);  // Advance the filter state.
        self.voltage_norm = (self.filter.get_output() as u16 >> 1) as i16;  // Retrieve and normalize the filter output.
        self.voltage_mv = norm_to_value(self.voltage_norm, self.max_voltage_mv);  // Convert normalized voltage to millivolts.
    }

    /// Retrieves the normalized voltage value.
    ///
    /// # Returns
    /// A reference to the normalized voltage.
    pub fn voltage_norm(&self) -> i16 {
        self.voltage_norm
    }

    /// Retrieves the voltage in millivolts.
    ///
    /// # Returns
    /// The voltage in millivolts.
    pub fn voltage_mv(&self) -> i32 {
        self.voltage_mv
    }

    /// Retrieves the maximum voltage in millivolts.
    ///
    /// # Returns
    /// The maximum voltage in millivolts.
    pub fn max_voltage_mv(&self) -> i32 {
        self.max_voltage_mv
    }
}