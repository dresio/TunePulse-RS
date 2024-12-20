// Implements the SupplyVoltage module, managing and filtering supply voltage measurements
// for accurate and stable voltage monitoring in the system.

// Key Features:
// - Processes raw supply voltage readings from ADC
// - Applies a low-pass filter to smooth voltage data
// - Scales filtered output to obtain voltage in millivolts
// - Provides access to normalized and scaled voltage values

// Detailed Operation:
// The SupplyVoltage struct handles raw ADC readings by passing them through a low-pass filter
// to eliminate noise and smooth the voltage signal. The filtered output is then normalized
// and scaled based on the maximum expected voltage to provide accurate millivolt measurements.
// This setup ensures reliable voltage monitoring for the system.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

use super::lpf::FilterLPF; // Imports the low-pass filter implementation from the parent module
use super::norm_to_value; // Imports the normalization to value conversion function from the parent module

/// Manages supply voltage measurements with low-pass filtering
pub struct SupplyVoltage {
    /// Instance of low-pass filter for smoothing voltage measurements
    filter: FilterLPF,

    /// Maximum voltage in millivolts for scaling
    max_voltage_mv: i32,

    /// Current normalized voltage value
    voltage_norm: i16,

    /// Current voltage measurement in millivolts
    voltage_mv: i32,
}

impl SupplyVoltage {
    /// Constructs a `SupplyVoltage` object with the specified filter constant and maximum voltage
    pub fn new(k_filter: u8, max_sup_voltage: i32) -> Self {
        SupplyVoltage {
            max_voltage_mv: max_sup_voltage, // Sets the maximum supply voltage
            filter: FilterLPF::new(0, k_filter), // Initializes the low-pass filter with initial value and filter constant
            voltage_norm: 0,                     // Initializes the normalized voltage to zero
            voltage_mv: 0,                       // Initializes the millivolt voltage to zero
        }
    }

    /// Updates the voltage measurement by processing the filter and scaling the output
    pub fn tick(&mut self, vsup_adc: u16) -> &Self {
        self.filter.tick(vsup_adc); // Advances the filter state with the new ADC reading
        self.voltage_norm = (self.filter.get_output() >> 1) as i16; // Retrieves and normalizes the filter output
        self.voltage_mv = norm_to_value(self.voltage_norm, self.max_voltage_mv); // Converts normalized voltage to millivolts
        self
    }

    /// Retrieves the normalized voltage value
    pub fn voltage_norm(&self) -> i16 {
        self.voltage_norm // Returns the current normalized voltage
    }

    /// Retrieves the voltage in millivolts
    pub fn voltage_mv(&self) -> i32 {
        self.voltage_mv // Returns the current voltage measurement in millivolts
    }

    /// Retrieves the maximum voltage in millivolts
    pub fn max_voltage_mv(&self) -> i32 {
        self.max_voltage_mv // Returns the maximum supply voltage in millivolts
    }
}
