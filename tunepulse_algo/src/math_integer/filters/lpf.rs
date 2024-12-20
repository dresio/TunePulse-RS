// Implements the position filtering module, providing low-pass filtering
// functionality to smooth out sensor data and reduce noise in position measurements.

// Key Features:
// - Implements a low-pass filter (LPF) for smoothing position data.
// - Allows dynamic adjustment of the filter coefficient (`alpha`).
// - Efficiently updates filtered output with minimal computational overhead.
// - Provides additional 8bit for error storage improoving accuracy of result over time

// Detailed Operation:
// The `FilterLPF` struct implements a low-pass filter to smooth incoming position
// data. It maintains an internal state (`temp`) that holds the scaled filtered value.
// The filter operates by calculating the difference between the current input and the
// previous filtered value, scaling this difference by the filter coefficient (`alpha`),
// and updating the internal state accordingly. The `tick` method performs the filtering
// operation, while `get_output` retrieves the current filtered value. The `set_alpha`
// method allows dynamic adjustment of the filter coefficient to modify the filter's
// responsiveness.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

// Defining the PositionFilter struct that implements the position filtering logic.
pub struct FilterLPF {
    // Filter coefficient (0u..255u = 0.0f..1.0f)
    alpha: i32, 
    output: u16,
    temp: i32, // Stores scaled filtered value
}

impl FilterLPF {
    /// Constructor to initialize the filter with the input and alpha
    pub fn new(input_default: u16, alpha: u8) -> FilterLPF {
        FilterLPF {
            alpha: alpha as i32,
            output: input_default,
            temp: (input_default as i32) << 16,
        }
    }

    /// Math call
    pub fn tick(&mut self, input: u16) -> u16 {
        // Convert the input to a 32-bit integer and shift left by 16 bits to allow wrapping as i32
        let current: i32 = (input as i32) << 16;

        // LPF filter math: filtered = alpha * (input - prev)
        // For integer alpha u8: filtered = alpha * (input - prev) / 256 + current

        // Get difference between previous (temp is i32 scaled) and current
        let diff: i32 = self.temp.wrapping_sub(current) as i32;

        // Downscale difference to allow alpha scale
        let diff: i32 = diff >> 8;

        // Calculate rest part of lpf, temp now will be as prev for next iter (i32 scaled)
        self.temp = (diff * self.alpha).wrapping_add(current);

        // Convert to u32 to allow correct bitshift and scale back to u16
        self.output = (self.temp as u32 >> 16) as u16;

        self.output
    }

    /// Function to retrieve the output value
    pub fn get_output(&self) -> u16 {
        self.output
    }

    /// Function to retrieve the output value
    pub fn set_alpha(&mut self, alpha: u8) {
        self.alpha = alpha as i32;
    }
}
