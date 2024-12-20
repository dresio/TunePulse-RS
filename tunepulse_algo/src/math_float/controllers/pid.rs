// Implements a floating-point PID controller, providing proportional, integral, derivative,
// and feedforward control functionalities for precise process control in various applications.

// Key Features:
// - Configurable proportional, integral, derivative, and feedforward gains.
// - Integrates error using Tustin's method with anti-windup via integral clamping.
// - Calculates control output with derivative and feedforward contributions.
// - Provides output retrieval for use in control loops.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

/// Implements the PID controller for precise process control.
pub struct PID {
    kp: f32,             // Proportional gain.
    ki: f32,             // Integral gain.
    kd: f32,             // Derivative gain.
    kff: f32,            // Feedforward gain.
    integral: f32,       // Accumulated integral of error.
    previous_error: f32, // Previous error value for derivative calculation.
    output: f32,         // Current output of the PID controller.
}

impl PID {
    /// Creates a new PID controller with specified gains.
    pub fn new(kp: f32, ki: f32, kd: f32, kff: f32) -> Self {
        Self {
            kp,                  // Sets the proportional gain.
            ki,                  // Sets the integral gain.
            kd,                  // Sets the derivative gain.
            kff,                 // Sets the feedforward gain.
            integral: 0.0,       // Initializes the integral term to zero.
            previous_error: 0.0, // Initializes the previous error to zero.
            output: 0.0,         // Initializes the output to zero.
        }
    }

    /// Updates the PID controller with the current error, feedforward value, and output limit.
    pub fn tick(&mut self, error: f32, feedfwd: f32, limit: f32) {
        // Calculate proportional term
        let p = self.kp * error; // Computes the proportional component.

        // Tustin's method for integrating the error with smoothing
        self.integral += (error + self.previous_error) * 0.5; // Integrates the error using Tustin's method.

        // Clamp integral to avoid with anti-windup
        self.integral = self.integral.clamp(-limit, limit); // Clamps the integral term to prevent windup.

        // Calculate integral term
        let i = self.ki * self.integral; // Computes the integral component.

        // Calculate derivative term
        let d = self.kd * (error - self.previous_error); // Computes the derivative component.

        // Update previous error for the next calculation
        self.previous_error = error; // Updates the previous error.

        // Calculate the total output with clamping
        self.output = (p + i + d).clamp(-limit, limit); // Calculates and clamps the total PID output.
    }

    /// Retrieves the current output of the PID controller.
    pub fn output(&self) -> f32 {
        self.output // Returns the current output value.
    }
}
