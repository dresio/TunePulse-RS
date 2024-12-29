// Implements the Space Vector Pulse Width Modulation (SVPWM) voltage calculations,
// including Clarke transforms and voltage adjustments for three-phase systems in motor control applications.

// Key Features:
// - Performs inverse and direct Clarke transforms to convert between two-phase (alpha-beta) and three-phase (A-B-C) systems.
// - Calculates SVPWM voltages based on sine and cosine references and available voltage.
// - Supports dual and triple current conversion methods.
// - Ensures voltage scaling and clamping to prevent overvoltage conditions.

// Detailed Operation:
// This module provides functions to perform Clarke transforms, converting between two-phase (alpha-beta)
// and three-phase (A-B-C) representations. The `inverse_clarke_tf` function computes phase duty from
// sine and cosine inputs, while the `direct_clarke_tf` function calculates alpha and beta components from
// phase currents. The `voltage_ab2abc` function calculates SVPWM voltages, scaling them based on available voltage
// and applying necessary offsets to ensure safe operation. Additionally, the module includes functions for
// dual and triple current conversions, facilitating different motor control scenarios.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pub mod duty {
    /// Calculates SVPWM voltages based on sine and cosine references and available voltage.
    /// Additionally, SVPWM allows excluding zero duty PWM.
    ///
    /// Limitations: May burn upper-side switches if full-scale voltage > supply voltage.
    #[inline]
    pub fn ab2abc(voltg_sin: i16, voltg_cos: i16) -> (i16, i16, i16) {
        const MAX_OUTPUT: i32 = i16::MAX as i32;
        // Inverse Clarke transform
        let (mut voltg_a, mut voltg_b, mut voltg_c) = super::inverse_clarke_tf(voltg_sin, voltg_cos); // Transforms sine and cosine voltages to three-phase voltages

        // Find the minimum and maximum phase voltages
        let voltg_min: i32 = voltg_a.min(voltg_b).min(voltg_c); // Determines the minimum voltage among phases
        let voltg_max: i32 = voltg_a.max(voltg_b).max(voltg_c); // Determines the maximum voltage among phases

        let voltg_offset: i32; // Initializes voltage offset

        let voltg_full_scale: i32 = voltg_max - voltg_min; // Calculates the full scale voltage range

        // Automatic constraining and bottom clamping if available voltage isn't enough
        if voltg_full_scale > MAX_OUTPUT {
            // Calculate scaling factor (fixed point based on i32, scale resolution: 15bit)
            let voltg_scale = (MAX_OUTPUT << 15) / voltg_full_scale; // Determines scaling factor to fit available voltage

            // Apply scale to all channels
            voltg_a = (voltg_a * voltg_scale) >> 15; // Scales voltage A
            voltg_b = (voltg_b * voltg_scale) >> 15; // Scales voltage B
            voltg_c = (voltg_c * voltg_scale) >> 15; // Scales voltage C

            // Apply scale to voltg_min to allow correct clamping
            let voltg_min_scaled: i32 = (voltg_min * voltg_scale) >> 15; // Scales minimum voltage
            voltg_offset = -voltg_min_scaled; // Sets voltage offset based on scaled minimum voltage
        } else {
            // Calculate reference voltage to shift all phase voltages
            voltg_offset = (MAX_OUTPUT - voltg_max - voltg_min) >> 1; // Determines voltage offset for shifting
        }

        // If zero voltage is required - activate maximum brake
        if voltg_full_scale != 0 {
            // Shift all phase voltages by reference voltage
            voltg_a += voltg_offset; // Applies offset to voltage A
            voltg_b += voltg_offset; // Applies offset to voltage B
            voltg_c += voltg_offset; // Applies offset to voltage C
        }

        return (voltg_a as i16, voltg_b as i16, voltg_c as i16); // Returns the final adjusted voltages
    }
}

pub mod current {
    /// Converts dual current measurements from ABC to AB system.
    /// Third component calculated based on Kirchhoff's current law (Ia + Ib + Ic = 0)
    #[inline]
    pub fn dual(curnt_a: i16, curnt_b: i16) -> (i16, i16) {
        let (i_alpha, i_beta) =
            super::direct_clarke_tf(curnt_a, curnt_b, -(curnt_a.saturating_add(curnt_b))); // Perform direct Clarke transform with dual currents
        (i_alpha, i_beta) // Return the alpha and beta current components
    }

    /// Converts triple current measurements from ABC to AB system.
    #[inline]
    pub fn triple(curnt_a: i16, curnt_b: i16, curnt_c: i16) -> (i16, i16) {
        let (i_alpha, i_beta) = super::direct_clarke_tf(curnt_a, curnt_b, curnt_c); // Perform direct Clarke transform with triple currents
        (i_alpha, i_beta) // Return the alpha and beta current components
    }
}

/// Precalculated sqrt(3)/2
const SQRT3: f64 = 1.7320508075688772;
/// Precalculated scaling factor for sqrt(3) in i16 format
const SQRT3DIV2: i32 = (SQRT3 / 2.0f64 * (1u32 << 16) as f64) as i32;

/// Performs the inverse Clarke transform to calculate phase values (A, B, C)
/// from the `sin` and `cos` values.
fn inverse_clarke_tf(sin: i16, cos: i16) -> (i32, i32, i32) {
    let sin: i32 = sin as i32; // Convert sine input to i32
    let cos: i32 = cos as i32; // Convert cosine input to i32

    // Convert beta value component to a scaled value using SQRT3DIV2
    let beta_sqrt3_div2: i32 = (SQRT3DIV2 * cos) >> 16; // Scale the cosine component

    // Set phase A value to the alpha component
    let a: i32 = sin; // Assign sine value to phase A

    // Calculate phase B value: -1/2 * V_alpha + sqrt(3)/2 * V_beta
    let b: i32 = -(sin >> 1) + beta_sqrt3_div2; // Compute phase B voltage

    // Calculate phase C value: -1/2 * V_alpha - sqrt(3)/2 * V_beta
    let c: i32 = -(sin >> 1) - beta_sqrt3_div2; // Compute phase C voltage

    (a, b, c) // Return the calculated phase voltages
}

/// Performs the direct Clarke transform to calculate the `alpha` and `beta` components
/// from the phase values `a`, `b`, and `c`.
fn direct_clarke_tf(a: i16, b: i16, c: i16) -> (i16, i16) {
    let alpha = a; // Alpha component is directly the phase A value

    let b = b as i32; // Convert phase B to i32 for calculation
    let c = c as i32; // Convert phase C to i32 for calculation

    // Beta component: (V_B - V_C) * sqrt(3)/2 / 2
    // Using scaling with SQRT3DIV2 and a right shift to maintain precision.
    let beta = ((b - c) * SQRT3DIV2) >> 16; // Calculate beta component
    let beta = beta as i16; // Convert beta back to i16

    (alpha, beta) // Return the alpha and beta components
}
