/// Array representing the sine values of the first quarter wave (0 to 90 degrees in radians) as i1.15
/// This quarter-wave sine table allows for easy computation of sine and cosine values for any angle.
/// The array has 257 values, including the endpoint, to accommodate interpolation if necessary.
const SINE_QUARTER_WAVE: [i16; 257] = [
    0x0000, 0x00C9, 0x0192, 0x025B, 0x0324, 0x03ED, 0x04B6, 0x057E, 0x0647, 0x0710, 0x07D9, 0x08A1,
    0x096A, 0x0A32, 0x0AFB, 0x0BC3, 0x0C8B, 0x0D53, 0x0E1B, 0x0EE3, 0x0FAB, 0x1072, 0x1139, 0x1200,
    0x12C7, 0x138E, 0x1455, 0x151B, 0x15E1, 0x16A7, 0x176D, 0x1833, 0x18F8, 0x19BD, 0x1A82, 0x1B46,
    0x1C0B, 0x1CCF, 0x1D93, 0x1E56, 0x1F19, 0x1FDC, 0x209F, 0x2161, 0x2223, 0x22E4, 0x23A6, 0x2467,
    0x2527, 0x25E7, 0x26A7, 0x2767, 0x2826, 0x28E5, 0x29A3, 0x2A61, 0x2B1E, 0x2BDB, 0x2C98, 0x2D54,
    0x2E10, 0x2ECC, 0x2F86, 0x3041, 0x30FB, 0x31B4, 0x326D, 0x3326, 0x33DE, 0x3496, 0x354D, 0x3603,
    0x36B9, 0x376F, 0x3824, 0x38D8, 0x398C, 0x3A3F, 0x3AF2, 0x3BA4, 0x3C56, 0x3D07, 0x3DB7, 0x3E67,
    0x3F16, 0x3FC5, 0x4073, 0x4120, 0x41CD, 0x4279, 0x4325, 0x43D0, 0x447A, 0x4523, 0x45CC, 0x4674,
    0x471C, 0x47C3, 0x4869, 0x490E, 0x49B3, 0x4A57, 0x4AFA, 0x4B9D, 0x4C3F, 0x4CE0, 0x4D80, 0x4E20,
    0x4EBF, 0x4F5D, 0x4FFA, 0x5097, 0x5133, 0x51CE, 0x5268, 0x5301, 0x539A, 0x5432, 0x54C9, 0x555F,
    0x55F4, 0x5689, 0x571D, 0x57B0, 0x5842, 0x58D3, 0x5963, 0x59F3, 0x5A81, 0x5B0F, 0x5B9C, 0x5C28,
    0x5CB3, 0x5D3D, 0x5DC6, 0x5E4F, 0x5ED6, 0x5F5D, 0x5FE2, 0x6067, 0x60EB, 0x616E, 0x61F0, 0x6271,
    0x62F1, 0x6370, 0x63EE, 0x646B, 0x64E7, 0x6562, 0x65DD, 0x6656, 0x66CE, 0x6745, 0x67BC, 0x6831,
    0x68A5, 0x6919, 0x698B, 0x69FC, 0x6A6C, 0x6ADB, 0x6B4A, 0x6BB7, 0x6C23, 0x6C8E, 0x6CF8, 0x6D61,
    0x6DC9, 0x6E30, 0x6E95, 0x6EFA, 0x6F5E, 0x6FC0, 0x7022, 0x7082, 0x70E1, 0x7140, 0x719D, 0x71F9,
    0x7254, 0x72AE, 0x7306, 0x735E, 0x73B5, 0x740A, 0x745E, 0x74B1, 0x7503, 0x7554, 0x75A4, 0x75F3,
    0x7640, 0x768D, 0x76D8, 0x7722, 0x776B, 0x77B3, 0x77F9, 0x783F, 0x7883, 0x78C6, 0x7908, 0x7949,
    0x7989, 0x79C7, 0x7A04, 0x7A41, 0x7A7C, 0x7AB5, 0x7AEE, 0x7B25, 0x7B5C, 0x7B91, 0x7BC4, 0x7BF7,
    0x7C29, 0x7C59, 0x7C88, 0x7CB6, 0x7CE2, 0x7D0E, 0x7D38, 0x7D61, 0x7D89, 0x7DB0, 0x7DD5, 0x7DF9,
    0x7E1C, 0x7E3E, 0x7E5E, 0x7E7E, 0x7E9C, 0x7EB9, 0x7ED4, 0x7EEF, 0x7F08, 0x7F20, 0x7F37, 0x7F4C,
    0x7F61, 0x7F74, 0x7F86, 0x7F96, 0x7FA6, 0x7FB4, 0x7FC1, 0x7FCD, 0x7FD7, 0x7FE0, 0x7FE8, 0x7FEF,
    0x7FF5, 0x7FF9, 0x7FFC, 0x7FFE, 0x7FFF,
];

/// Computes the sine and cosine values for a given normalized angle.
///
/// ### Arguments
/// * `angle` - The input angle represented in i1.31 format, normalized to the range `[-Pi, Pi]`
///             using `[i32::MIN, i32::MAX]`. This allows for higher precision angle representation.
///
/// ### Returns
/// * A tuple `(sine, cosine)` - The sine and cosine values as `i1.15`.
///
/// ### Notes
/// * The function uses a quarter-wave lookup table for computational efficiency.
/// * The lookup is performed in 4 quadrants, reducing the memory footprint while allowing
///   for full 360-degree coverage.
pub const fn angle2sincos(angle: i16) -> (i16, i16) {
    // Get the top 10 bits (1024 points resolution per full wave)
    let angle_uint = (angle as u16) >> 6;

    // Map the normalized angle to the index of the quarter wave array (0 to 255)
    let index = angle_uint & 0xFF;

    // Retrieve sine values from the quarter-wave sine lookup table
    let a = SINE_QUARTER_WAVE[index as usize];
    let b = SINE_QUARTER_WAVE[256 - index as usize];

    // Determine the quadrant from the top 2 bits of the angle
    let quadrant = angle_uint >> 8;

    // Based on the quadrant, determine the correct sine and cosine values
    match quadrant {
        0 => (a, b),   // First quadrant: 0 to PI/2
        1 => (b, -a),  // Second quadrant: PI/2 to PI
        2 => (-a, -b), // Third quadrant: PI to 3*PI/2
        _ => (-b, a),  // Fourth quadrant: 3*PI/2 to 2*PI
    }
}

/// Scales sine and cosine values by a given scale factor in i1.15 format.
///
/// ### Arguments
/// * `input` - A tuple `(sine, cosine)` representing sine and cosine components as `i1.15`.
/// * `scale` - A scaling factor in `i1.15`. Typically used to adjust amplitude.
///
/// ### Returns
/// * A tuple `(scaled_sine, scaled_cosine)` representing the scaled values.
/// 
/// ### Notes
/// * Uses `i32` internally to avoid overflow during scaling.
pub fn scale_sincos(input: (i16, i16), scale: i16) -> (i16, i16) {
    // Convert to i32 to avoid overflow during calculations
    let (a, b) = (input.0 as i32, input.1 as i32);
    let scale = scale as i32;

    // Scale the sine and cosine values, shifting right to retain `i16` precision
    let a = ((a * scale) >> 15) as i16;
    let b = ((b * scale) >> 15) as i16;
    
    // Return the scaled sine and cosine values
    (a, b)
}

/// Rotates a vector represented by sine and cosine components as `i1.15` using another vector (offset),
/// also represented by sine and cosine components.
///
/// ### Arguments
/// * `source` - A tuple `(source_sin, source_cos)` representing the sine and cosine components
///              of the vector to be rotated as `i1.15`.
/// * `offset` - A tuple `(offset_sin, offset_cos)` representing the sine and cosine components
///              of the rotation angle as `i1.15`.
///
/// ### Returns
/// * A tuple `(out_sin, out_cos)` - The sine and cosine components of the rotated vector as `i1.15`.
/// 
/// ### Notes
/// * Uses `i32` internally for calculations to avoid overflow.
/// * The final results are converted back to `i1.15` for consistency.
pub fn rotate_sincos(source: (i16, i16), offset: (i16, i16)) -> (i16, i16) {
    // Convert input types to i32 to avoid overflow during calculations.
    let source_sin: i32 = source.0 as i32;
    let source_cos: i32 = source.1 as i32;
    let offset_sin: i32 = offset.0 as i32;
    let offset_cos: i32 = offset.1 as i32;

    // Calculate the sine component of the rotated vector:
    // out_sin = source_sin * offset_cos + source_cos * offset_sin
    let out_sin: i32 = source_sin * offset_cos + source_cos * offset_sin;

    // Calculate the cosine component of the rotated vector:
    // out_cos = source_cos * offset_cos - source_sin * offset_sin
    let out_cos: i32 = source_cos * offset_cos - source_sin * offset_sin;

    // Scale back to `i16` by shifting right 15 bits
    let out_sin = (out_sin >> 15) as i16;
    let out_cos = (out_cos >> 15) as i16;

    // Return the rotated sine and cosine components
    (out_sin, out_cos)
}