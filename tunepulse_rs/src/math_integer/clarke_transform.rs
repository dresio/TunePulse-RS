/// Performs the direct Clarke transform to calculate the `alpha` and `beta` components
/// from the phase values `a`, `b`, and `c`.
///
/// # Parameters
/// - `a`: Phase A component.
/// - `b`: Phase B component.
/// - `c`: Phase C component.
///
/// # Returns
/// A tuple `(alpha, beta)` representing the calculated components for the α and β axes.
#[inline]
pub fn direct_clarke_transform(a: i16, b: i16, c: i16) -> (i16, i16) {
    // Alpha component is calculated as phase A value.
    let alpha = a;

    let b = b as i32;
    let c = c as i32;
    // Beta component: (2 * V_B - V_A - V_C) / sqrt(3)
    // Using scaling with SQRT3DIV2 and a right shift to maintain precision.
    let beta = ((b - c) * self::SQRT3DIV2) >> 16;
    let beta = beta as i16;
    
    (alpha, beta)
}

/// Performs the inverse Clarke transform to calculate phase values (A, B, C)
/// from the `sin` and `cos` values. The function uses a precalculated
/// `SQRT3DIV2` constant, which is derived from √3/2 and scaled to an i16 format.
///
/// # Parameters
/// - `sin`: Alpha component of the value.
/// - `cos`: Beta component of the value.
///
/// # Returns
/// A tuple `(a, b, c)` representing the calculated phase values for A, B, and C.
#[inline]
pub fn inverse_clarke_transform(sin: i16, cos: i16) -> (i32, i32, i32) {
    let sin: i32 = sin as i32;
    let cos: i32 = cos as i32;

    // Convert beta value component to a scaled value using SQRT3DIV2
    let beta_sqrt3_div2: i32 = (self::SQRT3DIV2 * cos) >> 16;

    // Set phase A value to the alpha component
    let a: i32 = sin;

    // Calculate phase B value: -1/2 * V_alpha + sqrt(3)/2 * V_beta
    let b: i32 = -(sin >> 1) + beta_sqrt3_div2;

    // Calculate phase C value: -1/2 * V_alpha - sqrt(3)/2 * V_beta
    let c: i32 = -(sin >> 1) - beta_sqrt3_div2;

    (a, b, c)
}

/// Precalculated sqrt(3)/2
const SQRT3: f64 = 1.7320508075688772;
/// Precalculated scaling factor for sqrt(3) in i16 format
const SQRT3DIV2: i32 = (SQRT3 / 2.0f64 * (1u32 << 16) as f64) as i32;
