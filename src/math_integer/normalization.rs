/// Converts normalized value to milliamps, millivolts, etc.
/// 
/// # Arguments
/// * `value_norm` - The normalized value [i16]
/// * `full_scale` - The maximum full scale value in milliamps, millivolts, etc. (only positive range) [i32] 
/// 
/// # Returns
/// The value in milliamps, millivolts, etc. [i32]
pub const fn norm_to_value(value_norm: i16, full_scale: i32) -> i32 {
    const BIT_SHIFT: u8 = 9;  // Define bit shift for resolution enhancement
    
    // Calculate scaling factor to prevent overflow
    let scale = full_scale >> (15 - BIT_SHIFT);
    
    // Scale and shift the normalized value to the target unit
    ((value_norm as i32 * scale) >> BIT_SHIFT) as i32
}

/// Converts milliamps, millivolts, etc. to normalized value.
/// 
/// # Arguments
/// * `value` - The value in milliamps, millivolts, etc. [i32]
/// * `full_scale` - The maximum full scale value in milliamps, millivolts, etc. (only positive range) [i32]
/// 
/// # Returns
/// The normalized value [i16]
pub const fn value_to_norm(value: i32, full_scale: i32) -> i16 {
    const BIT_SHIFT: u8 = 9;  // Define bit shift for resolution enhancement
    
    // Calculate scaling factor to prevent overflow
    let scale = full_scale >> (15 - BIT_SHIFT);
    
    // Shift and scale the value to normalized range
    ((value << BIT_SHIFT) / scale) as i16
}
