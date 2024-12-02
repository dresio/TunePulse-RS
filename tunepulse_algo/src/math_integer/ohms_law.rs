/// Calculate current in milliamps (mA) from voltage (mV) and resistance (mΩ).
/// 
/// # Arguments
/// * `voltage_mv` - The voltage in millivolts [i32]
/// * `resistance_mohm` - The resistance in milliohms [i32]
/// 
/// # Returns
/// The current in milliamps [i32]
pub const fn current(voltage_mv: i32, resistance_mohm: i32) -> i32 {
    // I = V / R, ensuring we prevent division by zero
    if resistance_mohm == 0 {
        0
    } else {
        (voltage_mv * 1000) / resistance_mohm
    }
}

/// Calculate voltage in millivolts (mV) from current (mA) and resistance (mΩ).
/// 
/// # Arguments
/// * `current_ma` - The current in milliamps [i32]
/// * `resistance_mohm` - The resistance in milliohms [i32]
/// 
/// # Returns
/// The voltage in millivolts [i32]
pub const fn voltage(current_ma: i32, resistance_mohm: i32) -> i32 {
    // V = I * R
    (current_ma * resistance_mohm) / 1000
}

/// Calculate resistance in milliohms (mΩ) from voltage (mV) and current (mA).
/// 
/// # Arguments
/// * `voltage_mv` - The voltage in millivolts [i32]
/// * `current_ma` - The current in milliamps [i32]
/// 
/// # Returns
/// The resistance in milliohms [i32]
pub const fn resistance(voltage_mv: i32, current_ma: i32) -> i32 {
    // R = V / I, ensuring we prevent division by zero
    if current_ma == 0 {
        0
    } else {
        (voltage_mv * 1000) / current_ma
    }
}

/// Calculate power in milliwatts (mW) from voltage (mV) and current (mA).
/// 
/// # Arguments
/// * `voltage_mv` - The voltage in millivolts [i32]
/// * `current_ma` - The current in milliamps [i32]
/// 
/// # Returns
/// The power in milliwatts [i32]
pub const fn power(voltage_mv: i32, current_ma: i32) -> i32 {
    // P = (V * I) / 1000
    (voltage_mv * current_ma) / 1000
}