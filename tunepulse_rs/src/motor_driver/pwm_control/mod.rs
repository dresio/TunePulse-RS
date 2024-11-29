pub mod motor_selector;

pub mod phase_selector;

use crate::math_integer::clarke_transform as math;


/// Enum for PhasePattern representing different PWM patterns
#[derive(Debug, Clone, Copy)]
pub enum PhasePattern {
    ABCD = 0b11100100, // Pattern 0: {0, 1, 2, 3}
    ACDB = 0b01111000, // Pattern 1: {0, 2, 3, 1}
    ADBC = 0b10011100, // Pattern 2: {0, 3, 1, 2}
    DCAB = 0b01001011, // Pattern 3: {3, 2, 0, 1}
}


#[derive(Debug, Clone, Copy)]
// Enumeration for motor types
pub enum MotorType {
    UNDEFINED, // No motor type selected
    DC,        // Direct Current motor
    STEPPER,   // Stepper motor
    BLDC,      // Brushless DC motor
}
