mod motor_selector;

mod phase_selector;

use motor_selector::MotorSelector;
use phase_selector::PhaseSelector;

use crate::math_integer::clarke_transform::inverse_clarke_transform;
use crate::math_integer::trigonometry as math;

/// Enum for PhasePattern representing different PWM patterns
#[derive(Debug, Clone, Copy)]
pub enum PhasePattern {
    ABCD = 0b11100100, // Pattern 0: {0, 1, 2, 3}
    ACDB = 0b01111000, // Pattern 1: {0, 2, 3, 1}
    ADBC = 0b10011100, // Pattern 2: {0, 3, 1, 2}
    DCAB = 0b01001011, // Pattern 3: {3, 2, 0, 1}
}

// Enumeration for motor types with full steps amount per rotation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)] // Ensures the enum is represented as a u16
pub enum MotorType {
    UNDEFINED = 1,        // No motor type selected
    DC = u16::MAX as u32, // Direct Current motor - Crunch
    BLDC = 3,             // Brushless DC motor
    STEPPER = 4,          // Stepper motor
}

// Class to handle different types of motor controls
pub struct MotorPWM {
    motor_sel: MotorSelector,
    phase_sel: PhaseSelector,
}

impl MotorPWM {
    // Constructor for MotorPWM
    pub fn new(motor: MotorType, connection: PhasePattern) -> Self {
        MotorPWM {
            motor_sel: MotorSelector::new(motor),
            phase_sel: PhaseSelector::new(connection),
        }
    }

    // Function to update motor control based on mode
    pub fn tick(&mut self, voltg_ab: (i16, i16)) -> [i16; 4] {
        // Update the motor selector state
        let motor_pwm = self.motor_sel.tick(voltg_ab, 25000);
        // Update the phase selector state
        self.phase_sel.tick(motor_pwm)
    }

    pub fn tick_angle(&mut self, voltg_ang: (i16, i16)) -> [i16; 4] {
        let voltage_ab = math::angle2sincos(voltg_ang.0);
        let voltage_ab_scaled = math::scale_sincos(voltage_ab, voltg_ang.1);
        self.tick(voltage_ab_scaled)
    }

    #[inline(always)]
    pub fn change_motor_mode(&mut self, motor: MotorType) {
        self.motor_sel.change_mode(motor);
    }

    #[inline(always)]
    pub fn change_phase_mode(&mut self, connection: PhasePattern) {
        self.phase_sel.change_mode(connection as u8);
    }
}
