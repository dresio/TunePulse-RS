// Implements the motor PWM control module, handling motor and phase selection,
// and managing PWM patterns for different motor types.

// Key Features:
// - Defines PhasePattern enum for different PWM patterns
// - Defines MotorType enum for various motor types
// - Implements MotorPWM struct to manage motor and phase selectors
// - Provides methods to update motor control and change motor or phase modes

// Detailed Operation:
// The motor_pwm module manages PWM signals for different motor types using MotorSelector and PhaseSelector.
// It defines enums for PhasePattern and MotorType to represent different configurations.
// The MotorPWM struct initializes selectors based on motor type and phase pattern,
// and provides methods to update PWM signals based on input voltages or angles.
// It uses mathematical transformations for voltage calculations and allows dynamic changing
// of motor and phase modes.

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

mod motor_selector; // Imports the motor_selector module
mod phase_selector; // Imports the phase_selector module

use motor_selector::MotorSelector; // Imports the MotorSelector struct from motor_selector module
use phase_selector::PhaseSelector; // Imports the PhaseSelector struct from phase_selector module

use crate::math_integer::clarke_transform::inverse_clarke_transform; // Imports inverse_clarke_transform from clarke_transform module
use crate::math_integer::trigonometry as math; // Imports trigonometry module as math

use super::calibration::angle_calibrator::AngleCalibrator;
use super::{ControlMode, DriverStatus, Motor, MotorDriver, MotorType, PhasePattern};

pub struct DriverPWM {
    // COMMON
    /// Duty of brake mode
    brake: i16,

    /// Selector for motor types
    motor_type: MotorSelector,
    /// Selector for phase patterns
    phase_sel: PhaseSelector,

    control_mode: ControlMode,

    calibrator: AngleCalibrator,

    status: DriverStatus,

    // ####### Related to step-dir driver ########
    /// Motor resistance
    pub angle: i16,
    /// Motor resistance
    current: i16,

    /// Motor rotation direction
    pub direction: isize,

    ch_1234: [i16; 4],
}

impl DriverPWM {
    #[inline(always)]
    fn normal_run(&mut self, voltage_ab: (i16, i16), supply: i16) -> (i16, i16) {
        match self.control_mode {
            ControlMode::CurrentAB => {
                let sincos_ab = math::angle2sincos(voltage_ab.0); // Converts angle to sine and cosine voltages
                math::scale_sincos(sincos_ab, voltage_ab.1) // Scales sine and cosine voltages based on input
            }
            ControlMode::VoltageAB => voltage_ab,
        }
    }
}

impl MotorDriver for DriverPWM {
    fn new(motor: Motor, control_mode: ControlMode) -> DriverPWM {
        // Create a calibrator (adjust if there is a different constructor)
        let calibrator = AngleCalibrator::new(1);

        DriverPWM {
            brake: 0,
            calibrator,
            angle: 0,
            current: 0,
            direction: motor.direction,
            control_mode,
            status: DriverStatus::Ready,
            motor_type: MotorSelector::new(motor.pole_type), // Initializes motor selector with motor type
            phase_sel: PhaseSelector::new(motor.connection), // Initializes phase selector with phase pattern
            ch_1234: [0; 4],
        }
    }

    fn tick(&mut self, voltage_ab: (i16, i16), supply: i16, current: [u16; 4]) -> [i16; 4] {
        let voltage_ab = match self.status {
            DriverStatus::Ready => voltage_ab,
            DriverStatus::Error => (0, 0),
            DriverStatus::Calibrating => (0, 0),
        };
        let voltage_ab = self.normal_run(voltage_ab, supply);
        let motor_voltages = self.motor_type.tick(voltage_ab, supply);
        self.ch_1234 = self.phase_sel.tick(motor_voltages);
        self.ch_1234
    }

    fn calibrate(&mut self) -> bool {
        self.status = DriverStatus::Calibrating;
        // self.calibrator.calibrate()
        false
    }

    fn enable(&mut self, flag: bool) {
        // Just store the flag in an internal field
        // If DriverPWM does not have it yet, add a `enabled: bool` field.
        // self.enabled = flag;
    }

    fn is_ready(&self) -> bool {
        matches!(self.status, DriverStatus::Ready) // Returns true if Ready
    }

    fn get_current(&mut self) -> (i16, i16) {
        // Return AB current for PWM driver
        (self.current, 0)
    }

    #[inline(always)]
    fn change_motor_mode(&mut self, motor_type: MotorType) -> bool {
        self.motor_type.change_mode(motor_type); // Updates motor selector with new motor type
        true
    }

    /// Changes the phase pattern mode to the specified connection
    #[inline(always)]
    fn change_phase_mode(&mut self, connection: PhasePattern) -> bool {
        self.phase_sel.change_mode(connection as u8); // Updates phase selector with new phase pattern
        true
    }

    fn change_control_mode(&mut self, mode: ControlMode) -> bool {
        // If no field for control_mode, add it to DriverPWM struct and update here
        self.control_mode = mode;
        true
    }

    fn get_output(&self) -> [i16; 4] {
        self.ch_1234
    }
}
