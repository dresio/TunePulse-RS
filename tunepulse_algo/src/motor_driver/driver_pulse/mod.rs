pub mod angle2pulse;
use angle2pulse::Angle2Pulse;

use crate::math_integer::trigonometry as math; // Imports trigonometry module as math

use super::calibration::angle_calibrator::AngleCalibrator;
use super::{ControlMode, DriverStatus, Motor, MotorDriver, MotorType, PhasePattern};

pub struct DriverPulse {
    // COMMON
    /// Duty of brake mode
    enable: i16,
    motor: Motor,

    control_mode: ControlMode,

    calibrator: AngleCalibrator,

    status: DriverStatus,

    angle2pulse: Angle2Pulse,

    // ####### Related to step-dir driver ########
    /// Motor resistance
    pub angle: i16,
    /// Motor resistance
    current: i16,

    /// Motor rotation direction
    pub direction: isize,

    ch_1234: [i16; 4],
}

impl DriverPulse {
    #[inline(always)]
    fn normal_run(&mut self, voltage_ab: (i16, i16), supply: i16) -> (i16, i16) {
        match self.control_mode {
            ControlMode::CurrentAB => (0, 0),
            ControlMode::VoltageAB => (0, 0),
        }
    }
}

impl MotorDriver for DriverPulse {
    fn new(motor: Motor, control_mode: ControlMode) -> DriverPulse {
        // Create a calibrator (adjust if there is a different constructor)
        let calibrator = AngleCalibrator::new(1);
        DriverPulse {
            enable: 0,
            calibrator,
            angle: 0,
            current: 0,
            direction: motor.direction,
            motor,
            control_mode,
            status: DriverStatus::Ready,
            ch_1234: [0; 4],
            angle2pulse: Angle2Pulse::new(4),
        }
    }

    fn tick(&mut self, voltage_ab: (i16, i16), supply: i16, current: [u16; 4]) -> [i16; 4] {
        let voltage_ab = match self.status {
            DriverStatus::Ready => voltage_ab,
            DriverStatus::Error => (0, 0),
            DriverStatus::Calibrating => (0, 0),
        };
        let current_ab = self.normal_run(voltage_ab, supply);
        let pulse = self.angle2pulse.tick(current_ab.0);
        self.ch_1234 = [self.enable, pulse.0 as i16, pulse.1 as i16, voltage_ab.1];
        self.ch_1234
    }

    fn calibrate(&mut self) -> bool {
        self.status = DriverStatus::Calibrating;
        // self.calibrator.calibrate()
        false
    }

    fn enable(&mut self, flag: bool) {
        self.enable = flag as i16;
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
        self.motor.pole_type == motor_type
    }

    /// Changes the phase pattern mode to the specified connection
    #[inline(always)]
    fn change_phase_mode(&mut self, connection: PhasePattern) -> bool {
        self.motor.connection == connection // Updates phase selector with new phase pattern
    }

    fn change_control_mode(&mut self, mode: ControlMode) -> bool {
        // If no field for control_mode, add it to DriverPWM struct and update here
        self.control_mode == mode
    }

    fn get_output(&self) -> [i16; 4] {
        self.ch_1234
    }
}
