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
    fn mode_check(&mut self, ab: (i16, i16)) -> (i16, i16) {
        match self.control_mode {
            ControlMode::CurrentAB => ab,
            ControlMode::VoltageAB => (0, 0),
        }
    }
}

impl MotorDriver for DriverPulse {
    fn new(motor: Motor, control_mode: ControlMode) -> DriverPulse {
        DriverPulse {
            enable: 0,
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

    fn tick_control(&mut self, ab_inpt: (i16, i16), supply: i16) -> [i16; 4] {
        let voltage_ab = match self.status {
            DriverStatus::Ready => ab_inpt,
            DriverStatus::Error => (0, 0),
            DriverStatus::Calibrating => (0, 0),
        };
        let current_ab = self.mode_check(voltage_ab);
        let pulse = self.angle2pulse.tick(current_ab.0);
        self.ch_1234 = [self.enable, pulse.0 as i16, pulse.1 as i16, voltage_ab.1];
        self.ch_1234
    }

    fn tick_current(&mut self, current: [i16; 4]) -> (i16, i16) {
        (0, 0)
    }

    fn calibrate(&mut self) -> bool {
        self.status = DriverStatus::Calibrating;
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

    fn get_control(&self) -> [i16; 4] {
        self.ch_1234
    }
}
