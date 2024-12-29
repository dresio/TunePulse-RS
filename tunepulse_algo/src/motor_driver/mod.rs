pub mod driver_pulse; // Module handling pulse-related logic
pub mod driver_pwm; // Module handling PWM-related logic

pub mod calibration;
pub use calibration::angle_calibrator::AngleCalibrator;
pub use driver_pwm::DriverPWM;

pub struct Motor {
    /// Motor pole count
    pub pole_count: usize,
    /// Motor pole type (DC/BLDC/STEPPER)
    pub pole_type: MotorType,
    /// Motor connection type (ABCD/DBAC/etc)
    pub connection: PhasePattern,
    /// Direction of rotation
    pub direction: isize,
    /// Resistance of the motor
    pub resistance: i32,
    /// Inductance of the motor
    pub inductance: i32,
    /// Maximum allowed current for motor (optional)
    pub max_current: i32,
}

impl Motor {
    pub fn new(resistance: i32) -> Motor {
        let resistance = if resistance <= 0 { 1 } else { resistance };
        Motor {
            pole_count: 1,
            pole_type: MotorType::UNDEFINED,
            connection: PhasePattern::NONE,
            direction: 0,
            resistance,
            inductance: 1,
            max_current: 1,
        }
    }
}

/// MotorType enumeration with predefined motor types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum MotorType {
    UNDEFINED = 1,
    DC = u16::MAX as u32,
    BLDC = 3,
    STEP = 4,
}

/// PhasePattern enumeration for PWM patterns
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhasePattern {
    ABCD = 0b11100100,
    ACDB = 0b01111000,
    ADBC = 0b10011100,
    DCAB = 0b01001011,
    NONE = 0b00000000,
}

/// PhasePattern enumeration for PWM patterns
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    /// As Sin and Cos in alpha-beta coordinate system
    VoltageAB,
    /// As angle of current + DC current amplitude
    CurrentAB,
}

/// Represents the motor's overall calibration status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverStatus {
    /// Motor is currently undergoing calibration.
    Calibrating,
    /// Motor calibration completed successfully and ready for normal operation.
    Ready,
    /// An error occurred during calibration or normal operation.
    Error,
}

/// Common interface for motor drivers
pub trait MotorDriver {
    /// Constructor for new driver
    fn new(motor: Motor, control_mode: ControlMode) -> Self;

    /// Updates motor control based on the current mode and input voltages
    fn tick_control(&mut self, ab_inpt: (i16, i16), supply: i16) -> [i16; 4];

    /// Updates motor control based on the current mode and input voltages
    fn tick_current(&mut self, current: [i16; 4]) -> (i16, i16);

    /// Run calibration cycle
    fn calibrate(&mut self) -> bool;

    /// Enable or disable motor
    fn enable(&mut self, flag: bool);

    /// Calibration is done and everything works well without errors
    fn is_ready(&self) -> bool;

    /// Will return ab current if PWM driver and 0 if Pulse driver
    fn get_current(&mut self) -> (i16, i16);

    fn get_control(&self) -> [i16; 4];

    /// Changes the motor type mode
    fn change_motor_mode(&mut self, motor_type: MotorType) -> bool;

    /// Changes the phase pattern mode
    fn change_phase_mode(&mut self, phase_seq: PhasePattern) -> bool;

    /// Changes the phase pattern mode
    fn change_control_mode(&mut self, mode: ControlMode) -> bool;
}
