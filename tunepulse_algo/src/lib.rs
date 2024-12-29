#![no_std]

pub mod inputs_dump;
use inputs_dump::DataInputs;

pub mod math_integer;
pub mod motor_driver;

pub mod analog;

use defmt_rtt as _; // Use the defmt_rtt crate for logging via RTT (Real-Time Transfer)

use motor_driver::{
    AngleCalibrator, ControlMode, DriverPWM, DriverStatus, Motor, MotorDriver, MotorType,
    PhasePattern,
};

use crate::math_integer::filters::lpf::FilterLPF;
use crate::math_integer::motion::position_integrator::Position;

use analog::supply_voltage::SupplyVoltage;

/// The main driver struct for the motor, holding all the state required for operation and calibration.
pub struct MotorController {
    motor: DriverPWM,   // Motor interface using PWM signals for control
    frequency: u16,     // Update frequency (ticks per second)
    position: Position, // Current encoder position reading

    driver_status: DriverStatus, // Current motor status (Calibrating, Ready, or Error)

    angle_el: u16,  // Electrical angle of the motor (0..65535), used to control phase
    amplitude: i16, // Amplitude (voltage magnitude) used during calibration
    direction: i16, // Current rotation direction (1 for forward, -1 for backward)
    speed: i16,     // Speed (steps per tick) during calibration

    angle_calibrator: AngleCalibrator,
    filter: FilterLPF,
    supply: SupplyVoltage,
    ticker: i32,
    sup_check: usize,
}

// Constants used during calibration
impl MotorController {
    /// Create a new MotorDriver instance.
    ///
    /// # Arguments
    /// * `motor` - Motor type configuration
    /// * `connection` - Phase pattern configuration
    /// * `frequency` - Number of ticks per second
    pub fn new(
        motor_type: MotorType,
        connection: PhasePattern,
        frequency: u16,
        max_sup_voltage: i32,
        resistance: i32,
    ) -> Self {
        let mut motor = Motor::new(resistance);
        motor.pole_type = motor_type;
        motor.connection = connection;
        let control_mode = ControlMode::CurrentAB;

        Self {
            motor: DriverPWM::new(motor, control_mode), // Initialize MotorPWM with given type and phase connection
            frequency,                                  // Store the update frequency
            position: Position::new(),                  // Initialize encoder position to 0

            driver_status: DriverStatus::Calibrating, // Start in Calibrating mode

            angle_el: 0, // Initial electrical angle is 0

            amplitude: 0,

            direction: 0, // No direction initially
            speed: 0,     // Use the predefined calibration speed

            angle_calibrator: AngleCalibrator::new(frequency),
            filter: FilterLPF::new(0, 0),

            supply: SupplyVoltage::new(200, max_sup_voltage),
            ticker: 0,
            sup_check: 100,
        }
    }

    /// Main update method.
    ///
    /// # Arguments
    /// * `voltg_angle` - (angle, amplitude) tuple for normal operation
    /// * `encoder_pos` - current encoder position from the sensor
    ///
    /// This method decides whether to run normal operation or calibration logic based on the motor status.
    pub fn tick(&mut self, current: i32, input: DataInputs) -> [i16; 4] {
        self.position.tick(input.angle_raw); // Update the internal position from the sensor
        let sup_adc = self.supply.tick(input.supply_adc).voltage_norm();
        self.amplitude = current as i16; // ma
                                         // let sup_adc = self.supply.voltage_norm();
        match self.driver_status {
            DriverStatus::Ready => {
                self.ticker += 1;

                // If calibration is complete, run normal operation logic
                let filtered_pos = self.filter.tick(self.position.angle());

                self.angle_el = self.angle_calibrator.get_correction(filtered_pos).1;
            }
            DriverStatus::Error => {
                // If in error state, stop driving the motor by setting amplitude to 0
                self.amplitude = 0;
            }
            DriverStatus::Calibrating => {
                if self.sup_check > 0 {
                    self.sup_check -= 1;
                    if self.sup_check == 0 {
                        if self.supply.voltage_mv() < 8000 {
                            defmt::warn!(
                                "SUPPLY is not enough: {}mV while at least 8000mV is needed",
                                self.supply.voltage_mv());
                        } else {
                            defmt::info!("SUPPLY is OK: {}mV", self.supply.voltage_mv());
                        };
                    };
                };
                // If still calibrating, run the calibration logic
                self.angle_el = self.angle_calibrator.tick(self.position.position());
                if self.angle_calibrator.is_ready() {
                    self.driver_status = DriverStatus::Ready
                }
            }
        }

        // Compute the PWM signals based on the current angle_el and amplitude
        self.motor
            .tick_control((self.angle_el as i16, self.amplitude), sup_adc)
    }

    /// Change the motor type mode.
    #[inline(always)]
    pub fn change_motor_mode(&mut self, motor: MotorType) {
        self.motor.change_motor_mode(motor); // Delegate to motor instance
    }

    /// Change the phase pattern mode.
    #[inline(always)]
    pub fn change_phase_mode(&mut self, connection: PhasePattern) {
        self.motor.change_phase_mode(connection); // Delegate to motor instance
    }

    /// Get current PWM signals.
    #[inline(always)]
    pub fn get_pwm(&mut self) -> [i16; 4] {
        self.motor.get_control()
    }
}
