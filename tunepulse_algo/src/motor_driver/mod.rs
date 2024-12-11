use defmt_rtt as _; // Use the defmt_rtt crate for logging via RTT (Real-Time Transfer)
use pwm_control::MotorPWM; // Import MotorPWM struct from the pwm_control module

pub mod pulse_control; // Module handling pulse-related logic
pub mod pwm_control; // Module handling PWM-related logic

pub mod calibration;

use calibration::angle_calibrator::AngleCalibrator;

/// Represents the motor's overall calibration status.
enum MotorStatus {
    /// Motor is currently undergoing calibration.
    Calibrating,
    /// Motor calibration completed successfully and ready for normal operation.
    Ready,
    /// An error occurred during calibration or normal operation.
    Error,
}

/// The main driver struct for the motor, holding all the state required for operation and calibration.
pub struct MotorDriver {
    motor: MotorPWM,   // Motor interface using PWM signals for control
    frequency: u16,    // Update frequency (ticks per second)
    pwm: [i16; 4],     // Current PWM output sent to the motor
    pub position: i32, // Current encoder position reading

    motor_status: MotorStatus, // Current motor status (Calibrating, Ready, or Error)

    angle_el: u16, // Electrical angle of the motor (0..65535), used to control phase
    amplitude: i16, // Amplitude (voltage magnitude) used during calibration
    direction: i16, // Current rotation direction (1 for forward, -1 for backward)
    speed: i16,     // Speed (steps per tick) during calibration

    angle_calibrator: AngleCalibrator,
}

// Constants used during calibration
impl MotorDriver {
    const CAL_SPEED: i16 = 32; // Speed used during calibration steps (angle increments per tick)

    /// Create a new MotorDriver instance.
    ///
    /// # Arguments
    /// * `motor` - Motor type configuration
    /// * `connection` - Phase pattern configuration
    /// * `frequency` - Number of ticks per second
    pub fn new(
        motor: pwm_control::MotorType,
        connection: pwm_control::PhasePattern,
        frequency: u16,
    ) -> Self {
        Self {
            motor: MotorPWM::new(motor, connection), // Initialize MotorPWM with given type and phase connection
            frequency,                               // Store the update frequency
            position: 0,                             // Initialize encoder position to 0

            motor_status: MotorStatus::Calibrating, // Start in Calibrating mode

            angle_el: 0, // Initial electrical angle is 0

            pwm: [0; 4],     // Initialize PWM outputs to zero
            amplitude: 6400, // Initial amplitude used during calibration, arbitrary chosen value

            direction: 0,           // No direction initially
            speed: Self::CAL_SPEED, // Use the predefined calibration speed

            angle_calibrator: AngleCalibrator::new(frequency),
        }
    }

    //---------------------------------------------------------
    // tick() Method Steps:
    //
    // 1. Update the encoder position from the given input.
    // 2. If the motor is ready (calibration done), run normal logic via tick_run().
    // 3. If the motor is in error state, set amplitude to 0.
    // 4. Otherwise (still calibrating), run the calibration logic via tick_calibrate().
    // 5. After updating angle and amplitude based on the chosen logic, compute the final PWM signals.
    //---------------------------------------------------------

    /// Main update method.
    ///
    /// # Arguments
    /// * `voltg_angle` - (angle, amplitude) tuple for normal operation
    /// * `encoder_pos` - current encoder position from the sensor
    ///
    /// This method decides whether to run normal operation or calibration logic based on the motor status.
    pub fn tick(&mut self, voltg_angle: (i16, i16), encoder_pos: i32) -> [i16; 4] {
        self.position = encoder_pos; // Update the internal position from the sensor

        match self.motor_status {
            MotorStatus::Ready => {
                // If calibration is complete, run normal operation logic
                self.tick_run(voltg_angle);
            }
            MotorStatus::Error => {
                // If in error state, stop driving the motor by setting amplitude to 0
                self.amplitude = 0;
            }
            MotorStatus::Calibrating => {
                // If still calibrating, run the calibration logic
                self.angle_el = self.angle_calibrator.tick(encoder_pos);
                if self.angle_calibrator.is_ready() {
                    self.motor_status = MotorStatus::Ready
                }
            }
        }

        // Compute the PWM signals based on the current angle_el and amplitude
        self.pwm = self
            .motor
            .tick_angle((self.angle_el as i16, self.amplitude));
        self.pwm // Return the updated PWM array
    }

    //---------------------------------------------------------
    // tick_run() Method Steps:
    //
    // 1. When calibration is done, normal operation simply sets the motor angle and amplitude directly.
    // 2. The motor runs according to the external control inputs provided.
    //---------------------------------------------------------

    /// Normal operation method after calibration is complete.
    ///
    /// # Arguments
    /// * `_voltg_angle` - (angle, amplitude) for normal run mode
    fn tick_run(&mut self, _voltg_angle: (i16, i16)) {
        // self.angle_el = _voltg_angle.0 as u16; // Update the electrical angle for normal operation
        self.move_at_speed(_voltg_angle.0);
        self.amplitude = _voltg_angle.1; // Update the amplitude for normal operation
    }


    //---------------------------------------------------------
    // move_at_speed() Method Steps:
    //
    // 1. Adjust angle_el by a given increment each tick.
    // 2. Wrapping arithmetic ensures the angle stays within 0..65535.
    //---------------------------------------------------------

    /// Inline method for rotating the motor at a given increment per tick.
    ///
    /// # Arguments
    /// * `increment` - how much to add to angle_el per tick
    #[inline(always)]
    fn move_at_speed(&mut self, increment: i16) {
        let new_angle = self.angle_el.wrapping_add(increment as u16); // Wrap around if overflow
        self.angle_el = new_angle; // Update the motor's electrical angle
    }

    //---------------------------------------------------------
    // is_ready() Method Steps:
    //
    // 1. Simply returns true if the motor_status is Ready.
    //---------------------------------------------------------

    /// Check if calibration is complete (motor is ready for normal operation).
    pub fn is_ready(&self) -> bool {
        matches!(self.motor_status, MotorStatus::Ready) // Returns true if Ready
    }

    //---------------------------------------------------------
    // get_data() Method Steps:
    //
    // 1. Returns a reference to the calibration data array.
    //---------------------------------------------------------

    // /// Get the calibration data array for analysis.
    // pub fn get_data(&self) -> &[i32; Self::CAL_TABLE_SIZE] {
    //     &self.cal_table // Return reference to data array
    // }

    //---------------------------------------------------------
    // change_motor_mode() and change_phase_mode() Steps:
    //
    // 1. These methods allow changing the motor mode and phase pattern at runtime.
    // 2. They simply delegate the call to the MotorPWM instance.
    //---------------------------------------------------------

    /// Change the motor type mode.
    #[inline(always)]
    pub fn change_motor_mode(&mut self, motor: pwm_control::MotorType) {
        self.motor.change_motor_mode(motor); // Delegate to motor instance
    }

    /// Change the phase pattern mode.
    #[inline(always)]
    pub fn change_phase_mode(&mut self, connection: pwm_control::PhasePattern) {
        self.motor.change_phase_mode(connection); // Delegate to motor instance
    }

    //---------------------------------------------------------
    // get_pwm() Method Steps:
    //
    // 1. Returns the current PWM output array.
    //---------------------------------------------------------

    /// Get current PWM signals.
    #[inline(always)]
    pub fn get_pwm(&mut self) -> [i16; 4] {
        self.pwm // Return the current PWM array
    }
}
