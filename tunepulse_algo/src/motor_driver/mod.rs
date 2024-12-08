use defmt_rtt as _;  // Use the defmt_rtt crate for logging via RTT (Real-Time Transfer)
use pwm_control::MotorPWM;  // Import MotorPWM struct from the pwm_control module

pub mod pulse_control; // Module handling pulse-related logic
pub mod pwm_control;   // Module handling PWM-related logic

/// Represents the motor's overall calibration status.
enum MotorStatus {
    /// Motor is currently undergoing calibration.
    Calibrating,
    /// Motor calibration completed successfully and ready for normal operation.
    Ready,
    /// An error occurred during calibration or normal operation.
    Error,
}

/// Represents the current stage of the calibration process.
enum CalStage {
    Setup,          // Initial setup stage for calibration
    Settle,         // Stage where the motor is allowed to settle before starting calibration steps
    /// Test the motor's ability to respond linearly and consistently by performing a few test steps.
    FirstStep,
    SearchZero,     // Stage where the system searches for a reference "zero" position
    /// Perform the full 200-step calibration cycle to map the motor's electrical angle to its mechanical position in the clockwise direction.
    FullRotationCW,
    /// Perform the full 200-step calibration cycle to map the motor's electrical angle to its mechanical position in the counterclockwise direction.
    FullRotationCCW,
}

/// Represents the state within each calibration cycle quarter:
/// (Setup -> Rotating -> Waiting -> Sampling)
#[derive(Copy, Clone)]
enum CalSamplingState {
    /// Initial setup state before starting rotation and sampling.
    Setup,
    /// Motor is rotating during a calibration step.
    Rotating,
    /// Motor is waiting (stationary) for a specified settling time.
    Waiting,
    /// Motor is sampling (oversampling the encoder position) for averaging.
    Sampling,
}

/// The main driver struct for the motor, holding all the state required for operation and calibration.
pub struct MotorDriver {
    motor: MotorPWM,   // Motor interface using PWM signals for control
    frequency: u16,    // Update frequency (ticks per second)
    pwm: [i16; 4],     // Current PWM output sent to the motor
    pub position: i32, // Current encoder position reading

    motor_status: MotorStatus,   // Current motor status (Calibrating, Ready, or Error)
    calibration_stage: CalStage, // Current stage of the overall calibration process
    cal_cycle_stage: CalSamplingState, // Current sub-stage of the calibration cycle (Setup, Rotating, Waiting, Sampling)

    angle_el: u16,        // Electrical angle of the motor (0..65535), used to control phase
    cal_idx: u16,         // Index for counting steps during calibration cycles
    data: [i32; 200],     // Array for storing sampled encoder data during full calibration
    oversampled_pos: i32, // Accumulator for averaging positions during oversampling (Sampling stage)

    time_in_state: u16,   // Counter for how many ticks remain in the current calibration sub-stage

    amplitude: i16, // Amplitude (voltage magnitude) used during calibration
    cal_step: u16,  // Defines how many steps to move in one sub-cycle
    direction: i16, // Current rotation direction (1 for forward, -1 for backward)
    speed: i16,     // Speed (steps per tick) during calibration

    init_pos: i32,    // Position recorded at the start of a calibration step
    temp_pos: i32,    // Temporary position for measuring movement increments
    cal_dif_max: i32, // Maximum difference in step measurement for consistency checks
    cal_dif_min: i32, // Minimum difference in step measurement for consistency checks
}

// Constants used during calibration
impl MotorDriver {
    const CAL_OVERSEMPLING: u16 = 64;    // Number of samples per oversampling period for averaging
    const CAL_SETTLING_TIME: u16 = 256;  // Number of ticks to wait for the motor to settle before sampling
    const CAL_SPEED: i16 = 32;           // Speed used during calibration steps (angle increments per tick)
    const CAL_FIRST_STEP_USTEPS: u16 = 16;
    //---------------------------------------------------------
    // Description of the Calibration Algorithm and Steps:
    //
    // The calibration algorithm involves several stages:
    // 1. Settle Stage: Let the motor stand still and record an initial position. This ensures that any mechanical slack or noise settles.
    // 2. Setup Stage: Prepare for testing motor responsiveness.
    // 3. FirstStep Stage: Move the motor a set number of small steps to test linearity and consistency. Verify that each step is uniform.
    //    - If the steps are inconsistent, report an error.
    //    - If consistent, determine the direction and proceed.
    // 4. SearchZero Stage: Move the motor until it reaches a reference zero point (e.g., crossing zero position).
    // 5. FullRotationCW Stage: Rotate the motor through a full calibrated range (e.g., a full electrical rotation) in the clockwise direction,
    //    collecting data at discrete intervals.
    // 6. FullRotationCCW Stage: Rotate back through the full range in the counterclockwise direction, also collecting data.
    //
    // Each calibration cycle (e.g., each quarter turn or incremental step) follows these internal sub-states:
    // - Setup: Initialize counters and accumulators for the cycle.
    // - Rotating: Rotate the motor at a specified speed and direction for a defined number of steps.
    // - Waiting: Pause to allow the system to stabilize after rotation.
    // - Sampling: Perform oversampling of the encoder position multiple times to get a stable averaged position reading.
    //
    // After completing these stages and verifying consistency, the motor is deemed calibrated and ready for normal operation.
    //---------------------------------------------------------


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
            calibration_stage: CalStage::Settle,    // Begin with the Settle stage
            cal_cycle_stage: CalSamplingState::Setup, // Initialize the calibration cycle state to Setup

            angle_el: 0, // Initial electrical angle is 0
            cal_idx: 0,  // Start index at 0

            data: [0; 200],     // Data array for storing calibration samples, initialized to 0
            oversampled_pos: 0, // Oversampling accumulator is initially 0
            time_in_state: 0,   // No time spent in current state initially

            pwm: [0; 4],     // Initialize PWM outputs to zero
            amplitude: 6400, // Initial amplitude used during calibration, arbitrary chosen value

            cal_step: 0,          // Initialize calibration steps counter
            direction: 0,         // No direction initially
            speed: Self::CAL_SPEED, // Use the predefined calibration speed

            init_pos: 0,          // Initial position placeholder
            temp_pos: 0,          // Temporary position placeholder
            cal_dif_max: i32::MIN,// Initialize to very small number for comparison
            cal_dif_min: i32::MAX,// Initialize to very large number for comparison
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
            _ => {
                // If still calibrating, run the calibration logic
                self.tick_calibrate();
            }
        }

        // Compute the PWM signals based on the current angle_el and amplitude
        self.pwm = self.motor.tick_angle((self.angle_el as i16, self.amplitude));
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
        self.amplitude = _voltg_angle.1;       // Update the amplitude for normal operation
    }

    //---------------------------------------------------------
    // tick_calibrate() Method Steps:
    //
    // 1. Calls run_calibration_cycle() to perform one cycle of movement, waiting, and sampling.
    // 2. If the cycle returns a stable position (not i32::MIN), proceed with logic depending on the current calibration stage.
    // 3. Move through the defined stages of calibration (Settle, Setup, FirstStep, SearchZero, FullRotationCW, FullRotationCCW).
    // 4. Validate motion consistency, detect errors, and finally transition to MotorStatus::Ready.
    //---------------------------------------------------------

    /// High-level calibration method.
    ///
    /// This drives the calibration through its main stages, calling `run_calibration_cycle()`
    /// and then handling the transitions between calibration steps.
    fn tick_calibrate(&mut self) {
        let stable_pos = self.run_calibration_cycle(self.cal_step); // Perform a calibration cycle and get stable position

        if stable_pos != i32::MIN {
            // If we have a valid stable position reading:
            match self.calibration_stage {
                CalStage::Settle => {
                    // After settling, move to the Setup stage
                    self.cal_idx = 10; // Arbitrary index setting for demonstration
                    self.calibration_stage = CalStage::Setup;
                    return;
                }

                CalStage::Setup => {
                    // Prepare to test the motor's motion
                    if Self::iter(&mut self.cal_idx) {
                        // If cal_idx reached 0, record initial positions
                        self.init_pos = stable_pos;
                        self.temp_pos = stable_pos;

                        // Reset difference tracking
                        self.cal_dif_max = i32::MIN;
                        self.cal_dif_min = i32::MAX;

                        // Set up for the FirstStep stage (16 steps)
                        self.cal_step = u16::MAX / Self::CAL_FIRST_STEP_USTEPS;
                        self.cal_idx = Self::CAL_FIRST_STEP_USTEPS;
                        self.calibration_stage = CalStage::FirstStep;
                        defmt::println!("Calibration: TEST MOTION");
                    }
                }

                CalStage::FirstStep => {
                    // Check the difference in position after each step to ensure consistency
                    let dif = self.temp_pos - stable_pos;
                    self.temp_pos = stable_pos;
                    self.cal_dif_min = self.cal_dif_min.min(dif);
                    self.cal_dif_max = self.cal_dif_max.max(dif);

                    if Self::iter(&mut self.cal_idx) {
                        // After completing all test steps, analyze results
                        let travel = self.init_pos - self.temp_pos; // Total travel during test
                        let direction = travel.signum(); // Determine direction of motion
                        self.direction = direction as i16;

                        // Average step size
                        let avg_step = (travel * direction) / Self::CAL_FIRST_STEP_USTEPS as i32;
                        let diviation = self.cal_dif_max - self.cal_dif_min;

                        if avg_step < diviation * 2 {
                            // If the variation is too large, calibration fails
                            defmt::println!("Calibration: ERROR");
                            self.motor_status = MotorStatus::Error;
                            return;
                        }

                        // Proceed with a known direction
                        defmt::println!("Calibration: DIRECTION: {}", direction);

                        // Prepare for the SearchZero stage
                        self.cal_idx = 0;
                        self.cal_step = u16::MAX / 4;
                        self.speed = Self::CAL_SPEED * self.direction; // Move at a calibrated speed

                        defmt::println!("Calibration: GO TO ZERO");
                        self.calibration_stage = CalStage::SearchZero;
                    }
                }

                CalStage::SearchZero => {
                    // Move until crossing zero
                    if stable_pos < 0 {
                        // Once zero is found, start full rotation calibration in CW direction
                        self.calibration_stage = CalStage::FullRotationCW;
                        self.speed = -self.speed; // Adjust speed direction
                        self.cal_idx = 0;
                        defmt::println!("Calibration: GO CALIBRATION RUN CW");
                    }
                }

                CalStage::FullRotationCW => {
                    // Perform a full rotation in CW direction
                    if stable_pos > u16::MAX as i32 {
                        // Once we exceed the maximum range, switch to CCW run
                        self.calibration_stage = CalStage::FullRotationCCW;
                        defmt::println!("Calibration: CW POLE COUNT: {}", self.cal_idx / 4);
                        defmt::println!("Calibration: GO CALIBRATION RUN CCW");
                        self.cal_idx = 0;
                        self.speed = -self.speed;
                        return;
                    }
                    // Increment index as we collect data (data storage commented out)
                    self.cal_idx += 1;
                }

                CalStage::FullRotationCCW => {
                    // Perform a full rotation in CCW direction
                    if stable_pos < 0 {
                        // Once we return to zero, calibration is complete
                        self.motor_status = MotorStatus::Ready;
                        defmt::println!("Calibration: CCW POLE COUNT: {}", self.cal_idx / 4);
                        defmt::println!("NORMAL RUN INITIATED");
                        return;
                    }
                    self.cal_idx += 1;
                    // TODO: handle data storing if needed
                }
            }
        }
    }

    //---------------------------------------------------------
    // run_calibration_cycle() Method Steps:
    //
    // 1. Depending on the cal_cycle_stage, perform:
    //    - Setup: Initialize sampling and timing counters.
    //    - Rotating: Rotate the motor at a given speed for a defined time.
    //    - Waiting: Let the motor stand still for settling time.
    //    - Sampling: Collect oversampled encoder values and average them.
    //
    // 2. Return i32::MIN until the full cycle (including Sampling) completes.
    // 3. Once Sampling is complete, return the averaged stable position.
    //---------------------------------------------------------

    /// Executes one complete cycle of moving, waiting, and sampling for calibration.
    ///
    /// # Arguments
    /// * `steps` - Number of steps to be used in rotation calculation.
    ///
    /// Returns `i32::MIN` if the cycle is not complete yet.
    /// Returns the averaged stable position once sampling is done.
    fn run_calibration_cycle(&mut self, steps: u16) -> i32 {
        match self.cal_cycle_stage {
            CalSamplingState::Setup => {
                // Prepare for a new calibration cycle
                self.oversampled_pos = 0;           // Reset oversampling accumulator
                self.cal_cycle_stage = CalSamplingState::Rotating; // Next state: Rotating
                self.time_in_state = steps / self.speed.abs() as u16; // Calculate how long to rotate
                i32::MIN // Not finished yet
            }

            CalSamplingState::Rotating => {
                // Rotate the motor for the specified time
                self.move_at_speed(self.speed); // Advance angle by 'speed' each tick

                if Self::iter(&mut self.time_in_state) {
                    // Once done rotating, move to Waiting state
                    self.cal_cycle_stage = CalSamplingState::Waiting;
                    self.time_in_state = Self::CAL_SETTLING_TIME; // Settle time
                }
                i32::MIN // Still not finished
            }

            CalSamplingState::Waiting => {
                // Motor is now stationary, waiting
                if Self::iter(&mut self.time_in_state) {
                    // After waiting, move to Sampling state
                    self.cal_cycle_stage = CalSamplingState::Sampling;
                    self.time_in_state = Self::CAL_OVERSEMPLING; // Perform oversampling
                }
                i32::MIN // Still waiting
            }

            CalSamplingState::Sampling => {
                // Oversample the encoder position to get a stable reading
                if Self::cal_oversampling(self.position, &mut self.time_in_state, &mut self.oversampled_pos) {
                    // Once oversampling is complete:
                    self.cal_cycle_stage = CalSamplingState::Setup; // Reset cycle stage
                    return self.oversampled_pos; // Return the averaged stable position
                }
                i32::MIN // Still sampling
            }
        }
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

    /// Get the calibration data array for analysis.
    pub fn get_data(&self) -> &[i32; 200] {
        &self.data // Return reference to data array
    }

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

    //---------------------------------------------------------
    // cal_oversampling() Method Steps:
    //
    // 1. Accumulate the current position into an integral.
    // 2. Decrement the iteration counter each time.
    // 3. When iterations reach zero, compute the average by dividing the integral by CAL_OVERSEMPLING.
    // 4. Return true when done, false otherwise.
    //---------------------------------------------------------

    /// Inline method for oversampling the encoder position data.
    ///
    /// # Arguments
    /// * `position` - current encoder position
    /// * `iter` - remaining iterations of oversampling
    /// * `integral` - accumulator for summing positions
    ///
    /// Returns true when oversampling completes, false otherwise.
    #[inline(always)]
    fn cal_oversampling(position: i32, iter: &mut u16, integral: &mut i32) -> bool {
        *integral += position; // Add current position to the integral
        *iter -= 1;            // Decrease iteration count
        if *iter == 0 {
            // If no more iterations left, finalize averaging
            *integral = *integral / Self::CAL_OVERSEMPLING as i32; // Compute the average
            return true; // Oversampling complete
        }
        false // Still sampling
    }

    //---------------------------------------------------------
    // iter() Method Steps:
    //
    // 1. Decrement the counter if it is greater than 0.
    // 2. Return false while counting down.
    // 3. Return true when the counter reaches zero.
    //---------------------------------------------------------

    /// Inline method to decrement a counter until it reaches zero.
    ///
    /// Returns:
    /// * `false` while counter > 0
    /// * `true` when the counter reaches 0
    #[inline(always)]
    fn iter(counter: &mut u16) -> bool {
        if *counter > 0 {
            *counter -= 1; // Decrement the counter
            false // Still have ticks left
        } else {
            true // Counter reached zero
        }
    }
}
