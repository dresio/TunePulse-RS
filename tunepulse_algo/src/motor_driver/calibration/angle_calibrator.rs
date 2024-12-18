use defmt_rtt as _; // Use the defmt_rtt crate for logging via RTT (Real-Time Transfer)

use super::CalibrationTable;

/// Represents the current stage of the calibration process.
enum CalStage {
    /// Test the motor's ability to respond linearly and consistently by performing a few test steps.
    Reset = 0, // Initial setup stage for calibration
    Setup = 1, // Stage where the motor is allowed to settle before starting calibration steps
    Pass0 = 2,
    Pass1 = 3, // First pass of the calibration
    Pass2 = 4, // Second pass of the calibration
    Check = 5, // State for verifying the calibration
    Ready = 6, // Calibration is complete and ready
    Error = 7, // An error occurred during calibration
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
pub struct AngleCalibrator {
    frequency: u16,    // Update frequency (ticks per second)
    pub position: i32, // Current encoder position reading

    calibration_stage: CalStage, // Current stage of the overall calibration process
    cal_cycle_stage: CalSamplingState, // Current sub-stage of the calibration cycle (Setup, Rotating, Waiting, Sampling)

    angle_el: u16,    // Electrical angle of the motor (0..65535), used to control phase
    ang_el_step: u16, // Defines how many steps to move in one sub-cycle

    cal_idx: usize, // Index for counting steps during calibration cycles
    // cal_table: [i32; Self::CAL_TABLE_SIZE], // Array for storing sampled encoder data during full calibration
    oversampled_pos: i32, // Accumulator for averaging positions during oversampling (Sampling stage)

    time_in_state: usize, // Counter for how many ticks remain in the current calibration sub-stage

    direction: isize, // Current rotation direction (1 for forward, -1 for backward)
    speed: isize,     // Speed (steps per tick) during calibration
    settling_time: usize, // Settling time in milliseconds

    init_pos: i32, // Position recorded at the start of a calibration step
    temp_pos: i32, // Temporary position for measuring movement increments
    dif_max: i32,  // Maximum difference in step measurement for consistency checks
    dif_min: i32,  // Minimum difference in step measurement for consistency checks

    cal_table: CalibrationTable<200>,
    el_step_idx: u16,
}

// Constants used during calibration
impl AngleCalibrator {
    const CAL_SETTLING_TIME_US: usize = 25000; // Settling time in milliseconds
    const CAL_SPEED_US: usize = 2500; // Speed in angle increments per millisecond

    const CAL_OVERSEMPLING: usize = 100; // Number of samples per oversampling period for averaging
    const CAL_FIRST_STEP_USTEPS: u16 = 16;
    const CAL_POINTS_PER_360EL: u16 = 4;

    //---------------------------------------------------------
    // Description of the Calibration Algorithm and Steps:
    //
    // The calibration algorithm involves several stages:
    // 1. Settle Stage: Let the motor stand still and record an initial position. This ensures that any mechanical slack or noise settles.
    // 2. Setup Stage: Prepare for testing motor responsiveness.
    // 3. FirstStep Stage: Move the motor a set number of small steps to test linearity and consistency. Verify that each step is uniform.
    //    - If the steps are inconsistent, report an error.
    //    - If consistent, determine the direction and proceed.
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
    pub fn new(frequency: u16) -> Self {
        let settling_time = Self::calculate_settling_time(frequency, Self::CAL_SETTLING_TIME_US);

        Self {
            frequency,   // Store the update frequency
            position: 0, // Initialize encoder position to 0

            calibration_stage: CalStage::Setup, // Begin with the Settle stage
            cal_cycle_stage: CalSamplingState::Setup, // Initialize the calibration cycle state to Setup

            angle_el: 0, // Initial electrical angle is 0
            cal_idx: 0,  // Start index at 0

            // cal_table: [0; Self::CAL_TABLE_SIZE], // Data array for storing calibration samples, initialized to 0
            oversampled_pos: 0, // Oversampling accumulator is initially 0
            time_in_state: 0,   // No time spent in current state initially

            ang_el_step: 0, // Initialize calibration steps counter
            direction: 0,   // No direction initially
            speed: 1,       // Use the predefined calibration speed
            settling_time,  // Use the calculated settling time

            init_pos: 0,       // Initial position placeholder
            temp_pos: 0,       // Temporary position placeholder
            dif_max: i32::MIN, // Initialize to very small number for comparison
            dif_min: i32::MAX, // Initialize to very large number for comparison

            cal_table: CalibrationTable::new(),
            el_step_idx: 0,
            
        }
    }

    //---------------------------------------------------------
    // tick_calibrate() Method Steps:
    //
    // 1. Calls run_calibration_cycle() to perform one cycle of movement, waiting, and sampling.
    // 2. If the cycle returns a stable position (not i32::MIN), proceed with logic depending on the current calibration stage.
    // 3. Move through the defined stages of calibration (Settle, Setup, FirstStep, FullRotationCW, FullRotationCCW).
    // 4. Validate motion consistency, detect errors, and finally transition to MotorStatus::Ready.
    //---------------------------------------------------------

    /// High-level calibration method.
    ///
    /// This drives the calibration through its main stages, calling `run_calibration_cycle()`
    /// and then handling the transitions between calibration steps.
    pub fn tick(&mut self, encoder_pos: i32) -> u16 {
        self.position = encoder_pos; // Update the internal position from the sensor
                                     // defmt::println!("Angle: {}", encoder_pos);
        let stable_pos = self.run_sampling_cycle(self.ang_el_step); // Perform a calibration cycle and get stable position

        if stable_pos != i32::MIN {
            // defmt::println!("Angle: {}", stable_pos);
            // If we have a valid stable position reading:
            match self.calibration_stage {
                CalStage::Setup => {
                    // After settling, move to the Setup stage
                    self.cal_idx = 10; // Arbitrary index setting for demonstration
                    self.calibration_stage = CalStage::Reset;
                    self.speed = Self::calculate_speed(self.frequency, Self::CAL_SPEED_US);
                    return self.angle_el;
                }

                CalStage::Reset => {
                    // Prepare to test the motor's motion
                    if Self::iter(&mut self.cal_idx) {
                        // If cal_idx reached 0, record initial positions
                        self.init_pos = stable_pos;
                        self.temp_pos = stable_pos;

                        // Reset difference tracking
                        self.dif_max = i32::MIN;
                        self.dif_min = i32::MAX;

                        // Set up for the FirstStep stage (16 steps)
                        self.ang_el_step = u16::MAX / Self::CAL_FIRST_STEP_USTEPS;
                        self.cal_idx = Self::CAL_FIRST_STEP_USTEPS as usize;
                        self.calibration_stage = CalStage::Pass0;
                        defmt::info!("CALIBRATION: Test single pole motion");
                    }
                }

                CalStage::Pass0 => {
                    // Check the difference in position after each step to ensure consistency
                    let dif = self.temp_pos - stable_pos;
                    self.temp_pos = stable_pos;
                    self.dif_min = self.dif_min.min(dif);
                    self.dif_max = self.dif_max.max(dif);

                    if Self::iter(&mut self.cal_idx) {
                        // After completing all test steps, analyze results
                        let travel = self.init_pos - self.temp_pos; // Total travel during test

                        let direction = travel.signum(); // Determine direction of motion
                        self.direction = direction as isize;

                        // Average step size
                        let avg_step = (travel * direction) / Self::CAL_FIRST_STEP_USTEPS as i32;
                        let deviation = self.dif_max - self.dif_min;

                        if avg_step < deviation {
                            // If the variation is too large, calibration fails
                            defmt::error!("CALIBRATION: Too much deviation while moving");
                            self.calibration_stage = CalStage::Error;
                            return self.angle_el;
                        }

                        // Proceed with a known direction
                        defmt::debug!("CALIBRATION: Detected motion direction: {}", self.direction);

                        // Prepare for the Pass1 stage
                        self.calibration_stage = CalStage::Pass1;
                        self.ang_el_step = u16::MAX / Self::CAL_POINTS_PER_360EL;
                        self.speed = -self.speed * self.direction; // Adjust speed direction
                        self.cal_idx = 0;
                        self.init_pos = stable_pos;
                        self.cal_table.reset(Self::CAL_POINTS_PER_360EL);
                        defmt::info!(
                            "CALIBRATION: Full rotation in positive direction with sampling"
                        );
                    }
                }

                // Perform a full rotation in positive direction
                CalStage::Pass1 => {
                    // Make some margin to allow full rotation calibration
                    let avg_step = (stable_pos - self.init_pos) / (self.cal_idx as i32 + 1);
                    if stable_pos - self.init_pos > u16::MAX as i32 + (avg_step / 3) {
                        // Once we exceed the maximum range, switch to CCW run
                        self.calibration_stage = CalStage::Pass2;
                        defmt::debug!("CALIBRATION: Position count: {}", self.cal_idx);
                        defmt::info!(
                            "CALIBRATION: Full rotation in negative direction with sampling"
                        );
                        self.speed = -self.speed;
                        return self.angle_el;
                    }
                    // if self.cal_idx != Self::CAL_TABLE_SIZE as u16 {
                    self.cal_table.fill_first(self.cal_idx, stable_pos as u16);
                    // }
                    // Increment index as we collect data (data storage commented out)
                    self.cal_idx += 1;
                    // if self.cal_idx == Self::CAL_TABLE_SIZE as u16 {
                    //     defmt::println!("Calibration: ERROR POLE COUNT");
                    //     self.motor_status = MotorStatus::Error;
                    // }
                }

                CalStage::Pass2 => {
                    // Perform a full rotation in CCW direction
                    self.cal_idx -= 1;
                    self.cal_table.fill_second(self.cal_idx, stable_pos as u16);

                    if self.cal_idx == 0 {
                        // Once we return to zero, calibration is complete
                        // self.motor_status = MotorStatus::Ready;
                        self.calibration_stage = CalStage::Check;
                        defmt::info!("CALIBRATION: Finished. Next => NORMAL RUN");

                        self.angle_el = 0;
                        // self.speed = 0;
                    };
                }

                CalStage::Check => {
                    self.cal_table.check();
                    self.calibration_stage = CalStage::Ready;
                    // self.calibration_stage = CalStage::Setup;
                }

                CalStage::Error => {}
                CalStage::Ready => {}
            }
        }
        return self.angle_el;
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
    fn run_sampling_cycle(&mut self, steps: u16) -> i32 {
        match self.cal_cycle_stage {
            CalSamplingState::Setup => {
                // Prepare for a new calibration cycle
                self.oversampled_pos = 0; // Reset oversampling accumulator
                self.cal_cycle_stage = CalSamplingState::Rotating; // Next state: Rotating
                self.time_in_state = steps as usize / self.speed.abs() as usize; // Calculate how long to rotate
                self.el_step_idx = self
                    .angle_el
                    .wrapping_add((steps as i32 * self.speed.signum() as i32) as u16);
                i32::MIN // Not finished yet
            }

            CalSamplingState::Rotating => {
                // Rotate the motor for the specified time
                self.move_at_speed(self.speed); // Advance angle by 'speed' each tick

                if Self::iter(&mut self.time_in_state) {
                    // Once done rotating, move to Waiting state
                    self.cal_cycle_stage = CalSamplingState::Waiting;
                    self.time_in_state = self.settling_time; // Settle time
                    self.angle_el = self.el_step_idx; // Fine tune angle to
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
                if Self::cal_oversampling(
                    self.position,
                    &mut self.time_in_state,
                    &mut self.oversampled_pos,
                ) {
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
    fn move_at_speed(&mut self, increment: isize) {
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
        matches!(self.calibration_stage, CalStage::Ready) // Returns true if Ready
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
    fn cal_oversampling(position: i32, iter: &mut usize, integral: &mut i32) -> bool {
        *integral += position; // Add current position to the integral
        *iter -= 1; // Decrease iteration count
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
    fn iter(counter: &mut usize) -> bool {
        if *counter > 0 {
            *counter -= 1; // Decrement the counter
            false // Still have ticks left
        } else {
            true // Counter reached zero
        }
    }

    #[inline(always)]
    pub fn get_correction(&self, pos: u16) -> (u16, u16) {
        self.cal_table.correct_pos(pos)
    }

    /// Calculate speed in ticks per millisecond.
    ///
    /// # Arguments
    /// * `frequency` - Number of ticks per second
    /// * `speed_ms` - Desired speed in milliseconds
    ///
    /// Returns the calculated speed in ticks.
    #[inline(always)]
    fn calculate_speed(frequency: u16, speed_us: usize) -> isize {
        ((frequency as usize * speed_us) / 1000000) as isize
    }

    /// Calculate settling time in ticks based on frequency and milliseconds.
    ///
    /// # Arguments
    /// * `frequency` - Number of ticks per second
    /// * `settling_ms` - Desired settling time in milliseconds
    ///
    /// Returns the calculated settling time in ticks.
    #[inline(always)]
    fn calculate_settling_time(frequency: u16, settling_us: usize) -> usize {
        (frequency as usize * settling_us) / 1000000
    }
}
