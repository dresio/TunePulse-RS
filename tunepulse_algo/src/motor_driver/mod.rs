use pwm_control::MotorPWM;

pub mod pulse_control;
pub mod pwm_control;

pub struct MotorDriver {
    motor: MotorPWM,
    frequency: u16,

    // ready: bool,

    setup: u16,
    iter: u16,

    pwm: [i16; 4],

    pub position: i32,

    pub direction: i16,

    pub start_pos: i32,
    pub pole_count: u16,
}

impl MotorDriver {
    const TESTS_COUNT: u16 = 4;
    // Constructor for MotorPWM
    pub fn new(
        motor: pwm_control::MotorType,
        connection: pwm_control::PhasePattern,
        frequency: u16,
    ) -> Self {
        MotorDriver {
            motor: MotorPWM::new(motor, connection),
            frequency,
            setup: Self::TESTS_COUNT + 2,
            iter: 0,
            // ready: false,
            position: 0,
            start_pos: 0,
            pole_count: 50,
            pwm: [0; 4],
            direction: 0,
        }
    }

    // Function to update motor control based on mode
    pub fn tick(&mut self, voltg_angle: (i16, i16), encoder_pos: i32) -> [i16; 4] {
        // Update the motor selector setup
        self.position = encoder_pos;
        let mut angle_el = self.position - self.start_pos;
        angle_el = angle_el % (u16::MAX as i32);
        angle_el = angle_el * self.pole_count as i32;
        let angle_el = (angle_el as i16).wrapping_add(16384 * self.direction);
        let voltg_angle = (angle_el as i16, voltg_angle.1);

        self.pwm = if self.setup == 0 {
            self.motor.tick_angle(voltg_angle)
        } else {
            self.tick_setup()
        };
        self.pwm

        // Update the phase selector state
    }

    fn tick_setup(&mut self) -> [i16; 4] {
        const CAL_SEQUENCE: [i16; 4] = [0, 16384, -32768, -16384];
        if self.iter == 0 {
            self.iter = self.frequency / Self::TESTS_COUNT as u16;
            self.setup -= 1;
            if self.setup == Self::TESTS_COUNT {
                self.start_pos = self.position;
            } else if self.setup == 0 {
                let pos_diff = self.start_pos - self.position;
                self.direction = Self::calc_directrion(pos_diff);
                // self.pole_count = Self::calc_pole_count(pos_diff);
                return [0; 4];
            }
            let angle = CAL_SEQUENCE[(self.setup + 1) as usize % 4];
            self.pwm = self.motor.tick_angle((angle, 6400));
        }
        self.iter -= 1;
        self.pwm
    }

    #[inline(always)]
    fn calc_directrion(diff: i32) -> i16 {
        if diff > 128 {
            1
        } else if diff < -128 {
            -1
        } else {
            0
        }
    }

    #[inline(always)]
    fn calc_pole_count(diff: i32) -> u16 {
        let diff = diff.abs();
        if diff == 0 {return 0;}
        ((((u16::MAX as i32) * Self::TESTS_COUNT as i32 ) / diff) ) as u16
    }

    #[inline(always)]
    pub fn change_motor_mode(&mut self, motor: pwm_control::MotorType) {
        self.motor.change_motor_mode(motor);
    }

    #[inline(always)]
    pub fn change_phase_mode(&mut self, connection: pwm_control::PhasePattern) {
        self.motor.change_phase_mode(connection);
    }

    pub fn is_ready(&self) -> bool {
        self.setup == 0
    }

    pub fn get_pwm(&mut self) -> [i16;4] {
        self.pwm
    }
}
