// Motor status constants
pub mod motor_status {
    pub const INIT: i8 = 0;
    pub const RUN: i8 = 1;
    pub const IDLE: i8 = 2;
    pub const BRAKE: i8 = 3;
    pub const ERROR: i8 = -1;
}

// pub struct MotorDriver {
//     // COMMON
//     /// Driver type
//     driver_type: bool,
//     /// Duty of brake mode
//     brake: i16,

//     // ####### Related to PWM driver ########
//     /// Motor type (DC/BLDC/STEPPER)
//     motor_type: u8,
//     /// Motor connection (DC/BLDC/STEPPER)
//     motor_connection: u8,
//     /// Motor type selector (DC/BLDC/STEPPER)
//     motor_sel: MotorSelector,
//     /// Motor phase connection selector
//     phase_sel: PhaseSelector,

//     // ####### Related to step-dir driver ########
//     /// Motor resistance
//     pub angle: i16,
//     /// Motor resistance
//     current: i16,
//     /// Output microstepping
//     ustep: i32,
//     /// Driver steps
//     pub steps: u16,
//     /// Motor rotation direction
//     pub direction: bool,
// }

// impl MotorDriver {
//     // Constructor to initialize all fields with default values
//     pub fn new() -> Self {
//         MotorDriver {
//             driver_type: false,
//             brake: 0,
//             motor_type: 0,
//             motor_connection: 0,
//             motor_sel: MotorSelector::new(
//                 MotorType::BLDC,
//                 VectorAxes2I16 { sin: 100, cos: 200 },
//                 0,
//                 0,
//             ),
//             phase_sel: PhaseSelector::new(PhasePattern::ABCD as u8),
//             angle: 0,
//             current: 0,
//             ustep: 0,
//             steps: 0,
//             direction: false,
//         }
//     }
// }

#[derive(Debug)]
struct CurrentSenseAB {
    abcd_input: [i16; 4],
    ab_output: (i16, i16),
    // brake_channel: i16,
    bipolar_probe: bool,
    probes_amount: u8,

    motor_type: MotorType,
}

impl CurrentSenseAB {
    /// Constructor to create a new instance of `CurrentSenseAB`.
    ///
    /// # Arguments
    /// - `current_channels` - An array of four `i16` values representing the input current channels.
    /// - `motor_type` - The type of motor (`MotorType`) being used, which affects the output calculation.
    ///
    /// # Returns
    /// - A new `CurrentSenseAB` instance with the initial calculated output channels.
    fn new(bipolar_probe: bool, probes_amount: u8) -> Self {
        CurrentSenseAB {
            abcd_input: [0; 4],
            ab_output: (i16::MIN, i16::MIN),
            // brake_channel: 0,
            motor_type: MotorType::UNDEFINED,
            bipolar_probe,
            probes_amount,
        }
    }


    fn tick(&mut self, currents: [i16; 4]) {
        self.abcd_input = currents;
        match self.bipolar_probe {
            false => self.tick_unipolar(),
            true => self.tick_bipolar(),
        };
    }

    fn tick_bipolar(&mut self) {
        match self.probes_amount {
            1 => self.tick_bipolar_single(),
            2 => self.tick_bipolar_dual(),
            3 => self.tick_bipolar_triple(),
            4 => self.tick_bipolar_quad(),
            _ => self.ab_output = (0, 0),
        }
    }

    fn tick_unipolar(&mut self) {
        match self.probes_amount {
            4 => self.tick_unipolar_quad(),
            _ => self.ab_output = (0, 0),
        }
    }

    fn tick_bipolar_single(&mut self) {
        self.ab_output = if let MotorType::DC = self.motor_type {
            (Self::dc_single_bipolar(self.abcd_input[0]), 0)
        } else {
            (0, 0)
        };
    }

    fn tick_bipolar_dual(&mut self) {
        self.ab_output = match self.motor_type {
            MotorType::UNDEFINED => (0, 0),
            MotorType::DC => (
                Self::dc_dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                0,
            ),
            MotorType::STEPPER => (
                Self::dc_single_bipolar(self.abcd_input[0]),
                Self::dc_single_bipolar(self.abcd_input[1]),
            ),
            MotorType::BLDC => Self::bldc_dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
        };
    }

    fn tick_bipolar_triple(&mut self) {
        self.ab_output = match self.motor_type {
            MotorType::UNDEFINED => (0, 0),
            MotorType::DC => (
                Self::dc_dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                0,
            ),
            MotorType::STEPPER => (
                Self::dc_dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                Self::dc_single_bipolar(self.abcd_input[2]),
            ),
            MotorType::BLDC => Self::bldc_triple_bipolar(
                self.abcd_input[0],
                self.abcd_input[1],
                self.abcd_input[2],
            ),
        };
    }

    fn tick_bipolar_quad(&mut self) {
        self.ab_output = match self.motor_type {
            MotorType::UNDEFINED => (0, 0),
            MotorType::DC => (
                Self::dc_dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                0,
            ),
            MotorType::STEPPER => (
                Self::dc_dual_bipolar(self.abcd_input[0], self.abcd_input[1]),
                Self::dc_dual_bipolar(self.abcd_input[2], self.abcd_input[3]),
            ),
            MotorType::BLDC => Self::bldc_triple_bipolar(
                self.abcd_input[0],
                self.abcd_input[1],
                self.abcd_input[2],
            ),
        };
    }

    fn tick_unipolar_quad(&mut self) {
        self.ab_output = match self.motor_type {
            MotorType::UNDEFINED => (0, 0),
            MotorType::DC => (
                Self::dc_dual_unipolar(self.abcd_input[0], self.abcd_input[1]),
                0,
            ),
            MotorType::STEPPER => (
                Self::dc_dual_unipolar(self.abcd_input[0], self.abcd_input[1]),
                Self::dc_dual_unipolar(self.abcd_input[2], self.abcd_input[3]),
            ),
            MotorType::BLDC => (0, 0),
        };
    }

    /// Calculates the averaged current for two unipolar input channels.
    ///
    /// # Arguments
    /// - `curnt_a` - The first current value for phase A.
    /// - `curnt_b` - The second current value for phase A.
    ///
    /// # Returns
    /// - The averaged current value for phase A.
    ///
    /// # Description
    /// This function sums the two input current values (`curnt_a` and `curnt_b`) and calculates the average. It assumes that both channels are measuring current in a unipolar configuration.
    #[inline(always)]
    fn dc_dual_unipolar(curnt_a: i16, curnt_b: i16) -> i16 {
        let curnt_combo = curnt_a - curnt_b;
        curnt_combo as i16
    }

    /// Returns the current value for a single bipolar current sensor channel.
    ///
    /// # Arguments
    /// - `curnt_a` - The current value for the channel.
    ///
    /// # Returns
    /// - The current value directly.
    ///
    /// # Description
    /// This function simply returns the provided current value. It is used for bipolar single-channel sensing.
    #[inline(always)]
    fn dc_single_bipolar(curnt: i16) -> i16 {
        curnt
    }

    /// Computes the averaged current for two bipolar input channels.
    ///
    /// # Arguments
    /// - `curnt_a` - The first current value.
    /// - `curnt_b` - The second current value.
    ///
    /// # Returns
    /// - The averaged current value.
    ///
    /// # Description
    /// This function calculates the average of two bipolar current values, useful for current balancing in a bipolar configuration.
    #[inline(always)]
    fn dc_dual_bipolar(curnt_a: i16, curnt_b: i16) -> i16 {
        let curnt_combo = (curnt_a as i32 - curnt_b as i32) >> 1;
        curnt_combo as i16
    }

    /// Computes the current vector components for a BLDC motor using a dual bipolar configuration.
    ///
    /// # Arguments
    /// - `current` - An array of four `i16` values representing the current channels.
    ///   - `current[0]` and `current[1]` represent the phase currents.
    ///
    /// # Returns
    /// - A tuple `(i_alpha, i_beta)` representing the current vector components after performing a Clarke transform.
    ///
    /// # Description
    /// The Clarke transform is used to convert the phase currents into vector components, which are more convenient for field-oriented control.
    fn bldc_dual_bipolar(curnt_a: i16, curnt_b: i16) -> (i16, i16) {
        let (i_alpha, i_beta) =
            direct_clarke_transform(curnt_a, curnt_b, -(curnt_a.saturating_add(curnt_b)));
        (i_alpha, i_beta)
    }

    /// Computes the current vector components for a BLDC motor using a triple bipolar configuration.
    ///
    /// # Arguments
    /// - `current` - An array of four `i16` values representing the current channels.
    ///   - `current[0..2]` represent the phase currents for a three-phase system.
    ///
    /// # Returns
    /// - A tuple `(i_alpha, i_beta)` representing the current vector components after performing a Clarke transform.
    ///
    /// # Description
    /// This function uses three phase currents and calculates the vector components through a Clarke transform. It is specifically used for three-phase BLDC motors.
    fn bldc_triple_bipolar(curnt_a: i16, curnt_b: i16, curnt_c: i16) -> (i16, i16) {
        let (i_alpha, i_beta) = direct_clarke_transform(curnt_a, curnt_b, curnt_c);
        (i_alpha, i_beta)
    }
}
