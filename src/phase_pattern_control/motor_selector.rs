#[derive(Debug, Clone, Copy)]
// Structure to represent two components of voltage
pub struct VectorAxes2I16 {
    pub sin: i16, // Sinusoidal component
    pub cos: i16, // Cosine component
}


// Class to handle different types of motor controls
pub struct MotorSelector {
    pub voltg: VectorAxes2I16, // Input for alpha component of voltage
    pub voltg_sup: i16,          // Input for supply voltage
    pub voltg_brak: i16,         // Input for brake voltage
    pub mode: MotorType,         // Input for motor type mode
    ch_abcd: [i16; 4], // Array to store voltages for four channels
}

impl MotorSelector {
    // Constructor for SelectorMotorType
    pub fn new(mode: MotorType, voltg_norm: VectorAxes2I16, sup_norm: i16, brake: i16) -> Self {
        MotorSelector {
            mode,
            voltg: voltg_norm,
            voltg_sup: sup_norm,
            voltg_brak: brake,
            ch_abcd: [0; 4],
        }
    }

    // Function to calculate coil voltages
    fn math_coil(voltg_ref: i16) -> (i16, i16) {
        let mut voltg_a = voltg_ref;
        let mut voltg_b = voltg_ref;

        // If the reference voltage is equal to the disable state, do not change anything
        if voltg_ref == i16::MIN {
            return (voltg_a, voltg_b);
        } else if voltg_ref < 0 {
            // If the reference voltage is negative, set to reverse drive mode
            voltg_a = 0;
            voltg_b = voltg_ref.abs();
        } else {
            // If the reference voltage is zero or positive, set to direct drive mode
            voltg_b = 0;
        }

        return (voltg_a, voltg_b);
    }

    fn math_svpwm(
        &mut self,
        voltg_sin: i16,
        voltg_cos: i16,
        voltg_available: i16,
    ) -> (i16, i16, i16) {
        let voltg_sin = voltg_sin as i32;
        let voltg_cos: i32 = voltg_cos as i32;
        let voltg_available: i32 = voltg_available as i32;

        // ############# INVERSE CLARKE TRANSFORM ###################
        // Precalculated sqrt(3)/2 scaled to i16 (sqrt(3)/2 * 2^16)
        const SQRT3: f64 = 1.7320508075688772;
        const SQRT3DIV2: i32 = (SQRT3 / 2.0f64 * (1u32 << 16) as f64) as i32;

        // Convert beta voltage component to a scaled value using SQRT3DIV2
        let voltg_beta_sqrt3_div2: i32 = (SQRT3DIV2 * voltg_cos) >> 16;

        // Set phase A voltage to the alpha component
        let mut voltg_a: i32 = voltg_sin;

        // Calculate phase B voltage: -1/2 * V_alpha + sqrt(3)/2 * V_beta
        let mut voltg_b: i32 = -(voltg_sin >> 1) + voltg_beta_sqrt3_div2;

        // Calculate phase C voltage: -1/2 * V_alpha - sqrt(3)/2 * V_beta
        let mut voltg_c: i32 = -(voltg_sin >> 1) - voltg_beta_sqrt3_div2;

        // Find the minimum and maximum phase voltages
        let voltg_min: i32 = voltg_a.min(voltg_b).min(voltg_c);
        let voltg_max: i32 = voltg_a.max(voltg_b).max(voltg_c);

        let voltg_offset: i32;

        let voltg_full_scale: i32 = voltg_max - voltg_min;

        // Automatic constraining and bottom clamping if avaliable voltage isn't enough
        if voltg_full_scale > voltg_available {
            // Calculate scaling factor (fixed point based on i32, size of scale: 15bit)
            let voltg_scale = (voltg_available << 15) / voltg_full_scale;

            // Apply scale to all channels
            voltg_a = (voltg_a * voltg_scale) >> 15;
            voltg_b = (voltg_b * voltg_scale) >> 15;
            voltg_c = (voltg_c * voltg_scale) >> 15;

            // Apply scale to voltg_min to allow correct clamping
            let voltg_min: i32 = ((voltg_min) * voltg_scale) >> 15;
            voltg_offset = -voltg_min;
        } else {
            // Calculate reference voltage to shift all phase voltages
            voltg_offset = (voltg_available - voltg_max - voltg_min) >> 1;
        }

        // Shift all phase voltages by reference voltage
        voltg_a += voltg_offset;
        voltg_b += voltg_offset;
        voltg_c += voltg_offset;

        return (voltg_a as i16, voltg_b as i16, voltg_c as i16);
    }

    /// Function to handle one-phase motor control
    #[inline(always)]
    fn tick1phase(&mut self) {
        (self.ch_abcd[0], self.ch_abcd[1]) = Self::math_coil(self.voltg.sin);
        // Set unused phase to brake voltage (optional)
        self.ch_abcd[2] = self.voltg_brak;
        self.ch_abcd[3] = self.voltg_brak;
    }

    /// Function to handle two-phase motor control
    #[inline(always)]
    fn tick2phase(&mut self) {
        (self.ch_abcd[0], self.ch_abcd[1]) = Self::math_coil(self.voltg.sin);
        (self.ch_abcd[2], self.ch_abcd[3]) = Self::math_coil(self.voltg.cos);
    }

    /// Function to control 3Phase 3Wire motor with SVPWM algorithm
    #[inline(always)]
    fn tick3phase(&mut self) {
        (self.ch_abcd[0], self.ch_abcd[1], self.ch_abcd[2]) =
            self.math_svpwm(self.voltg.sin, self.voltg.cos, self.voltg_sup);
        // Set unused phase to brake voltage (optional)
        self.ch_abcd[3] = self.voltg_brak;
    }

    // Function to update motor control based on mode
    pub fn tick(&mut self) {
        match self.mode {
            MotorType::DC => self.tick1phase(),
            MotorType::STEPPER => self.tick2phase(),
            MotorType::BLDC => self.tick3phase(),
        }
    }

    // Function to get PWM channel values
    pub fn pwm_channels(&self) -> [i16; 4] {
        self.ch_abcd
    }

    // Function to directly set voltage channels - DO NOT USE SIMULTANEOUSLY WITH tick()
    pub fn dev(&mut self, voltages: [i16; 4]) {
        self.ch_abcd.copy_from_slice(&voltages);
    }
}

#[derive(Debug, Clone, Copy)]
// Enumeration for motor types
pub enum MotorType {
    DC,      // Direct Current motor
    STEPPER, // Stepper motor
    BLDC,    // Brushless DC motor
}
