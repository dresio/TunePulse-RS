// Defining the PositionFilter struct that implements the position filtering logic.
pub struct FilterLPF {
    // Inputs
    alpha: u32, // Filter coefficient (0..255 = 0.0..1.0)

    // Outputs
    output: u16,

    // Local variables
    temp: u32, // Stores scaled filtered value
}

impl FilterLPF {
    const RESOLUTION: u8 = 7; // Additional bits for better filtering (min 0 max 7)

    /// Constructor to initialize the filter with the input and alpha
    pub fn new(input_default: u16, alpha: u8) -> FilterLPF {
        FilterLPF {
            alpha: alpha as u32,
            output: input_default,
            temp: (input_default as u32) << FilterLPF::RESOLUTION,
        }
    }

    /// Math call
    pub fn tick(&mut self, input: u16) {
        // Convert the input to a 32-bit integer and shift left by the resolution value to scale it
        let current: u32 = (input as u32) << Self::RESOLUTION;
        let previous = self.temp;

        // ############ LPF FILTER WITH INTEGER MATH ######################
        self.temp = self.lpf(current, previous, self.alpha);

        // Scale the filtered value back down and store it as the output
        self.output = (self.temp >> Self::RESOLUTION) as u16;
    }

    pub fn tick_oflw(&mut self, input: u16) {
        // Constants for calculations
        const THRESHOLD_POS: u32 = 1u32 << (16 + FilterLPF::RESOLUTION - 1); // Positive bias
        const THRESHOLD_NEG: u32 = THRESHOLD_POS.wrapping_neg(); // Negative bias
        const MASK: u32 = 2 * THRESHOLD_POS - 1; // Mask to constrain output temp

        // Convert the input to a 32-bit integer and shift left by the resolution value to scale it
        let current: u32 = (input as u32) << Self::RESOLUTION;

        // ########## ZERO CROSS OFFSET CALCULATION #######################
        // Calculate difference to detect zero-cross condition
        let diff: u32 = THRESHOLD_POS.wrapping_add((self.temp as i32 - current as i32) as u32);

        // Asset offset depending on input values to prevent wrong calculations
        let offst = if diff <= MASK {
            0 // If difference less than half of maximum allowable value then no offst is needed
        } else if diff >> 31 != 0 {
            THRESHOLD_NEG // If value is negative (sign bit is 1) set offst as -THRESHOLD
        } else {
            THRESHOLD_POS // Otherwise set offst as +THRESHOLD
        };

        // ############ LPF FILTER WITH INTEGER MATH ######################
        let prevs_sub_offst = self.temp.wrapping_sub(offst);
        let curnt_add_offst = current.wrapping_add(offst);

        // ############ LPF FILTER WITH INTEGER MATH ######################
        self.temp = self.lpf(curnt_add_offst, prevs_sub_offst, self.alpha);

        // Adjust the filtered value within the range defined by the MASK.
        self.temp = self.temp.wrapping_add(offst) & MASK;

        // Scale the filtered value back down and store it as the output
        self.output = (self.temp >> Self::RESOLUTION) as u16;
    }

    /// Function to retrieve the output value
    pub fn get_output(&self) -> u16 {
        self.output
    }

    /// Function to retrieve the output value
    pub fn set_alpha(&mut self, alpha: u8) {
        self.alpha = alpha as u32;
    }

    #[inline]
    fn lpf(&mut self, current: u32, previous: u32, alpha: u32) -> u32 {
        // ############ LPF FILTER WITH INTEGER MATH ######################
        // LPF filter: y = alpha * y + (1 - alpha) * x
        let output = alpha * previous + (256 - alpha) * current;

        // Compensate alpha u8 size multiplication and return
        return output >> 8;
    }
}
