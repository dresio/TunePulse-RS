// Defining the PositionFilter struct that implements the position filtering logic.
pub struct AngleFilter {
    // Inputs
    pub alpha: u8, // Filter coefficient (0..255 = 0.0..1.0)

    // Outputs
    output: u16,

    // Local variables
    temp: u32, // Stores scaled filtered value
}

impl AngleFilter {
    const RESOLUTION: u8 = 7; // Additional bits for better filtering (min 0 max 7)

    /// Constructor to initialize the filter with the input and alpha
    pub fn new(input_default: u16, alpha: u8) -> AngleFilter {
        AngleFilter {
            alpha,
            output: input_default,
            temp: (input_default as u32) << AngleFilter::RESOLUTION,
        }
    }

    /// Math call
    pub fn tick(&mut self, input: u16) {
        // Constants for calculations
        const THRESHOLD_POS: u32 = 1u32 << (16 + AngleFilter::RESOLUTION - 1); // Positive bias
        const THRESHOLD_NEG: u32 = THRESHOLD_POS.wrapping_neg(); // Negative bias
        const MASK: u32 = 2 * THRESHOLD_POS - 1; // Mask to constrain output temp

        // Convert the input to a 32-bit integer and shift left by the resolution value to scale it
        let current: u32 = (input as u32) << AngleFilter::RESOLUTION;

        // ########## ZERO CROSS OFFSET CALCULATION #######################
        // Calculate difference to detect zero-cross condition
        let diff: u32 = THRESHOLD_POS.wrapping_add((self.temp as i32 - current as i32) as u32);

        // Asset offset depending on input values to bias calculations
        let bias = if diff <= MASK {
            0 // If difference less than half of maximum allowable value then no bias is needed
        } else if diff >> 31 != 0 {
            THRESHOLD_NEG // If value is negative (sign bit is 1) set bias as -THRESHOLD
        } else {
            THRESHOLD_POS // Otherwise set bias as +THRESHOLD
        };

        // ############ LPF FILTER WITH INTEGER MATH ######################
        let prevs_sub_bias = self.temp.wrapping_sub(bias);
        let curnt_add_bias = current.wrapping_add(bias);

        let alpha: u32 = self.alpha as u32;

        // LPF filter: y = alpha * y + (1 - alpha) * x
        self.temp = alpha * prevs_sub_bias + (256u32 - alpha) * curnt_add_bias;

        // Compensate alpha size multiplication
        self.temp >>= 8;

        // Adjust the filtered value within the range defined by the MASK.
        self.temp = self.temp.wrapping_add(bias) & MASK;

        // Scale the filtered value back down and store it as the output
        self.output = (self.temp >> AngleFilter::RESOLUTION) as u16;
    }

    /// Function to retrieve the output value
    pub fn get_output(&self) -> u16 {
        self.output
    }
}
