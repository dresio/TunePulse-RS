// Defining the PositionFilter struct that implements the position filtering logic.
pub struct FilterLPF {
    alpha: i32, // Filter coefficient (0..255 = 0.0..1.0)
    output: u16,
    temp: i32, // Stores scaled filtered value
}

impl FilterLPF {
    /// Constructor to initialize the filter with the input and alpha
    pub fn new(input_default: u16, alpha: u8) -> FilterLPF {
        FilterLPF {
            alpha: alpha as i32,
            output: input_default,
            temp: (input_default as i32) << 16,
        }
    }

    /// Math call
    pub fn tick(&mut self, input: u16) -> u16 {
        // Convert the input to a 32-bit integer and shift left by 16 bits to allow wrapping as i32
        let current: i32 = (input as i32) << 16;

        // LPF filter math: filtered = alpha * (input - prev)
        // For integer alpha u8: filtered = alpha * (input - prev) / 256 + current 
        
        // Get difference between previous (temp is i32 scaled) and current
        let diff: i32 = self.temp.wrapping_sub(current) as i32;
        
        // Downscale difference to allow alpha scale
        let diff: i32 = diff >> 8;

        // Calculate rest part of lpf, temp now will be as prev for next iter (i32 scaled)
        self.temp = ((diff * self.alpha)).wrapping_add(current);

        // Convert to u32 to allow correct bitshift and scale back to u16
        self.output = (self.temp as u32 >> 16) as u16;

        self.output
    }

    /// Function to retrieve the output value
    pub fn get_output(&self) -> u16 {
        self.output
    }

    /// Function to retrieve the output value
    pub fn set_alpha(&mut self, alpha: u8) {
        self.alpha = alpha as i32;
    }
}
