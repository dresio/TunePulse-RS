// Defining the PositionFilter struct that implements the position filtering logic.
pub struct AngleFilter {
    // Inputs
    pub alpha: u8,  // Filter coefficient (0..255 = 0.0..1.0)

    // Outputs
    output: u16,

    // Local variables
    temp: u32, // Stores scaled filtered value
}

impl AngleFilter {
    // Constant resolution that defines the scaling factor for better filtering
    const RESOLUTION: u8 = 7; // Additional bits for better filtering (min 0 max 7)

    // Constructor to initialize the filter with the input and alpha
    pub fn new(input_default: u16, alpha: u8) -> AngleFilter {
        AngleFilter {
            alpha,
            output: input_default,
            temp: (input_default as u32) << AngleFilter::RESOLUTION,
        }
    }

    pub fn tick(&mut self, input: u16) -> u16 {
        // Constants for calculations
        const THRESHOLD_POS: u32 = 1u32 << (16 + AngleFilter::RESOLUTION - 1);
        const THRESHOLD_NEG: u32 = THRESHOLD_POS.wrapping_neg();
        const MASK: u32 = 2 * THRESHOLD_POS - 1;

        // Convert the input to a 32-bit integer and shift left by the resolution value to scale it
        let current: u32 = (input as u32) << AngleFilter::RESOLUTION;

        // ########## ZERO CROSS OFFSET CALCULATION #######################
        // Calculate difference to detect zero-cross condition
        let mut diff: u32 = THRESHOLD_POS.wrapping_add((self.temp as i32 - current as i32) as u32);

        // Asset offset depending on input values to bias calculations
        diff = if diff <= MASK {
            0 // Если разница в допустимом диапазоне, смещение отсутствует
        } else if diff >> 31 != 0 {
            // Если старший бит установлен, значение отрицательное, поэтому корректируем на -THRESHOLD
            THRESHOLD_NEG
        } else {
            THRESHOLD_POS // Иначе смещение равно THRESHOLD
        };

        // ######## Фильтр нижних частот с целочисленной арифметикой   ###############  
        let prevs_sub_diff = self.temp.wrapping_sub(diff);
        let curnt_add_diff = current.wrapping_add(diff);

        let alpha: u32 = self.alpha as u32;

        self.temp = alpha * prevs_sub_diff + (256u32 - alpha) * curnt_add_diff;
        self.temp >>= 8;

        // Корректировка отфильтрованного значения в заданном диапазоне
        self.temp = self.temp.wrapping_add(diff) & MASK;

        // Масштабирование обратно и сохранение в выходное значение
        self.output = (self.temp >> AngleFilter::RESOLUTION) as u16;
        return self.output;
    }

    
    // Function to retrieve the output value
    pub fn get_output(&self) -> u16 {
        self.output
    }

}
