/// Type alias for storing data from 6 channels:
/// - `AdcData[0-3]`: `ich1`-`ich4` - current measurement for channel 1-4
/// - `AdcData[4]`: `vsup` - supply voltage
/// - `AdcData[5]`: `vtemp` - temperature sensor voltage
type AdcData = [u16; 6];  // Define a type alias representing an array of six 16-bit unsigned integers for storing ADC data from 6 channels.

pub struct NormalizeADC {
    /// Reference voltage to allow correction
    vref: u16,  // Store the reference voltage used for ADC value correction.

    /// Type alias for storing data from 6 channels:
    /// - `AdcData[0-3]`: `ich1`-`ich4` - current measurement for channel 1-4
    /// - `AdcData[4]`: `vsup` - supply voltage
    /// - `AdcData[5]`: `vtemp` - temperature sensor voltage
    adc: AdcData,  // Store the raw ADC data from 6 channels, as defined by the `AdcData` alias.

    /// Normalized supply voltage out
    vsup: u16,  // Store the normalized value for the supply voltage.

    /// Normalized temperature sensor voltage out
    vtemp: u16,  // Store the normalized value for the temperature sensor voltage.

    /// Normalized current channels voltage out
    current1234: [u16; 4],  // Store the normalized current values for channels 1 to 4.

    /// Vref reference voltage (from internal variables or from datasheet)
    vref_cal: u32,  // Store the calibrated reference voltage as a 32-bit unsigned integer.

    /// Compensation factor
    k_factor: u32,  // Store the compensation factor for correcting ADC values.
}

impl NormalizeADC {
    const K_BITSHIFT: u32 = 15;  // Define a constant for bit-shifting compensation factor to maintain precision during calculations.

    pub fn new(vref_cal: u32) -> Self {  // Public constructor method to initialize a new instance of NormalizeADC.
        NormalizeADC {
            vref: 0,  // Initialize the reference voltage to 0.
            adc: [0; 6],  // Initialize the ADC data array with all values set to 0.
            vref_cal: vref_cal << Self::K_BITSHIFT,  // Shift the calibrated reference voltage by `K_BITSHIFT` to maintain precision.
            vsup: 0,  // Initialize the normalized supply voltage to 0.
            vtemp: 0,  // Initialize the normalized temperature sensor voltage to 0.
            current1234: [0; 4],  // Initialize the normalized current values for channels 1 to 4 to 0.
            k_factor: (u16::MAX >> 1) as u32,  // Set the initial compensation factor to half the maximum 16-bit value.
        }
    }

    fn update_k(&mut self) {  // Private method to update the compensation factor `k_factor`.
        self.k_factor = self.vref_cal / (self.vref as u32);  // Update `k_factor` based on the reference voltage calibration and current reference voltage.
    }

    fn adjust_adc(&self, adc_val: u16) -> u16 {  // Private method to adjust an ADC value using the compensation factor.
        let adc_val = adc_val as u32;  // Convert the 16-bit ADC value to 32-bit for calculation.
        let corrected_adc = (adc_val * self.k_factor) >> Self::K_BITSHIFT;  // Calculate the corrected ADC value by applying `k_factor` and shifting.

        if corrected_adc >> 16 != 0 {  // Check if the corrected value exceeds 16 bits.
            0xFFFF  // If it exceeds, return the maximum possible 16-bit value to prevent overflow.
        } else {
            corrected_adc as u16  // Otherwise, return the corrected value as a 16-bit integer.
        }
    }

    pub fn tick(&mut self) {  // Public method to update the normalized values based on current ADC readings.
        self.update_k();  // Update the compensation factor `k_factor` based on the current reference voltage.

        // ########## Adjust voltage values ###########################
        for i in 0..4 {  // Iterate over the first four ADC channels (current channels).
            self.current1234[i] = self.adjust_adc(self.adc[i]);  // Adjust and store the normalized current values for each channel.
        }
        self.vsup = self.adjust_adc(self.adc[4]);  // Adjust and store the normalized supply voltage.
        self.vtemp = self.adjust_adc(self.adc[5]);  // Adjust and store the normalized temperature sensor voltage.
    }
}

/// Calculates a calibrated reference voltage (VREF).
///
/// # Parameters
/// - `design_vdda_mv`: The designed VDDA voltage in millivolts.
/// - `cal_val`: The calibration value for the VREF.
/// - `cal_vdda_mv`: The calibration voltage for VDDA in millivolts.
/// - `cal_bits`: The bit resolution of the VREF calibration.
///
/// # Returns
/// The calibrated reference voltage as a 32-bit unsigned integer.
pub fn vref_calc_calibrated(design_vdda_mv: u32, cal_val: u32, cal_vdda_mv: u32, cal_bits: u32) -> u32 {
    ((cal_val << (16 - cal_bits)) * cal_vdda_mv)  // Adjust the calibration value to 16 bits and multiply by the calibration voltage.
        / design_vdda_mv // Divide by the designed VDDA voltage to get the calibrated VREF.
}

/// Calculates an approximate reference voltage (VREF).
///
/// # Parameters
/// - `design_vdda_mv`: The designed VDDA voltage in millivolts.
/// - `vref_mv`: The reference voltage in millivolts.
///
/// # Returns
/// The approximate reference voltage as a 32-bit unsigned integer.
pub const fn vref_calc_approximated(design_vdda_mv: u32, vref_mv: u32) -> u32 {
    (vref_mv * (u16::MAX >> 1) as u32) / design_vdda_mv // Calculate the approximate VREF using the designed VDDA voltage and reference voltage.
}