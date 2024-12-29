pub mod duty {
    /// Calculates coil voltages based on the reference voltage
    /// Best for center-alligned PWM, distributes switching between all 4 mosfets, eliminates close to zero duty issue
    #[inline(always)]
    pub fn center(voltg_ref: i16) -> (i16, i16) {
        if voltg_ref == i16::MIN || voltg_ref == 0 {
            return (voltg_ref, voltg_ref); // Returns disabled or zero voltage if reference is disabled or zero
        }
        // Calculate duty as difference between duties on both channels aligned to midpoint. Output wave vill be like
        // TIM: ↑↑↑↓↓↓↑↑↑↓↓↓↑↑↑↓↓↓ or ↑↑↑↓↓↓↑↑↑↓↓↓↑↑↑↓↓↓
        // CH1: _‾‾‾‾__‾‾‾‾___‾‾____‾‾__ where `_` is LOW and `‾` is HIGH
        // CH2: __‾‾____‾‾___‾‾‾‾__‾‾‾‾_ where `_` is LOW and `‾` is HIGH
        // OUT: -↑--↑--↑--↑--↓--↓--↓--↓- where `-` is recuperation, `↑` is direct and `↓` is reverse polarity on coil
        // This way single period of center alligned PWM will give twice switching frequency. Also both half-bridges will work thus reducing overheat of each one

        // This approach is oposite to edge switching which has to deal with ~0% duty PWM at ~0% and at ~100% current,
        // while this will deal with ~0% duty PWM only at ~100% current
        // ~0% duty PWM is issue because it may be represented as VERY high frequency signal AND may be too short to allow driver react correctly

        let duty: i16 = voltg_ref >> 1; // Calculates duty cycle based on reference voltage
        const MIDPOINT: i16 = i16::MAX >> 1; // Defines the midpoint for PWM alignment
        return (MIDPOINT + duty, MIDPOINT - duty); // Returns adjusted voltages based on duty cycle
    }

    /// Calculates coil voltages based on the reference voltage
    #[inline(always)]
    pub fn edge(voltg_ref: i16) -> (i16, i16) {
        if voltg_ref == i16::MIN || voltg_ref == 0 {
            return (voltg_ref, voltg_ref); // Returns disabled or zero voltage if reference is disabled or zero
        }

        if voltg_ref < 0 {
            return (0, -voltg_ref);
        } else {
            return (voltg_ref, 0);
        }
    }
}

pub mod current {
    /// Calculates the averaged current for two unipolar input channels.
    ///
    /// This function sums the two input current values (`current_a` and `current_b`) and calculates the average. It assumes that both channels are measuring current in a unipolar configuration.
    #[inline(always)]
    pub fn dual_unipolar(current_a: i16, current_b: i16) -> i16 {
        let current_combo = current_a - current_b;
        current_combo as i16
    }

    /// Returns the current value for a single bipolar current sensor channel.
    ///
    /// This function simply returns the provided current value. It is used for bipolar single-channel sensing.
    #[inline(always)]
    pub fn single_bipolar(current: i16) -> i16 {
        current
    }

    /// Computes the averaged current for two bipolar input channels.
    ///
    /// This function calculates the average of two bipolar current values, useful for current balancing in a bipolar configuration.
    #[inline(always)]
    pub fn dual_bipolar(current_a: i16, current_b: i16) -> i16 {
        let current_combo = (current_a as i32 - current_b as i32) >> 1;
        current_combo as i16
    }
}
