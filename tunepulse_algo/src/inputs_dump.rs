// TODO: Make possible to set mandatory fields

/* **Description:**
This code implements a double-buffering approach for storing ADC and other sensor data. 
It uses two data buffers (`DataInputs`) and a set of flags to track which buffer 
is currently being updated and which is ready to be read. By using a bitmask 
to track the completion of all data fields, the code ensures that whenever 
a buffer is marked as "ready," it contains a complete and coherent set of data. 
The reading function (`get_data()`) uses a lock bit to prevent the buffer from being 
modified during the brief read operation, thus eliminating the need for 
heavier synchronization mechanisms. This allows short interrupt routines to safely 
capture snapshots of the data while the main loop or other tasks run freely. 
As a result, every time data is fetched, it is guaranteed to be fully updated and consistent.

**What this code does:**

- Implements a double-buffering system for ADC and sensor data.
- Ensures that each buffer is fully updated before it becomes available to reading tasks.
- Minimizes synchronization overhead by using a lock bit only during the brief data read, 
allowing short interrupt routines to read data without risking partial updates.
- Allows main computations or other code paths to run freely without blocking on data reads.
- Guarantees that each time data is fetched, it is complete, up-to-date, and consistent.
*/

// Data structure holding various ADC readings and raw angle measurements
#[derive(Clone, Copy, Default)]
pub struct DataInputs {
    pub supply_adc: u16,      // Supply ADC reading
    pub temper_adc: u16,      // Temperature ADC reading
    pub currnt_adc: [u16; 4], // Current ADC readings (4 channels)
    pub angle_raw: u16,       // Raw angle measurement
}

// Structure for managing two buffers of DataInputs and related flags
pub struct InputsDump { 
    buffers: [DataInputs; 2], // Two buffers: one being updated, one ready for reading
    idx2update: usize,        // Index of the buffer currently being updated
    flags: [u32; 2], // Flags array holding field completion and lock status for each buffer
    iter: usize,
    prev_iter: usize,
}

// Enum defining inverted bit masks for each data field and a lock bit
#[repr(u32)]
enum FieldBit {
    SUPPLY = !(1 << 0),  // Inverted mask for the supply field bit
    TEMP = !(1 << 1),    // Inverted mask for the temperature field bit
    CURRENT = !(1 << 2), // Inverted mask for the current array field bit
    ANGLE = !(1 << 3),   // Inverted mask for the angle field bit
    LOCK = !(1 << 31),   // Inverted mask for the lock bit at the most significant bit
}

// Combine all field bits using inverted logic to represent a fully unfilled (all fields pending) state
const ALL_FIELDS: u32 = !(FieldBit::SUPPLY as u32
    & FieldBit::TEMP as u32
    & FieldBit::CURRENT as u32
    & FieldBit::ANGLE as u32);

impl InputsDump {
    // Create a new InputsDump with both buffers cleared and ready to be filled
    pub fn new() -> Self {
        Self {
            buffers: [DataInputs::default(), DataInputs::default()], // Initialize both buffers to default
            idx2update: 0,                                           // Start updating buffer 0
            flags: [ALL_FIELDS, 0], // Buffer 0: all fields pending; Buffer 1: ready (no fields pending)
            iter: 0,
            prev_iter: 0,
        }
    }

    // Check if the given buffer index is ready (no fields pending)
    #[inline(always)]
    fn is_ready(&self, idx: usize) -> bool {
        self.flags[idx] == 0 // If flags are 0, the buffer is completely filled (ready)
    }

    // Get the opposite buffer index (if idx=0 return 1, if idx=1 return 0)
    #[inline(always)]
    fn get_opposite(&self, idx: usize) -> usize {
        1 - idx
    }

    // Clear a particular field bit in the flags for the specified buffer
    #[inline(always)]
    fn clear_field_bit(&mut self, idx: usize, bit: FieldBit) {
        self.flags[idx] &= bit as u32; // Use the inverted bit to clear the corresponding field
    }

    // Check if all fields of the current buffer are filled; if both buffers are ready, reinitialize one for updating
    fn check_fill(&mut self, idx: usize) {
        // If both buffers are ready (no pending fields),
        // re-initialize the opposite buffer for new data collection
        if self.is_ready(0) && self.is_ready(1) {
            let idx = self.get_opposite(idx);
            self.flags[idx] = ALL_FIELDS; // Reset the opposite buffer to all fields pending
            self.idx2update = idx; // Switch to updating the opposite buffer
            self.iter = self.iter.wrapping_add(1);
        }
    }

    // Set the supply_adc field in the currently updating buffer
    pub fn set_supply_adc(&mut self, value: u16) {
        let idx = self.idx2update; // Get the currently updating buffer index
        self.buffers[idx].supply_adc = value; // Store the supply ADC value
        self.clear_field_bit(idx, FieldBit::SUPPLY); // Mark the supply field as filled
        self.check_fill(idx); // Check if buffer filling is complete or if we need to switch
    }

    // Set the temper_adc field in the currently updating buffer
    pub fn set_temper_adc(&mut self, value: u16) {
        let idx = self.idx2update; // Get the currently updating buffer index
        self.buffers[idx].temper_adc = value; // Store the temperature ADC value
        self.clear_field_bit(idx, FieldBit::TEMP); // Mark the temperature field as filled
        self.check_fill(idx); // Check if buffer filling is complete or if we need to switch
    }

    // Set the currnt_adc field in the currently updating buffer
    pub fn set_current_adc(&mut self, values: [u16; 4]) {
        let idx = self.idx2update; // Get the currently updating buffer index
        self.buffers[idx].currnt_adc = values; // Store the current ADC array
        self.clear_field_bit(idx, FieldBit::CURRENT); // Mark the current field as filled
        self.check_fill(idx); // Check if buffer filling is complete or if we need to switch
    }

    // Set the angle_raw field in the currently updating buffer
    pub fn set_angle_raw(&mut self, value: u16) {
        let idx = self.idx2update; // Get the currently updating buffer index
        self.buffers[idx].angle_raw = value; // Store the angle data
        self.clear_field_bit(idx, FieldBit::ANGLE); // Mark the angle field as filled
        self.check_fill(idx); // Check if buffer filling is complete or if we need to switch
    }

    // Check if the given buffer index is ready (no fields pending)
    #[inline(always)]
    pub fn is_updated(&self, idx: usize) -> bool {
        self.iter != self.prev_iter
    }

    // Get a fully updated DataInputs from the opposite buffer.
    // This method briefly sets a lock bit to prevent modifications during reading.
    #[inline(always)]
    pub fn get_data(&mut self) -> DataInputs {
        let ready_idx = self.get_opposite(self.idx2update); // Get the opposite buffer which should be ready
        self.flags[ready_idx] |= FieldBit::LOCK as u32; // Set the lock bit on the ready buffer
        let data = self.buffers[ready_idx]; // Copy the data from the locked buffer
        self.prev_iter = self.iter;
        self.flags[ready_idx] &= !(FieldBit::LOCK as u32); // Clear the lock bit after reading
        data // Return the copied data
    }
}
