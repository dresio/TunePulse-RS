/// Represents a single data point in its raw binary form for Real-Time Transfer (RTT)
///
/// This struct is marked with `repr(C, packed)` to ensure:
/// - `repr(C)`: The struct has a C-compatible memory layout
/// - `packed`: No padding bytes are inserted between fields
#[repr(C, packed)]
pub struct RawDataPoint {
    /// Unique identifier for the data point.
    /// Used to distinguish between different types of measurements or data sources.
    id: u8,

    /// Timestamp or sequence number for the data point.
    /// Typically implemented as a monotonically increasing counter.
    timestamp: u32,

    /// The actual measurement or value of the data point.
    /// Stored as a 32-bit floating point number.
    value: f32,
}

impl RawDataPoint {
    /// Create a new `RawDataPoint` with the specified `id`, `timestamp`, and `value`.
    pub fn new(id: u8, timestamp: u32, value: f32) -> Self {
        Self {
            id,
            timestamp,
            value,
        }
    }
}
