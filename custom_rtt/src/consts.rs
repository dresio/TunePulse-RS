use crate::plot_struct::RawDataPoint;

/// RTT buffer size (default: 1024).
pub(crate) const BUF_SIZE: usize = 1024;
pub(crate) const PLOT_STRUCT_SIZE: usize = core::mem::size_of::<RawDataPoint>();
pub(crate) const PLOT_BUF_SIZE: usize = (PLOT_STRUCT_SIZE * 2) + 1; //add 1 for some random packing buffer, anything less then this causes the entire thing to break
pub(crate) const DEFMT_CHANNEL: usize = 0;
pub(crate) const PLOT_CHANNEL: usize = 1;
