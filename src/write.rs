use alloc::string::String;
use core::fmt::Write as _;

use crate::{format_value, Format, ParamFile, ParamType, DEFAULT_COMPONENT};

/// Writes a parameter file in the given format, parameters in file order.
///
/// Every value is the shortest text that reads back to the same stored value
/// (see [`format_value`]), so `parse(write(f))` returns the same values
/// bit for bit. Call [`ParamFile::sort`] first for the name order the ground
/// stations save in.
///
/// * Mission Planner: `NAME,VALUE`.
/// * MAVProxy: the name padded to 16 columns, a space, the value, like
///   `mavparm.save`, but without its `%f` rounding to six decimals.
/// * QGroundControl: the `# Onboard parameters for Vehicle N` header and
///   `vehicle<TAB>component<TAB>name<TAB>value<TAB>type` lines. A parameter
///   from an untyped file is written as `MAV_PARAM_TYPE_REAL32` (9) with
///   component 1, the vehicle id defaults to 1.
pub fn write(file: &ParamFile, format: Format) -> String {
    let mut out = String::new();
    match format {
        Format::MissionPlanner => {
            for p in &file.params {
                let _ = writeln!(out, "{},{}", p.name, format_value(p));
            }
        }
        Format::MavProxy => {
            for p in &file.params {
                let _ = writeln!(out, "{:<16} {}", p.name, format_value(p));
            }
        }
        Format::QGroundControl => {
            let vehicle = file.vehicle_id.unwrap_or(1);
            let _ = writeln!(out, "# Onboard parameters for Vehicle {vehicle}");
            out.push_str("#\n# Vehicle-Id Component-Id Name Value Type\n");
            for p in &file.params {
                let ty = p.ty.unwrap_or(ParamType::Real32);
                let _ = writeln!(
                    out,
                    "{}\t{}\t{}\t{}\t{}",
                    vehicle,
                    p.component.unwrap_or(DEFAULT_COMPONENT),
                    p.name,
                    format_value(p),
                    ty as u8
                );
            }
        }
    }
    out
}
