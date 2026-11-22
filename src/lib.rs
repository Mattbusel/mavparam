//! Drone parameter files: read, write and diff the three formats people
//! actually exchange.
//!
//! | Format | Written by | Line shape |
//! |---|---|---|
//! | [`Format::MissionPlanner`] | Mission Planner, ArduPilot `Tools/Frame_params` | `NAME,VALUE` |
//! | [`Format::MavProxy`] | MAVProxy `param save`, ArduPilot default `.parm` files | `NAME VALUE` (spaces or tab) |
//! | [`Format::QGroundControl`] | QGroundControl "Save to file" | `vehicle<TAB>component<TAB>NAME<TAB>VALUE<TAB>MAV_PARAM_TYPE` |
//!
//! Values are compared the way the autopilot stores them. A MAVLink
//! parameter travels as a 32-bit float (`PARAM_VALUE.param_value`), so
//! `0.1`, `0.100000001` and `0.100000001490116119` are the same stored
//! value and [`diff`] does not report them as a change.
//!
//! ```
//! let a = mavparam::parse("ATC_RAT_RLL_P,0.1\nANGLE_MAX,3000\n").unwrap();
//! let b = mavparam::parse(
//!     "1\t1\tATC_RAT_RLL_P\t0.100000001490116119\t9\n1\t1\tANGLE_MAX\t4500\t4\n",
//! )
//! .unwrap();
//! let d = mavparam::diff(&a, &b, &mavparam::DiffOptions::default());
//! assert_eq!(d.changed.len(), 1);
//! assert_eq!(d.changed[0].name(), "ANGLE_MAX");
//! ```
#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

extern crate alloc;

mod diff;
mod read;
mod value;
mod write;

#[cfg(feature = "metadata")]
pub mod metadata;

pub use diff::{diff, Change, Diff, DiffOptions};
pub use read::{parse, parse_as, sniff, Error, ErrorKind};
pub use value::{format_f32, format_value, values_equal};
pub use write::write;

use alloc::string::String;
use alloc::vec::Vec;

/// Longest parameter name MAVLink can carry: `PARAM_VALUE.param_id` is `char[16]`.
pub const MAX_NAME_LEN: usize = 16;

/// The autopilot component id a parameter belongs to when the file does not say
/// (`MAV_COMP_ID_AUTOPILOT1`).
pub const DEFAULT_COMPONENT: u8 = 1;

/// A parameter file format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Format {
    /// `NAME,VALUE` per line, `#` comments. Mission Planner and ArduPilot `.param` files.
    MissionPlanner,
    /// `NAME VALUE` per line separated by spaces or a tab, `#` comments.
    /// MAVProxy `param save` and ArduPilot `.parm` default files.
    MavProxy,
    /// QGroundControl `.params`: tab separated vehicle id, component id,
    /// name, value and `MAV_PARAM_TYPE`.
    QGroundControl,
}

impl Format {
    /// The usual file extension (without the dot).
    pub fn extension(self) -> &'static str {
        match self {
            Format::MissionPlanner => "param",
            Format::MavProxy => "parm",
            Format::QGroundControl => "params",
        }
    }

    /// Guesses the format from a file extension: `params` is QGroundControl,
    /// `param` Mission Planner, `parm` MAVProxy. Case insensitive.
    pub fn from_extension(ext: &str) -> Option<Format> {
        if ext.eq_ignore_ascii_case("params") {
            Some(Format::QGroundControl)
        } else if ext.eq_ignore_ascii_case("param") {
            Some(Format::MissionPlanner)
        } else if ext.eq_ignore_ascii_case("parm") {
            Some(Format::MavProxy)
        } else {
            None
        }
    }
}

/// `MAV_PARAM_TYPE`, the storage type of a parameter on the vehicle.
///
/// <https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE>
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
pub enum ParamType {
    /// 8-bit unsigned integer.
    Uint8 = 1,
    /// 8-bit signed integer.
    Int8 = 2,
    /// 16-bit unsigned integer.
    Uint16 = 3,
    /// 16-bit signed integer.
    Int16 = 4,
    /// 32-bit unsigned integer.
    Uint32 = 5,
    /// 32-bit signed integer.
    Int32 = 6,
    /// 64-bit unsigned integer.
    Uint64 = 7,
    /// 64-bit signed integer.
    Int64 = 8,
    /// 32-bit IEEE float.
    Real32 = 9,
    /// 64-bit IEEE float.
    Real64 = 10,
}

impl ParamType {
    /// Converts the numeric `MAV_PARAM_TYPE` value.
    pub fn from_u8(v: u8) -> Option<ParamType> {
        Some(match v {
            1 => ParamType::Uint8,
            2 => ParamType::Int8,
            3 => ParamType::Uint16,
            4 => ParamType::Int16,
            5 => ParamType::Uint32,
            6 => ParamType::Int32,
            7 => ParamType::Uint64,
            8 => ParamType::Int64,
            9 => ParamType::Real32,
            10 => ParamType::Real64,
            _ => return None,
        })
    }

    /// True for the integer types.
    pub fn is_integer(self) -> bool {
        !matches!(self, ParamType::Real32 | ParamType::Real64)
    }

    /// The inclusive value range of an integer type, as f64.
    /// `None` for floats. 64-bit bounds are rounded to the nearest f64.
    pub fn integer_range(self) -> Option<(f64, f64)> {
        Some(match self {
            ParamType::Uint8 => (0.0, u8::MAX as f64),
            ParamType::Int8 => (i8::MIN as f64, i8::MAX as f64),
            ParamType::Uint16 => (0.0, u16::MAX as f64),
            ParamType::Int16 => (i16::MIN as f64, i16::MAX as f64),
            ParamType::Uint32 => (0.0, u32::MAX as f64),
            ParamType::Int32 => (i32::MIN as f64, i32::MAX as f64),
            ParamType::Uint64 => (0.0, u64::MAX as f64),
            ParamType::Int64 => (i64::MIN as f64, i64::MAX as f64),
            ParamType::Real32 | ParamType::Real64 => return None,
        })
    }
}

/// One parameter.
#[derive(Debug, Clone, PartialEq)]
pub struct Param {
    /// Parameter name, at most [`MAX_NAME_LEN`] bytes.
    pub name: String,
    /// The value. For untyped and `REAL32` parameters this is already rounded
    /// to the nearest f32, because that is what the autopilot stores.
    /// Integer types hold the exact integer (exact up to 2^53).
    pub value: f64,
    /// Storage type, when the file records it (QGroundControl does).
    pub ty: Option<ParamType>,
    /// MAVLink component id, when the file records it (QGroundControl does).
    pub component: Option<u8>,
}

impl Param {
    /// An untyped parameter, value rounded to f32 as an autopilot would store it.
    pub fn new(name: &str, value: f64) -> Param {
        Param {
            name: name.into(),
            value: value as f32 as f64,
            ty: None,
            component: None,
        }
    }

    /// The component id, or [`DEFAULT_COMPONENT`] when the file did not say.
    pub fn component_or_default(&self) -> u8 {
        self.component.unwrap_or(DEFAULT_COMPONENT)
    }
}

/// A parsed parameter file, in file order.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamFile {
    /// The format it was read as, and the default format it is written as.
    pub format: Format,
    /// The vehicle (system) id from a QGroundControl file.
    pub vehicle_id: Option<u8>,
    /// The parameters in the order they appear. A name can repeat; the last one wins.
    pub params: Vec<Param>,
}

impl ParamFile {
    /// An empty file of the given format.
    pub fn new(format: Format) -> ParamFile {
        ParamFile {
            format,
            vehicle_id: None,
            params: Vec::new(),
        }
    }

    /// The last parameter with this name on the default component (or no component).
    pub fn get(&self, name: &str) -> Option<&Param> {
        self.params
            .iter()
            .rev()
            .find(|p| p.name == name && p.component_or_default() == DEFAULT_COMPONENT)
    }

    /// Names that appear more than once for the same component, each reported once.
    pub fn duplicates(&self) -> Vec<&str> {
        let mut seen = alloc::collections::BTreeMap::new();
        let mut out: Vec<&str> = Vec::new();
        for p in &self.params {
            let n = seen
                .entry((p.component_or_default(), p.name.as_str()))
                .or_insert(0u32);
            *n += 1;
            if *n == 2 {
                out.push(p.name.as_str());
            }
        }
        out
    }

    /// Sorts by component, then name, the order Mission Planner, MAVProxy and
    /// QGroundControl all save in. Stable, so a repeated name keeps its last-wins order.
    pub fn sort(&mut self) {
        self.params.sort_by(|a, b| {
            (a.component_or_default(), a.name.as_str())
                .cmp(&(b.component_or_default(), b.name.as_str()))
        });
    }

    /// Writes this file in its own format.
    pub fn to_text(&self) -> String {
        write(self, self.format)
    }
}
