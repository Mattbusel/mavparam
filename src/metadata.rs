//! ArduPilot parameter metadata from `apm.pdef.json` (feature `metadata`).
//!
//! ArduPilot publishes one file per vehicle, generated from the firmware
//! source, for example
//! <https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.json>.
//! It holds each parameter's display name, units, recommended range, enum
//! values, bitmask bits and the `RebootRequired` and `ReadOnly` flags.
//!
//! ```
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let json = r#"{"Copter": {"ANGLE_MAX": {"DisplayName": "Angle Max",
//!   "Range": {"low": "1000", "high": "8000"}, "Units": "cdeg"}}}"#;
//! let meta = mavparam::metadata::Metadata::from_json(json)?;
//! let file = mavparam::parse("ANGLE_MAX,9000\n")?;
//! let found = meta.check(&file);
//! assert_eq!(found.len(), 1);
//! assert_eq!(found[0].to_string(), "ANGLE_MAX = 9000: outside the documented range 1000 to 8000 cdeg");
//! # Ok(()) }
//! ```
//!
//! ArduPilot ranges are recommendations: the firmware stores a value outside
//! them. The check reports them so a person looks, it does not mean the
//! vehicle rejected the value.

use std::collections::BTreeMap;
use std::fmt;

use serde_json::Value;

use crate::value::is_integral;
use crate::{format_f32, format_value, Param, ParamFile, ParamType};

/// What `apm.pdef.json` says about one parameter.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ParamInfo {
    /// Short human name.
    pub display_name: String,
    /// Long description.
    pub description: String,
    /// Units, such as `cdeg`, `Hz` or `s/rad`.
    pub units: Option<String>,
    /// Documented `(low, high)` range, inclusive.
    pub range: Option<(f64, f64)>,
    /// Suggested step size.
    pub increment: Option<f64>,
    /// Named values, for enum parameters.
    pub values: Vec<(f64, String)>,
    /// Named bits, for bitmask parameters.
    pub bitmask: Vec<(u32, String)>,
    /// The new value takes effect only after a reboot.
    pub reboot_required: bool,
    /// Set by the firmware (calibration ids, counters); writing it does nothing useful.
    pub read_only: bool,
    /// The audience level: `Standard`, `Advanced` or `User`.
    pub user: Option<String>,
}

impl ParamInfo {
    /// The name of an enum value, if `value` is one of the listed values.
    pub fn value_name(&self, value: f64) -> Option<&str> {
        self.values
            .iter()
            .find(|(v, _)| *v as f32 == value as f32)
            .map(|(_, n)| n.as_str())
    }
}

/// Parameter metadata for one vehicle type, keyed by parameter name.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Metadata {
    params: BTreeMap<String, ParamInfo>,
}

/// `apm.pdef.json` could not be read.
#[derive(Debug)]
pub struct MetadataError(String);

impl fmt::Display for MetadataError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "apm.pdef.json: {}", self.0)
    }
}

impl std::error::Error for MetadataError {}

fn num(v: Option<&Value>) -> Option<f64> {
    match v? {
        Value::String(s) => s.trim().parse().ok(),
        Value::Number(n) => n.as_f64(),
        _ => None,
    }
}

fn text(v: Option<&Value>) -> Option<String> {
    v.and_then(Value::as_str).map(str::to_owned)
}

fn flag(v: Option<&Value>) -> bool {
    match v {
        Some(Value::String(s)) => s.eq_ignore_ascii_case("true"),
        Some(Value::Bool(b)) => *b,
        _ => false,
    }
}

fn info(o: &serde_json::Map<String, Value>) -> ParamInfo {
    let range = o
        .get("Range")
        .and_then(|r| Some((num(r.get("low"))?, num(r.get("high"))?)));
    let mut values: Vec<(f64, String)> = o
        .get("Values")
        .and_then(Value::as_object)
        .map(|m| {
            m.iter()
                .filter_map(|(k, v)| Some((k.trim().parse().ok()?, v.as_str()?.to_owned())))
                .collect()
        })
        .unwrap_or_default();
    values.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut bitmask: Vec<(u32, String)> = o
        .get("Bitmask")
        .and_then(Value::as_object)
        .map(|m| {
            m.iter()
                .filter_map(|(k, v)| {
                    let bit: u32 = k.trim().parse().ok()?;
                    (bit < 64).then(|| (bit, v.as_str().unwrap_or("").to_owned()))
                })
                .collect()
        })
        .unwrap_or_default();
    bitmask.sort_by_key(|b| b.0);
    ParamInfo {
        display_name: text(o.get("DisplayName")).unwrap_or_default(),
        description: text(o.get("Description")).unwrap_or_default(),
        units: text(o.get("Units")),
        range,
        increment: num(o.get("Increment")),
        values,
        bitmask,
        reboot_required: flag(o.get("RebootRequired")),
        read_only: flag(o.get("ReadOnly")),
        user: text(o.get("User")),
    }
}

/// The bits of a bitmask value. ArduPilot keeps bitmasks in signed
/// `AP_Int8`, `AP_Int16` and `AP_Int32`, so the top bit reads back negative:
/// a negative value is taken as two's complement in its type's width (32 bits
/// when the file has no type).
fn bitmask_bits(p: &Param) -> Option<u64> {
    let v = p.value;
    if !is_integral(v) {
        return None;
    }
    if v >= 0.0 {
        return (v < 18_446_744_073_709_551_616.0).then_some(v as u64);
    }
    match p.ty {
        Some(ParamType::Int8) if v >= i8::MIN as f64 => Some((v as i8) as u8 as u64),
        Some(ParamType::Int16) if v >= i16::MIN as f64 => Some((v as i16) as u16 as u64),
        Some(ParamType::Int64) if v >= i64::MIN as f64 => Some((v as i64) as u64),
        Some(ParamType::Int8 | ParamType::Int16 | ParamType::Int64) => None,
        _ if v >= i32::MIN as f64 => Some((v as i32) as u32 as u64),
        _ => None,
    }
}

/// Something about a parameter value that a person should look at.
#[derive(Debug, Clone, PartialEq)]
pub enum Problem {
    /// The name is not in the metadata: a typo, a parameter from another
    /// vehicle type or firmware version, or one that no longer exists.
    Unknown,
    /// Outside the documented range.
    OutOfRange {
        /// Documented low end.
        low: f64,
        /// Documented high end.
        high: f64,
    },
    /// An enum parameter set to a value that is not one of the listed values.
    /// Only checked when the parameter has no `Range`: with a range, the
    /// listed values are presets.
    NotAListedValue,
    /// A bitmask parameter with bits set that the metadata does not define.
    UndefinedBits {
        /// The undefined bits that are set.
        bits: u64,
    },
    /// A bitmask parameter that is not a whole number, or is negative beyond
    /// its type's width.
    NotABitmask,
}

/// A [`Problem`] found in one parameter.
#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    /// The parameter as it was in the file.
    pub param: Param,
    /// The problem.
    pub problem: Problem,
    /// The parameter's units, for display.
    pub units: Option<String>,
}

impl fmt::Display for Finding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} = {}: ", self.param.name, format_value(&self.param))?;
        match &self.problem {
            Problem::Unknown => write!(f, "not in the parameter metadata"),
            Problem::OutOfRange { low, high } => {
                write!(
                    f,
                    "outside the documented range {} to {}",
                    format_f32(*low as f32),
                    format_f32(*high as f32)
                )?;
                if let Some(u) = &self.units {
                    write!(f, " {u}")?;
                }
                Ok(())
            }
            Problem::NotAListedValue => write!(f, "not one of the listed values"),
            Problem::UndefinedBits { bits } => write!(f, "sets undefined bits 0x{bits:x}"),
            Problem::NotABitmask => write!(f, "not a valid bitmask value"),
        }
    }
}

impl Metadata {
    /// Reads `apm.pdef.json`: an object of groups, each an object of
    /// parameter name to its description. Entries that are not objects (the
    /// `json.version` stamp) are skipped.
    pub fn from_json(json: &str) -> Result<Metadata, MetadataError> {
        let root: Value = serde_json::from_str(json).map_err(|e| MetadataError(e.to_string()))?;
        let groups = root
            .as_object()
            .ok_or_else(|| MetadataError("top level is not an object".into()))?;
        let mut params = BTreeMap::new();
        for group in groups.values() {
            let Some(group) = group.as_object() else {
                continue;
            };
            for (name, v) in group {
                if let Some(o) = v.as_object() {
                    params.insert(name.clone(), info(o));
                }
            }
        }
        if params.is_empty() {
            return Err(MetadataError("no parameters found".into()));
        }
        Ok(Metadata { params })
    }

    /// The metadata for one parameter.
    pub fn get(&self, name: &str) -> Option<&ParamInfo> {
        self.params.get(name)
    }

    /// Number of parameters described.
    pub fn len(&self) -> usize {
        self.params.len()
    }

    /// True when no parameters are described.
    pub fn is_empty(&self) -> bool {
        self.params.is_empty()
    }

    /// Problems with one parameter's value. Comparisons are done as f32, so
    /// a value of `0.3` against a documented high of `0.3` is in range even
    /// though `0.3f32` is slightly above `0.3f64`.
    pub fn check_param(&self, p: &Param) -> Option<Problem> {
        let Some(info) = self.get(&p.name) else {
            return Some(Problem::Unknown);
        };
        let v = p.value as f32;
        if let Some((low, high)) = info.range {
            if v < low as f32 || v > high as f32 {
                return Some(Problem::OutOfRange { low, high });
            }
        }
        if !info.bitmask.is_empty() {
            let Some(bits) = bitmask_bits(p) else {
                return Some(Problem::NotABitmask);
            };
            let defined = info.bitmask.iter().fold(0u64, |m, (b, _)| m | (1u64 << b));
            let extra = bits & !defined;
            if extra != 0 {
                return Some(Problem::UndefinedBits { bits: extra });
            }
        } else if info.range.is_none()
            && !info.values.is_empty()
            && info.value_name(p.value).is_none()
        {
            // With a Range as well, the Values are presets (MOT_SPIN_ARM lists
            // 0, 0.1 and 0.2 inside a 0 to 0.2 range), not the only legal values.
            return Some(Problem::NotAListedValue);
        }
        None
    }

    /// Problems with every parameter in a file, in file order.
    pub fn check(&self, file: &ParamFile) -> Vec<Finding> {
        file.params
            .iter()
            .filter_map(|p| {
                self.check_param(p).map(|problem| Finding {
                    param: p.clone(),
                    problem,
                    units: self.get(&p.name).and_then(|i| i.units.clone()),
                })
            })
            .collect()
    }
}
