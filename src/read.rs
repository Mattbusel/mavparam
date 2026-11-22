use alloc::string::String;
use core::fmt;

use crate::value::is_integral;
use crate::{Format, Param, ParamFile, ParamType, MAX_NAME_LEN};

/// A line that could not be read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    /// 1-based line number.
    pub line: usize,
    /// What was wrong with it.
    pub kind: ErrorKind,
}

/// What was wrong with a line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ErrorKind {
    /// Not `NAME VALUE` / `NAME,VALUE` (Mission Planner, MAVProxy), or not
    /// five tab separated fields (QGroundControl).
    Fields,
    /// The name is longer than 16 bytes, which MAVLink cannot carry
    /// (`PARAM_VALUE.param_id` is `char[16]`).
    NameTooLong,
    /// The value is not a number.
    Value,
    /// A QGroundControl vehicle or component id that is not 0 to 255.
    Id,
    /// A QGroundControl type column that is not a `MAV_PARAM_TYPE` (1 to 10).
    Type,
    /// An integer-typed value with a fraction, or outside its type's range.
    IntegerValue,
    /// An ArduPilot defaults-file directive (`@include`, `@READONLY`), which
    /// this version does not expand.
    Directive,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let what = match self.kind {
            ErrorKind::Fields => {
                "expected NAME VALUE, NAME,VALUE or five tab separated QGroundControl fields"
            }
            ErrorKind::NameTooLong => {
                "parameter name longer than 16 bytes (MAVLink param_id is char[16])"
            }
            ErrorKind::Value => "value is not a number",
            ErrorKind::Id => "vehicle or component id is not 0 to 255",
            ErrorKind::Type => "type is not a MAV_PARAM_TYPE (1 to 10)",
            ErrorKind::IntegerValue => "value does not fit its integer MAV_PARAM_TYPE",
            ErrorKind::Directive => "@include and @READONLY directives are not supported",
        };
        write!(f, "line {}: {}", self.line, what)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

const QGC_HEADER: &str = "# Onboard parameters";

/// Guesses the format of a parameter file from its content.
///
/// A `# Onboard parameters` header or a first data line of five tab separated
/// fields is QGroundControl, a first data line with a comma is Mission
/// Planner, anything else is MAVProxy style `NAME VALUE`. An empty file is
/// Mission Planner.
pub fn sniff(text: &str) -> Format {
    for line in text.lines() {
        let t = line.trim();
        if t.starts_with(QGC_HEADER) {
            return Format::QGroundControl;
        }
        let data = strip_comment(t);
        if data.is_empty() {
            continue;
        }
        if line.split('\t').count() == 5 {
            return Format::QGroundControl;
        }
        if data.contains(',') {
            return Format::MissionPlanner;
        }
        return Format::MavProxy;
    }
    Format::MissionPlanner
}

/// Reads a parameter file in whatever format [`sniff`] finds.
pub fn parse(text: &str) -> Result<ParamFile, Error> {
    parse_as(text, sniff(text))
}

/// Reads a parameter file as the given format.
///
/// Mission Planner and MAVProxy files are read like pymavlink `mavparm.load`:
/// everything after `#` is a comment, a comma counts as whitespace, and each
/// remaining line must be exactly a name and a value. `0x` hex values are
/// accepted. Unlike `mavparm`, a bad line is an error, not a skipped line.
///
/// QGroundControl files are read like `ParameterManager`: lines starting with
/// `#` are skipped and every other line is `vehicle, component, name, value,
/// type` separated by tabs.
pub fn parse_as(text: &str, format: Format) -> Result<ParamFile, Error> {
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let mut file = ParamFile::new(format);
    for (i, line) in text.lines().enumerate() {
        let n = i + 1;
        let err = |kind| Error { line: n, kind };
        let p = match format {
            Format::QGroundControl => {
                if line.trim().is_empty() || line.trim_start().starts_with('#') {
                    continue;
                }
                let (p, vehicle) = parse_qgc_line(line).map_err(err)?;
                file.vehicle_id.get_or_insert(vehicle);
                p
            }
            Format::MissionPlanner | Format::MavProxy => {
                let data = strip_comment(line).trim();
                if data.is_empty() {
                    continue;
                }
                parse_name_value(data).map_err(err)?
            }
        };
        file.params.push(p);
    }
    Ok(file)
}

fn strip_comment(line: &str) -> &str {
    match line.find('#') {
        Some(i) => &line[..i],
        None => line,
    }
}

fn check_name(name: &str) -> Result<String, ErrorKind> {
    if name.is_empty() {
        return Err(ErrorKind::Fields);
    }
    if name.len() > MAX_NAME_LEN {
        return Err(ErrorKind::NameTooLong);
    }
    Ok(name.into())
}

fn parse_number(s: &str) -> Result<f64, ErrorKind> {
    let (neg, body) = match s.strip_prefix('-') {
        Some(b) => (true, b),
        None => (false, s.strip_prefix('+').unwrap_or(s)),
    };
    if let Some(hex) = body.strip_prefix("0x").or_else(|| body.strip_prefix("0X")) {
        let v = u64::from_str_radix(hex, 16).map_err(|_| ErrorKind::Value)? as f64;
        return Ok(if neg { -v } else { v });
    }
    s.parse::<f64>().map_err(|_| ErrorKind::Value)
}

fn parse_name_value(data: &str) -> Result<Param, ErrorKind> {
    let mut it = data
        .split(|c: char| c == ',' || c.is_whitespace())
        .filter(|s| !s.is_empty());
    let (Some(name), Some(value)) = (it.next(), it.next()) else {
        return Err(ErrorKind::Fields);
    };
    if name.starts_with('@') {
        return Err(ErrorKind::Directive);
    }
    match it.next() {
        None => {}
        Some(extra) if extra.starts_with('@') => return Err(ErrorKind::Directive),
        Some(_) => return Err(ErrorKind::Fields),
    }
    let name = check_name(name)?;
    let value = parse_number(value)?;
    Ok(Param {
        name,
        value: value as f32 as f64,
        ty: None,
        component: None,
    })
}

fn parse_qgc_line(line: &str) -> Result<(Param, u8), ErrorKind> {
    let mut f = line.split('\t');
    let fields: [&str; 5] = [
        f.next().unwrap_or(""),
        f.next().ok_or(ErrorKind::Fields)?,
        f.next().ok_or(ErrorKind::Fields)?,
        f.next().ok_or(ErrorKind::Fields)?,
        f.next().ok_or(ErrorKind::Fields)?,
    ];
    if f.next().is_some() {
        return Err(ErrorKind::Fields);
    }
    let vehicle: u8 = fields[0].trim().parse().map_err(|_| ErrorKind::Id)?;
    let component: u8 = fields[1].trim().parse().map_err(|_| ErrorKind::Id)?;
    let name = check_name(fields[2].trim())?;
    let ty = fields[4]
        .trim()
        .parse::<u8>()
        .ok()
        .and_then(ParamType::from_u8)
        .ok_or(ErrorKind::Type)?;
    let raw = parse_number(fields[3].trim())?;
    let value = match ty {
        ParamType::Real32 => raw as f32 as f64,
        ParamType::Real64 => raw,
        t => {
            let (lo, hi) = t.integer_range().unwrap_or((f64::MIN, f64::MAX));
            if !is_integral(raw) || raw < lo || raw > hi {
                return Err(ErrorKind::IntegerValue);
            }
            raw
        }
    };
    Ok((
        Param {
            name,
            value,
            ty: Some(ty),
            component: Some(component),
        },
        vehicle,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sniffs() {
        assert_eq!(sniff("#NOTE: x\nACRO_Y_RATE,120\n"), Format::MissionPlanner);
        assert_eq!(
            sniff("FRAME_TYPE\t0\nFS_THR_ENABLE   1\n"),
            Format::MavProxy
        );
        assert_eq!(sniff("1\t1\tATT_EN\t0\t6\n"), Format::QGroundControl);
        assert_eq!(
            sniff("# Onboard parameters for Vehicle 1\n"),
            Format::QGroundControl
        );
    }

    #[test]
    fn mavparm_rules() {
        let f = parse("A 1 # trailing\n\n# c\nB,0x10\nC\t-2.5\n").unwrap();
        let v: alloc::vec::Vec<_> = f
            .params
            .iter()
            .map(|p| (p.name.as_str(), p.value))
            .collect();
        assert_eq!(v, [("A", 1.0), ("B", 16.0), ("C", -2.5)]);
    }

    #[test]
    fn errors() {
        assert_eq!(parse("A\n").unwrap_err().kind, ErrorKind::Fields);
        assert_eq!(parse("A 1 2\n").unwrap_err().kind, ErrorKind::Fields);
        assert_eq!(parse("A x\n").unwrap_err().kind, ErrorKind::Value);
        assert_eq!(
            parse("ABCDEFGHIJKLMNOPQ 1\n").unwrap_err(),
            Error {
                line: 1,
                kind: ErrorKind::NameTooLong
            }
        );
        assert_eq!(
            parse("A 1 @READONLY\n").unwrap_err().kind,
            ErrorKind::Directive
        );
        assert_eq!(
            parse("@include x.parm\n").unwrap_err().kind,
            ErrorKind::Directive
        );
        let q = "# Onboard parameters for Vehicle 1\n";
        assert_eq!(
            parse(&alloc::format!("{q}1\t1\tA\t1.5\t6\n"))
                .unwrap_err()
                .kind,
            ErrorKind::IntegerValue
        );
        assert_eq!(
            parse(&alloc::format!("{q}1\t1\tA\t300\t1\n"))
                .unwrap_err()
                .kind,
            ErrorKind::IntegerValue
        );
        assert_eq!(
            parse(&alloc::format!("{q}1\t1\tA\t1\t11\n"))
                .unwrap_err()
                .kind,
            ErrorKind::Type
        );
        assert_eq!(
            parse(&alloc::format!("{q}1\t1\tA\t1\n")).unwrap_err().kind,
            ErrorKind::Fields
        );
        assert_eq!(
            parse(&alloc::format!("{q}1\t256\tA\t1\t9\n"))
                .unwrap_err()
                .line,
            2
        );
    }
}
