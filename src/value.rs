use alloc::format;
use alloc::string::String;

use crate::{Param, ParamType};

fn abs(v: f64) -> f64 {
    if v < 0.0 {
        -v
    } else {
        v
    }
}

/// The shortest decimal text that parses back to the same f32.
///
/// `0.1f32` prints as `0.1`, not `0.100000001490116119`. Values below 1e-5 or
/// at or above 1e16 use exponent form (`1e-7`, `3e20`), which every reader
/// in this crate, MAVProxy, Mission Planner and QGroundControl parse. Negative
/// zero prints as `0`, NaN as `nan`.
pub fn format_f32(v: f32) -> String {
    if v == 0.0 {
        return "0".into();
    }
    if v.is_nan() {
        return "nan".into();
    }
    let a = abs(v as f64);
    if (1e-5..1e16).contains(&a) {
        format!("{v}")
    } else {
        format!("{v:e}")
    }
}

fn format_f64(v: f64) -> String {
    if v == 0.0 {
        return "0".into();
    }
    if v.is_nan() {
        return "nan".into();
    }
    if (1e-5..1e16).contains(&abs(v)) {
        format!("{v}")
    } else {
        format!("{v:e}")
    }
}

/// The text a writer puts in the value column: an exact integer for integer
/// types, the shortest round-trip f64 for `REAL64`, and the shortest
/// round-trip f32 otherwise.
pub fn format_value(p: &Param) -> String {
    match p.ty {
        Some(ParamType::Real64) => format_f64(p.value),
        Some(t) if t.is_integer() && is_integral(p.value) => {
            if p.value < 0.0 {
                format!("{}", p.value as i64)
            } else {
                format!("{}", p.value as u64)
            }
        }
        _ => format_f32(p.value as f32),
    }
}

pub(crate) fn is_integral(v: f64) -> bool {
    // f64 values at or above 2^52 have no fractional part.
    if abs(v) >= 4_503_599_627_370_496.0 {
        return v.is_finite();
    }
    v == (v as i64) as f64
}

/// Whether two parameters hold the same stored value.
///
/// Two integer-typed parameters, or two `REAL64` ones, compare exactly.
/// Everything else compares as f32, because that is what `PARAM_VALUE`
/// carries and what ArduPilot stores: `0.1` from a Mission Planner file equals
/// `0.100000001490116119` from an older QGroundControl export. NaN equals NaN,
/// and `-0` equals `0`.
pub fn values_equal(a: &Param, b: &Param) -> bool {
    if a.value.is_nan() && b.value.is_nan() {
        return true;
    }
    let exact = match (a.ty, b.ty) {
        (Some(x), Some(y)) => {
            (x.is_integer() && y.is_integer()) || (x == ParamType::Real64 && y == ParamType::Real64)
        }
        _ => false,
    };
    if exact {
        a.value == b.value
    } else {
        a.value as f32 == b.value as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shortest_f32() {
        assert_eq!(format_f32(0.1), "0.1");
        assert_eq!(format_f32("0.100000001490116119".parse().unwrap()), "0.1");
        assert_eq!(format_f32(-0.0), "0");
        assert_eq!(format_f32(360.0), "360");
        assert_eq!(format_f32(16777216.0), "16777216");
        assert_eq!(format_f32(1e-7), "1e-7");
        assert_eq!(format_f32(3e20), "3e20");
        assert_eq!(format_f32(0.2222222), "0.2222222");
    }

    #[test]
    fn every_written_f32_reads_back_exactly() {
        // 200k pseudo-random bit patterns across every exponent, both signs.
        let mut x: u32 = 0x1234_5678;
        for _ in 0..200_000 {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            let v = f32::from_bits(x);
            if !v.is_finite() {
                continue;
            }
            let s = format_f32(v);
            let back: f32 = s.parse().unwrap();
            assert!(back == v, "{v:?} wrote {s}");
        }
    }

    #[test]
    fn f32_equality() {
        let a = Param::new("A", 0.1);
        let mut b = Param::new("A", 0.0);
        b.value = 0.100000001;
        assert!(values_equal(&a, &b));
        b.value = 0.1001;
        assert!(!values_equal(&a, &b));
    }
}
