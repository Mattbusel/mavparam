use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::{values_equal, Param, ParamFile};

/// How [`diff`] decides two values are the same.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DiffOptions {
    /// Also treat values within this absolute difference as unchanged.
    pub abs_tolerance: Option<f64>,
    /// Also treat values within this fraction of the larger magnitude as
    /// unchanged (`0.01` is one percent).
    pub rel_tolerance: Option<f64>,
}

/// A parameter present in both files with different values.
#[derive(Debug, Clone, PartialEq)]
pub struct Change {
    /// The parameter in the first file.
    pub old: Param,
    /// The parameter in the second file.
    pub new: Param,
}

impl Change {
    /// The parameter name.
    pub fn name(&self) -> &str {
        &self.new.name
    }
}

/// The result of [`diff`], each list sorted by component then name.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Diff {
    /// In the second file only.
    pub added: Vec<Param>,
    /// In the first file only.
    pub removed: Vec<Param>,
    /// In both, with a different stored value.
    pub changed: Vec<Change>,
}

impl Diff {
    /// No additions, removals or changes.
    pub fn is_empty(&self) -> bool {
        self.added.is_empty() && self.removed.is_empty() && self.changed.is_empty()
    }
}

fn index(f: &ParamFile) -> BTreeMap<(u8, &str), &Param> {
    // Last occurrence wins, as when the file is loaded onto a vehicle.
    f.params
        .iter()
        .map(|p| ((p.component_or_default(), p.name.as_str()), p))
        .collect()
}

fn abs(v: f64) -> f64 {
    if v < 0.0 {
        -v
    } else {
        v
    }
}

fn same(a: &Param, b: &Param, o: &DiffOptions) -> bool {
    if values_equal(a, b) {
        return true;
    }
    let d = abs(a.value - b.value);
    if let Some(t) = o.abs_tolerance {
        if d <= t {
            return true;
        }
    }
    if let Some(r) = o.rel_tolerance {
        let m = if abs(a.value) > abs(b.value) {
            abs(a.value)
        } else {
            abs(b.value)
        };
        if d <= r * m {
            return true;
        }
    }
    false
}

/// Compares two parameter files, which can be in different formats.
///
/// Parameters match by name and component; a file without component ids
/// (Mission Planner, MAVProxy) counts as component 1, the autopilot. Values
/// compare as the autopilot stores them (see [`values_equal`]) and then
/// against the optional tolerances. A name repeated within a file counts once,
/// with its last value.
pub fn diff(a: &ParamFile, b: &ParamFile, options: &DiffOptions) -> Diff {
    let ia = index(a);
    let ib = index(b);
    let mut d = Diff::default();
    for (k, pa) in &ia {
        match ib.get(k) {
            None => d.removed.push((*pa).clone()),
            Some(pb) if !same(pa, pb, options) => d.changed.push(Change {
                old: (*pa).clone(),
                new: (*pb).clone(),
            }),
            Some(_) => {}
        }
    }
    for (k, pb) in &ib {
        if !ia.contains_key(k) {
            d.added.push((*pb).clone());
        }
    }
    d
}
