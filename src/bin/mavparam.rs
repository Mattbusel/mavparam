//! `mavparam`: diff, convert and check drone parameter files.

use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::process::ExitCode;

use mavparam::metadata::{Metadata, Problem};
use mavparam::{diff, format_value, parse, write, DiffOptions, Format, Param, ParamFile};

const USAGE: &str = "\
mavparam: diff, convert and check drone parameter files

Formats (detected from the content):
  Mission Planner / ArduPilot .param   NAME,VALUE
  MAVProxy / ArduPilot .parm           NAME VALUE
  QGroundControl .params               vehicle<TAB>component<TAB>NAME<TAB>VALUE<TAB>TYPE

Usage:
  mavparam diff A B [--meta apm.pdef.json] [--abs X] [--rel X] [--json]
      Values compare as float32, as the autopilot stores them.
      Exit 0 when the files match, 1 when they differ.
  mavparam convert IN OUT [--to param|parm|params] [--types REF.params] [--sort]
      Output format from --to or OUT's extension. '-' is stdin or stdout.
      --types takes MAV_PARAM_TYPE from a QGroundControl export of the
      same vehicle, for writing .params from an untyped file.
  mavparam check FILE [--meta apm.pdef.json]
      Reports repeated names and, with --meta, values outside the documented
      range, values not in an enum, undefined bitmask bits and unknown names.
      Exit 0 when clean, 1 when something was found.

ArduPilot metadata: https://autotest.ardupilot.org/Parameters/<Vehicle>/apm.pdef.json
(Vehicle is ArduCopter, ArduPlane, Rover, ArduSub, AntennaTracker or Blimp).
";

struct Args {
    pos: Vec<String>,
    opts: BTreeMap<String, String>,
    flags: Vec<String>,
}

fn parse_args(raw: &[String]) -> Result<Args, String> {
    let mut a = Args {
        pos: vec![],
        opts: BTreeMap::new(),
        flags: vec![],
    };
    let mut it = raw.iter();
    while let Some(s) = it.next() {
        match s.as_str() {
            "--json" | "--sort" => a.flags.push(s.clone()),
            "--meta" | "--abs" | "--rel" | "--to" | "--types" => {
                let v = it.next().ok_or_else(|| format!("{s} needs a value"))?;
                a.opts.insert(s.clone(), v.clone());
            }
            o if o.starts_with("--") => return Err(format!("unknown option {o}")),
            _ => a.pos.push(s.clone()),
        }
    }
    Ok(a)
}

fn read_text(path: &str) -> Result<String, String> {
    if path == "-" {
        let mut s = String::new();
        std::io::stdin()
            .read_to_string(&mut s)
            .map_err(|e| format!("stdin: {e}"))?;
        return Ok(s);
    }
    std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))
}

fn load(path: &str) -> Result<(ParamFile, String), String> {
    let text = read_text(path)?;
    let f = parse(&text).map_err(|e| format!("{path}: {e}"))?;
    Ok((f, text))
}

fn load_meta(a: &Args) -> Result<Option<Metadata>, String> {
    match a.opts.get("--meta") {
        None => Ok(None),
        Some(p) => Metadata::from_json(&read_text(p)?)
            .map(Some)
            .map_err(|e| format!("{p}: {e}")),
    }
}

fn format_name(f: Format) -> &'static str {
    match f {
        Format::MissionPlanner => "Mission Planner",
        Format::MavProxy => "MAVProxy",
        Format::QGroundControl => "QGroundControl",
    }
}

fn label(p: &Param) -> String {
    match p.component {
        Some(c) if c != mavparam::DEFAULT_COMPONENT => format!("{}@{}", p.name, c),
        _ => p.name.clone(),
    }
}

fn shown(p: &Param, meta: Option<&Metadata>) -> String {
    let v = format_value(p);
    match meta
        .and_then(|m| m.get(&p.name))
        .and_then(|i| i.value_name(p.value))
    {
        Some(n) => format!("{v} ({n})"),
        None => v,
    }
}

/// A MAVProxy `param save` dump writes every value with `%f`, six decimals,
/// which loses precision below 1e-6 and on long fractions.
fn six_decimal_dump(f: &ParamFile, text: &str) -> bool {
    if f.format != Format::MavProxy || f.params.is_empty() {
        return false;
    }
    let six = text
        .lines()
        .filter_map(|l| l.split_whitespace().nth(1))
        .filter(|v| v.split_once('.').is_some_and(|(_, d)| d.len() == 6))
        .count();
    six * 10 >= f.params.len() * 9
}

fn notes(m: Option<&Metadata>, p: &Param) -> Vec<String> {
    let mut out = vec![];
    let Some(m) = m else { return out };
    if let Some(info) = m.get(&p.name) {
        if info.reboot_required {
            out.push("reboot required".into());
        }
        if info.read_only {
            out.push("read-only".into());
        }
    }
    match m.check_param(p) {
        Some(Problem::OutOfRange { low, high }) => {
            let u = m
                .get(&p.name)
                .and_then(|i| i.units.as_deref())
                .map(|u| format!(" {u}"))
                .unwrap_or_default();
            out.push(format!(
                "outside {} to {}{u}",
                mavparam::format_f32(low as f32),
                mavparam::format_f32(high as f32)
            ));
        }
        Some(Problem::NotAListedValue) => out.push("not a listed value".into()),
        Some(Problem::UndefinedBits { bits }) => out.push(format!("undefined bits 0x{bits:x}")),
        Some(Problem::NotABitmask) => out.push("not a valid bitmask".into()),
        Some(Problem::Unknown) => out.push("not in metadata".into()),
        None => {}
    }
    out
}

fn tag(n: &[String]) -> String {
    n.iter().map(|s| format!("  [{s}]")).collect()
}

fn cmd_diff(a: &Args) -> Result<u8, String> {
    let [pa, pb] = a.pos.as_slice() else {
        return Err("diff needs two files".into());
    };
    let (fa, ta) = load(pa)?;
    let (fb, tb) = load(pb)?;
    let meta = load_meta(a)?;
    let num = |k: &str| -> Result<Option<f64>, String> {
        a.opts
            .get(k)
            .map(|v| {
                v.parse::<f64>()
                    .map_err(|_| format!("{k}: not a number: {v}"))
            })
            .transpose()
    };
    let opts = DiffOptions {
        abs_tolerance: num("--abs")?,
        rel_tolerance: num("--rel")?,
    };
    let d = diff(&fa, &fb, &opts);
    let m = meta.as_ref();

    if a.flags.iter().any(|f| f == "--json") {
        let p = |p: &Param| {
            serde_json::json!({
                "name": p.name, "component": p.component_or_default(),
                "value": format_value(p), "notes": notes(m, p),
            })
        };
        let out = serde_json::json!({
            "a": {"path": pa, "format": format_name(fa.format), "count": fa.params.len()},
            "b": {"path": pb, "format": format_name(fb.format), "count": fb.params.len()},
            "added": d.added.iter().map(p).collect::<Vec<_>>(),
            "removed": d.removed.iter().map(p).collect::<Vec<_>>(),
            "changed": d.changed.iter().map(|c| serde_json::json!({
                "name": c.new.name, "component": c.new.component_or_default(),
                "old": format_value(&c.old), "new": format_value(&c.new),
                "notes": notes(m, &c.new),
            })).collect::<Vec<_>>(),
        });
        println!("{out:#}");
    } else {
        for (path, f, t) in [(pa, &fa, &ta), (pb, &fb, &tb)] {
            if six_decimal_dump(f, t) {
                eprintln!(
                    "note: {path} looks like a MAVProxy `param save` dump, which rounds every \
                     value to 6 decimals; --abs 5e-7 ignores that rounding"
                );
            }
        }
        println!(
            "--- {pa} ({}, {} params)\n+++ {pb} ({}, {} params)",
            format_name(fa.format),
            fa.params.len(),
            format_name(fb.format),
            fb.params.len()
        );
        let mut w = 16;
        for n in d
            .changed
            .iter()
            .map(|c| &c.new)
            .chain(&d.added)
            .chain(&d.removed)
        {
            w = w.max(label(n).len());
        }
        for c in &d.changed {
            println!(
                "~ {:<w$}  {} -> {}{}",
                label(&c.new),
                shown(&c.old, m),
                shown(&c.new, m),
                tag(&notes(m, &c.new))
            );
        }
        for p in &d.added {
            println!("+ {:<w$}  {}{}", label(p), shown(p, m), tag(&notes(m, p)));
        }
        for p in &d.removed {
            println!("- {:<w$}  {}", label(p), shown(p, m));
        }
        println!(
            "{} changed, {} added, {} removed",
            d.changed.len(),
            d.added.len(),
            d.removed.len()
        );
    }
    Ok(if d.is_empty() { 0 } else { 1 })
}

fn cmd_convert(a: &Args) -> Result<u8, String> {
    let [input, output] = a.pos.as_slice() else {
        return Err("convert needs IN and OUT".into());
    };
    let (mut f, _) = load(input)?;
    let to = match a.opts.get("--to") {
        Some(t) => Format::from_extension(t.trim_start_matches('.'))
            .ok_or_else(|| format!("--to {t}: use param, parm or params"))?,
        None => std::path::Path::new(output)
            .extension()
            .and_then(|e| e.to_str())
            .and_then(Format::from_extension)
            .ok_or_else(|| {
                format!("{output}: cannot tell the format from the extension, pass --to")
            })?,
    };
    if let Some(r) = a.opts.get("--types") {
        let (rf, _) = load(r)?;
        let types: BTreeMap<(u8, &str), &Param> = rf
            .params
            .iter()
            .map(|p| ((p.component_or_default(), p.name.as_str()), p))
            .collect();
        if f.vehicle_id.is_none() {
            f.vehicle_id = rf.vehicle_id;
        }
        for p in &mut f.params {
            if p.ty.is_none() {
                if let Some(t) = types.get(&(p.component_or_default(), p.name.as_str())) {
                    p.ty = t.ty;
                    p.component = t.component;
                }
            }
        }
    }
    if a.flags.iter().any(|f| f == "--sort") {
        f.sort();
    }
    if to == Format::QGroundControl {
        let untyped = f.params.iter().filter(|p| p.ty.is_none()).count();
        if untyped > 0 {
            eprintln!(
                "note: {untyped} parameters have no type in {input} and are written as REAL32 (9); \
                 --types REF.params copies types from a QGroundControl export of the same vehicle"
            );
        }
    }
    let text = write(&f, to);
    if output == "-" {
        std::io::stdout()
            .write_all(text.as_bytes())
            .map_err(|e| e.to_string())?;
    } else {
        std::fs::write(output, text).map_err(|e| format!("{output}: {e}"))?;
    }
    Ok(0)
}

fn cmd_check(a: &Args) -> Result<u8, String> {
    let [path] = a.pos.as_slice() else {
        return Err("check needs one file".into());
    };
    let (f, text) = load(path)?;
    let meta = load_meta(a)?;
    let mut found = 0;
    println!(
        "{path}: {}, {} params",
        format_name(f.format),
        f.params.len()
    );
    if six_decimal_dump(&f, &text) {
        println!("note: values have 6 decimals, like a MAVProxy `param save` dump; small values may be rounded");
    }
    for d in f.duplicates() {
        println!("{d}: set more than once, the last value wins");
        found += 1;
    }
    if let Some(m) = &meta {
        let all = m.check(&f);
        let unknown = all.iter().filter(|x| x.problem == Problem::Unknown).count();
        for x in all.iter().filter(|x| x.problem != Problem::Unknown) {
            println!("{x}");
            found += 1;
        }
        for x in all.iter().filter(|x| x.problem == Problem::Unknown) {
            println!("{x}");
        }
        found += unknown;
        for p in &f.params {
            if m.get(&p.name).is_some_and(|i| i.read_only) {
                println!(
                    "{} = {}: read-only, set by the firmware; loading it does nothing useful",
                    p.name,
                    format_value(p)
                );
            }
        }
    }
    println!("{found} findings");
    Ok(if found == 0 { 0 } else { 1 })
}

fn main() -> ExitCode {
    let raw: Vec<String> = std::env::args().skip(1).collect();
    let (cmd, rest) = match raw.split_first() {
        Some((c, r)) => (c.as_str(), r),
        None => {
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    if matches!(cmd, "-h" | "--help" | "help") {
        print!("{USAGE}");
        return ExitCode::SUCCESS;
    }
    if matches!(cmd, "-V" | "--version") {
        println!("mavparam {}", env!("CARGO_PKG_VERSION"));
        return ExitCode::SUCCESS;
    }
    let result = parse_args(rest).and_then(|a| match cmd {
        "diff" => cmd_diff(&a),
        "convert" => cmd_convert(&a),
        "check" => cmd_check(&a),
        other => Err(format!("unknown command {other}; try mavparam --help")),
    });
    match result {
        Ok(code) => ExitCode::from(code),
        Err(e) => {
            eprintln!("mavparam: {e}");
            ExitCode::from(2)
        }
    }
}
