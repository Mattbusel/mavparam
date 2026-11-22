//! The `mavparam` binary on the sample files (sources in tests/formats.rs).
#![cfg(feature = "cli")]

use std::path::PathBuf;
use std::process::Command;

fn data(name: &str) -> String {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
        .to_string_lossy()
        .into_owned()
}

fn run(args: &[&str]) -> (i32, String, String) {
    let o = Command::new(env!("CARGO_BIN_EXE_mavparam"))
        .args(args)
        .output()
        .unwrap();
    (
        o.status.code().unwrap(),
        String::from_utf8(o.stdout).unwrap(),
        String::from_utf8(o.stderr).unwrap(),
    )
}

fn tmp(name: &str) -> String {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    dir.join(name).to_string_lossy().into_owned()
}

#[test]
fn diff_across_formats_with_metadata() {
    let a = tmp("edited.param");
    let text = std::fs::read_to_string(data("qgc_arducopter_4.6.params")).unwrap();
    let edited = text
        .replace("FRAME_CLASS\t1\t", "FRAME_CLASS\t2\t")
        .replace("ATC_RAT_RLL_P\t0.135\t", "ATC_RAT_RLL_P\t0.4\t")
        .replace("2\t1\tSTAT_BOOTCNT\t134\t4\n", "");
    let f = mavparam::parse(&edited).unwrap();
    std::fs::write(&a, mavparam::write(&f, mavparam::Format::MissionPlanner)).unwrap();

    let (code, out, _) = run(&[
        "diff",
        &data("qgc_arducopter_4.6.params"),
        &a,
        "--meta",
        &data("apm.pdef.subset.json"),
    ]);
    assert_eq!(code, 1, "{out}");
    assert!(
        out.contains("~ ATC_RAT_RLL_P     0.135 -> 0.4  [outside 0 to 0.35 s/rad]"),
        "{out}"
    );
    assert!(
        out.contains("~ FRAME_CLASS       1 (Quad) -> 2 (Hexa)  [reboot required]"),
        "{out}"
    );
    assert!(out.contains("- STAT_BOOTCNT      134"), "{out}");
    assert!(out.ends_with("2 changed, 0 added, 1 removed\n"), "{out}");

    let (code, out, _) = run(&["diff", &data("qgc_arducopter_4.6.params"), &a, "--json"]);
    assert_eq!(code, 1);
    let v: serde_json::Value = serde_json::from_str(&out).unwrap();
    assert_eq!(v["changed"].as_array().unwrap().len(), 2);
    assert_eq!(v["removed"][0]["name"], "STAT_BOOTCNT");
}

#[test]
fn convert_then_diff_is_clean() {
    for ext in ["param", "parm", "params"] {
        let out = tmp(&format!("ardusub.{ext}"));
        let (code, _, err) = run(&["convert", &data("qgc_ardusub_3.5.2.params"), &out]);
        assert_eq!(code, 0, "{err}");
        let (code, stdout, _) = run(&["diff", &data("qgc_ardusub_3.5.2.params"), &out]);
        assert_eq!(code, 0, "{stdout}");
        assert!(stdout.ends_with("0 changed, 0 added, 0 removed\n"));
    }
}

#[test]
fn convert_takes_types_from_a_reference() {
    let (code, out, err) = run(&[
        "convert",
        &data("mp_3dr_iris+.param"),
        "-",
        "--to",
        "params",
    ]);
    assert_eq!(code, 0);
    assert!(err.contains("37 parameters have no type"), "{err}");
    assert!(out.contains("1\t1\tACRO_Y_RATE\t120\t9\n"), "{out}");

    let mp = tmp("subset.param");
    std::fs::write(&mp, "ANGLE_MAX,3500\nATC_RAT_RLL_P,0.15\n").unwrap();
    let (code, out, err) = run(&[
        "convert",
        &mp,
        "-",
        "--to",
        "params",
        "--types",
        &data("qgc_arducopter_4.6.params"),
    ]);
    assert_eq!(code, 0);
    assert!(err.is_empty(), "{err}");
    assert!(out.starts_with("# Onboard parameters for Vehicle 2\n"));
    assert!(out.contains("2\t1\tANGLE_MAX\t3500\t4\n"), "{out}");
    assert!(out.contains("2\t1\tATC_RAT_RLL_P\t0.15\t9\n"), "{out}");
}

#[test]
fn check_reports_and_exits() {
    let (code, out, _) = run(&["check", &data("mp_3dr_iris+.param")]);
    assert_eq!(code, 0, "{out}");
    assert!(out.contains("Mission Planner, 37 params"));

    let bad = tmp("bad.param");
    std::fs::write(&bad, "FRAME_CLASS,99\nLOG_BITMASK,16384\nFRAME_CLASS,1\n").unwrap();
    let (code, out, _) = run(&["check", &bad, "--meta", &data("apm.pdef.subset.json")]);
    assert_eq!(code, 1);
    assert!(out.contains("FRAME_CLASS: set more than once"), "{out}");
    assert!(
        out.contains("LOG_BITMASK = 16384: sets undefined bits 0x4000"),
        "{out}"
    );

    let (code, _, err) = run(&["check", &data("missing.param")]);
    assert_eq!(code, 2);
    assert!(err.starts_with("mavparam: "));
}

#[test]
fn mavproxy_dump_gets_a_precision_note() {
    let (code, _, err) = run(&[
        "diff",
        &data("mavproxy_heli.parm"),
        &data("mavproxy_heli.parm"),
    ]);
    assert_eq!(code, 0);
    assert!(err.contains("rounds every value to 6 decimals"), "{err}");
}
