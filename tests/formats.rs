//! Tests on real parameter files. Each file in tests/data is a short excerpt
//! of a public export, kept byte for byte (line endings included):
//!
//! * `qgc_ardusub_3.5.2.params`: first 30 lines of
//!   <https://github.com/bluerobotics/bluerobotics.github.io/blob/master/downloads/brov2-heavy-3_5_2.params>,
//!   a BlueROV2 ArduSub 3.5.2 export from an older QGroundControl, which wrote
//!   floats with 18 decimals (`0.100000001490116119`).
//! * `qgc_arducopter_4.6.params`: header plus 17 lines of
//!   <https://github.com/JacopoPan/aerial-autonomy-stack/blob/main/start_here/docs/params/x650-arducopter-4.6.params>,
//!   an ArduCopter 4.6.3 export from a current QGroundControl, which writes the
//!   shortest `%g` text (`3.6e+02`).
//! * `qgc_px4_1.17.params`: first 24 lines of
//!   <https://github.com/mavlink/qgroundcontrol/blob/master/src/Comms/MockLink/PX4MockLink.params>.
//! * `mp_3dr_iris+.param`: all of
//!   <https://github.com/ArduPilot/ardupilot/blob/master/Tools/Frame_params/3DR_Iris+.param>.
//! * `mavproxy_heli.parm`: first 24 lines of
//!   <https://github.com/tridge/cuav/blob/master/script/SITL/aircraft/Heli/heli.parm>,
//!   a MAVProxy `param save` dump (`%-16.16s %f`).
//! * `ap_copter_defaults.parm`: first 20 lines of
//!   <https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/default_params/copter.parm>
//!   (tab and space separators mixed).
//! * `apm.pdef.subset.json`: 10 entries of
//!   <https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.json>
//!   (fetched 2026-11-22), structure unchanged.

use mavparam::{
    diff, parse, parse_as, sniff, values_equal, write, DiffOptions, Format, Param, ParamFile,
    ParamType,
};

const ARDUSUB: &str = include_str!("data/qgc_ardusub_3.5.2.params");
const COPTER46: &str = include_str!("data/qgc_arducopter_4.6.params");
const PX4: &str = include_str!("data/qgc_px4_1.17.params");
const IRIS: &str = include_str!("data/mp_3dr_iris+.param");
const HELI: &str = include_str!("data/mavproxy_heli.parm");
const DEFAULTS: &str = include_str!("data/ap_copter_defaults.parm");

fn get<'a>(f: &'a ParamFile, name: &str) -> &'a Param {
    f.get(name).unwrap_or_else(|| panic!("{name} missing"))
}

fn assert_same_values(a: &ParamFile, b: &ParamFile) {
    assert_eq!(a.params.len(), b.params.len());
    for (x, y) in a.params.iter().zip(&b.params) {
        assert_eq!(x.name, y.name);
        assert_eq!(x.value.to_bits(), y.value.to_bits(), "{}", x.name);
    }
}

#[test]
fn sniffs_every_sample() {
    assert_eq!(sniff(ARDUSUB), Format::QGroundControl);
    assert_eq!(sniff(COPTER46), Format::QGroundControl);
    assert_eq!(sniff(PX4), Format::QGroundControl);
    assert_eq!(sniff(IRIS), Format::MissionPlanner);
    assert_eq!(sniff(HELI), Format::MavProxy);
    assert_eq!(sniff(DEFAULTS), Format::MavProxy);
}

#[test]
fn qgc_old_18_decimal_export() {
    let f = parse(ARDUSUB).unwrap();
    assert_eq!(f.vehicle_id, Some(1));
    assert_eq!(f.params.len(), 22);
    let p = get(&f, "ACCEL_Z_I");
    assert_eq!(p.value, 0.1f32 as f64);
    assert_eq!(p.ty, Some(ParamType::Real32));
    assert_eq!(p.component, Some(1));
    let t = get(&f, "ACRO_TRAINER");
    assert_eq!((t.value, t.ty), (2.0, Some(ParamType::Int8)));
    assert_eq!(get(&f, "ANGLE_MAX").ty, Some(ParamType::Int16));
    // The writers drop the float noise.
    let mp = write(&f, Format::MissionPlanner);
    assert!(mp.contains("ACCEL_Z_I,0.1\n"), "{mp}");
    assert!(mp.contains("ACRO_EXPO,0.3\n"));
    assert!(mp.contains("AHRS_RP_P,0.2\n"));
}

#[test]
fn qgc_current_export_with_exponents() {
    let f = parse(COPTER46).unwrap();
    assert_eq!(f.vehicle_id, Some(2));
    assert_eq!(get(&f, "ACRO_RP_RATE").value, 360.0);
    assert_eq!(get(&f, "ATC_ACCEL_P_MAX").value, 30000.0);
    assert_eq!(get(&f, "ATC_RAT_RLL_P").value, 0.135f32 as f64);
    let id = get(&f, "INS_ACC_ID");
    assert_eq!((id.value, id.ty), (3866898.0, Some(ParamType::Int32)));
    assert_eq!(get(&f, "LOG_BITMASK").value, 180222.0);
}

#[test]
fn px4_mock_export() {
    let f = parse(PX4).unwrap();
    assert_eq!(f.vehicle_id, Some(1));
    assert_eq!(get(&f, "BAT1_CAPACITY").value, -1.0);
    let n = get(&f, "BAT1_N_CELLS");
    assert_eq!((n.value, n.ty), (4.0, Some(ParamType::Int32)));
}

#[test]
fn mission_planner_frame_file() {
    let f = parse(IRIS).unwrap();
    assert_eq!(f.format, Format::MissionPlanner);
    assert_eq!(f.params.len(), 37);
    assert_eq!(get(&f, "ACRO_Y_RATE").value, 120.0);
    assert_eq!(get(&f, "ATC_RAT_PIT_IMAX").value, 0.2222222f32 as f64);
    assert_eq!(get(&f, "ATC_RAT_PIT_IMAX").ty, None);
}

#[test]
fn mavproxy_dump_and_defaults_file() {
    let f = parse(HELI).unwrap();
    assert_eq!(f.format, Format::MavProxy);
    assert_eq!(f.params.len(), 24);
    assert_eq!(get(&f, "ACCEL_Z_FILT").value, 20.0);
    let d = parse(DEFAULTS).unwrap();
    assert_eq!(d.params.len(), 20);
    assert_eq!(get(&d, "FRAME_TYPE").value, 0.0);
    assert_eq!(get(&d, "COMPASS_OFS_Z").value, -18.0);
    assert_eq!(get(&d, "RC1_MAX").value, 2000.0);
}

#[test]
fn every_sample_round_trips_through_every_format() {
    for text in [ARDUSUB, COPTER46, PX4, IRIS, HELI, DEFAULTS] {
        let f = parse(text).unwrap();
        for to in [
            Format::MissionPlanner,
            Format::MavProxy,
            Format::QGroundControl,
        ] {
            let out = write(&f, to);
            assert_eq!(sniff(&out), to);
            let back = parse(&out).unwrap();
            assert_same_values(&f, &back);
            assert!(diff(&f, &back, &DiffOptions::default()).is_empty());
            if to == Format::QGroundControl {
                // Types and ids survive a QGroundControl round trip.
                for (x, y) in f.params.iter().zip(&back.params) {
                    assert_eq!(x.ty.unwrap_or(ParamType::Real32), y.ty.unwrap());
                }
                assert_eq!(back.vehicle_id, Some(f.vehicle_id.unwrap_or(1)));
            }
        }
    }
}

#[test]
fn qgc_round_trip_is_text_stable() {
    // A file this crate wrote reads and writes back to the same bytes.
    let once = write(&parse(COPTER46).unwrap(), Format::QGroundControl);
    let twice = write(&parse(&once).unwrap(), Format::QGroundControl);
    assert_eq!(once, twice);
    assert!(once.contains("2\t1\tACRO_RP_RATE\t360\t9\n"));
    assert!(once.contains("2\t1\tINS_ACC_ID\t3866898\t6\n"));
}

#[test]
fn cross_format_diff_ignores_float_noise() {
    // The first ArduSub values typed the way a person writes them in a .param
    // file. The QGroundControl export says 0.100000001490116119 and
    // 0.300000011920928955; the vehicle stores the same f32 either way.
    let mp = parse(
        "ACCEL_Z_D,0\nACCEL_Z_FF,0\nACCEL_Z_FILT,20\nACCEL_Z_I,0.1\nACCEL_Z_IMAX,100\n\
         ACCEL_Z_P,0.5\nACRO_BAL_PITCH,1\nACRO_BAL_ROLL,1\nACRO_EXPO,0.3\nACRO_RP_P,4.5\n\
         ACRO_TRAINER,2\nACRO_YAW_P,4.5\nAHRS_COMP_BETA,0.1\nAHRS_EKF_TYPE,2\n\
         AHRS_GPS_GAIN,1\nAHRS_GPS_MINSATS,6\nAHRS_GPS_USE,1\nAHRS_ORIENTATION,16\n\
         AHRS_RP_P,0.2\nAHRS_WIND_MAX,0\nAHRS_YAW_P,0.2\nANGLE_MAX,4500\n",
    )
    .unwrap();
    let qgc = parse(ARDUSUB).unwrap();
    let d = diff(&mp, &qgc, &DiffOptions::default());
    assert!(d.is_empty(), "{d:?}");

    // A real change still shows.
    let edited = parse(&ARDUSUB.replace("ANGLE_MAX\t4500", "ANGLE_MAX\t3000")).unwrap();
    let d = diff(&mp, &edited, &DiffOptions::default());
    assert_eq!(d.changed.len(), 1);
    assert_eq!(d.changed[0].name(), "ANGLE_MAX");
    assert_eq!(
        (d.changed[0].old.value, d.changed[0].new.value),
        (4500.0, 3000.0)
    );
}

#[test]
fn added_removed_changed_and_tolerance() {
    let a = parse("A,1\nB,2\nC,0.123456\n").unwrap();
    let b = parse("B 2.5\nC 0.1234561\nD 4\n").unwrap();
    let d = diff(&a, &b, &DiffOptions::default());
    assert_eq!(
        d.removed.iter().map(|p| &p.name[..]).collect::<Vec<_>>(),
        ["A"]
    );
    assert_eq!(
        d.added.iter().map(|p| &p.name[..]).collect::<Vec<_>>(),
        ["D"]
    );
    assert_eq!(
        d.changed.iter().map(|c| c.name()).collect::<Vec<_>>(),
        ["B", "C"]
    );

    let o = DiffOptions {
        abs_tolerance: Some(5e-7),
        ..Default::default()
    };
    assert_eq!(diff(&a, &b, &o).changed.len(), 1);
    let o = DiffOptions {
        rel_tolerance: Some(0.25),
        ..Default::default()
    };
    assert!(diff(&a, &b, &o).changed.is_empty());
}

#[test]
fn integer_types_compare_exactly() {
    // 16777217 is not an f32. Two typed INT32 values compare as integers.
    let q = "# Onboard parameters for Vehicle 1\n";
    let a = parse(&format!("{q}1\t1\tX_ID\t16777216\t6\n")).unwrap();
    let b = parse(&format!("{q}1\t1\tX_ID\t16777217\t6\n")).unwrap();
    assert!(!values_equal(&a.params[0], &b.params[0]));
    assert_eq!(write(&b, Format::MissionPlanner), "X_ID,16777217\n");
    // An untyped file can only hold what the f32 wire format carries.
    let c = parse("X_ID,16777217\n").unwrap();
    assert!(values_equal(&a.params[0], &c.params[0]));
}

#[test]
fn components_and_repeats() {
    let q = "# Onboard parameters for Vehicle 1\n\
             1\t1\tCAM_MODE\t1\t6\n1\t100\tCAM_MODE\t2\t6\n";
    let qgc = parse(q).unwrap();
    let mp = parse("CAM_MODE,1\n").unwrap();
    let d = diff(&mp, &qgc, &DiffOptions::default());
    assert!(d.changed.is_empty());
    assert_eq!(d.added.len(), 1);
    assert_eq!(d.added[0].component, Some(100));

    let rep = parse("A,1\nB,2\nA,3\n").unwrap();
    assert_eq!(rep.duplicates(), ["A"]);
    assert_eq!(rep.get("A").unwrap().value, 3.0);
    let d = diff(&rep, &parse("A,3\nB,2\n").unwrap(), &DiffOptions::default());
    assert!(d.is_empty());
}

#[test]
fn crlf_and_bom() {
    let f = parse("\u{feff}#NOTE: x\r\nA,1\r\nB,0.5\r\n").unwrap();
    assert_eq!(f.params.len(), 2);
    let f = parse_as(
        "# Onboard parameters for Vehicle 3\r\n3\t1\tA\t1\t9\r\n",
        Format::QGroundControl,
    )
    .unwrap();
    assert_eq!((f.vehicle_id, f.params[0].value), (Some(3), 1.0));
}

#[test]
fn errors_name_the_line() {
    let e = parse("A,1\nTHIS_NAME_IS_TOO_LONG,2\n").unwrap_err();
    assert_eq!(e.line, 2);
    assert_eq!(
        e.to_string(),
        "line 2: parameter name longer than 16 bytes (MAVLink param_id is char[16])"
    );
}

#[cfg(feature = "metadata")]
mod metadata {
    use super::*;
    use mavparam::metadata::{Metadata, Problem};

    const PDEF: &str = include_str!("data/apm.pdef.subset.json");

    #[test]
    fn loads_pdef() {
        let m = Metadata::from_json(PDEF).unwrap();
        assert_eq!(m.len(), 10);
        let fc = m.get("FRAME_CLASS").unwrap();
        assert!(fc.reboot_required);
        assert_eq!(fc.value_name(1.0), Some("Quad"));
        assert!(m.get("INS_ACC_ID").unwrap().read_only);
        let r = m.get("ATC_RAT_RLL_P").unwrap();
        assert_eq!(
            (r.range, r.units.as_deref()),
            (Some((0.0, 0.35)), Some("s/rad"))
        );
        assert_eq!(r.increment, Some(0.005));
        assert_eq!(m.get("LOG_BITMASK").unwrap().bitmask.len(), 20);
    }

    #[test]
    fn real_export_is_clean_where_described() {
        let m = Metadata::from_json(PDEF).unwrap();
        let f = parse(COPTER46).unwrap();
        let found = m.check(&f);
        assert!(
            found.iter().all(|x| x.problem == Problem::Unknown),
            "{found:?}"
        );
        // The excerpt holds 17 params; 10 are in the metadata subset.
        assert_eq!(found.len(), 7);
    }

    #[test]
    fn flags_range_enum_and_bits() {
        let m = Metadata::from_json(PDEF).unwrap();
        let f = parse(
            "ATC_RAT_RLL_P,0.5\nFRAME_CLASS,99\nLOG_BITMASK,16384\nINS_GYRO_FILTER,20\n\
             ACRO_RP_RATE,0\nSERIAL1_PROTOCOL,-1\n",
        )
        .unwrap();
        let found: Vec<String> = m.check(&f).iter().map(|x| x.to_string()).collect();
        assert_eq!(
            found,
            [
                "ATC_RAT_RLL_P = 0.5: outside the documented range 0 to 0.35 s/rad",
                "FRAME_CLASS = 99: not one of the listed values",
                "LOG_BITMASK = 16384: sets undefined bits 0x4000",
                "ACRO_RP_RATE = 0: outside the documented range 1 to 1080 deg/s",
            ]
        );
    }

    #[test]
    fn negative_bitmask_is_twos_complement() {
        // ArduPilot keeps bitmasks in signed AP_Int types, so the top bit
        // reads back as a negative number.
        let m = Metadata::from_json(
            r#"{"G": {"OPT": {"Bitmask": {"0": "A", "31": "B"}}, "O8": {"Bitmask": {"0": "A"}}}}"#,
        )
        .unwrap();
        let q = |line: &str| parse(&format!("1\t1\t{line}\n")).unwrap();
        assert!(m.check(&q("OPT\t-2147483647\t6")).is_empty());
        assert_eq!(
            m.check(&q("OPT\t-2147483646\t6"))[0].problem,
            Problem::UndefinedBits { bits: 2 }
        );
        // INT8 -128 is bit 7 only.
        assert_eq!(
            m.check(&q("O8\t-128\t2"))[0].problem,
            Problem::UndefinedBits { bits: 0x80 }
        );
        // Untyped: bit 31 alone is exact in f32.
        assert!(m.check(&parse("OPT,-2147483648\n").unwrap()).is_empty());
        assert_eq!(
            m.check(&parse("OPT,-4294967296\n").unwrap())[0].problem,
            Problem::NotABitmask
        );
        assert_eq!(
            m.check(&parse("O8,1.5\n").unwrap())[0].problem,
            Problem::NotABitmask
        );
    }

    #[test]
    fn range_compares_as_f32() {
        // 0.35f32 is 0.3499999940395355 and 0.35 as written; both are in range.
        let m = Metadata::from_json(PDEF).unwrap();
        let f = parse("ATC_RAT_RLL_P,0.35\n").unwrap();
        assert!(m.check(&f).is_empty());
        let m =
            Metadata::from_json(r#"{"G": {"X": {"Range": {"low": "0", "high": "0.3"}}}}"#).unwrap();
        // 0.3f32 is 0.30000001192092896, above 0.3 as an f64.
        assert!(m.check(&parse("X,0.3\n").unwrap()).is_empty());
        assert_eq!(m.check(&parse("X,0.3001\n").unwrap()).len(), 1);
    }
}

#[test]
fn readme_library_example() {
    let mp = mavparam::parse("ATC_RAT_RLL_P,0.1\nANGLE_MAX,3000\n").unwrap();
    let qgc =
        mavparam::parse("1\t1\tATC_RAT_RLL_P\t0.100000001490116119\t9\n1\t1\tANGLE_MAX\t4500\t4\n")
            .unwrap();
    let d = mavparam::diff(&mp, &qgc, &mavparam::DiffOptions::default());
    assert_eq!(d.changed.len(), 1);
    assert_eq!(d.changed[0].name(), "ANGLE_MAX");
    let text = mavparam::write(&qgc, mavparam::Format::MissionPlanner);
    assert_eq!(text, "ATC_RAT_RLL_P,0.1\nANGLE_MAX,4500\n");
}
