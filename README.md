# mavparam: diff drone parameter files the way the autopilot stores them

[![crates.io](https://img.shields.io/crates/v/mavparam.svg)](https://crates.io/crates/mavparam)
[![docs.rs](https://docs.rs/mavparam/badge.svg)](https://docs.rs/mavparam)
[![CI](https://github.com/Mattbusel/mavparam/actions/workflows/ci.yml/badge.svg)](https://github.com/Mattbusel/mavparam/actions/workflows/ci.yml)

**`0.1` and `0.100000001490116119` are the same parameter value. A text diff
says they are not.**

A MAVLink parameter is a 32-bit float on the wire (`PARAM_VALUE.param_value`)
and ArduPilot stores it that way. Ground stations then write it to disk however
they like: a `.param` file says `0.1`, older QGroundControl releases with 18
decimals (`0.100000001490116119`), current QGroundControl as the shortest `%g`
text (`3.6e+02`), MAVProxy `param save` as `%f`, six decimals, rounding
anything smaller away. Compare those files as text, or as 64-bit doubles, and
you get changes that are not changes. In a real BlueROV2 ArduSub 3.5.2 export
from QGroundControl ([brov2-heavy-3_5_2.params](https://github.com/bluerobotics/bluerobotics.github.io/blob/master/downloads/brov2-heavy-3_5_2.params),
604 parameters), 40 of the 171 float values parse to a different double than
the same value written short.

mavparam reads all three formats, compares every value as the f32 the vehicle
holds, and writes each format back out without losing a bit. Here a real
ArduCopter 4.6.3 QGroundControl export
([x650-arducopter-4.6.params](https://github.com/JacopoPan/aerial-autonomy-stack/blob/main/start_here/docs/params/x650-arducopter-4.6.params),
1,067 parameters), converted to a Mission Planner `.param`, is compared with a
copy of the export with three values edited and one line deleted:

```console
$ mavparam convert x650.params before_update.param
$ mavparam diff before_update.param after_update.params --meta apm.pdef.json
--- before_update.param (Mission Planner, 1067 params)
+++ after_update.params (QGroundControl, 1066 params)
~ ATC_RAT_RLL_P     0.135 -> 0.4  [outside 0 to 0.35 s/rad]
~ FRAME_CLASS       1 (Quad) -> 2 (Hexa)  [reboot required]
~ INS_GYRO_FILTER   20 -> 40
- STAT_BOOTCNT      134
3 changed, 0 added, 1 removed
```

The two sides are different formats. The diff matches them by name (and
MAVLink component), finds no change in the 1,063 values that were not edited,
and, with ArduPilot's published metadata, says which changes need a reboot and
which leave the documented range. Exit code 0 means identical, 1 means
different, so it drops into CI or a pre-flight script as is.

## What it reads

| Format | Written by | Line |
|---|---|---|
| Mission Planner `.param` | Mission Planner, ArduPilot `Tools/Frame_params` | `NAME,VALUE` |
| MAVProxy `.parm` | MAVProxy `param save`, ArduPilot default param files | `NAME VALUE` (spaces or tab) |
| QGroundControl `.params` | QGroundControl, ArduPilot and PX4 | `vehicle<TAB>component<TAB>NAME<TAB>VALUE<TAB>MAV_PARAM_TYPE` |

The format is sniffed from the content, not the file name. Mission Planner and
MAVProxy files are read with the same rules as pymavlink
[`mavparm.load`](https://github.com/ArduPilot/pymavlink/blob/master/mavparm.py)
(`#` comments anywhere on a line, comma or whitespace, `0x` hex values).
QGroundControl files follow `ParameterManager` in
[QGroundControl](https://github.com/mavlink/qgroundcontrol): five tab separated
fields with the `MAV_PARAM_TYPE` number, which mavparam keeps.

## Use

```sh
cargo install mavparam --features cli
```

```sh
mavparam diff a.param b.params                     # any format against any format
mavparam diff a.parm b.parm --abs 5e-7             # ignore MAVProxy's 6-decimal rounding
mavparam diff a.params b.params --json             # machine readable
mavparam convert vehicle.params vehicle.param      # format from the extension
mavparam convert frame.param - --to params --types vehicle.params
mavparam check vehicle.params --meta apm.pdef.json
```

ArduPilot publishes metadata per vehicle at
`https://autotest.ardupilot.org/Parameters/<ArduCopter|ArduPlane|Rover|ArduSub|...>/apm.pdef.json`.

As a library:

```toml
[dependencies]
mavparam = "0.1"
```

```rust
let mp = mavparam::parse("ATC_RAT_RLL_P,0.1\nANGLE_MAX,3000\n")?;
let qgc = mavparam::parse(
    "1\t1\tATC_RAT_RLL_P\t0.100000001490116119\t9\n1\t1\tANGLE_MAX\t4500\t4\n",
)?;
let d = mavparam::diff(&mp, &qgc, &mavparam::DiffOptions::default());
assert_eq!(d.changed.len(), 1); // ANGLE_MAX only
assert_eq!(d.changed[0].name(), "ANGLE_MAX");

// Write any file in any format; values read back bit for bit.
let text = mavparam::write(&qgc, mavparam::Format::MissionPlanner);
assert_eq!(text, "ATC_RAT_RLL_P,0.1\nANGLE_MAX,4500\n");
```

With `features = ["metadata"]`, `mavparam::metadata::Metadata::from_json`
loads `apm.pdef.json` and `check` returns the values outside the documented
range, enum values that are not listed, undefined bitmask bits and names the
metadata does not know.

## What it gets right

* **Float32 equality.** Untyped and `REAL32` values compare as f32. Two
  integer-typed values (QGroundControl `INT32`, for example a 24-bit device id)
  compare exactly, and so do two `REAL64` values. `-0` equals `0`.
* **Lossless writers.** Every value is written as the shortest text that reads
  back to the same stored value, checked over 200,000 random f32 bit patterns
  in the test suite. Integer types are written as integers, never `4.000000`.
  The MAVProxy writer keeps the 16 column layout but drops the `%f` rounding.
* **Strict reading.** A malformed line is an error with its line number.
  `mavparm.load` prints "Invalid line" and carries on, so a half-read file can
  load silently. Names longer than 16 bytes are rejected, because MAVLink's
  `param_id` is `char[16]` and the vehicle cannot hold them. Integer-typed
  QGroundControl values with a fraction or outside their type's range are
  rejected too.
* **Range checks in f32.** `0.3f32` is `0.30000001192...`, above a documented
  high of `0.3` as a double. mavparam does not flag it.
* **Components.** A QGroundControl file can hold parameters for several
  components. They diff separately; a Mission Planner or MAVProxy file counts
  as component 1, the autopilot.
* **`no_std`.** Reading, writing and diffing need `alloc` only, and build for
  `thumbv7em-none-eabihf`. `#![forbid(unsafe_code)]`, no dependencies outside
  the `metadata` feature (`serde_json`).

## What to know

* ArduPilot ranges are recommendations, not limits the firmware enforces, and
  several parameters use `0` for "disabled" below their range. Checking the
  x650 ArduCopter 4.6.3 export above against the ArduCopter metadata fetched
  on 2026-11-22 reports 8 values outside a range (`BRD_VSERVO_MIN = 0`,
  `FENCE_TOTAL = 0` and others) and one unlisted enum value
  (`BRD_TYPE = 43`): things to look at, not errors. The published file is
  merged across frame types, so some ranges (the `ATC_RAT_*` ones) are the
  helicopter controller's. When a parameter has both a range and listed
  values, the values are presets and only the range is checked.
* Use the metadata for the firmware you run. The same check reports 143 of
  the 1,067 names as unknown, because the published file is newer than 4.6:
  it has `ATC_ANGLE_MAX` and `MAV_SYSID` where 4.6 has `ANGLE_MAX` and
  `SYSID_THISMAV`.
* A Mission Planner or MAVProxy file has no types. Converting one to
  `.params` writes `REAL32` (9) unless `--types` points at a QGroundControl
  export of the same vehicle.
* Not yet supported: ArduPilot defaults-file directives (`@include`,
  `@READONLY`) are reported as errors rather than expanded, and PX4 metadata
  (`parameters.json`) is not loaded.

## License

MIT or Apache-2.0, at your option.
