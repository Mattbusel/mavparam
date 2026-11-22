# Changelog

## 0.1.0

* Readers and writers for Mission Planner `.param`, MAVProxy `.parm` and
  QGroundControl `.params`, with format sniffing.
* `diff` with float32 equality and optional absolute or relative tolerance.
* `metadata` feature: ArduPilot `apm.pdef.json` ranges, enum values, bitmasks,
  reboot-required and read-only flags.
* `mavparam` CLI (`--features cli`): `diff`, `convert`, `check`.
