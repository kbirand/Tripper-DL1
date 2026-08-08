# Tripper-DL1 — notes for Claude

ESP32-S3 (XIAO) BLE sensor pucks for an e-bike: BNO055 IMU + BMP280 barometer
+ Talaria CAN tap, streaming 5 Hz telemetry to the Tripper iOS app (sibling
repo `../Tripper`). Two builds, wire-compatible on purpose: **Full**
(`tripper_puck.ino` — GPS, OLED, buttons) and **Light** (`tripper_light.ino` —
IMU + baro + CAN only). The rider currently runs Light.

**Keep the two `.ino` files in step** — packet structs, CAN decode, UUIDs and
the OTA block are duplicated by design, and every change lands in both.

## The verdict that shapes everything (settled 2026-08-08 — do not relitigate)

The BNO055's fusion **invents attitude its own sensors contradict**. Proven by
the v0x04 raw-accel channel on `Tripper_20260807_232541.trip`: on 118 s of
straight, steady riding the raw accelerometer's implied roll was −0.19° ± 1.04°
while the fusion's roll wandered ±13.6° (to −24° at 69 km/h), correlation
+0.018 between them. Corner sign inverted in 14/15 corners. Mount, axes, gyro,
barometer, I2C (flat quatRejects) all cleared. Full history and the ruled-out
table: `docs/lean-investigation.md`; what a correct instrument would look like:
`docs/direct-attitude-sensing.md`.

Consequences baked into the code:

- The quaternion/linear-accel channel is **diagnostic only** — kept in the
  packet deliberately (a wrong channel you record is a diagnosable channel).
  The app computes lean and slope itself from gyro + CAN speed + baro.
- **No sensor purchase fixes lean** — in a balanced corner the net force runs
  through the bike, so any gravity-referenced device reads upright (measured:
  lateral g 0.015 while moving). Lean = `atan(v·ψ̇/g)` + gyro, period.
- **Slope's distance axis must be GPS-scaled, never raw wheel** — the driven
  wheel over-read 152% on a climb at 60% slip (`docs/lean-investigation.md`,
  "Slope's distance").

## Sign conventions (empirically anchored — see the iOS repo's CLAUDE.md too)

Sensor frame: X forward, Y rider's left, Z up. So a right-hand turn is
**negative** gyro Z (right-hand rule), and the app's right-positive lean scale
means its kinematic term negates gyro.z. Right roll rate = +gyro X. Raw-accel
roll, right-positive, = `atan2(+accY, +accZ)`; the bike parked on its
(left-side) kickstand reads ≈ −9.5° and anchors the sign.

## Packet protocol — the additive rule

Never reorder or resize existing fields. New fields append; the app gates on
**length**, not version, so old apps parse new packets and drop the tail.
Telemetry is 86 B, `ver 0x05`: byte-identical layout to 0x04, but the
gyro/raw-accel fields carry the **mean over the 200 ms packet window** (~20
samples at 100 Hz) instead of the newest sample, and the baro is read at 5 Hz
(was 1 Hz — it staircased under the app's 10 m slope fit). Status is 15 B:
14 + appended `otaState` (0 off · 1 AP up · 1+n = n WiFi clients).

Control writes: `0x01` marker · `0x02` mount zero (refused until accel cal
usable) · `0x03` identify · `0x04` ride state · `0x05` + [on u8] WiFi
flashing mode.

## Flashing & wireless diagnostics

`docs/flashing.md` is the single reference (arduino-cli commands for USB and
WiFi, values, troubleshooting). Short version: FQBN `esp32:esp32:XIAO_ESP32S3`;
the app's Settings toggle raises a SoftAP (`Tripper-Light-OTA` /
`Tripper-Puck-OTA`, pass `tripper-ota`), puck at `192.168.4.1`, ArduinoOTA
upload password `tripper`. While the AP is up, `http://192.168.4.1/` is a
status page and `/log` serves an 8 KB RAM ring of debug lines **recorded since
boot** (reset reason in the `[boot]` line). The mode never survives a power
cycle, by design. First OTA-capable flash must go over USB.

## Analysing rides — tools/

`.trip` files (recorded by the app) are `"TRIP"` + version byte + LZFSE JSON
`{session, samples}`. `tools/triplib.py` decodes them (macOS libcompression
via ctypes; elsewhere set `LZFSE_BIN` to a built lzfse CLI) and derives the
standard signals. `tools/harness.py` is the replay harness:

```bash
python3 tools/harness.py ride.trip        # needs numpy
```

It runs the step-1 fusion diagnostic, scores every lean/slope candidate
against corner-sign and straight-line metrics, and prints regression windows.
**Test filter ideas here, in seconds — rides are for collecting datasets, not
for testing ideas.** Add candidates to `LEAN_CANDIDATES` / `SLOPE_CANDIDATES`.

Baselines to beat, from `Tripper_20260807_232541.trip` (5 Hz, pre-v0x05
firmware): shipped estimator 15/15 corners, straight sd 2.78°, >5° 7.5%; the
v0x05 + app-fix pipeline replayed at sd 2.59°, >5° 4.4%.

## Open items

- **Validation ride pending** on v0x05 firmware + the fixed app — compare
  against the Aug 7 baseline with the harness, especially the window-mean
  gyro's effect on straight-line sd.
- Bias-learning gains (2 s settle, 0.5 pull) are first guesses; tune from
  ride data, not intuition.
- `docs/lean-investigation.md` still says "cause not yet identified" — the
  2026-08-08 analysis answered it (fusion at fault); the doc deserves a
  closing section.
- Recordings made before 2026-08-08 carry lean with the **mirrored sign**
  (the L/R fix); don't compare signs across that boundary.

## Conventions

- Comments explain *why*; anything sign- or convention-shaped cites the ride
  that anchored it.
- Solo project, direct on `main`; commit/push only when asked.
