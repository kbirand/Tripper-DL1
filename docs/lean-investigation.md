# Lean and slope: what is wrong, what has been ruled out, what to check next

**Status:** open. Two rides investigated, five hypotheses eliminated, the cause
not yet identified. Firmware v0x04 exists to make the next ride diagnosable.

Read this before touching attitude code. It exists so the next person — most
likely us in three weeks — does not re-run experiments that have already been
run, and does not re-adopt an explanation that has already been disproved.

---

## The symptom

Lean is unusable, and slope is worse than it looks. From
`Tripper_2026-08-03_16-10-18.trip` (401 s, 2.9 km, max 95 km/h, puck IMU and
CAN at 5 Hz, GPS at 1 Hz):

| | 2026-07-31 ride (pre-fix) | 2026-08-03 ride (post-fix) |
|---|---|---|
| straight-line mean lean | −1 … −30° wandering | −4.4° (sd 4.1) |
| straight riding reading \|lean\| > 10° | 48% | 9.0% (>5°: 45%) |
| correlation vs `atan(v·ω/g)` | −0.52 | −0.20 |
| corners with the correct sign | 0/14 | 2/14 |

Commit 7ac31e7 (body-side `qRef`, tilt-only reference) is a genuine fix and its
reasoning still holds — it is simply not what is producing the readings above.

Two windows worth keeping as regression cases, both spotted by eye during
playback and both confirmed in the data:

- **t = 230–239 s, dead straight** (course 340–359°, 30–50 km/h). Reported lean
  sits at −8 to −12° for ten continuous seconds. The puck's own gyro says the
  bike is not rolling during that time.
- **t = 353–365 s, descending.** Reported slope +1.3°. Puck barometer −1.5°,
  phone barometer −2.9°, GPS altitude −6.0 m over 92 m. Three independent
  witnesses against one.

Over the whole ride, slope is off by −3.1° on average with ±5.2° of scatter
against the barometric road grade. Its +0.41 correlation looks respectable and
hides exactly this.

## Ruled out — do not revisit without new evidence

| Hypothesis | How it was tested | Result |
|---|---|---|
| Mount, axes or zero wrong | Roll and pitch at all six standstills in the ride | Return to ≈0 every time (−2.6…+0.9°). A static error would be present at rest too |
| Gyro drift | Signed gyro mean at each standstill | ≤0.13 deg/s. Would take minutes to build 10°, and the errors come and go rather than accumulate |
| Frame algebra still wrong (residual mount yaw) | Regressed attitude-implied body rates onto the raw gyro | M ≈ diag(0.81, 0.18, 0.94), off-diagonals small — the body-side `qRef` is doing what it claims |
| The bike's own acceleration read as tilt | Subtracted `atan(a/g)` using CAN dv/dt from reported slope | Made it **worse**: correlation 0.41 → 0.25, sd 5.21 → 7.54 |
| Vibration level | Correlated \|roll\| against local gyro spread and accel shake on straight rows | +0.13 / +0.30, and the worst window (3:50) has *below*-average vibration |
| Puck not rigidly on the bike | `puckGyroZ` vs GPS course rate; `puckGyroX` at corner entries | Z is up, X is forward, gyro sees every corner. Hardware and mounting are sound |

Two explanations were offered to the rider and then withdrawn, both recorded
here so they are not re-proposed: *coordinated-turn geometry* (true, but cannot
produce a wrong reading on a straight road, so it does not explain 3:50), and
*acceleration mistaken for tilt* (tested and refuted, row 4 above).

## Established as healthy

- **The gyro.** Bias ≤0.13 deg/s at rest, correct turn direction, and it
  matches the attitude's own rate of change at r = +0.87. The *changes* in
  attitude are right; the absolute angle is not.
- **The mount, the axes and the mount zero**, per the table above.
- **The barometer.** Grade from puck baro over distance travelled tracks GPS
  slope at +0.75 and needs no attitude at all.

## The ceiling on lean, independent of this bug

In a steady corner the force a leaned bike feels runs straight through the
bike — that is what stops it falling over. Measured on this ride: lateral g
averages +0.015 (sd 0.12) while moving above 20 km/h, above 0.2 g on only 10%
of rows. **Any device that infers tilt by feeling gravity therefore reads
"upright" mid-corner**, from a $5 accelerometer to a dedicated inclinometer.

So lean must ultimately come from `atan(v·ψ̇/g)` — CAN speed and the gyro's
vertical-axis rate — with the gyro filling in the transitions. Validated
offline against this recording at 5 Hz (the firmware would run 100 Hz), τ = 1–2 s:

| | as shipped | gyro + kinematic reference |
|---|---|---|
| corner sign agreement | 2/14 | 10/12 |
| straight-line mean | −4.4° (sd 4.1) | −0.14° (sd 2.9) |
| max lean | — | 23° (plausible) |

This is necessary but **not sufficient**: it does not explain or fix the 3:50
straight-line window.

## What v0x04 adds, and why

Everything in the packet that bears on attitude — quaternion, linear accel,
even the calibration byte — is a *product* of the BNO055's fusion. They
corroborate each other by construction, whatever the fusion decides. That is
why the investigation ran out of evidence: every remaining witness was the
accused.

v0x04 (86 bytes) appends the **raw accelerometer** (offset 78, sensor frame,
gravity included, sampled in the same 10 ms tick as the quaternion) and
**`quatRejects`** (offset 84). See the packet table in `README.md`.

## What to check when the next ride lands

1. **`acc` direction vs reported attitude, across t = 230–239 s equivalent** —
   a straight, steady stretch. Low-pass `acc` over ~2 s and compare its
   direction with the roll/pitch the quaternion reports.
   - Raw accel points true down, quaternion says −10° → **the fusion is at
     fault.** Stop using its attitude; run our own filter on raw accel + gyro,
     or move to a BNO085/BNO086.
   - Raw accel is itself tilted while the bike is provably straight → **the
     accelerometer is being fed a tilted "down"** — mount flex, resonance or
     rectified vibration. A new chip would read the same nonsense; fix the
     mechanics.
2. **`acc − lin`** — the fusion's own gravity vector. If it disagrees with the
   quaternion's down, the fusion is internally inconsistent, which is
   conclusive on its own.
3. **`quatRejects`** — flat is healthy. Climbing mid-ride means the attitude is
   being *held* across glitched I2C reads rather than tracking the bike.

### Bench test that needs no ride

Bike upright on the stand, puck powered, motor running so it vibrates. Watch
reported lean for a minute. If it wanders more than a degree while the bike is
provably upright, the fusion (or the mount) is confirmed at fault and no
algorithm layered on its output will save it. Then have someone push the bike
in a straight line and watch again.

## Sensor replacement — only after step 1 answers

- **Slope:** buy nothing. The BMP280 already on the puck, plus CAN speed, beats
  the IMU's pitch today. Altitude over distance needs no attitude, so it cannot
  be corrupted by any of this.
- **Lean:** no sensor fixes it — see "the ceiling" above. The BNO055's gyro is
  already good enough.
- **If replacing anyway:** BNO085/BNO086 is the sensible successor — same I2C,
  same 3.3 V, Adafruit breakout, far less opaque fusion, exposes raw sensors
  properly. Not a drop-in code swap: different library (SH-2), about a day.

## Reproducing the analysis

`.trip` files are `"TRIP"` + a version byte + an LZFSE-compressed JSON payload
of `{session, samples}`. On macOS, `compression_decode_buffer` from
`/usr/lib/libcompression.dylib` with `COMPRESSION_LZFSE = 0x801` decompresses
the body; no third-party library needed. The fields used above are
`rollDegrees`, `pitchDegrees`, `imuSlopeDegrees`, `puckGyro{X,Y,Z}`,
`canSpeedKmh`, `puckBaroAltitudeMeters`, `latitude`/`longitude` and
`lateralG`.

Course and turn rate should be derived from GPS **positions**, not from
`headingDegrees` — the two agree here (median offset −0.3°) but the position
track is the one that can be trusted at low speed. Sanity-check any GPS
reference against CAN speed first: on this ride they correlate at +0.95.
