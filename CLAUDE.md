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
Telemetry is 92 B, `ver 0x07`: the 0x06 layout plus `canOdo_km` on the tail
(the bike's odometer, whole km, from CAN `0x402[2:4]` — see "The odometer"
below). 0x06 was 0x05 plus `accDev_mg`.
0x05 was byte-identical to 0x04 with a semantics change — the gyro/raw-accel
fields carry the **mean over the 200 ms packet window** (~20 samples at
100 Hz) instead of the newest sample, and the baro is read at 5 Hz (was 1 Hz —
it staircased under the app's 10 m slope fit). Status is 15 B:
14 + appended `otaState` (0 off · 1 AP up · 1+n = n WiFi clients).

Control writes: `0x01` marker · `0x02` mount zero (refused until accel cal
usable) · `0x03` identify · `0x04` ride state · `0x05` + [on u8] WiFi
flashing mode · `0x06` + [on u8] raw stream (boots **on**) · `0x07` + [mode u8]
ride-mode override (0 release · 1 Eco · 2 Sport) · `0x08` + [level u8] regen
override (0 release · 1–4). Status is 16 B since 0x07: `ovrState` on the tail,
bits 3:0 mode · bits 7:4 regen, reporting what the puck IS holding rather than
what was asked for.

### `accDev_mg` — the peak beside the mean (0x06, 2026-08-09)

Largest `|‖a‖ − 1 g|` seen at 100 Hz inside the packet's window, in mg.

It exists because **the app had a gate it could not trip**. The estimator only
takes its "down" reference from the accelerometer while `‖a‖` is near 1 g, but
it was testing the window MEAN — the one number structurally incapable of
showing a jolt inside its own window. The 100 Hz corpus settled it: real `‖a‖`
was outside 0.8–1.2 g on **17–20% of moving samples**, the means on **0.6–1.3%**.
The gate had been tuned against a signal already smoothed into compliance.

The general lesson, worth more than the field: *a summary statistic can hide
exactly the event a gate on that statistic is meant to catch.* When the app
needs to know something about the window rather than its average, compute it
on the puck — a mean cannot be un-averaged downstream.

Not `maxG_mg`, which is the peak of the fusion's LINEAR accel and therefore a
product of the fusion this channel exists to cross-check.

## The puck now transmits — bike control (0x07, 2026-08-10)

`0x490` is a **command from the dash to the motor controller**, not the echo it
was documented as for months. Two bench measurements settled it: on every mode
button press `0x490` moved 20–27 ms *before* `0x202`, and injecting `0x490`
with Eco while the dash went on sending Sport drove the controller's demand
floor 1100 → 750. So the bike obeys whoever spoke last on that ID, and
`TWAI_MODE_LISTEN_ONLY` became `TWAI_MODE_NORMAL` in both builds.

**"It never transmits, so it cannot disturb the bike" is no longer true.** What
replaces it: nothing is sent unless the rider asks, and every failure path
releases within 100 ms — BLE disconnect, CAN stale, or the rider pressing the
handlebar button, which always wins.

**React, never poll.** The controller never latches, and the dash re-asserts at
5 Hz forever, so the override answers each dash frame the instant it arrives.
Free-running at 20 Hz lost 24% of samples (a current limit oscillating between
750 and 1100 several times a second); replying on receipt held 297/297.

**CONFIRMED ON HARDWARE 2026-08-10.** Flashed over WiFi and tested with the
bike on a stand: the app sets the mode and the bike obeys. `tx` climbs by
exactly 5/s — one reply per dash frame, no waste — with `txerr=0`, `boff=0` and
the driver never leaving `run` over ~1000 transmits. The transmit path is clean;
`rxerr` blips to 4-5 occasionally and self-clears, which is what two nodes
sharing an arbitration ID look like.

**The bike's display cannot be updated, and never will be.** The dash renders
its own internal state and ignores the bus completely — during the 30 s
injection test `0x202` reported Eco throughout while the screen stayed on Sport,
so the dash ignores the CONTROLLER's frame too, not just ours. While an override
is held the bike has two disagreeing instruments and the app is the honest one
(its mode comes from `0x202`). Pressing the handlebar button releases the
override AND realigns the display, so the disagreement cannot get stuck.

**Regen is commandable but unverifiable.** It moves nothing else on the bus, the
dash never listens so its display cannot confirm it, and regen current is
unmeasurable because `0x203` power and `0x302` current are both unsigned. The
bike also inhibits regen above ~90% SOC. Nothing is confirmed here — the
spin-down test in `TODO.md` is what would settle it.

## The raw stream — characteristic `…0005` (added 2026-08-09)

100 Hz IMU samples, un-averaged, ten per notification at 10 Hz (148 B on the
wire, ~1.5 kB/s). **This is the most important thing added this week** and the
reason is worth keeping: the 5 Hz packet's window mean is a boxcar filter,
first sidelobe only 13 dB down, and suspension resonance (2–5 Hz) plus wheel
hop (10–20 Hz) sit above the 2.5 Hz Nyquist of a 5 Hz output. They therefore
**alias into the road-grade band** and no downstream filter can separate them
again. Every slope algorithm shipped before this was tuned against data that
had already destroyed the evidence needed to judge it.

Batch layout is in the README ("Raw stream"). Two rules:

- **Store raw, decide later.** Pressure in pascals not altitude, quaternion
  unzeroed, samples not means. A decision made at record time cannot be undone.
- **Time comes from the puck.** Each batch carries `t0_ms` from the ESP32's own
  millis; BLE delivers in bursts, so the phone's arrival time would smear every
  derivative. The phone's clock anchors ride start and nothing else.

Additive: a phone that never subscribes sees no change at all.

## What the first 100 Hz corpus said (2026-08-09, two Istanbul rides)

38,010 and 44,280 samples, exactly 100 Hz, **zero gaps, no truncation**. The
recorder needs no defending. Findings, all replayable from the two `.trip`
files — quote these rather than re-deriving them:

**The aliasing worry was right about slope and wrong about lean.** Share of
variance above 2.5 Hz (the 5 Hz output's Nyquist), moving rows:

| axis | >2.5 Hz | dominant |
|---|---|---|
| `gy` pitch rate | **67–71%** | 5–10 Hz (33–37%), peak 1.8–2.2 Hz |
| `az` | **92–94%** | 5–20 Hz, peak 8.5–11 Hz |
| `gx` roll rate | 2.4% | 0–1 Hz (90%) |
| `gz` yaw rate | 2.3% | 0–1 Hz (94%) |

Steering and roll are genuinely slow signals. **Lean was never at risk from
the 5 Hz packet** — don't spend effort there.

**The boxcar mean is exactly right for the gyro integration.** Integrating
pitch rate at 100 Hz versus from the 5 Hz means diverges by **3.1° over 379 s**
and 5.1° over 441 s (~0.01 °/s). The mean over a window IS the integral over
it, so the fusion's timing channel never lost anything. The 200 ms mean does
discard 63–64% of `gy` power and 91–92% of `az` power — that loss matters to
everything NONLINEAR downstream (gates, `atan2` of a direction), not to the
integrator. This corrects the founding claim that every earlier slope
algorithm was judged on ruined data: true of the accelerometer paths, false of
the gyro one.

**The vibration is structural, not wheel hop.** The `az` peak sits at 8.5–11 Hz
at every speed while its ratio to wheel rotation falls 3.7 → 1.35 from 8 to
60 km/h. A second, softer peak at 2.3–3.0 Hz in `gy` is the suspension's pitch
mode.

**The estimator layer earned its place immediately.** The braking gate fired
38 and 37 times, longest run 3.5 s and 3.9 s — the 4 s dead-reckoning shelf
life was never reached, so that rule is correctly sized. Slope was withdrawn
on 2.5% and 4.3% of rows (longest blackout 2.7 s and 7.3 s) — while
`imuSlopeDegrees` in `samples.json` showed **no blanks at all**, because the
sample writer holds the previous value. **Coverage numbers taken from
`imuSlopeDegrees` are worthless; read the `est` layer.**

**Roughness vs slope error is unsettled.** One ride says the roughest quartile
doubles the error (1.64° vs 0.84°, r = +0.31); the other says nothing
(r = +0.03). Two rides disagreeing is not a result.

### Ride provenance — read before quoting these two files

- `Tripper_2026-08-09_12-48-03` (Harbiye) was recorded with the bike's **gear
  ratio set to 1:7.5 when the bike is 1:8.4**. Its CAN speed reads **13.6%
  high** and everything derived from it inherits that. Correct by ×0.880
  before using it as a reference for anything.
- `Tripper_2026-08-09_12-54-59` (Tepebaşı) had the ratio correct and still
  reads **4.7% high** (×0.955) — that residual is the loaded rolling
  circumference of the rear knobby, ~1.93 m against a 2.019 m nominal for an
  80/100-19. Normal squash, not a fault.
- Measured against GPS Doppler speed, flat across every speed bin in both
  rides, so it is a scale error and nothing subtler. `wheelScale` learned
  0.866 and 0.951 against truths of 0.880 and 0.955 — **the learner was right
  to 1.5% both times, from a GPS with a fix on 10% of rows.** It was briefly
  written off as broken; it was measuring the gear ratio error correctly.
  What was genuinely wrong was learning at walking pace with no GPS speed in
  the last 13 seconds (0.361, kept for the next ride) — fixed app-side.

None of this belongs in firmware or app constants: every bike has different
rims, tyres and gearing, which is exactly why the app learns it.

## Flashing & wireless diagnostics

`docs/flashing.md` is the single reference (arduino-cli commands for USB and
WiFi, values, troubleshooting). Short version: FQBN `esp32:esp32:XIAO_ESP32S3`;
the app's Settings toggle raises a SoftAP (`Tripper-Light-OTA` /
`Tripper-Puck-OTA`, pass `tripper-ota`), puck at `192.168.4.1`, ArduinoOTA
upload password `tripper`. While the AP is up, `http://192.168.4.1/` is a
status page and `/log` serves an 8 KB RAM ring of debug lines **recorded since
boot** (reset reason in the `[boot]` line). The mode never survives a power
cycle, by design. First OTA-capable flash must go over USB.

**Flashing from a phone** (added 2026-08-09): while the AP is up,
`http://192.168.4.1/update` serves an HTML upload form — pick a `.bin` in
Safari and go, no toolchain, bike untouched. ArduinoOTA speaks espota, which
only Arduino tooling implements, which is why this exists. The firmware also
answers `/hotspot-detect.html` (and 404s) with Apple's success body, or iOS
decides there is no internet and drops back to cellular. The current app image
is committed at `releases/tripper_light.ino.bin` — rebuild and commit it in the
same commit as any firmware change. Only `*.ino.bin` (~1.2 MB, magic `0xE9`) is
flashable; `merged.bin` is a whole-flash image and will not work over OTA.

## Analysing rides — tools/

`.trip` files (recorded by the app) are `"TRIP"` + version byte + LZFSE JSON
`{session, samples}`. `tools/triplib.py` decodes them (macOS libcompression
via ctypes; elsewhere set `LZFSE_BIN` to a built lzfse CLI) and derives the
standard signals. `tools/harness.py` is the replay harness:

```bash
python3 tools/harness.py ride.trip        # needs numpy
```

`triplib.load_raw(path)` returns the 100 Hz layers from a `.trip` that has
them: `t` (rebuilt from the puck's clock), `gx/gy/gz` deg/s, `ax/ay/az` g,
per-batch `slow` context, and `est` — what the estimator believed on every
telemetry row (slope, lean, barometer anchor, fused angle, gate and gravity
flags, wheelScale). That last layer exists because a Python replica of the
Swift gate fired at different times than the real one on 2026-08-08, which made
every number it produced unciteable. Read what the app recorded; do not
re-implement it.

`harness.py` runs the step-1 fusion diagnostic, scores every lean/slope candidate
against corner-sign and straight-line metrics, and prints regression windows.
**Test filter ideas here, in seconds — rides are for collecting datasets, not
for testing ideas.** Add candidates to `LEAN_CANDIDATES` / `SLOPE_CANDIDATES`.

Baselines to beat, from `Tripper_20260807_232541.trip` (5 Hz, pre-v0x05
firmware): shipped estimator 15/15 corners, straight sd 2.78°, >5° 7.5%; the
v0x05 + app-fix pipeline replayed at sd 2.59°, >5° 4.4%.

## Open items

- **Validation ride pending on `ver 0x06` + the gated wheelScale learner.**
  Both were written 2026-08-09 after the offroad rides and neither has been
  ridden. Check that `puck_accel_dev_g` actually fires the trust gate (expect
  it above 0.2 g on roughly a fifth of moving rows) and that `wheelScale`
  settles instead of flailing.
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
- **A comment describing a wire format is part of that format.** The raw slow
  row was documented as 28 B while the code wrote 26; the first decoder trusted
  the comment and read every batch two bytes off. If a struct changes, the
  comment, the README table and the Python decoder change in the same commit.
- Claims about data get checked against the data before they are stated. Several
  hours went into fixes that were reported before being verified, and the rider
  had to catch them.
- Solo project, direct on `main`; commit/push only when asked.
