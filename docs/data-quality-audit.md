# Data-quality audit — every channel, hunting instability

**Date:** 2026-08-08, against `Tripper_2026-08-08_15-55-02.trip` and
`Tripper_2026-08-08_16-30-22.trip` (v0x05 firmware, post-sign-fix app),
cross-checked where relevant against the four earlier rides of the same day.
Method: systematic sweep of timing, GPS, barometer, CAN, IMU, cross-channel
physics consistency, and session totals, per channel, with thresholds; every
flagged row inspected by hand before it was allowed to stand.

Findings ranked by how much they lie to the rider.

---

## 1. SERIOUS — the g-force channels are still built on the condemned fusion

`currentG`, `longitudinalG`, `lateralG`, `verticalG` (and the session `maxG`,
whose 100 Hz peak the firmware latches from the same source) are produced by
rotating the BNO055's **linear acceleration** by the BNO055's **quaternion** —
both outputs of the fusion that `lean-investigation.md` convicted. Lean and
slope were rebuilt from raw channels; the g-forces never were.

Measured consequences:

| check | expectation | observed |
|---|---|---|
| lateralG vs v·ψ̇/g, whole ride | strong correlation | **+0.01** (15-55), **−0.17** (16-30) |
| hardest braking rows, 15-55 | −0.66 g (from wheel dv/dt) | −0.14 g recorded |
| hardest braking rows, 16-30 | −0.24 g | **+0.32 g** — wrong sign |
| currentG parked, 15-55 | ~0 | +0.12 g phantom |
| corner magnitudes | — | plausible on one ride, ~2× high on the other — exactly the fusion's ride-to-ride mood |

**Fix (app):** derive the g-forces the same way lean was rescued — raw
accelerometer minus a gravity unit vector built from the estimator's own lean
and slope: `linear = rawAccel − ĝ(lean, slope)`; longitudinal = x, lateral =
−y (right-positive), vertical = z. At standstill this reads exactly zero by
construction. `maxG` needs a firmware follow-up eventually (the 100 Hz peak is
latched from fusion linear-accel on the puck — candidate for v0x06:
latch `max(|rawAccel| − 1g)` instead).

## 2. MODERATE — phone GPS fixes with 40–50 m accuracy are swallowed whole

Ride 16-30, t=198.4 s: a fix with `horizontalAccuracy` 47 m lands ~30 m
sideways from the track (implied speed 168 km/h against a wheel reading 64),
plus 9.7 m/s of fake GPS-altitude change. The rows before it carry hAcc 39–47;
once accuracy recovers to <16 the track snaps back. On the Light build these
phone fixes feed the odometer (`distanceMeters`), `gpsSlope`, and the
`wheelScale` learner (the learner's bounds and gain kept it safe — verified —
but the odometer and gpsSlope take the hit).

**Fix (app):** gate odometer accumulation and wheelScale lessons on
`horizontalAccuracy` ≲ 20 m; let the track renderer keep bad fixes if it wants
the continuity, but never let them teach anything.

Related, lower severity: phone-GPS droughts of 15–22 s appear once per ride
(t=85 on 15-55, t=180 on 16-30 — pocketed phone, hillside). During a drought
nothing learns and slope's distance axis rides on the wheel alone, which is
its designed degradation. The Full puck's own GNSS would remove this class
entirely; noted, not urgent.

## 3. MODERATE — single-row pressure burps at constant speed

Ride 15-55, t=67.4–67.8: mid-descent at a steady 41 km/h, the barometer blips
+0.6 m for two rows and back; recorded slope swings −10% → +2.6% → −12% inside
1.5 s. This is *not* the braking/acceleration transient (speed constant — the
aero gate correctly ignores it): it is a pressure gust — wind, a passing
vehicle, a wall. Seven such steps >3 m/s on 16-30, two on 15-55.

**Fix (app, cheap):** median-of-3 on the altitude samples entering the fit
window. Kills every single-row burp for 0.4 s of added latency, which the 1 s
display smoothing already dwarfs. The least-squares fit is otherwise defencelss
against a single wild point in a 10 m window.

## 4. INFORMATIONAL — measured constants worth knowing

- **Wheel over-read vs GPS ground: +13.6% / +14.8%** on the two rides —
  stable, consistent with the investigation's 13%; this is the tyre/gearing
  calibration `wheelScale` divides out. Its stability across rides is
  reassuring: the learner is converging on a real constant.
- **Phone-vs-puck barometer relative drift: ±5–8 m over a ride.** Two sensors,
  two enclosures, weather — expected; nothing uses them interchangeably.
- **Heart rate coverage 67% / 34% of rows** — the known watch under-sampling
  saga (see the iOS repo's CLAUDE.md); no new information, not a puck issue.
- **Timing:** row cadence clean (one 0.55 s hiccup in 4330 rows); GPS-era
  duplicates are by design (fix-scoped fields).
- **CAN:** no speed dropouts, no kickstand flicker, SOC monotone, cell spread
  sane, rpm/speed ratio steady outside genuine wheelspin. The bus tap is
  clean.
- **IMU:** no gyro spikes, no freefall/clipping accel rows, quatRejects flat
  (boot-time only), gyro calibration 3/3 throughout. The v0x05 window-mean
  telemetry is behaving.

## What was checked and found healthy (so it isn't re-checked)

Row timing and monotonicity; CAN internal consistency (P≈V·I, rpm/speed,
SOC, cells, kickstand); gyro/accel outliers; quatRejects trend; calibration
status; verticalG zero at rest; session-vs-integrated distance (ratio matches
the known wheel over-read); baro-vs-pressure channel agreement.

## Recommended order of work

1. Rebuild g-forces from raw accel + estimator attitude (fixes four channels
   and the g-g display; session maxG follows with a firmware v0x06).
2. hAcc gate on odometer + wheelScale learning.
3. Median-of-3 on the fit's altitude input.
4. Eventually: Full-build GNSS on the bike to retire phone-GPS droughts.
