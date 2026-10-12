#!/usr/bin/env python3
"""Repair standstill slope in rides recorded before the 2026-08-18 estimator fixes.

WHAT WAS WRONG
--------------
Two faults in the app's stopped-bike pitch channel, both fixed in
BikeAttitudeEstimator on 2026-08-18, both visible only while the bike was
stationary on a steep, loose climb:

  * The wheel's dv/dt was subtracted from the gravity reference unguarded. A
    rear wheel spinning under a stationary bike reports 0 -> 40 km/h and back,
    and the collapse read as ~1 g of braking that never happened, inflating
    the grade (174% at 00:19:21 on 2026-08-16 where the accelerometer held 33°).
  * The slope-source crossfade anchored on `?? 0`. A switch that landed while
    the estimator had withdrawn faded up from an imaginary flat road, ramping
    0 -> 33° over a second (00:18:51.0 and 00:18:52.6 on the same ride).

WHY THIS REPAIR IS DEFENSIBLE
-----------------------------
Neither fault can be undone by replaying the estimator in Python — the result
would depend on internal filter state, and a replica that fires at different
times than the Swift produces numbers nobody can cite. This script does not
replay anything. It repairs only rows where the bike is PROVABLY stationary:

    CAN speed below 2.5 km/h (the app's own gravity-mode threshold) and the
    puck's window-PEAK accelDeviation inside the app's own 0.2 g gate, with
    loose gyro and |a| sanity bounds.

On such a row the accelerometer is a true inclinometer — there is no
acceleration to confuse with tilt — so the slope is `atan2(aX, hypot(aY, aZ))`
minus the puck's mount offset, with no estimator state involved. That is
exactly what the fixed app computes there.

The mount offset is not assumed. It is recovered from the ride itself as the
median of (accelerometer pitch - recorded slope) over rows that are stationary
AND have had no wheel movement for three seconds — rows where the channel was
demonstrably working. Across the four 2026-08-16 rides it measured -2.28° with
an interquartile range of 0.06°, identical in all four.

Rows are repaired only where the recorded value differs from that truth by
more than `THRESHOLD`; below it the difference is the app's legitimate
smoothing lag and rewriting would add noise, not accuracy. Each repaired
stretch is extended by `BLEND_SECONDS` on both sides and rebuilt through the
same 0.5 s low-pass the app uses, seeded from the app's own value at the
stretch's start, so the repair enters and leaves continuously instead of
stepping.

Everything else is untouched: moving rows, lean, the barometer, CAN, GPS.

USAGE
-----
    python3 tools/repair_standstill_slope.py RIDE.trip [...]     # dry run
    python3 tools/repair_standstill_slope.py --apply DIR ...

`--apply` rewrites in place after copying each original into a sibling
`pre-stillfix/` directory.
"""
import argparse
import glob
import math
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
import triplib  # noqa: E402

THRESHOLD = 3.0        # deg; below this the disagreement is smoothing lag
BLEND_SECONDS = 0.5    # lead-in / lead-out around each repaired stretch
BRIDGE_SECONDS = 2.0   # longest run of untrustworthy rows bridged by interpolation
TAU = 0.5              # s, the app's thetaSmoothTau
QUIET_SECONDS = 3.0    # wheel-still time required to measure the mount offset


def columns(samples, key):
    return np.array([s.get(key) if s.get(key) is not None else np.nan
                     for s in samples], dtype=float)


def analyse(samples):
    """Return time, recorded slope, accelerometer truth, its bridged form, and the still mask."""
    t = columns(samples, "elapsedSeconds")
    deg = columns(samples, "imuSlopeDegrees")
    v = columns(samples, "canSpeedKmh")
    dev = columns(samples, "puckAccelDeviationG")
    ax = columns(samples, "puckAccelX")
    ay = columns(samples, "puckAccelY")
    az = columns(samples, "puckAccelZ")
    gx = columns(samples, "puckGyroX")
    gy = columns(samples, "puckGyroY")
    gz = columns(samples, "puckGyroZ")

    norm = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)
    pitch = np.degrees(np.arctan2(ax, np.hypot(ay, az)))
    gyro = np.sqrt(gx ** 2 + gy ** 2 + gz ** 2)

    # Trust the accelerometer on the same terms the app itself does, plus the
    # bike not going anywhere. `dev` is the puck's window PEAK departure from
    # 1 g, so <= 0.2 certifies the whole 200 ms, not an average of it — that is
    # the app's `accelIsUsable`. The gyro and |a| bounds are loose on purpose:
    # a stationary bike being shoved on a hill rotates a few deg/s and its |a|
    # wanders a few per cent, and tight limits here excluded precisely the rows
    # carrying the worst spikes (00:19:21.0 on 2026-08-16: 0.0 km/h, dev 0.093,
    # gyro 2.9 deg/s — stationary by every measure, and reading 60 deg).
    still = ((v < 2.5) & (gyro < 10) & (np.abs(norm - 1) < 0.06)
             & (dev <= 0.2) & np.isfinite(pitch) & np.isfinite(deg))

    # The mount offset, measured where the channel was demonstrably working:
    # stationary, and no wheel movement at all for the previous QUIET_SECONDS.
    dt = float(np.median(np.diff(t)))
    back = max(1, int(round(QUIET_SECONDS / dt)))
    quiet = (still & (v == 0) & (gyro < 2) & (np.abs(norm - 1) < 0.02))
    for i in np.where(quiet)[0]:
        if i >= back and np.any(v[i - back:i] > 0):
            quiet[i] = False
    if quiet.sum() < 50:
        raise ValueError("too few quiet standstill rows to measure the mount offset")
    offset = float(np.median((pitch - deg)[quiet]))
    truth = pitch - offset

    # Bridge the untrustworthy rows before looking for disagreements, not
    # after. Those rows — the jolts, where the puck's gate shut — are where the
    # worst readings live, and a rule that can only START from a trustworthy
    # row cannot reach a burst made entirely of untrustworthy ones (00:19:11.6
    # on 2026-08-16 sat at 6° for 0.6 s inside a stretch reading 33°). A gap
    # bounded on both sides by the accelerometer, and shorter than
    # BRIDGE_SECONDS, is one stationary moment with a shake in the middle; a
    # longer one is the bike riding away, and stays unbridged.
    bridged = np.where(still, truth, np.nan)
    edges = np.flatnonzero(still)
    if len(edges) >= 2:
        for a, b in zip(edges, edges[1:]):
            if b - a > 1 and t[b] - t[a] <= BRIDGE_SECONDS:
                span = np.arange(a + 1, b)
                bridged[span] = np.interp(t[span], [t[a], t[b]], [truth[a], truth[b]])
    return t, deg, truth, bridged, still, offset, quiet.sum(), dt


def stretches(mask, t, gap):
    """Contiguous groups of True in `mask`, merged across gaps up to `gap` s."""
    idx = np.where(mask)[0]
    out = []
    i = 0
    while i < len(idx):
        j = i
        while j + 1 < len(idx) and t[idx[j + 1]] - t[idx[j]] <= gap:
            j += 1
        out.append((idx[i], idx[j]))
        i = j + 1
    return out


def _write(samples, i, value):
    samples[i]["imuSlopeDegrees"] = value
    if samples[i].get("pitchDegrees") is not None:
        samples[i]["pitchDegrees"] = value
    if samples[i].get("imuSlopePercent") is not None:
        samples[i]["imuSlopePercent"] = math.tan(math.radians(value)) * 100


def repair(samples):
    t, deg, truth, bridged, still, offset, nquiet, dt = analyse(samples)
    have = np.isfinite(bridged)
    bad = have & (np.abs(deg - bridged) > THRESHOLD)
    if not bad.any():
        return 0, 0, offset, nquiet, 0.0

    lead = max(1, int(round(BLEND_SECONDS / dt)))
    touched = 0
    worst_step = 0.0
    written = np.zeros(len(t), dtype=bool)

    for a, b in stretches(bad, t, gap=BRIDGE_SECONDS):
        # Extend into neighbouring rows the accelerometer still covers, so the
        # repair has clean ground to start and finish on.
        lo, hi = a, b
        while lo > 0 and have[lo - 1] and a - lo < lead:
            lo -= 1
        while hi + 1 < len(t) and have[hi + 1] and hi - b < lead:
            hi += 1
        seg = np.arange(lo, hi + 1)
        target = bridged[seg]
        if not np.isfinite(target).all():
            continue

        # Seeded on the measured value, not on `deg[lo]`: seeding on the app's
        # own number made the filter inherit the error it is here to remove and
        # walk out of it over a second and a half, leaving the worst spike at
        # 148% instead of the 70% the accelerometer was reading.
        y = np.empty(len(seg))
        v = target[0]
        for n in range(len(seg)):
            v += (target[n] - v) * dt / (TAU + dt)
            y[n] = v
        for n, i in enumerate(seg):
            _write(samples, i, y[n])
            written[i] = True
            touched += 1

        # Blend across the boundaries FROM the repaired value TOWARDS the
        # recorded one. Never by adding a residual to it — an added residual
        # pushed one exit row to 208%, worse than the spike being repaired.
        for direction, edge, value in ((-1, lo, y[0]), (1, hi, y[-1])):
            last = value
            for n in range(1, lead + 1):
                i = edge + direction * n
                if i < 0 or i >= len(t) or written[i]:
                    break
                f = n / (lead + 1)
                last = value * (1 - f) + deg[i] * f
                _write(samples, i, last)
                written[i] = True
                touched += 1
            else:
                i = edge + direction * (lead + 1)
                if 0 <= i < len(t):
                    worst_step = max(worst_step, abs(last - deg[i]))
    return touched, int(bad.sum()), offset, nquiet, worst_step


def trip_version(path):
    with open(path, "rb") as f:
        head = f.read(5)
    if head[:4] != b"TRIP":
        raise ValueError(f"{path}: not a .trip file")
    return head[4]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--apply", action="store_true")
    args = ap.parse_args()

    files = []
    for p in args.paths:
        files.extend(sorted(glob.glob(os.path.join(p, "*.trip")))
                     if os.path.isdir(p) else [p])
    if not files:
        sys.exit("no .trip files found")

    for path in files:
        version = trip_version(path)
        session, samples = triplib.load_trip(path)
        touched, bad, offset, nquiet, step = repair(samples)
        name = os.path.basename(path)
        print(f"{name}: mount offset {offset:+.2f}° (from {nquiet} quiet rows) — "
              f"{bad} rows beyond {THRESHOLD:.0f}°, {touched} rewritten incl. blend, "
              f"worst exit step {step:.2f}°" + ("" if args.apply else "   [dry run]"))
        if not args.apply or not touched:
            continue
        backup_dir = os.path.join(os.path.dirname(os.path.abspath(path)),
                                  "pre-stillfix")
        os.makedirs(backup_dir, exist_ok=True)
        backup = os.path.join(backup_dir, name)
        if not os.path.exists(backup):
            shutil.copy2(path, backup)
        triplib.save_trip(path, session, samples, version=version)
        print(f"    written; previous version kept at {backup}")


if __name__ == "__main__":
    main()
