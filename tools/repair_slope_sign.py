#!/usr/bin/env python3
"""Repair the inverted slope sign in rides recorded before the 2026-08-18 fix.

WHAT WAS WRONG
--------------
The app derived attitude from the puck's quaternion with the aerospace ZYX
euler extraction, whose pitch term `asin(2(wy - zx))` assumes Y right and
Z down. The puck's frame is Y LEFT and Z UP, so the extraction returns
nose-DOWN-positive: `puckFusionSlopeDegrees` is the true slope NEGATED.
Roll came out correct either way, which is why it survived unnoticed.

That inverted value only reached `imuSlopeDegrees` when
`BikeAttitudeEstimator.slopeDegrees` was nil — but nil there meant two
different things, and the consumer could not tell them apart: "no estimator
on this puck" (stand in for it) versus "the estimator is running and
deliberately withdrew this packet" (a refusal, which must not be overruled
by the one channel the estimator exists to replace). Stopped on a steep
climb, every shove of the rider's weight tripped the estimator's 0.2 g
accelerometer gate, so the slope alternated between the right answer and
minus itself about once a second.

Measured on Tripper_2026-08-16_14-40-56.trip over 4728 standstill rows,
where a stopped bike's accelerometer IS a true inclinometer:
`fusion pitch = -0.982 x accel pitch - 2.55`, r = -0.994.

WHAT THIS DOES
--------------
Rewrites each ride to hold what the FIXED app would have recorded:

  * `puckFusionSlopeDegrees` is negated on every row (the fix is
    `pitchSign = -1`, and that field is `pitchSign * euler.pitch`).
  * On rows where the app published the fusion value verbatim — detectable
    because `imuSlopeDegrees` equals `puckFusionSlopeDegrees` exactly —
    slope becomes the last value the estimator actually produced, if it is
    less than `HOLD_SECONDS` old; otherwise the negated fusion. Same tiers,
    same shelf life as PuckMotionMapper after the fix.
  * `pitchDegrees` follows slope (it always did) and `imuSlopePercent` is
    recomputed as `tan(deg) * 100`.

Untouched: every row the estimator answered for itself, lean/roll (the
fusion's roll was never inverted, and lean never fell back in these rides),
and every other channel.

USAGE
-----
    python3 tools/repair_slope_sign.py RIDE.trip [RIDE2.trip ...]   # dry run
    python3 tools/repair_slope_sign.py --apply DIR_OR_FILES ...

`--apply` rewrites in place after copying each original into a sibling
`pre-signfix/` directory, so a GPS Sync project keeps pointing at a working
file and the untouched recording is still there to check against.
"""
import argparse
import glob
import math
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import triplib  # noqa: E402

# Same four seconds the estimator allows its own dead reckoning: past that a
# held value stops describing the road under the bike.
HOLD_SECONDS = 4.0


def repair(samples):
    """Repair in place; return (rows_touched, rows_held, rows_negated)."""
    held_value = None
    held_at = None
    touched = negated = holds = 0

    for s in samples:
        fus = s.get("puckFusionSlopeDegrees")
        rep = s.get("imuSlopeDegrees")
        t = s.get("elapsedSeconds")

        # Whether the app fell back is decided on the ORIGINAL values, so read
        # it before negating anything.
        fell_back = (fus is not None and rep is not None
                     and abs(rep - fus) < 1e-3)

        if fus is not None:
            s["puckFusionSlopeDegrees"] = -fus

        if not fell_back:
            # The estimator's own answer: correct as recorded, and it becomes
            # the value held across the next refusal.
            if rep is not None and t is not None:
                held_value, held_at = rep, t
            continue

        touched += 1
        fresh = (held_value is not None and held_at is not None
                 and t is not None and t - held_at <= HOLD_SECONDS)
        if fresh:
            fixed = held_value
            holds += 1
        else:
            fixed = -fus
            negated += 1
        s["imuSlopeDegrees"] = fixed
        if s.get("pitchDegrees") is not None:
            s["pitchDegrees"] = fixed
        if s.get("imuSlopePercent") is not None:
            s["imuSlopePercent"] = math.tan(math.radians(fixed)) * 100

    return touched, holds, negated


def trip_version(path):
    """The file's own header byte — carried over, never defaulted."""
    with open(path, "rb") as f:
        head = f.read(5)
    if head[:4] != b"TRIP":
        raise ValueError(f"{path}: not a .trip file")
    return head[4]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help=".trip files or directories")
    ap.add_argument("--apply", action="store_true",
                    help="rewrite in place (originals copied to pre-signfix/)")
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
        touched, holds, negated = repair(samples)
        name = os.path.basename(path)
        if not touched:
            print(f"{name}: nothing to repair")
            continue
        print(f"{name}: {touched} rows ({100 * touched / len(samples):.1f}%) — "
              f"{holds} held, {negated} negated"
              + ("" if args.apply else "   [dry run]"))
        if not args.apply:
            continue
        backup_dir = os.path.join(os.path.dirname(os.path.abspath(path)),
                                  "pre-signfix")
        os.makedirs(backup_dir, exist_ok=True)
        backup = os.path.join(backup_dir, name)
        if not os.path.exists(backup):
            shutil.copy2(path, backup)
        triplib.save_trip(path, session, samples, version=version)
        print(f"    written; original kept at {os.path.relpath(backup, os.getcwd())}")


if __name__ == "__main__":
    main()
