#!/usr/bin/env python3
"""Replay harness for Tripper rides: replays candidate lean/slope filters
against a recorded .trip file and scores them, so filter ideas are tested
in seconds instead of rides. Needs numpy; decode notes in triplib.py.

Usage:
    python3 tools/harness.py <ride.trip|trip.json> [--tau 1.5] [--slope-tau 3.0]

Signals used (v0x04 packet required for raw-accel diagnostics):
    canSpeedKmh, puckGyro{X,Y,Z}, puckAccel{X,Y,Z}, puckBaroAltitudeMeters,
    puckFusionRollDegrees / puckFusionSlopeDegrees (BNO055 fusion, baseline),
    rollDegrees / imuSlopeDegrees (currently shipped values, baseline).

Scoring:
    LEAN  — corner sign agreement vs the kinematic reference atan(v*psi_dot/g),
            straight-line mean/sd and % of rows beyond 5/10 deg.
    SLOPE — correlation / bias / sd against the attitude-free barometric grade
            (baro altitude over CAN distance), the reference the July
            investigation validated at +0.75 vs GPS.
LEAN IS RIGHT-POSITIVE. Re-anchored on the two 2026-08-09 rides after the app
flipped on 2026-08-08; this file scored against the OLD left-positive
reference until 2026-08-09 and therefore marked the corrected app 0/27 on a
ride where it was right. Each sign was settled against the shipped
rollDegrees on both rides, not from any doc:
    kinematic reference  atan(-v*psi_dot/g)  -> kinematic_lean(-1)  +0.94/+0.95
    roll rate            +puckGyroX                                 +0.93/+0.92
    accel roll           atan2(ay, az)  (already right-positive)     +0.98/+0.99
Recordings made before 2026-08-08 carry lean MIRRORED and cannot be scored
here without flipping them first.
"""
import argparse
import numpy as np
from triplib import Ride, G


# --------------------------------------------------------------------------
# Candidate filters. Each takes a Ride and returns a per-sample array (deg).
# Add new ideas here; they will be scored automatically.
# --------------------------------------------------------------------------

def lean_kinematic_raw(r, **kw):
    """Pure kinematic reference atan(v*psi_dot/g), no smoothing."""
    return r.kinematic_lean(-1)


def lean_complementary(r, tau=1.5, **kw):
    """Gyro-integrated roll pulled toward the kinematic reference with time
    constant tau — the filter the July doc validated offline (10/12 corners).
    Roll rate is +gyroX (forward axis) in the right-positive convention."""
    kin = r.kinematic_lean(-1)
    rate = +r.gx  # deg/s
    a = r.dt / (tau + r.dt)
    out = np.zeros(len(kin))
    for i in range(1, len(kin)):
        pred = out[i - 1] + (rate[i] if np.isfinite(rate[i]) else 0.0) * r.dt
        ref = kin[i] if np.isfinite(kin[i]) else pred
        out[i] = (1 - a) * pred + a * ref
    return out


def lean_complementary_speedgate(r, tau=1.5, **kw):
    """Same, but below 3 km/h the reference becomes the raw-accel roll
    (gravity is trustworthy at standstill, kinematic reference is not)."""
    kin = r.kinematic_lean(-1)
    acc_roll, _ = r.accel_roll_pitch(tau=1.0)
    ref_arr = np.where(r.v > 3.0, kin, acc_roll)
    rate = +r.gx
    a = r.dt / (tau + r.dt)
    out = np.zeros(len(kin))
    for i in range(1, len(kin)):
        pred = out[i - 1] + (rate[i] if np.isfinite(rate[i]) else 0.0) * r.dt
        ref = ref_arr[i] if np.isfinite(ref_arr[i]) else pred
        out[i] = (1 - a) * pred + a * ref
    return out


def slope_baro(r, slope_tau=3.0, **kw):
    """Barometric grade, smoothed: d(baroAlt)/d(distance). Attitude-free."""
    return r.lp(r.baro_slope(tau_alt=slope_tau), 1.0)


def slope_baro_gyro(r, slope_tau=3.0, **kw):
    """Baro grade as the slow reference, pitch gyro (-gyroY, lateral axis)
    filling in transitions — complementary, tau = slope_tau."""
    ref = r.baro_slope(tau_alt=slope_tau)
    rate = -r.gy  # deg/s, sign convention checked against baro reference
    a = r.dt / (slope_tau + r.dt)
    out = np.zeros(len(ref))
    for i in range(1, len(ref)):
        pred = out[i - 1] + (rate[i] if np.isfinite(rate[i]) else 0.0) * r.dt
        rf = ref[i] if np.isfinite(ref[i]) else pred
        out[i] = (1 - a) * pred + a * rf
    return out


LEAN_CANDIDATES = {
    "kinematic(raw)": lean_kinematic_raw,
    "complementary": lean_complementary,
    "compl+lowspeed": lean_complementary_speedgate,
}
SLOPE_CANDIDATES = {
    "baro(smoothed)": slope_baro,
    "baro+gyro": slope_baro_gyro,
}


# --------------------------------------------------------------------------
# Scoring
# --------------------------------------------------------------------------

def score_lean(r, arr, corners, straight, kin):
    agree = tot = 0
    for a, b in corners:
        km = np.nanmean(kin[a:b + 1])
        rm = np.nanmean(arr[a:b + 1])
        if abs(km) > 2:
            tot += 1
            if np.sign(km) == np.sign(rm) and abs(rm) > 1:
                agree += 1
    s = arr[straight]
    return dict(corners=f"{agree}/{tot}",
                str_mean=np.nanmean(s), str_sd=np.nanstd(s),
                gt5=100 * np.nanmean(np.abs(s) > 5),
                gt10=100 * np.nanmean(np.abs(s) > 10),
                maxlean=np.nanmax(np.abs(arr[r.v > 10])))


def score_slope(r, arr, ref):
    m = (r.v > 5) & np.isfinite(arr) & np.isfinite(ref)
    if m.sum() < 10:
        return dict(corr=np.nan, bias=np.nan, sd=np.nan)
    err = arr[m] - ref[m]
    return dict(corr=np.corrcoef(arr[m], ref[m])[0, 1],
                bias=np.nanmean(err), sd=np.nanstd(err))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trip")
    ap.add_argument("--tau", type=float, default=1.5)
    ap.add_argument("--slope-tau", type=float, default=3.0)
    args = ap.parse_args()

    r = Ride(args.trip)
    n = len(r.t)
    print(f"ride: {n} samples, {r.t[-1]:.0f}s, "
          f"{r.session.get('distanceMeters', 0)/1000:.1f} km, "
          f"max {np.nanmax(r.v):.0f} km/h, dt={r.dt:.2f}s")
    pv = r.col("puckPacketVersion")
    has_raw = np.isfinite(r.col("puckAccelX")).any()
    print(f"packet v{int(np.nanmax(pv)) if np.isfinite(pv).any() else '?'}, "
          f"raw accel {'present' if has_raw else 'MISSING (pre-v0x04 ride)'}\n")

    kin = r.kinematic_lean(-1)
    corners = r.corners()
    straight = r.straight_mask()
    baro_ref = r.baro_slope()

    # ---- step-1 diagnostic (fusion vs mechanics), only with raw accel ----
    if has_raw:
        acc_roll, acc_pitch = r.accel_roll_pitch(tau=2.0)
        s = acc_roll[straight]
        fus = r.col("puckFusionRollDegrees")[straight]
        print("STEP-1 DIAGNOSTIC (straight, steady rows: "
              f"{straight.sum()} = {straight.sum()*r.dt:.0f}s)")
        print(f"  raw-accel roll : mean {np.nanmean(s):+.2f}  sd {np.nanstd(s):.2f}  "
              f"|>10deg| {100*np.nanmean(np.abs(s)>10):.1f}%")
        print(f"  BNO fusion roll: mean {np.nanmean(fus):+.2f}  sd {np.nanstd(fus):.2f}  "
              f"|>10deg| {100*np.nanmean(np.abs(fus)>10):.1f}%")
        mm = straight & np.isfinite(acc_roll) & np.isfinite(r.col("puckFusionRollDegrees"))
        cc = np.corrcoef(acc_roll[mm], r.col("puckFusionRollDegrees")[mm])[0, 1]
        print(f"  corr(accel, fusion) on straights: {cc:+.3f}")
        print("  -> accel flat + fusion wandering + low corr = FUSION AT FAULT; "
              "accel wandering too = MOUNT/MECHANICS\n")

    # ---- lean table ----
    header = f"{'LEAN':<18}{'corners':>9}{'str mean':>10}{'str sd':>8}{'>5deg':>8}{'>10deg':>8}{'max':>7}"
    print(header); print("-" * len(header))
    rows = [("BNO fusion [base]", r.col("puckFusionRollDegrees")),
            ("shipped rollDeg", r.col("rollDegrees"))]
    rows += [(name, fn(r, tau=args.tau)) for name, fn in LEAN_CANDIDATES.items()]
    for name, arr in rows:
        sc = score_lean(r, arr, corners, straight, kin)
        print(f"{name:<18}{sc['corners']:>9}{sc['str_mean']:>+10.2f}{sc['str_sd']:>8.2f}"
              f"{sc['gt5']:>7.1f}%{sc['gt10']:>7.1f}%{sc['maxlean']:>7.1f}")

    # ---- slope table (two references: baro grade, and independent GPS slope;
    # note the baro candidate is near-circular vs the baro reference — the GPS
    # column is the honest one for it) ----
    print()
    gps_ref = r.col("gpsSlopeDegrees")
    header = (f"{'SLOPE':<24}{'baro corr':>10}{'bias':>7}{'sd':>6}"
              f"{'gps corr':>10}{'bias':>7}{'sd':>6}")
    print(header); print("-" * len(header))
    rows = [("BNO fusion [base]", r.col("puckFusionSlopeDegrees")),
            ("shipped imuSlope", r.col("imuSlopeDegrees"))]
    rows += [(name, fn(r, slope_tau=args.slope_tau)) for name, fn in SLOPE_CANDIDATES.items()]
    for name, arr in rows:
        sb = score_slope(r, arr, baro_ref)
        sg = score_slope(r, arr, gps_ref)
        print(f"{name:<24}{sb['corr']:>+10.2f}{sb['bias']:>+7.2f}{sb['sd']:>6.2f}"
              f"{sg['corr']:>+10.2f}{sg['bias']:>+7.2f}{sg['sd']:>6.2f}")

    # ---- regression windows: worst straight-line stretches per signal ----
    print("\nWorst straight-line windows (|roll|>8deg for >3s):")
    for name, arr in [("BNO fusion", r.col("puckFusionRollDegrees")),
                      ("shipped", r.col("rollDegrees")),
                      ("complementary", lean_complementary(r, tau=args.tau))]:
        runs = r.runs(straight & (np.abs(arr) > 8), min_dur=3.0)
        if not runs:
            print(f"  {name:<14}: none")
        else:
            w = "  ".join(f"t={r.t[a]:.0f}-{r.t[b]:.0f}s({np.nanmean(arr[a:b+1]):+.0f}deg)"
                          for a, b in runs[:5])
            print(f"  {name:<14}: {w}")


if __name__ == "__main__":
    main()
