"""Loader + signal helpers for Tripper .trip recordings.

Format: "TRIP" + 1 version byte + LZFSE payload -> JSON {session, samples}.
Decode order: macOS libcompression via ctypes (no dependency), else an lzfse
CLI built from github.com/lzfse/lzfse (path via LZFSE_BIN).

Needs numpy. Written 2026-08-08 alongside the fusion investigation; the sign
conventions and derived signals match docs/lean-investigation.md and the
app's BikeAttitudeEstimator.
"""
import ctypes
import ctypes.util
import json
import os
import subprocess
import tempfile

import numpy as np

LZFSE_BIN = os.environ.get("LZFSE_BIN", "lzfse")
COMPRESSION_LZFSE = 0x801
G = 9.80665


def _decode_lzfse(payload):
    """LZFSE-decode bytes: libcompression on macOS, lzfse CLI elsewhere."""
    lib = ctypes.util.find_library("compression")
    if lib:
        c = ctypes.CDLL(lib)
        c.compression_decode_buffer.restype = ctypes.c_size_t
        c.compression_decode_buffer.argtypes = [
            ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p,
            ctypes.c_size_t, ctypes.c_void_p, ctypes.c_int,
        ]
        cap = max(4 * len(payload), 1 << 22)
        while True:
            dst = ctypes.create_string_buffer(cap)
            n = c.compression_decode_buffer(dst, cap, payload, len(payload),
                                            None, COMPRESSION_LZFSE)
            if n == 0:
                raise RuntimeError("LZFSE decode failed")
            if n < cap:            # == cap means possibly truncated: grow
                return dst.raw[:n]
            cap *= 2
    # No libcompression (Linux): the lzfse CLI.
    with tempfile.NamedTemporaryFile(suffix=".lzfse", delete=False) as fin:
        fin.write(payload)
    out = fin.name + ".json"
    try:
        subprocess.run([LZFSE_BIN, "-decode", "-i", fin.name, "-o", out],
                       check=True)
        with open(out, "rb") as f:
            return f.read()
    finally:
        os.unlink(fin.name)
        if os.path.exists(out):
            os.unlink(out)


def load_trip(path):
    """Return (session_dict, samples_list) from a .trip file or a raw JSON."""
    with open(path, "rb") as f:
        head = f.read(5)
        body = f.read()
    if head[:4] == b"TRIP":
        d = json.loads(_decode_lzfse(body))
    else:
        d = json.load(open(path))
    return d["session"], d["samples"]


class Ride:
    """Column view over samples with the derived signals every analysis needs."""

    def __init__(self, path):
        self.session, self.samples = load_trip(path)
        self._cols = {}
        t = self.col("elapsedSeconds")
        self.t = t
        self.dt = float(np.median(np.diff(t)))
        self.v = self.col("canSpeedKmh")          # km/h
        self.vm = self.v / 3.6                     # m/s
        self.still = self.v < 0.5
        # gyro, bias-corrected with the whole-ride standstill mean
        # (deg/s; Z=up, X=forward — see docs/lean-investigation.md)
        self.gz = self.col("puckGyroZ") - np.nanmean(self.col("puckGyroZ")[self.still])
        self.gx = self.col("puckGyroX") - np.nanmean(self.col("puckGyroX")[self.still])
        self.gy = self.col("puckGyroY") - np.nanmean(self.col("puckGyroY")[self.still])

    def col(self, k):
        if k not in self._cols:
            self._cols[k] = np.array(
                [s.get(k) if s.get(k) is not None else np.nan for s in self.samples],
                dtype=float)
        return self._cols[k]

    def lp(self, x, tau):
        """First-order low-pass, NaN-tolerant (holds last value across gaps)."""
        a = self.dt / (tau + self.dt)
        y = np.array(x, dtype=float)
        started = False
        for i in range(len(y)):
            if not started:
                started = np.isfinite(y[i])
                continue
            y[i] = y[i - 1] + a * (y[i] - y[i - 1]) if np.isfinite(y[i]) else y[i - 1]
        return y

    # ---- derived signals ----------------------------------------------------
    def accel_roll_pitch(self, tau=2.0, zeroed=True):
        """Roll/pitch implied by the RAW accelerometer direction, low-passed.
        Sensor frame: X forward, Y rider's left, Z up (accel in g).
        NOTE: atan2(ay, az) is LEFT-positive; the app's display convention is
        right-positive (see the sign section in CLAUDE.md). zeroed=True
        subtracts the standstill median (mount zero)."""
        ax = self.lp(self.col("puckAccelX"), tau)
        ay = self.lp(self.col("puckAccelY"), tau)
        az = self.lp(self.col("puckAccelZ"), tau)
        roll = np.degrees(np.arctan2(ay, az))
        pitch = np.degrees(np.arctan2(-ax, np.sqrt(ay ** 2 + az ** 2)))
        if zeroed:
            roll -= np.nanmedian(roll[self.still])
            pitch -= np.nanmedian(pitch[self.still])
        return roll, pitch

    def kinematic_lean(self, sign=+1):
        """atan(v * psi_dot / g) from CAN speed + vertical-axis gyro (deg).
        sign=+1 is left-positive; sign=-1 matches the app's right-positive."""
        return np.degrees(np.arctan2(sign * self.vm * np.radians(self.gz), G))

    def straight_mask(self, yaw_thresh=1.5, vmin=20, tau=1.0):
        """Rows that are straight (low yaw rate) and moving."""
        return (np.abs(self.lp(self.gz, tau)) < yaw_thresh) & (self.v > vmin)

    def baro_slope(self, tau_alt=3.0, min_speed=3.0):
        """Road grade from puck baro altitude over CAN distance (deg).
        Attitude-free. Beware wheel slip on climbs — see 'Slope's distance'
        in docs/lean-investigation.md; this is fine as a reference on rides
        without sustained wheelspin."""
        alt = self.lp(self.col("puckBaroAltitudeMeters"), tau_alt)
        dalt = np.gradient(alt, self.t)
        grade = np.degrees(np.arctan2(dalt, np.maximum(self.vm, 0.1)))
        grade[self.v < min_speed] = np.nan
        return grade

    def corners(self, yaw_thresh=6.0, vmin=10, min_dur=1.5):
        """Contiguous cornering windows: |smoothed gz| above yaw_thresh deg/s."""
        gzl = self.lp(self.gz, 0.8)
        m = (np.abs(gzl) > yaw_thresh) & (self.v > vmin)
        out, start = [], None
        for i, f in enumerate(m):
            if f and start is None:
                start = i
            elif not f and start is not None:
                if self.t[i - 1] - self.t[start] >= min_dur:
                    out.append((start, i - 1))
                start = None
        if start is not None and self.t[len(m) - 1] - self.t[start] >= min_dur:
            out.append((start, len(m) - 1))
        return out

    def runs(self, mask, min_dur=3.0, max_gap=5):
        """Contiguous True runs of `mask` lasting at least min_dur seconds."""
        idx = np.where(mask)[0]
        if not len(idx):
            return []
        out, s0, prev = [], idx[0], idx[0]
        for i in idx[1:]:
            if i - prev > max_gap:
                if self.t[prev] - self.t[s0] >= min_dur:
                    out.append((s0, prev))
                s0 = i
            prev = i
        if self.t[prev] - self.t[s0] >= min_dur:
            out.append((s0, prev))
        return out
