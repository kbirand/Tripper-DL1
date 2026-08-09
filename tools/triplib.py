"""Shared loader + signal helpers for Tripper .trip recordings.

Decode chain: "TRIP" + 1 version byte + LZFSE payload -> JSON {session, samples}.

macOS decodes through libcompression via ctypes, which ships with the OS and
needs nothing installed; everywhere else it shells out to an lzfse CLI built
from github.com/lzfse/lzfse (override the path with $LZFSE_BIN). Do not drop
the ctypes path: a CLI-only version was committed once and made every tool
here unusable on the machine the rides are actually analysed on.

Needs numpy. Sign conventions and derived signals match the app's
BikeAttitudeEstimator — see docs/lean-investigation.md.
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
    """LZFSE-decode bytes: libcompression on macOS, the lzfse CLI elsewhere."""
    lib = ctypes.util.find_library("compression")
    if lib:
        c = ctypes.CDLL(lib)
        c.compression_decode_buffer.restype = ctypes.c_size_t
        c.compression_decode_buffer.argtypes = [
            ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p,
            ctypes.c_size_t, ctypes.c_void_p, ctypes.c_int,
        ]
        cap = max(8 * len(payload), 1 << 22)
        while True:
            dst = ctypes.create_string_buffer(cap)
            n = c.compression_decode_buffer(dst, cap, payload, len(payload),
                                            None, COMPRESSION_LZFSE)
            if n == 0:
                raise RuntimeError("LZFSE decode failed")
            if n < cap:            # == cap means possibly truncated: grow
                return dst.raw[:n]
            cap *= 2
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


def _load_json(path):
    """Whole decoded payload dict from a .trip, or an already-decoded json."""
    with open(path, "rb") as f:
        head = f.read(5)
        body = f.read()
    if head[:4] == b"TRIP":
        return json.loads(_decode_lzfse(body))
    return json.load(open(path))


def load_trip(path):
    """Return (session_dict, samples_list) from a .trip file (or a raw trip.json)."""
    d = _load_json(path)
    return d["session"], d["samples"]


class Ride:
    """Column view over samples with the derived signals every analysis needs."""

    def __init__(self, path):
        self.session, self.samples = load_trip(path)
        self.dt = None
        self._cols = {}
        t = self.col("elapsedSeconds")
        self.t = t
        self.dt = float(np.median(np.diff(t)))
        self.v = self.col("canSpeedKmh")          # km/h
        self.vm = self.v / 3.6                     # m/s
        self.still = self.v < 0.5
        # gyro, bias-corrected at standstill (deg/s; Z=up, X=forward per investigation)
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
        Sensor frame: X forward, Y lateral, Z up (accel in g units).

        atan2(ay, az) is RIGHT-POSITIVE — the same convention the app uses.
        This docstring claimed left-positive until 2026-08-09; measured against
        the shipped rollDegrees over every standstill row of the two
        2026-08-09 rides it correlates +0.979 and +0.990, means agreeing to
        0.4 deg. Do not re-invert it from the older prose.

        zeroed=True subtracts the standstill median (mount zero)."""
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

        PASS sign=-1 to match the app. Sensor Z is up, so a right turn is a
        NEGATIVE gyro Z and the right-positive convention negates it: sign=-1
        correlates +0.939/+0.946 with the shipped rollDegrees on the two
        2026-08-09 rides, sign=+1 exactly as strongly negative. The default
        stays +1 only so old call sites keep their old meaning; every scorer
        in harness.py passes -1."""
        return np.degrees(np.arctan2(sign * self.vm * np.radians(self.gz), G))

    def straight_mask(self, yaw_thresh=1.5, vmin=20, tau=1.0):
        """Rows that are straight (low yaw rate) and moving."""
        return (np.abs(self.lp(self.gz, tau)) < yaw_thresh) & (self.v > vmin)

    def baro_slope(self, tau_alt=3.0, min_speed=3.0):
        """Road grade from puck barometer altitude over CAN distance (deg).
        The attitude-free reference the investigation validated at +0.75 vs GPS."""
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


def save_trip(path, session, samples, version=1):
    """Write a .trip: "TRIP" + version byte + LZFSE-compressed JSON.

    Encoding still needs the lzfse CLI ($LZFSE_BIN) — libcompression's encoder
    is not wired up here because nothing in this repo writes .trip files
    routinely; reading is the hot path.
    """
    payload = json.dumps({"session": session, "samples": samples},
                         separators=(",", ":")).encode()
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as fin:
        fin.write(payload)
    out = fin.name + ".lzfse"
    subprocess.run([LZFSE_BIN, "-encode", "-i", fin.name, "-o", out], check=True)
    body = open(out, "rb").read()
    os.unlink(fin.name); os.unlink(out)
    with open(path, "wb") as f:
        f.write(b"TRIP")
        f.write(bytes([version]))
        f.write(body)


# ---------------------------------------------------------------- raw stream
# The 100 Hz flight recorder, added 2026-08-09. Layout mirrors
# RawFlightRecorder.swift and the firmware's RawBatch; see the Tripper-DL1
# README section "Raw stream" for the wire format the phone is repacking.

def load_raw(path):
    """Return the 100 Hz layers from a .trip, or None if it has none.

    Keys: t (s, elapsed from the first sample), gx/gy/gz (deg/s),
    ax/ay/az (g), plus 'slow' (per-batch context) and 'est' (what the
    estimator believed, one row per telemetry packet).

    Timestamps are rebuilt from the PUCK's own counter, never arrival time:
    BLE delivers in bursts, so arrival time would smear every derivative.
    """
    import base64, struct

    d = _load_json(path)
    raw = d.get("raw")
    if not raw:
        return None

    imu = np.frombuffer(base64.b64decode(raw["imu"]), dtype="<i2")
    n = len(imu) // 6
    imu = imu[: n * 6].reshape(n, 6)

    slow_b = base64.b64decode(raw["slow"])
    SLOW = "<IHHhhhhihBBBB"                     # 26 bytes; see RawFlightLog
    assert struct.calcsize(SLOW) == 26, struct.calcsize(SLOW)
    slow = [struct.unpack_from(SLOW, slow_b, i * 26)
            for i in range(len(slow_b) // 26)]

    # Rebuild the timeline batch by batch. A batch whose puck timestamp does
    # not follow the last one is a BLE drop: the gap is left in the time axis
    # rather than closed up, because closing it would invent motion that never
    # happened.
    t_ms = np.empty(n, dtype=np.float64)
    k = 0
    for (puck_ms, per10, count, *_rest) in slow:
        per = per10 / 10.0
        c = min(count, n - k)
        if c <= 0:
            break
        t_ms[k:k + c] = puck_ms + np.arange(c) * per
        k += c
    t_ms = t_ms[:k]
    imu = imu[:k]

    est_b = base64.b64decode(raw.get("estimator", ""))
    EST = "<ffffffBB"                            # 26 bytes
    assert struct.calcsize(EST) == 26, struct.calcsize(EST)
    est = [struct.unpack_from(EST, est_b, i * 26)
           for i in range(len(est_b) // 26)]

    return {
        "meta": {kk: raw[kk] for kk in
                 ("version", "sampleHz", "startedAt", "puckStartMs",
                  "batchCount", "sampleCount", "gapCount")
                 if kk in raw},
        "truncated": bool(raw.get("truncated")),
        "t":  (t_ms - t_ms[0]) / 1000.0,
        "gx": imu[:, 0] / 100.0, "gy": imu[:, 1] / 100.0, "gz": imu[:, 2] / 100.0,
        "ax": imu[:, 3] / 1000.0, "ay": imu[:, 4] / 1000.0, "az": imu[:, 5] / 1000.0,
        "slow": [{"puckMs": s[0], "periodMs": s[1] / 10.0, "count": s[2],
                  "quat": (s[3] / 16384, s[4] / 16384, s[5] / 16384, s[6] / 16384),
                  "pressPa": s[7], "tempC": s[8] / 10.0,
                  "cal": s[9:13]} for s in slow],
        # nan where the app had no value — a blank slope must not read as 0.
        "est": [{"t": e[0], "slope": e[1], "lean": e[2], "anchor": e[3],
                 "fused": e[4], "wheelScale": e[5],
                 "gated": bool(e[6] & 1), "gravity": bool(e[6] & 2)} for e in est],
    }
