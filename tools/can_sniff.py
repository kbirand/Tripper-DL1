#!/usr/bin/env python3
"""
CAN bring-up / reverse-engineering tool for the Waveshare USB-CAN-A.

The adapter speaks the "USB-CAN Analyzer" serial protocol, which python-can
exposes as the `seeedstudio` backend.

Everything here defaults to LISTEN-ONLY (silent) mode: the transceiver never
drives the bus and never even sends ACKs, so a wrong bitrate cannot spew error
frames at the bike's controller. Leaving silent mode requires --mode normal.

    scan   sweep candidate bitrates, report which one decodes traffic
    sniff  live table of IDs with a per-byte change map
    log    record to a file (.asc / .blf / .csv / .log) for later analysis

Usage:
    python3 can_sniff.py scan  -p /dev/cu.usbserial-510
    python3 can_sniff.py sniff -p /dev/cu.usbserial-510 -b 500000
    python3 can_sniff.py log   -p /dev/cu.usbserial-510 -b 500000 -o ride.asc
"""

import argparse
import sys
import time
from collections import OrderedDict

try:
    import can
except ImportError:
    sys.exit("python-can not installed:  pip install python-can pyserial")

# Ordered most-likely-first for vehicle buses.
DEFAULT_BITRATES = [500000, 250000, 125000, 1000000, 100000]


def open_bus(port, bitrate, frame_type="STD", mode="silent", serial_baud=2000000):
    """Open the adapter. `serial_baud` is the USB side; USB-CAN-A ships at 2 Mbaud."""
    return can.Bus(
        interface="seeedstudio",
        channel=port,
        baudrate=serial_baud,
        bitrate=bitrate,
        frame_type=frame_type,
        operation_mode=mode,
        timeout=0.05,
    )


def drain(bus, seconds):
    """Collect messages for `seconds`. Returns (count, {(id, ext), ...})."""
    count, ids = 0, set()
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg is not None:
            count += 1
            ids.add((msg.arbitration_id, msg.is_extended_id))
    return count, ids


def fmt_id(arb_id, extended):
    return f"{arb_id:08X}x" if extended else f"{arb_id:03X} "


# --------------------------------------------------------------------- decode
# Talaria signals, verified against the two ride logs in inklit/Talaria_CAN.
# See talaria.dbc for provenance on each one.


# 16S pack: 0x201's index bytes span 1..16 across both ride logs. That frame
# reports only the highest and lowest cell (bytes 0-1 > bytes 2-3 in 100% of
# 18,447 logged frames), so the individual 16 cells are never all visible.
CELL_COUNT = 16


def _u16(d, off):
    return int.from_bytes(d[off:off + 2], "little")


SIGNALS = [
    # (label, arbitration id, extractor, unit, format)
    ("Speed", 0x303, lambda d: _u16(d, 0) / 10, "km/h", "%6.1f"),
    ("Motor", 0x203, lambda d: _u16(d, 0), "rpm", "%6.0f"),
    ("Power", 0x203, lambda d: _u16(d, 2), "W", "%6.0f"),
    ("Current", 0x302, lambda d: _u16(d, 4) / 10, "A", "%6.1f"),
    ("Pack", 0x101, lambda d: _u16(d, 0) / 10, "V", "%6.1f"),
    ("Pack~", 0x302, lambda d: d[0], "V", "%6.0f"),
    ("Charge", 0x401, lambda d: d[0], "%", "%6.0f"),
    # 0x202[3:5] leads motor RPM by ~0.3 s on throttle and rests at a
    # mode-dependent floor. Note this sits at [2:4] on the firmware in the
    # inklit ride logs, where it is mode-linked only and does not track throttle.
    ("Demand", 0x202, lambda d: _u16(d, 3), "", "%6.0f"),
]

MODES = {1: "Eco", 2: "Sport"}

# 0x202 byte 0 is the vehicle state byte. Confirmed on both ride logs:
# bit 7 is a hard kickstand interlock (0.00% of moving samples, max speed
# 0.0 km/h while set) and bits 5:4 select the ride mode.
STATES = [
    ("Kickstand", 0x202, lambda d: "DOWN" if (d[0] >> 7) & 1 else "up"),
    ("Mode", 0x202, lambda d: MODES.get((d[0] >> 4) & 3, f"?{(d[0]>>4)&3}")),
    ("Standstill", 0x202, lambda d: "yes" if (d[0] >> 1) & 1 else "no"),
    ("State", 0x202, lambda d: f"0x{d[0]:02X}"),
]


def decode_panel(tracks, cells):
    """Named physical values from the last payload of each message."""
    out = []
    row = []
    for label, arb, fn, unit, fmt in SIGNALS:
        track = tracks.get((arb, False))
        if track is None or len(track.data) < 8:
            continue
        try:
            row.append(f"{label:>7} {fmt % fn(track.data)} {unit:<4}")
        except (IndexError, ValueError):
            continue
    for i in range(0, len(row), 4):
        out.append("  " + "  ".join(row[i:i + 4]))

    states = []
    for label, arb, fn in STATES:
        track = tracks.get((arb, False))
        if track is None or len(track.data) < 8:
            continue
        try:
            states.append(f"{label:>10} {fn(track.data):<6}")
        except (IndexError, ValueError):
            continue
    if states:
        out.append("  " + " ".join(states))

    if cells:
        (hi_i, hi_mv), (lo_i, lo_mv) = cells
        out.append(
            f"  {'Cells':>7} high #{hi_i:<2} {hi_mv/1000:.3f} V   "
            f"low #{lo_i:<2} {lo_mv/1000:.3f} V   spread {hi_mv-lo_mv:>3} mV   "
            f"est pack {CELL_COUNT * (hi_mv+lo_mv)/2/1000:5.2f} V"
        )
    return out


# --------------------------------------------------------------------------- scan


def cmd_scan(args):
    print(f"Port {args.port} · listen-only · {args.seconds:.1f}s per candidate")
    print("Bike must be ON and awake (screen lit) or there is nothing to hear.\n")
    print(f"{'bitrate':>9}  {'frames':>7}  {'ids':>4}   {'type':<4}")
    print("-" * 34)

    results = []
    for bitrate in args.bitrates:
        for frame_type in ("STD", "EXT"):
            try:
                bus = open_bus(args.port, bitrate, frame_type, "silent", args.serial_baud)
            except Exception as exc:
                sys.exit(f"could not open {args.port}: {exc}")
            bus.flush_buffer()
            count, ids = drain(bus, args.seconds)
            bus.shutdown()
            time.sleep(0.2)  # let the CH340 settle before reconfiguring

            results.append((bitrate, frame_type, count, len(ids)))
            flag = "  <-- traffic" if count else ""
            print(f"{bitrate:>9}  {count:>7}  {len(ids):>4}   {frame_type:<4}{flag}")

    best = max(results, key=lambda r: r[2])
    print()
    if best[2] == 0:
        print("No frames on any bitrate. In order of likelihood:")
        print("  1. CAN_H / CAN_L are swapped — swap the two wires and re-run.")
        print("  2. No shared ground between the adapter and the bike.")
        print("  3. The bus is asleep — wake the bike (throttle, brake, display on).")
        print("  4. The adapter's 120R termination jumper is ON, making the bus 40R.")
        return 1

    bitrate, frame_type, count, n_ids = best
    print(f"Locked: {bitrate} bit/s ({frame_type}) — {count} frames, {n_ids} unique IDs.")
    print("Polarity is correct; CAN_H and CAN_L are on the right terminals.\n")
    print(f"  python3 {sys.argv[0]} sniff -p {args.port} -b {bitrate} -f {frame_type}")
    return 0


# -------------------------------------------------------------------------- sniff


class Track:
    """Per-arbitration-ID statistics."""

    __slots__ = ("count", "first", "last", "data", "changes", "lo", "hi")

    def __init__(self, ts, data):
        self.count = 1
        self.first = self.last = ts
        self.data = bytearray(data)
        self.changes = [0] * len(data)
        self.lo = bytearray(data)
        self.hi = bytearray(data)

    def update(self, ts, data):
        self.count += 1
        self.last = ts
        if len(data) != len(self.data):  # variable DLC — restart the profile
            self.data = bytearray(data)
            self.changes = [0] * len(data)
            self.lo = bytearray(data)
            self.hi = bytearray(data)
            return
        for i, byte in enumerate(data):
            if byte != self.data[i]:
                self.changes[i] += 1
                self.data[i] = byte
            if byte < self.lo[i]:
                self.lo[i] = byte
            if byte > self.hi[i]:
                self.hi[i] = byte

    def rate(self):
        span = self.last - self.first
        return (self.count - 1) / span if span > 0.05 else 0.0

    def activity(self):
        """One char per byte: how lively is it?  . static  _-= mild  # busy"""
        out = []
        for n in self.changes:
            if n == 0:
                out.append(".")
                continue
            ratio = n / max(self.count - 1, 1)
            out.append("_" if ratio < 0.05 else "-" if ratio < 0.25 else "=" if ratio < 0.6 else "#")
        return "".join(out)


def cmd_sniff(args):
    try:
        bus = open_bus(args.port, args.bitrate, args.frame_type, args.mode, args.serial_baud)
    except Exception as exc:
        sys.exit(f"could not open {args.port}: {exc}")

    if args.mode != "silent":
        print(f"!! {args.mode} mode — the adapter WILL drive the bus. Ctrl-C to abort.")
        time.sleep(2)

    wanted = set(int(x, 16) for x in args.ids) if args.ids else None
    tracks = OrderedDict()
    cells = ()          # ((high_idx, mV), (low_idx, mV)) from the last 0x201
    total = 0
    started = time.time()
    next_draw = 0.0

    try:
        while True:
            if args.seconds and time.time() - started > args.seconds:
                break
            msg = bus.recv(timeout=0.05)
            if msg is not None:
                if wanted is None or msg.arbitration_id in wanted:
                    total += 1
                    key = (msg.arbitration_id, msg.is_extended_id)
                    track = tracks.get(key)
                    if track is None:
                        tracks[key] = Track(msg.timestamp, msg.data)
                    else:
                        track.update(msg.timestamp, msg.data)
                    if msg.arbitration_id == 0x201 and len(msg.data) >= 6:
                        d = msg.data
                        cells = ((d[4], _u16(d, 0)), (d[5], _u16(d, 2)))

            now = time.time()
            if now >= next_draw:
                next_draw = now + args.refresh
                draw(tracks, total, now - started, args, cells)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

    draw(tracks, total, time.time() - started, args, cells)
    print(f"\nStopped. {total} frames, {len(tracks)} unique IDs.")
    return 0


def draw(tracks, total, elapsed, args, cells=()):
    rows = sorted(tracks.items(), key=lambda kv: (kv[0][1], kv[0][0]))
    out = ["\033[H\033[J"]  # home + clear
    out.append(
        f"{args.port}  {args.bitrate} bit/s  {args.mode}   "
        f"{total} frames  {len(rows)} ids  {elapsed:5.1f}s\n"
    )
    if not args.raw:
        panel = decode_panel(tracks, cells or ())
        if panel:
            out.extend(panel)
            out.append("")
    out.append(f"{'ID':<10} {'cnt':>7} {'Hz':>6}  {'data':<24} {'activity':<9} {'range'}")
    out.append("-" * 88)
    for (arb_id, ext), t in rows:
        data = " ".join(f"{b:02X}" for b in t.data)
        span = ",".join(
            f"{i}:{t.lo[i]:02X}-{t.hi[i]:02X}" for i in range(len(t.data)) if t.changes[i]
        )
        out.append(
            f"{fmt_id(arb_id, ext):<10} {t.count:>7} {t.rate():>6.1f}  "
            f"{data:<24} {t.activity():<9} {span[:34]}"
        )
    out.append("\n. static   _ rare   - some   = often   # every frame     Ctrl-C to stop")
    sys.stdout.write("\n".join(out) + "\n")
    sys.stdout.flush()


# -------------------------------------------------------------------------- events


def cmd_events(args):
    """Learn which bytes are noisy, then report only meaningful changes.

    Run it, sit still through the baseline, then work one control at a time:
    the byte that moves is the one carrying it.
    """
    try:
        bus = open_bus(args.port, args.bitrate, args.frame_type, "silent", args.serial_baud)
    except Exception as exc:
        sys.exit(f"could not open {args.port}: {exc}")

    last = {}      # (id, byte) -> value
    noisy = set()  # bytes that moved on their own during the baseline
    started = time.time()

    print(f"Baseline: hold still and touch nothing for {args.baseline:.0f}s ...")
    try:
        while True:
            msg = bus.recv(timeout=0.05)
            now = time.time()
            learning = now - started < args.baseline

            if msg is not None and not msg.is_extended_id:
                arb = msg.arbitration_id
                for i, byte in enumerate(msg.data):
                    key = (arb, i)
                    prev = last.get(key)
                    last[key] = byte
                    if prev is None or prev == byte:
                        continue
                    if learning:
                        noisy.add(key)          # drifts by itself - ignore later
                    elif key not in noisy or args.all:
                        delta = prev ^ byte
                        bits = " ".join(f"bit{b}" for b in range(8) if delta >> b & 1)
                        print(f"  {now-started:7.1f}s  {arb:03X}[{i}]  "
                              f"0x{prev:02X} -> 0x{byte:02X}   {bits}")

            if learning and time.time() - started >= args.baseline:
                started -= 0  # keep the clock; just announce the transition
                print(f"Baseline done - ignoring {len(noisy)} self-changing bytes.")
                print("Now operate ONE control at a time (kickstand, brake, throttle, "
                      "lights, mode).\nCtrl-C to stop.\n")
                # prevent re-announcing
                args.baseline = -1
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
    print(f"\nStopped. Suppressed bytes: "
          f"{', '.join(f'{a:03X}[{b}]' for a, b in sorted(noisy)) or 'none'}")
    return 0


# ---------------------------------------------------------------------------- log


def cmd_log(args):
    try:
        bus = open_bus(args.port, args.bitrate, args.frame_type, "silent", args.serial_baud)
    except Exception as exc:
        sys.exit(f"could not open {args.port}: {exc}")

    print(f"Recording {args.port} @ {args.bitrate} -> {args.output}   Ctrl-C to stop")
    count = 0
    started = None
    with can.Logger(args.output) as logger:
        try:
            while True:
                if args.seconds and started and time.time() - started > args.seconds:
                    break
                msg = bus.recv(timeout=0.2)
                if msg is not None:
                    if started is None:
                        started = time.time()
                    logger(msg)
                    count += 1
                    if count % 50 == 0:
                        sys.stdout.write(f"\r{count} frames")
                        sys.stdout.flush()
        except KeyboardInterrupt:
            pass
        finally:
            bus.shutdown()
    print(f"\nWrote {count} frames to {args.output}")
    return 0


# --------------------------------------------------------------------------- main


def main():
    # Shared options, accepted on either side of the subcommand.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("-p", "--port", default="/dev/cu.usbserial-510", help="serial device")
    common.add_argument(
        "--serial-baud", type=int, default=2000000, help="USB-side baud (USB-CAN-A default 2000000)"
    )

    parser = argparse.ArgumentParser(
        description="Waveshare USB-CAN-A sniffer (listen-only by default)"
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", parents=[common], help="sweep bitrates and find the bus")
    s.add_argument("-s", "--seconds", type=float, default=2.0, help="listen time per candidate")
    s.add_argument(
        "--bitrates", type=int, nargs="+", default=DEFAULT_BITRATES, help="candidates to try"
    )
    s.set_defaults(func=cmd_scan)

    s = sub.add_parser("sniff", parents=[common], help="live ID table with byte-change map")
    s.add_argument("-b", "--bitrate", type=int, required=True)
    s.add_argument("-f", "--frame-type", choices=["STD", "EXT"], default="STD")
    s.add_argument("--mode", choices=["silent", "normal"], default="silent")
    s.add_argument("--ids", nargs="+", metavar="HEX", help="only show these arbitration IDs")
    s.add_argument("--raw", action="store_true", help="hide the decoded panel")
    s.add_argument("-s", "--seconds", type=float, default=0, help="stop after N seconds")
    s.add_argument("--refresh", type=float, default=0.4, help="redraw interval")
    s.set_defaults(func=cmd_sniff)

    s = sub.add_parser("events", parents=[common],
                       help="report byte changes only - use to find brake/throttle/lights")
    s.add_argument("-b", "--bitrate", type=int, required=True)
    s.add_argument("-f", "--frame-type", choices=["STD", "EXT"], default="STD")
    s.add_argument("--baseline", type=float, default=15.0,
                   help="seconds of do-nothing used to learn the noisy bytes")
    s.add_argument("--all", action="store_true", help="do not suppress noisy bytes")
    s.set_defaults(func=cmd_events)

    s = sub.add_parser("log", parents=[common], help="record frames to a file")
    s.add_argument("-b", "--bitrate", type=int, required=True)
    s.add_argument("-f", "--frame-type", choices=["STD", "EXT"], default="STD")
    s.add_argument("-o", "--output", required=True, help="ride.asc / ride.blf / ride.csv")
    s.add_argument("-s", "--seconds", type=float, default=0)
    s.set_defaults(func=cmd_log)

    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
