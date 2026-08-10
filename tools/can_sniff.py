#!/usr/bin/env python3
"""
CAN bring-up / reverse-engineering tool for the Waveshare USB-CAN-A.

The adapter speaks the "USB-CAN Analyzer" serial protocol, which python-can
exposes as the `seeedstudio` backend.

Everything here defaults to LISTEN-ONLY (silent) mode: the transceiver never
drives the bus and never even sends ACKs, so a wrong bitrate cannot spew error
frames at the bike's controller. Leaving silent mode requires --mode normal.

    scan        sweep candidate bitrates, report which one decodes traffic
    sniff       live table of IDs with a per-byte change map
    events      report byte changes only - press one control at a time
    transitions re-analyse a saved log the way `events` would have
    log         record to a file (.asc / .blf / .csv / .log) for later analysis
    send        TRANSMIT frames - the only command that drives the bus

Usage:
    python3 can_sniff.py scan  -p /dev/cu.usbserial-510
    python3 can_sniff.py sniff -p /dev/cu.usbserial-510 -b 500000
    python3 can_sniff.py log   -p /dev/cu.usbserial-510 -b 500000 -o ride.asc
    python3 can_sniff.py events -b 250000 --log press.asc -s 90
    python3 can_sniff.py transitions press.asc
    python3 can_sniff.py send  -b 250000 490#1160000000000000 --watch 2
"""

import argparse
import struct
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


def recv(bus, timeout=0.05):
    """bus.recv() that survives a truncated frame.

    The seeedstudio backend reads the 0xAA start byte and then reads the rest
    of the frame with no timeout guard, so if the serial read expires mid-frame
    it calls ord() on b'' and raises. A short recv timeout makes that likely -
    it killed a 100 s injection run on the bike at second 30. Dropping the torn
    frame is always right: the next one is 11 ms away on this bus.

    Keep the timeout comfortably above one frame time. It costs no latency,
    because recv returns the moment a frame lands, and this bus is never quiet.
    """
    try:
        return bus.recv(timeout=timeout)
    except (TypeError, struct.error):
        return None


def drain(bus, seconds):
    """Collect messages for `seconds`. Returns (count, {(id, ext), ...})."""
    count, ids = 0, set()
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = recv(bus)
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
    # Found 2026-08-10 by matching the bike's own display (400 km) against a
    # full-bus capture. Provisional until a ride shows it incrementing — this
    # bus carries round constants, so one static match is not proof.
    ("Odo", 0x402, lambda d: _u16(d, 2), "km", "%6.0f"),
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
            msg = recv(bus)
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


class ChangeDetector:
    """Per-byte change tracking with a learn-then-report split.

    Ordering is reported as an arrival sequence number, not a timestamp. The
    adapter hands us one serial stream, so the order bytes arrive IS the order
    the frames reached it, while host timestamps jitter with USB scheduling.
    When two frames change in the same burst, `seq` says which moved first and
    the clock does not - that ordering is what separates a command from its
    echo.
    """

    def __init__(self, ignore_ids=()):
        self.last = {}       # (id, byte index) -> value
        self.noisy = set()   # bytes that moved on their own during the baseline
        self.ignore_ids = set(ignore_ids)
        self.seq = 0

    def feed(self, arb, data, learning):
        """Consume one frame. Returns [(index, prev, new)] worth reporting."""
        self.seq += 1
        if arb in self.ignore_ids:
            return []
        out = []
        for i, byte in enumerate(data):
            key = (arb, i)
            prev = self.last.get(key)
            self.last[key] = byte
            if prev is None or prev == byte:
                continue
            if learning:
                self.noisy.add(key)
            elif key not in self.noisy:
                out.append((i, prev, byte))
        return out

    def suppressed(self):
        return ", ".join(f"{a:03X}[{b}]" for a, b in sorted(self.noisy)) or "none"


def bits_of(prev, new):
    delta = prev ^ new
    return " ".join(f"bit{b}" for b in range(8) if delta >> b & 1)


def annotate(arb, index, new):
    """Name the field when we already know what it is - keeps the eye honest."""
    if arb == 0x202 and index == 0:
        return (f"mode={MODES.get((new >> 4) & 3, '?')} "
                f"kickstand={'DOWN' if new >> 7 & 1 else 'up'}")
    if arb == 0x490 and index == 0:
        return f"mode={MODES.get((new >> 3) & 7, '?')} regen={new & 7}"
    return ""


class BurstPrinter:
    """Group changes into bursts so one button press reads as one block."""

    def __init__(self, gap, t0):
        self.gap = gap
        self.t0 = t0
        self.last_ts = None
        self.n = 0

    def emit(self, ts, seq, arb, index, prev, new):
        if self.last_ts is None or ts - self.last_ts > self.gap:
            self.n += 1
            print(f"\n--- event {self.n} @ {ts - self.t0:.2f}s "
                  f"{'-' * 46}")
        self.last_ts = ts
        note = annotate(arb, index, new)
        print(f"  #{seq:<7} {ts - self.t0:8.3f}s  {arb:03X}[{index}]  "
              f"0x{prev:02X} -> 0x{new:02X}  {bits_of(prev, new):<22} {note}")


def cmd_events(args):
    """Learn which bytes are noisy, then report only meaningful changes.

    Run it, sit still through the baseline, then work one control at a time:
    the byte that moves is the one carrying it. `--log` keeps the raw capture,
    because a session of button presses costs the rider's time and `transitions`
    can re-analyse the same file with different settings for free.
    """
    try:
        bus = open_bus(args.port, args.bitrate, args.frame_type, "silent", args.serial_baud)
    except Exception as exc:
        sys.exit(f"could not open {args.port}: {exc}")

    det = ChangeDetector(int(x, 16) for x in (args.ignore or ()))
    started = time.time()
    printer = BurstPrinter(args.gap, started)
    logger = can.Logger(args.log) if args.log else None
    announced = False

    print(f"Baseline: hold still and touch nothing for {args.baseline:.0f}s ...")
    sys.stdout.flush()
    try:
        while True:
            now = time.time()
            if args.seconds and now - started > args.seconds:
                break
            msg = recv(bus)
            learning = now - started < args.baseline

            if not announced and not learning:
                announced = True
                print(f"Baseline done - ignoring {len(det.noisy)} self-changing bytes.")
                print("Now operate ONE control at a time (mode, regen, kickstand, "
                      "brake, throttle).")
                print("Ctrl-C to stop.")
                sys.stdout.flush()

            if msg is None or msg.is_extended_id:
                continue
            if logger is not None:
                logger(msg)
            for index, prev, new in det.feed(msg.arbitration_id, msg.data, learning):
                printer.emit(now, det.seq, msg.arbitration_id, index, prev, new)
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
        if logger is not None:
            logger.stop()

    print(f"\nStopped. {printer.n} events. Suppressed bytes: {det.suppressed()}")
    if args.log:
        print(f"Raw capture kept at {args.log} - re-analyse with:  "
              f"python3 {sys.argv[0]} transitions {args.log}")
    return 0


def cmd_transitions(args):
    """Replay the `events` analysis over a saved log, offline and repeatable."""
    try:
        messages = list(can.LogReader(args.input))
    except Exception as exc:
        sys.exit(f"could not read {args.input}: {exc}")
    if not messages:
        sys.exit(f"{args.input} contains no frames")

    t0 = messages[0].timestamp
    det = ChangeDetector(int(x, 16) for x in (args.ignore or ()))
    printer = BurstPrinter(args.gap, t0)
    span = messages[-1].timestamp - t0
    print(f"{args.input}: {len(messages)} frames over {span:.1f}s, "
          f"baseline {args.baseline:.0f}s, burst gap {args.gap:.2f}s")

    for msg in messages:
        if msg.is_extended_id:
            continue
        learning = msg.timestamp - t0 < args.baseline
        for index, prev, new in det.feed(msg.arbitration_id, msg.data, learning):
            printer.emit(msg.timestamp, det.seq, msg.arbitration_id, index, prev, new)

    print(f"\n{printer.n} events. Suppressed bytes: {det.suppressed()}")
    return 0


# --------------------------------------------------------------------------- send


def parse_frame(spec):
    """cansend syntax: 490#1160000000000000  (hex id '#' hex data)."""
    if "#" not in spec:
        raise argparse.ArgumentTypeError(
            f"{spec!r}: expected ID#DATA, e.g. 490#1160000000000000"
        )
    id_text, data_text = spec.split("#", 1)
    data_text = data_text.replace(" ", "").replace(".", "")
    try:
        arb = int(id_text, 16)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{id_text!r} is not a hex CAN id")
    if len(data_text) % 2:
        raise argparse.ArgumentTypeError(f"{data_text!r} has an odd number of hex digits")
    try:
        data = bytes.fromhex(data_text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{data_text!r} is not hex")
    if len(data) > 8:
        raise argparse.ArgumentTypeError(f"{len(data)} data bytes; CAN 2.0 allows at most 8")
    return can.Message(arbitration_id=arb, data=data, is_extended_id=arb > 0x7FF)


def snapshot(bus, seconds):
    """Latest payload per standard ID over a short listen."""
    latest = {}
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = recv(bus)
        if msg is not None and not msg.is_extended_id:
            latest[msg.arbitration_id] = bytes(msg.data)
    return latest


def demand_of(data):
    """0x202[3:5] - the controller's current limit, and the witness that it
    actually accepted a mode command rather than merely displaying one."""
    return int.from_bytes(data[3:5], "little") if len(data) >= 5 else None


def state_line(latest):
    """The two bytes that carry mode and regen, decoded."""
    parts = []
    data = latest.get(0x490)
    if data:
        parts.append(f"0x490[0]=0x{data[0]:02X} {annotate(0x490, 0, data[0])}")
    data = latest.get(0x202)
    if data:
        parts.append(f"0x202[0]=0x{data[0]:02X} {annotate(0x202, 0, data[0])} "
                     f"demand={demand_of(data)}")
    return "   ".join(parts) or "no 0x202/0x490 seen"


def cmd_send(args):
    """Transmit frames and report whether the bike's own state bytes moved."""
    try:
        bus = open_bus(args.port, args.bitrate, args.frame_type, "normal", args.serial_baud)
    except Exception as exc:
        sys.exit(f"could not open {args.port}: {exc}")

    print("!! NORMAL mode - the adapter drives the bus and ACKs every frame.")
    for msg in args.frame:
        print(f"   will send {msg.arbitration_id:03X}#{msg.data.hex().upper()}"
              f"  x{args.repeat} every {args.interval * 1000:.0f} ms")
    if not args.now:
        print("   Ctrl-C within 3s to abort.")
        time.sleep(3)

    # Listening has to continue THROUGH the burst, not just bracket it. On this
    # bike an injected 0x490 is undone by the dash's own 5 Hz frame within
    # 100 ms of the last injected one, so a before/after pair straddles the
    # whole effect and reports "nothing changed" for a command that worked.
    timeline = []
    sent = 0
    try:
        for phase, seconds in (("before", args.watch),):
            _collect(bus, time.time() + seconds, phase, timeline)

        failed = False
        for _ in range(args.repeat):
            for msg in args.frame:
                try:
                    bus.send(msg)
                    sent += 1
                except Exception as exc:      # TX buffer full, bus-off
                    print(f"!! send failed after {sent} frames: {exc}")
                    failed = True
                    break
            if failed:
                break
            _collect(bus, time.time() + args.interval, "DURING", timeline)

        _collect(bus, time.time() + args.watch, "after", timeline)
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        bus.shutdown()

    print(f"sent    {sent} frames\n")
    _report(timeline)
    return 0


def _collect(bus, until, phase, timeline):
    while time.time() < until:
        msg = recv(bus)
        if msg is not None and not msg.is_extended_id:
            timeline.append((time.time(), phase, msg.arbitration_id, bytes(msg.data)))


def _report(timeline):
    """Per-phase state, then every change in the two bytes that carry it."""
    if not timeline:
        print("no frames heard - is the bus alive?")
        return
    t0 = timeline[0][0]
    for phase in ("before", "DURING", "after"):
        latest = {arb: data for _, p, arb, data in timeline if p == phase}
        if latest:
            print(f"  {phase:<7} {state_line(latest)}")

    print("\n  changes in 0x202 / 0x490:")
    prev = {}
    for ts, phase, arb, data in timeline:
        if arb not in (0x202, 0x490):
            continue
        key = (data[0], demand_of(data)) if arb == 0x202 else data[0]
        if prev.get(arb) == key:
            continue
        seen_before, prev[arb] = arb in prev, key
        if seen_before:
            extra = f" demand={demand_of(data)}" if arb == 0x202 else ""
            print(f"    {ts - t0:6.3f}s [{phase:>6}]  {arb:03X}[0]=0x{data[0]:02X}  "
                  f"{annotate(arb, 0, data[0])}{extra}")


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
                msg = recv(bus, 0.2)
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
    s.add_argument("--gap", type=float, default=0.25,
                   help="quiet time that separates one burst of changes from the next")
    s.add_argument("--ignore", nargs="+", metavar="HEX", help="ignore these ids entirely")
    s.add_argument("--log", metavar="FILE", help="also keep the raw capture (.asc/.blf/.csv)")
    s.add_argument("-s", "--seconds", type=float, default=0, help="stop after N seconds")
    s.set_defaults(func=cmd_events)

    s = sub.add_parser("transitions", help="re-run the events analysis over a saved log")
    s.add_argument("input", help="a log written by `log` or by `events --log`")
    s.add_argument("--baseline", type=float, default=15.0)
    s.add_argument("--gap", type=float, default=0.25)
    s.add_argument("--ignore", nargs="+", metavar="HEX", help="ignore these ids entirely")
    s.set_defaults(func=cmd_transitions)

    s = sub.add_parser("send", parents=[common],
                       help="TRANSMIT frames - the only command that drives the bus")
    s.add_argument("frame", nargs="+", type=parse_frame, metavar="ID#DATA",
                   help="e.g. 490#1160000000000000")
    s.add_argument("-b", "--bitrate", type=int, required=True)
    s.add_argument("-f", "--frame-type", choices=["STD", "EXT"], default="STD")
    s.add_argument("-n", "--repeat", type=int, default=1, help="send the set N times")
    s.add_argument("-i", "--interval", type=float, default=0.02, help="seconds between frames")
    s.add_argument("--watch", type=float, default=1.5, metavar="SEC",
                   help="listen this long before and after, and diff the state bytes")
    s.add_argument("--now", action="store_true", help="skip the 3s abort window")
    s.set_defaults(func=cmd_send)

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
