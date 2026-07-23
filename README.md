# Tripper Puck (DL1)

A matchbox-sized BLE telemetry puck for e-bikes — companion hardware for the
[Tripper iOS app](https://github.com/kbirand/Tripper). It streams 5 Hz GPS,
on-chip-fused IMU orientation, and barometric data to the phone over
Bluetooth LE; the app records, analyzes, and exports. The puck itself is
stateless: it powers from the bike's USB, boots in seconds, and needs no
interaction beyond a marker button.

**Status:** bench-complete and phone-verified (2026-07-21). Remaining:
Tripper-side `ExternalSensorSource` (Swift), enclosure, field ride.

## How it works

- **No battery** — powered by the e-bike's USB outlet; boots with the bike
- **No SD card** — the phone is the recorder; BLE is the only data path
- **No screen dependence** — the OLED is a convenience dashboard; every
  feature works headless
- **Mount-zero calibration** — hold a button 10 s (or tap *Zero pitch & roll*
  in the app) and the current orientation becomes 0/0/0 (reference quaternion,
  persisted to flash, survives reboots, applied to both display and telemetry)
- **Ride awareness** — while the app records, the OLED runs inverted and a
  trip-time screen joins the cycle; the state re-syncs on every reconnect, so
  link flaps and mid-ride reboots heal themselves

## Hardware

| Part | Role |
|---|---|
| Seeed XIAO ESP32-S3 | MCU, BLE 5.0, USB-C power |
| DFRobot Gravity 10DOF (BNO055 + BMP280) | On-chip sensor fusion + barometer, I²C |
| u-blox NEO-M8 GPS (GY-GPSU3 carrier) | 5 Hz position/speed/time, UART @ 115200 |
| SSD1306 0.91" OLED 128×32 | Clock / live-data screens + all status, I²C `0x3C` |
| 12 mm button | Marker (glove-friendly) |
| HW-483 button | Screen hold / attitude zero |

### Pin map (XIAO ESP32-S3)

| Pin | Function |
|---|---|
| D1 | Marker button → GND (internal pull-up) |
| D2 | Hold/zero button → GND (internal pull-up) |
| D4 / D5 | I²C SDA / SCL — BNO055 `0x28`, BMP280 `0x76`, OLED `0x3C` @ 100 kHz |
| D6 / D7 | UART TX→GPS RX / RX←GPS TX @ 115200 |
| D0, D3, D8–D10 | free |

The I²C bus **must run at 100 kHz** — the BNO055's clock-stretching upsets
ESP32 I²C at higher speeds (validated: 59,722 reads / 0 errors / 10 min).
The firmware bursts OLED frames at 400 kHz between sensor transactions.

### Cable connection diagram

Pin columns match the physical XIAO ESP32-S3 viewed from above, USB-C at
the top:

```
                      e-bike USB outlet / power bank
                                  │
                                  │ USB-C
                      ┌───────────┴───────────┐
 n/c ─────────────────┤ D0                 5V ├─ n/c
 Marker button ○──────┤ D1                GND ├────────● GND rail
 Hold/Zero button ○───┤ D2                3V3 ├────────● 3V3 rail
 n/c ─────────────────┤ D3                D10 ├─ n/c
 SDA bus ●────────────┤ D4                 D9 ├─ n/c
 SCL bus ●────────────┤ D5                 D8 ├─ n/c
 to GPS RX ◄──────────┤ D6 (TX)       (RX) D7 ├──────◄ from GPS TX
                      └───────────────────────┘
                            XIAO ESP32-S3

 ● 3V3 rail ─┬─ Gravity 10DOF VCC (red)     ● SDA bus (D4) ─┬─ Gravity SDA (blue)
             ├─ GPS VCC                                     └─ OLED SDA
             └─ OLED VCC
                                            ● SCL bus (D5) ─┬─ Gravity SCL (green)
 ● GND rail ─┬─ Gravity 10DOF GND (black)                   └─ OLED SCK
             ├─ GPS GND
             ├─ OLED GND
             ├─ Marker button ○ (2nd leg)
             └─ Hold/Zero button ○ (2nd leg)
```

Wiring notes:

- **GPS UART is a crossover** — XIAO TX (D6) feeds the GPS **RX** pin and
  vice versa. If the firmware reports "no NMEA data", these two are swapped.
- **Buttons need no resistors** — each connects its pin straight to GND;
  the firmware enables the ESP32's internal pull-ups.
- **The Gravity board's I²C comes from its 4-pin socket** (its back-side
  pad row is control pins only — no SDA/SCL there). Trim the cable to
  length and glue-lock the connector for vibration.
- **GPS patch antenna**: sky-facing, ≥ 15 mm from the XIAO and USB cable —
  the ESP32's RF noise measurably delays fixes.
- **GPS backup cell** (MS621 on the GY carrier): if it's flat, every power
  cycle factory-resets the module to 9600/1 Hz *and* forces a cold start
  (minutes to first fix instead of seconds). The firmware detects the revert
  and reconfigures automatically — boot log says `was at 9600 factory: BBR
  lost, check backup cell` — but only a healthy cell brings back hot starts.
  The cell trickle-charges while powered; if it never holds, replace it.

## Firmware

Arduino sketches in [`hardware/firmware/`](hardware/firmware/):

| Sketch | Purpose |
|---|---|
| [`tripper_puck`](hardware/firmware/tripper_puck/) | **Production firmware** — sensors → BLE + OLED |
| [`i2c_gate`](hardware/firmware/i2c_gate/) | BNO055 clock-stretch stress test (the go/no-go gate) |
| [`i2c_diag`](hardware/firmware/i2c_diag/) | Wiring diagnostic — line states + normal/swapped bus scans |
| [`gps_config_5hz`](hardware/firmware/gps_config_5hz/) | One-time GPS config: 5 Hz + 115200, saved to BBR (production firmware re-applies at boot) |
| [`gps_revert_factory`](hardware/firmware/gps_revert_factory/) | Test harness — reverts the GPS to 9600/1 Hz factory defaults (simulates a power cycle with a dead backup cell) to exercise `gpsBringup()`'s recovery path |

### Build & flash

```sh
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit BNO055" "Adafruit BMP280 Library" \
  "Adafruit SSD1306" "TinyGPSPlus" "NimBLE-Arduino"
cd hardware/firmware/tripper_puck
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 .
arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32S3 -p /dev/cu.usbmodem* .
```

### Controls & LED

| Input | Action |
|---|---|
| Marker button (D1) click | Marker counter++ in telemetry · MARK splash |
| Hold button (D2) click | Pin / unpin the current OLED screen (border = pinned) |
| Hold button (D2) 10 s hold | Zero roll/pitch/yaw at current orientation (progress bar → ZEROED) |

All status lives on the OLED, which alternates every 5 s between a GPS
clock screen (UTC+3) and a data screen (speed hero, roll/pitch,
satellites, g, fix dot, link state). While the app is recording a ride the
display runs inverted and a third screen joins the cycle: the trip time
(app-authoritative, so it tracks pauses and survives reconnects).

## BLE protocol

Device name `Tripper-DL1`. One service, three characteristics:

| UUID | Char | Direction |
|---|---|---|
| `8E7C1A20-0F5A-4B9C-9C90-54B1D2A70001` | *service* | |
| `…0002` | telemetry | notify + read, 5 Hz, 50 B |
| `…0003` | status | notify + read, 1 Hz, 14 B |
| `…0004` | control | write / write-no-response |

### Telemetry packet (50 bytes, little-endian, packed)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | ver | `0x01` |
| 1 | u8 | flags | bit0 fix valid · bit1 time valid |
| 2 | u32 | gpsTimeMs | UTC ms-of-day, `0xFFFFFFFF` if invalid |
| 6 | i32 | lat_e7 | degrees × 1e7 |
| 10 | i32 | lon_e7 | degrees × 1e7 |
| 14 | i32 | alt_cm | GPS altitude |
| 18 | i32 | baroAlt_cm | BMP280, std-atmosphere ref |
| 22 | u32 | press_pa | pressure |
| 26 | u16 | speed_cmps | cm/s |
| 28 | u16 | course_cdeg | degrees × 100 |
| 30 | u8 | sats | used in fix |
| 31 | u16 | hdop_c | HDOP × 100 |
| 33 | i16×4 | qw qx qy qz | quaternion × 16384, **mount-zeroed** |
| 41 | i16×3 | lin x y z | linear accel, mg (sensor frame) |
| 47 | i16 | maxG_mg | interval max \|lin\|, resets each packet |
| 49 | u8 | marker | increments per button press |

### Status packet (14 bytes)

`ver u8 · fix u8 · sats u8 · battPct u8 (0xFF = USB) · hdop_c u16 ·
uptime_s u32 · temp_x10 i16 · marker u8 · reserved u8`

### Control opcodes

| Byte | Payload | Action |
|---|---|---|
| `0x01` | — | Marker ack — MARK splash on the OLED |
| `0x02` | — | Zero roll/pitch/yaw at the current orientation — same as the 10 s button hold, saved to flash |
| `0x03` | — | Identify — OLED inverts for 2 s (relative to the ride-state invert) |
| `0x04` | `active u8 · elapsed_s u32` | Ride state — active inverts the OLED and adds the trip-time screen; elapsed seconds seed the timer. The app re-sends it on every reconnect |

## Bike CAN bus (Talaria)

Exploratory, not yet wired into the firmware. The bike's own CAN bus carries
battery and drivetrain data the puck's sensors can't see — pack voltage, cell
balance, motor current, state of charge. [`tools/`](tools/) holds the
bring-up and reverse-engineering kit.

**250 kbit/s, standard 11-bit IDs, 15 messages, ~92 frames/s.** Tap CAN_H and
CAN_L at the controller. A healthy bus reads 60 Ω across the pair with the
bike **off** (two 120 Ω terminators in parallel); 120 Ω means you're on one
end only. If the adapter has a 120 Ω termination jumper, pull it — the bus is
already terminated at both ends and a third resistor drops it to 40 Ω.

Everything defaults to **listen-only**: the transceiver never drives the bus
and never even ACKs, so a wrong bitrate cannot disturb the bike.

```sh
python3 -m venv .venv && ./.venv/bin/pip install -r tools/requirements.txt
./.venv/bin/python tools/can_sniff.py scan   -p /dev/cu.usbserial-510
./.venv/bin/python tools/can_sniff.py sniff  -p /dev/cu.usbserial-510 -b 250000
./.venv/bin/python tools/can_sniff.py events -p /dev/cu.usbserial-510 -b 250000
```

| Command | Purpose |
|---|---|
| `scan` | Sweeps bitrates × STD/EXT and reports which decodes traffic. Zero frames everywhere usually means CAN_H/CAN_L are swapped — harmless, just swap them |
| `sniff` | Live table: decoded physical values, per-ID rates, per-byte change map |
| `events` | Learns which bytes drift on their own, then prints only real changes. Operate one control at a time to find what carries it |
| `log` | Records to `.asc` / `.blf` / `.csv` for offline analysis |

### Decoded signals

Full provenance for each — how it was verified and what was ruled out — is in
the `CM_` comments of [`tools/talaria.dbc`](tools/talaria.dbc).

| Signal | Frame | Bytes | Scale |
|---|---|---|---|
| Speed | `0x303` | 0–1 LE | 0.1 km/h |
| Motor RPM | `0x203` | 0–1 LE | 1 rpm |
| Power | `0x203` | 2–3 LE | 1 W |
| Current | `0x302` | 4–5 LE | 0.1 A |
| Pack voltage | `0x101` | 0–1 LE | 0.1 V |
| Battery percentage | `0x401` | 0 | 1 % |
| Highest cell + index | `0x201` | 0–1 LE, 4 | 1 mV |
| Lowest cell + index | `0x201` | 2–3 LE, 5 | 1 mV |
| Kickstand | `0x202` | byte 0 bit 7 | 1 = down |
| Ride mode | `0x202` | byte 0 bits 5:4 | 1 Eco, 2 Sport |
| Throttle demand | `0x202` | 3–4 LE | units unconfirmed |

Speed and RPM hold a fixed 5.887 ratio at r = 0.999 across two independent
rides; integrating speed reproduces plausible ride distances. Cell high/low
averaged × 16 reproduces pack voltage to 0.02 V, confirming the 16S pack.

Traps worth knowing:

- **`0x103[0]` is not the battery percentage.** It reads a constant 100 while
  actual charge is elsewhere — almost certainly state of *health*.
- **`0x202[6:8]` is not an odometer.** It is monotonic, which is why it looks
  like one, but its rate has correlation −0.0005 with speed and its counts per
  km differ 34% between rides. No odometer has been found on the bus.
- **`0x490[0]` is not a temperature.** It is a ride-mode echo (r = 0.99).
- **Byte offsets vary by firmware.** Throttle demand sits at `0x202[3:5]` here
  but at `0x202[2:4]` on the bike in the reference logs, where it doesn't
  track throttle at all. Verify offsets before trusting this on another bike.

Horn and lights never reach the bus — they're switched 12 V circuits from the
bar switchgear. Brake levers appear on some bikes; aftermarket levers without
sensors produce nothing.

DBC groundwork from [inklit/Talaria_CAN](https://github.com/inklit/Talaria_CAN),
whose two ride logs were the reference data for every decode above.

## Docs

The full build story — shopping list (Turkey), decision records, validation
results, pitfalls — lives in [`hardware/build-guide.html`](hardware/build-guide.html).
