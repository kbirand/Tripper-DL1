# Tripper Puck (DL1)

![Tripper Puck wiring schematic — ESP32-S3 with the SN65HVD230 CAN transceiver,
Gravity 10DOF, NEO-8M GPS, SSD1306 OLED and two buttons, powered from the bike
through a 5 V BEC](wiring-schema.jpg)

A matchbox-sized BLE telemetry puck for e-bikes — companion hardware for the
[Tripper iOS app](https://github.com/kbirand/Tripper). It streams 5 Hz GPS,
on-chip-fused IMU orientation, and barometric data to the phone over
Bluetooth LE; the app records, analyzes, and exports. The puck itself is
stateless: it powers from the bike through a 5 V BEC, boots in seconds, and
needs no interaction beyond a marker button. It also reads the bike's own CAN
bus listen-only, adding battery, drivetrain and rider-control data to the
stream.

**Status:** bench-complete and phone-verified (2026-07-21). Remaining:
Tripper-side `ExternalSensorSource` (Swift), enclosure, field ride.

## How it works

- **No battery** — a 5 V BEC steps the bike's pack down to VUSB; boots with
  the bike
- **No SD card** — the phone is the recorder; BLE is the only data path
- **No screen dependence** — the OLED is a convenience dashboard; every
  feature works headless
- **Mount-zero calibration** — hold a button 10 s (or tap *Zero pitch & roll*
  in the app) and the current orientation becomes 0/0/0 (reference quaternion,
  persisted to flash, survives reboots, applied to both display and telemetry)
- **Ride awareness** — while the app records, the OLED runs inverted and a
  trip-time screen joins the rotation; the state re-syncs on every reconnect,
  so link flaps and mid-ride reboots heal themselves
- **Reads the bike** — listen-only CAN gives battery percentage, pack voltage,
  cell balance, motor current and rider controls without touching the bus

## Hardware

| Part | Role |
|---|---|
| Seeed XIAO ESP32-S3 | MCU, BLE 5.0, on-chip TWAI CAN controller |
| DFRobot Gravity 10DOF (BNO055 + BMP280) | On-chip sensor fusion + barometer, I²C |
| u-blox NEO-8M GPS (GYGPSV1 carrier) | 5 Hz position/speed/time, UART @ 115200 |
| SSD1306 0.91" OLED 128×32 | Clock / live-data / bike-CAN screens + all status, I²C `0x3C` |
| SN65HVD230 CAN transceiver | 3.3 V CAN PHY for the bike bus, D8/D9 |
| 12 mm button | Screen step / auto-cycle toggle |
| HW-483 button | Marker (click) / attitude zero (hold) |
| 5 V BEC (3 A) | Steps the bike's pack down to 5 V, feeds VUSB |

No SD card, no LiPo, no power switch, no status LED — the phone is the
recorder over BLE, the bike is the power source, and the OLED carries all
status. Physically the ESP32, CAN module and sensor board live at the rear;
the OLED, two buttons and GPS sit at the handlebar (GPS wants the sky view).

### Pin map (XIAO ESP32-S3)

| Pin | Function |
|---|---|
| D1 | Screen button → GND (internal pull-up) |
| D2 | Marker/zero button → GND (internal pull-up) |
| D4 / D5 | I²C SDA / SCL — BNO055 `0x28`, BMP280 `0x76`, OLED `0x3C` @ 100 kHz |
| D6 / D7 | UART TX→GPS RX / RX←GPS TX @ 115200 |
| D8 / D9 | CAN TX→CTX / RX←CRX (SN65HVD230), 250 kbit/s listen-only |
| D0, D3, D10 | free |

The I²C bus **must run at 100 kHz** — the BNO055's clock-stretching upsets
ESP32 I²C at higher speeds (validated: 59,722 reads / 0 errors / 10 min).
The firmware bursts OLED frames at 400 kHz between sensor transactions.

### Cable connection diagram

Pin columns match the physical XIAO ESP32-S3 viewed from above, USB-C at
the top:

```
                          USB-C (top edge) — flashing only
                      ┌───────────────────────┐
 n/c ─────────────────┤ D0                 5V ├──◄ BEC +5V ◄─ BEC 5V 3A ◄─ bike pack
 Screen button ○──────┤ D1                GND ├────────● GND rail ◄ BEC GND
 Marker/Zero button ○─┤ D2                3V3 ├────────● 3V3 rail
 n/c ─────────────────┤ D3                D10 ├─ n/c
 SDA bus ●────────────┤ D4                 D9 ├──────◄ CAN CRX
 SCL bus ●────────────┤ D5                 D8 ├──────► CAN CTX
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
             ├─ CAN module GND
             ├─ Screen button ○ (2nd leg)
             └─ Marker/Zero button ○ (2nd leg)
```

Wiring notes:

- **Power comes from a 5 V BEC**, not USB. The BEC steps the bike's traction
  pack down to 5 V / 3 A and feeds the XIAO's 5V (VUSB) pin. USB-C is only for
  flashing — **never plug USB in while the BEC is powered**: both drive VUSB
  and the two 5 V rails collide. Unplug one before the other.
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
| [`can_probe`](hardware/firmware/can_probe/) | CAN bring-up — decoded dashboard on the OLED and over BLE (`Tripper-CAN`, Nordic UART). Reports wirelessly on purpose: the bike and a USB host must never be connected at once |

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
| Screen button (D1) click | Step to the next OLED screen |
| Screen button (D1) 3 s hold | Toggle auto-cycling (thin border = cycling) |
| Marker button (D2) click | Marker counter++ in telemetry · MARK splash |
| Marker button (D2) 10 s hold | Zero roll/pitch/yaw at current orientation (progress bar → ZEROED) |

All status lives on the OLED. **Screens do not rotate on their own** — on a
moving bike the screen you picked should stay put, so D1 steps through them
and only a 3 s hold hands the stepping back to a 5 s timer (thin border while
it does). Three screens normally:

1. **Clock** — GPS time, UTC+3
2. **Data** — GPS speed hero, roll/pitch, satellites, g, fix dot, link state
3. **Bike CAN** — wheel speed hero, battery percentage, ride mode, kickstand;
   reads `no data` when no CAN frame has arrived in 2 s, `off` if the TWAI
   controller never came up

While the app is recording a ride the display runs inverted and a fourth
screen joins: the trip time (app-authoritative, so it tracks pauses and
survives reconnects). It disappears when the ride ends, and the screen index
falls back to the clock if it was showing.

## BLE protocol

Device name `Tripper-DL1`. One service, three characteristics:

| UUID | Char | Direction |
|---|---|---|
| `8E7C1A20-0F5A-4B9C-9C90-54B1D2A70001` | *service* | |
| `…0002` | telemetry | notify + read, 5 Hz, 70 B |
| `…0003` | status | notify + read, 1 Hz, 14 B |
| `…0004` | control | write / write-no-response |

### Telemetry packet (70 bytes, little-endian, packed)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | ver | `0x02` — `0x01` was the 50-byte pre-CAN packet |
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
| 50 | u8 | canFlags | bit0 live · bit1 kickstand down · bits3:2 ride mode (1 Eco, 2 Sport) |
| 51 | u16 | canSpeed_dkph | 0.1 km/h |
| 53 | u16 | canRpm | motor rpm |
| 55 | u16 | canPower_w | watts |
| 57 | u16 | canCurrent_da | 0.1 A |
| 59 | u16 | canPack_dv | 0.1 V |
| 61 | u8 | canSoc_pct | battery percentage |
| 62 | u16 | canDemand | throttle demand, units unconfirmed |
| 64 | u16 | cellHi_mv | highest cell, mV |
| 66 | u8 | cellHi_idx | its index, 1–16 |
| 67 | u16 | cellLo_mv | lowest cell, mV |
| 69 | u8 | cellLo_idx | its index, 1–16 |

Bytes 50–69 are the bike's CAN bus, read listen-only from a SN65HVD230 on
D8/D9 (see [Bike CAN bus](#bike-can-bus-talaria)). **The whole block is zeroed
and `canFlags` bit0 is clear whenever no frame has arrived in the last 2 s** —
transceiver unplugged, bike asleep, or bus fault. Gate every CAN field on that
bit rather than trusting a zero speed, or a parked bike and a disconnected
cable look identical.

Note `speed_cmps` at offset 26 is GPS ground speed in cm/s; `canSpeed_dkph`
at 51 is the bike's own wheel speed in 0.1 km/h. They are independent
measurements and will disagree — wheelspin, GPS lag, tyre circumference.

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

The bike's own CAN bus carries battery and drivetrain data the puck's sensors
can't see — pack voltage, cell balance, motor current, state of charge. The
production firmware reads it listen-only and appends it to the BLE telemetry
packet; [`tools/`](tools/) holds the host-side bring-up and reverse-engineering
kit that produced the decode.

**Hardware:** a 3.3 V **SN65HVD230** transceiver on D8 (GPIO7 → CTX) and
D9 (GPIO8 → CRX), CANH/CANL to the bike. Straight through, *not* a crossover
like the GPS UART. No ground wire is needed in the CAN tap — the puck and the
bike already share a ground through the BEC's negative rail. The ESP32-S3's own
TWAI controller is the CAN peripheral, so the module is only a physical layer.

**Remove the module's onboard 120 Ω terminator** (marked `121`, sitting across
CANH/CANL). The bike's bus is already terminated at both ends and reads 60 Ω;
a third resistor drops it to 40 Ω.

**Never connect USB and the bike at once.** Once CANH/CANL are spliced in, the
puck's ground *is* the bike's ground (shared through the BEC). Attaching a
mains-earthed host then ties the bike's battery negative to protective earth,
and USB 5 V collides with the BEC's 5 V on VUSB. Fit a 2-pin connector in the
CANH/CANL run, and either kill the BEC or unplug that connector before you
flash. Read telemetry over BLE instead.

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
