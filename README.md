# Tripper Puck (DL1)

![Tripper Puck wiring schematic — ESP32-S3 with the SN65HVD230 CAN transceiver,
Gravity 10DOF, NEO-8M GPS, SSD1306 OLED and two buttons, powered from the bike
through a 5 V BEC. Split across two enclosures: sensors and CAN at the rear,
display, GPS and buttons at the handlebar](wiring-schema.jpg)

The red callouts on the schematic mark the handlebar-split revision — the base
drawing still routes the OLED to D4/D5, which is superseded. Read the notes.

An editable version lives in [`hardware/tripper-puck.fzz`](hardware/tripper-puck.fzz)
(Fritzing). Every part is custom and embedded in the file, so it opens without
installing anything into your parts bin. The nets are the source of truth; the
part placement is machine-generated and worth tidying before you print it.

A matchbox-sized BLE telemetry puck for e-bikes — companion hardware for the
[Tripper iOS app](https://github.com/kbirand/Tripper). It streams on-chip-fused
IMU orientation and barometric data to the phone over Bluetooth LE, and reads
the bike's own CAN bus listen-only for battery, drivetrain and rider-control
data; the app records, analyzes, and exports. The puck itself is stateless: it
powers from the bike through a 5 V BEC and boots in seconds.

It comes in **two builds** that share one codebase and one BLE contract:

- **[Light build](#1--light-build)** — four parts in one box at the rear.
  IMU, barometer and bike CAN. The phone supplies position; the app is the UI.
- **[Full build](#2--full-build)** — adds a handlebar module: 5 Hz GPS, a
  0.91" OLED and two buttons. Strictly additive, so Light upgrades to Full
  without rewiring anything.

See [Choosing a build](#choosing-a-build) for the trade-offs.

**Status:** bench-complete and phone-verified (2026-07-21). Remaining:
Tripper-side `ExternalSensorSource` (Swift), enclosure, field ride.

## How it works

- **No battery** — a 5 V BEC steps the bike's pack down to VUSB; boots with
  the bike
- **No SD card** — the phone is the recorder; BLE is the only data path
- **No screen dependence** — the OLED is a convenience dashboard and every
  feature works headless. The Light build is exactly that claim taken to its
  conclusion: no screen at all, nothing lost from the recording
- **Mount-zero calibration** — the current orientation becomes 0/0/0 (reference
  quaternion, persisted to flash, survives reboots, applied to both display and
  telemetry). Triggered by a 10 s button hold on Full, or *Zero pitch & roll*
  in the app on either build
- **Ride awareness** — the app pushes ride state, so link flaps and mid-ride
  reboots heal themselves. On Full this inverts the OLED and adds a trip-time
  screen; on Light it is simply acknowledged
- **Reads the bike** — listen-only CAN gives battery percentage, pack voltage,
  cell balance, motor current and rider controls without touching the bus

## Choosing a build

Two configurations, one codebase, one BLE contract:

| | **Light build** | **Full build** |
|---|---|---|
| Physical form | one box at the rear | rear box **+ handlebar module** |
| Sensing | IMU, barometer, bike CAN | IMU, barometer, bike CAN, **GPS** |
| Display | none — the app is the UI | **0.91" OLED**, 4 screens |
| Buttons | none — the app sends control writes | **two** (screens · marker/zero) |
| Position from | the **phone's** GPS | the puck's own **5 Hz** GPS |
| Firmware | [`tripper_light`](hardware/firmware/tripper_light/) | [`tripper_puck`](hardware/firmware/tripper_puck/) |
| Parts | 4 | 8 |
| Wires to run | 6 | 14 |

**Start with the Light build.** It is the entire telemetry product — fused
orientation, g-force, barometric altitude and every decoded bike-CAN signal —
in four parts and one enclosure, and your phone already carries a GPS the app
already reads. The Full build buys 5 Hz position (the phone's is 1 Hz), a
dashboard you can read with the phone pocketed, and two hardware buttons.

**The Full build is strictly additive.** Nothing in the Light build gets
rewired to get there: the handlebar parts land on pins the Light build leaves
free, so it is an upgrade path rather than a different project.

The BLE packets are **byte-identical** across both, so the app needs no branch.
A Light puck reports its GPS block as invalid — the same state a Full puck
reports before it gets a fix, which the app already has to handle — and the
status packet's [capability byte](#status-packet-14-bytes) says which build is
on the other end.

## 1 · Light build

MCU, sensor board, CAN transceiver, power. One enclosure at the rear of the
bike; nothing on the handlebar.

| Part | Role |
|---|---|
| Seeed XIAO ESP32-S3 | MCU, BLE 5.0, on-chip TWAI CAN controller |
| DFRobot Gravity 10DOF (BNO055 + BMP280) | On-chip sensor fusion + barometer, I²C |
| SN65HVD230 CAN transceiver | 3.3 V CAN PHY for the bike bus, D8/D9 |
| 5 V BEC (3 A) | Steps the bike's pack down to 5 V, feeds VUSB |

No SD card, no LiPo, no power switch, no status LED, no screen — the phone is
the recorder *and* the interface over BLE, and the bike is the power source.

What you give up versus Full: no on-board position (the app uses phone GPS, so
rides still record), no glanceable display, and marker/mount-zero move to
[control writes](#control-opcodes) instead of buttons.

### Pin map — Light build

| Pin | Function |
|---|---|
| D4 / D5 | I²C SDA / SCL — BNO055 `0x28`, BMP280 `0x76` @ 100 kHz |
| D8 / D9 | CAN TX→CTX / RX←CRX (SN65HVD230), 250 kbit/s listen-only |
| D0, D1, D2, D3, D6, D7, D10 | free — D1, D2, D6, D7 are what the Full build claims |

I²C sits on the XIAO's **default pads**, so a bare `Wire.begin()` would work; the
firmware still names them explicitly via `PIN_SDA`/`PIN_SCL` so the pin map lives
in one place.

> **Troubleshooting — if every I²C device looks dead at once.** The first board
> used in this project had its **D4 pad clamped low**: it read 0 even with the
> internal pull-up enabled and nothing attached, shorted to ground somewhere on
> the board. An I²C line that cannot idle high is a bus no device can ever
> signal on, so it presents as *four* simultaneously dead sensors rather than
> one bad pin — which is why it is worth recognising. SDA was moved to D3 as a
> workaround until the board was replaced.
>
> To tell a dead pad from a wiring fault: pull the pin up, then down, and read
> it back. A pad the chip still controls follows both (1 then 0); one that reads
> 0 against the pull-up is tied to ground by something outside the GPIO.
> `padSweep()` in [`bench_imu_can`](hardware/firmware/bench_imu_can/) does this
> across the whole pad row every boot — one pin failing where six neighbours
> pass is the diagnosis — and
> [`i2c_diag`](hardware/firmware/i2c_diag/) localises it further.

### Wiring — Light build

Pin columns match the physical XIAO ESP32-S3 viewed from above, USB-C at
the top. Six wires plus the two CAN taps:

```
                          USB-C (top edge) — flashing only
                      ┌───────────────────────┐
 n/c ─────────────────┤ D0                 5V ├──◄ BEC +5V ◄─ BEC 5V 3A ◄─ bike pack
 n/c (Full: button 1) ┤ D1                GND ├────────● GND rail ◄ BEC GND
 n/c (Full: button 2) ┤ D2                3V3 ├────────● 3V3 rail
 n/c ─────────────────┤ D3                D10 ├─ n/c
 SDA bus ●────────────┤ D4                 D9 ├──────◄ CAN CRX
 SCL bus ●────────────┤ D5                 D8 ├──────► CAN CTX
 n/c (Full: GPS TX) ──┤ D6 (TX)       (RX) D7 ├─ n/c (Full: GPS RX)
                      └───────────────────────┘
                            XIAO ESP32-S3

 ● 3V3 rail ─┬─ Gravity 10DOF VCC (red)      ● SDA bus (D4) ── Gravity SDA (blue)
             └─ CAN module 3V3
                                             ● SCL bus (D5) ── Gravity SCL (green)
 ● GND rail ─┬─ Gravity 10DOF GND (black)
             └─ CAN module GND

 CAN module CANH ──◄ bike CAN_H          (2-pin connector — see the CAN section)
 CAN module CANL ──◄ bike CAN_L
```

## 2 · Full build

Everything in the Light build, plus a handlebar module: GPS for 5 Hz position,
an OLED for at-a-glance status, and two buttons. The GPS wants sky view and the
screen wants your eyeline, which is why these four live at the bars while the
MCU, sensor board and CAN transceiver stay in the rear box.

| Part it adds | Role |
|---|---|
| u-blox NEO-8M GPS (GYGPSV1 carrier) | 5 Hz position/speed/time, UART @ 115200 |
| SSD1306 0.91" OLED 128×32 | Clock / live-data / bike-CAN screens + all status, I²C `0x3C` |
| 12 mm button | Screen step / auto-cycle toggle |
| HW-483 button | Marker (click) / attitude zero (hold) |

### Pin map — additions

Only free pins are used, so the Light build's wiring is untouched:

| Pin | Function |
|---|---|
| D1 | Screen button → GND (internal pull-up) |
| D2 | Marker/zero button → GND (internal pull-up) |
| D6 / D7 | UART TX→GPS RX / RX←GPS TX @ 115200 |
| D3 / D10 | I²C bus 1 (`Wire1`) SDA / SCL — SSD1306 OLED `0x3C`. **Runs to the handlebar** |

The Light build's D4/D5 bus is untouched and stays **rear-enclosure only**: the
OLED gets its own bus rather than joining it.

**Two I²C buses, on purpose.** The sensors and the display sit in different
enclosures about a metre apart, so they get separate buses. Bus 0 stays short
— 15 cm inside the rear box — because the BNO055's clock-stretching already
forces it to **100 kHz** (validated: 59,722 reads / 0 errors / 10 min) and it
is the one bus whose failure costs real data. Bus 1 carries only the OLED out
to the handlebar; a metre of cable there can glitch the screen but can no
longer touch sensor fusion. Wire1 idles at 100 kHz and bursts frames at
400 kHz (`OLED_BUS_HZ` / `OLED_BURST_HZ` in the firmware) — if a long or
unshielded run tears the display, drop the burst to 100 kHz first.

### Wiring — Full build

Same as the Light build with the handlebar module added. The four new parts land on the
pins the Light build left free, so nothing already wired moves:

Signals marked **⇢ bar** leave the rear enclosure through the 8-pin handlebar
connector; everything else stays inside the rear box.

```
                          USB-C (top edge) — flashing only
                      ┌───────────────────────┐
 n/c ─────────────────┤ D0                 5V ├──◄ BEC +5V ◄─ BEC 5V 3A ◄─ bike pack
 Screen button ⇢ bar ─┤ D1                GND ├────────● GND rail ◄ BEC GND
 Marker/Zero  ⇢ bar ──┤ D2                3V3 ├────────● 3V3 rail
 OLED SDA     ⇢ bar ──┤ D3                D10 ├── OLED SCK  ⇢ bar
 Sensor SDA ●─────────┤ D4                 D9 ├──────◄ CAN CRX
 Sensor SCL ●─────────┤ D5                 D8 ├──────► CAN CTX
 to GPS RX  ⇢ bar ◄───┤ D6 (TX)       (RX) D7 ├──────◄ from GPS TX  ⇢ bar
                      └───────────────────────┘
                            XIAO ESP32-S3

 REAR enclosure                          HANDLEBAR enclosure
 ──────────────                          ───────────────────
 ● 3V3 rail ─┬─ Gravity 10DOF VCC (red)  OLED VCC / GND / SDA / SCK
             ├─ CAN module 3V3           GPS  VCC / GND / TX / RX
             └─── 3V3 ⇢ bar              Screen button ○ ─┐
                                         Marker/Zero   ○ ─┴─ to GND ⇢ bar
 ● Sensor SDA (D4) ── Gravity SDA (blue)
 ● Sensor SCL (D5) ── Gravity SCL (green)   ┌── 100 µF bulk cap across
                                            │   3V3/GND at this end
 ● GND rail ─┬─ Gravity 10DOF GND (black)  ─┘
             ├─ CAN module GND
             └─── GND ⇢ bar

 8-pin handlebar harness (M12 A-coded)
   1 3V3   2 GND   3 OLED SDA (D3)   4 OLED SCK (D10)
   5 GPS RX (◄D6)  6 GPS TX (►D7)    7 btn1 (D1)   8 btn2 (D2)
```

Wiring notes (both builds unless noted):

- **Power comes from a 5 V BEC**, not USB. The BEC steps the bike's traction
  pack down to 5 V / 3 A and feeds the XIAO's 5V (VUSB) pin. USB-C is only for
  flashing — **never plug USB in while the BEC is powered**: both drive VUSB
  and the two 5 V rails collide. Unplug one before the other.
- **GPS UART is a crossover** *(Full only)* — XIAO TX (D6) feeds the GPS **RX**
  pin and vice versa. If the firmware reports "no NMEA data", these two are
  swapped. Note the CAN pair on D8/D9 is *not* a crossover: CTX→CTX, CRX→CRX.
- **Buttons need external pull-ups now that they're a metre away** *(Full
  only)*. Each still connects its pin straight to GND and the firmware still
  enables the internal pull-up — but that pull-up is ~45 kΩ, and a metre of
  wire on a high-impedance input is an antenna for BEC and motor-controller
  switching noise. Add a **10 kΩ to 3V3 and a 100 nF to GND at the ESP32 end**
  of D1 and D2. On a breadboard with short leads you can skip this; on the bike
  you cannot.
- **The Gravity board's I²C comes from its 4-pin socket** (its back-side
  pad row is control pins only — no SDA/SCL there). Trim the cable to
  length and glue-lock the connector for vibration.
- **The handlebar box is 3.3 V only.** Both things in it — the SSD1306 and the
  NEO-8M carrier — run off the XIAO's own 3V3 regulator; the only 5 V anywhere
  in the build is the BEC feeding VUSB inside the rear box. Never run 5 V up to
  the bar. This also raises the stakes on the two rules below: there is no
  regulator at the far end to absorb cable drop, so whatever the metre of wire
  loses comes straight out of the GPS's supply margin.
- **The handlebar run** carries 8 conductors (see the harness table above) over
  roughly a metre. Rules for it:
  - **Shielded multicore**, shield to GND **at the rear end only** — grounding
    both ends makes a loop.
  - **SDA and SCL each twisted with their own GND return**, never twisted with
    each other; paired together they crosstalk. Same for GPS TX/RX.
  - **2.2 kΩ pull-ups on the OLED bus**, fitted at the ESP32 end, replacing the
    module's own 4.7–10 kΩ. A metre of cable adds ~100 pF and the weaker
    pull-ups can't pull the edges up through it. Sizing, if your run is longer:
    the pull-up and the bus capacitance set the rise time, which must stay under
    1 µs at 100 kHz, while the sink current (3.3 V ÷ R) must stay under 3 mA.

    | Run | Bus C | 2.2 kΩ | 1.5 kΩ |
    |---|---|---|---|
    | 1 m | ~150 pF | 0.27 µs | 0.19 µs |
    | 2 m | ~250 pF | 0.7 µs | 0.45 µs |
    | 3 m | ~350 pF | 1.2 µs ✗ | 0.63 µs |

    So **2.2 kΩ to ~2 m, 1.5 kΩ (2.2 mA sink) to ~3 m.** Past 3 m the 400 pF
    bus-capacitance ceiling binds and no pull-up value saves it.
  - **100 µF bulk capacitor at the handlebar end**, across 3V3/GND near the GPS.
    Its acquisition current spikes into the cable's inductance otherwise sag
    the rail and lengthen time-to-fix.
  - **≥ 26 AWG** on the 3V3 and GND conductors. The bar end draws ~60 mA, which
    is only ~16 mV of drop at 26 AWG — but thin 30 AWG hookup wire more than
    doubles that, and it comes off a 3.3 V rail with no headroom to spare.
  - **Past 2 m, drop `OLED_BURST_HZ` to 100 kHz** and move to the 1.5 kΩ
    pull-ups. Don't reach for an I²C buffer (P82B715 / P82B96) — the arithmetic
    above says plain I²C covers any run a motorcycle actually needs, and both
    parts are out of stock domestically anyway (the P82B715 is obsolete).
  - **Keep the cable thin.** 8 × 0.14 mm² (≈26 AWG) shielded is ~6 mm OD and
    still flexible; 0.25 mm² and up gets stiff enough to fatigue at the steering
    head. 0.14 mm² carries the ~60 mA with room to spare.
- **GPS patch antenna**: sky-facing, ≥ 15 mm from the XIAO and USB cable —
  the ESP32's RF noise measurably delays fixes. Moving the GPS to the handlebar
  enclosure satisfies this for free, since the ESP32 stays at the rear.
- **GPS backup cell** (MS621 on the GY carrier): if it's flat, every power
  cycle factory-resets the module to 9600/1 Hz *and* forces a cold start
  (minutes to first fix instead of seconds). The firmware detects the revert
  and reconfigures automatically — boot log says `was at 9600 factory: BBR
  lost, check backup cell` — but only a healthy cell brings back hot starts.
  The cell trickle-charges while powered; if it never holds, replace it.

### Connectors

Two enclosures means two harnesses, and they are deliberately different sizes
so they cannot be cross-plugged.

| Run | Conductors | Connector |
|---|---|---|
| Bike → rear enclosure | +5 V, GND, CAN_H, CAN_L | **M8 4-pin**, IP67 |
| Rear → handlebar enclosure | **3V3**, GND, SDA, SCL, GPS TX, GPS RX, btn1, btn2 | **M12 8-pin A-coded**, IP67 |

The two runs carry different voltages — 5 V to the rear box, 3.3 V onward to the
bar — which is a second reason the connectors are deliberately different sizes.

**Gender follows the live side.** Whichever end stays energised when the pair
is separated gets the *female* sockets — recessed contacts can't be shorted
against the frame. On the bike run that's the cable (it goes back to the BEC),
so the rear enclosure carries a **male** panel receptacle and the cable carries
a **female** plug. The handlebar run is dead on both sides once the bike
connector is pulled, so either orientation is safe there; keep the panel male
for consistency.

Parts that work, sourced domestically (prices July 2026): rear panel
**ATTEND 219A-04MSR** ≈ ₺243, bike cable **SIGNAL ASM08AF04001** ≈ ₺282. If you
also want the BEC and CAN tap to separate from the 4-core, add
**ASM08AM04001** + a second **ASM08AF04001** — but note these field-assembly
bodies take a *single* cable gland, so the BEC pair and CAN pair must be merged
into one jacket before the connector either way. Unless that junction needs to
come apart, solder and adhesive-lined heat-shrink is more vibration-tolerant
and free.

**Keep the pin assignment identical at both ends of every run.** With two
connectors in the harness it is easy to mirror one and feed 5 V into CAN_H.

## Firmware

Arduino sketches in [`hardware/firmware/`](hardware/firmware/):

| Sketch | Purpose |
|---|---|
| [`tripper_light`](hardware/firmware/tripper_light/) | **Light-build firmware** — IMU + baro + CAN → BLE. No GPS/OLED/button code at all; marker and mount-zero arrive as control writes. Same UUIDs and same 70-byte packet as the Full build |
| [`tripper_puck`](hardware/firmware/tripper_puck/) | **Full-build firmware** — sensors + GPS → BLE + OLED |
| [`bench_imu_can`](hardware/firmware/bench_imu_can/) | Bring-up rig for a half-built puck — IMU + baro + CAN only, no OLED/buttons/GPS. Talks to the BNO055 registers directly (a library `begin()` can't tell a phantom ACK from a chip), counts every failed read, and re-probes missing sensors every 3 s so a wire soldered mid-run comes up on its own. Same BLE UUIDs and packets as production |
| [`i2c_gate`](hardware/firmware/i2c_gate/) | BNO055 clock-stretch stress test (the go/no-go gate) |
| [`i2c_diag`](hardware/firmware/i2c_diag/) | Wiring diagnostic — line states + normal/swapped bus scans |
| [`gps_config_5hz`](hardware/firmware/gps_config_5hz/) | One-time GPS config: 5 Hz + 115200, saved to BBR (production firmware re-applies at boot) |
| [`gps_revert_factory`](hardware/firmware/gps_revert_factory/) | Test harness — reverts the GPS to 9600/1 Hz factory defaults (simulates a power cycle with a dead backup cell) to exercise `gpsBringup()`'s recovery path |
| [`can_probe`](hardware/firmware/can_probe/) | CAN bring-up — decoded dashboard on the OLED and over BLE (`Tripper-CAN`, Nordic UART). Reports wirelessly on purpose: the bike and a USB host must never be connected at once |

### Build & flash

```sh
arduino-cli core install esp32:esp32

# Light build — no SSD1306 or TinyGPSPlus needed
arduino-cli lib install "Adafruit BNO055" "Adafruit BMP280 Library" "NimBLE-Arduino"
arduino-cli compile -b esp32:esp32:XIAO_ESP32S3 hardware/firmware/tripper_light
arduino-cli upload  -b esp32:esp32:XIAO_ESP32S3 -p /dev/cu.usbmodem* hardware/firmware/tripper_light

# Full build — adds the display and GPS libraries
arduino-cli lib install "Adafruit SSD1306" "TinyGPSPlus"
arduino-cli compile -b esp32:esp32:XIAO_ESP32S3 hardware/firmware/tripper_puck
arduino-cli upload  -b esp32:esp32:XIAO_ESP32S3 -p /dev/cu.usbmodem* hardware/firmware/tripper_puck
```

⚠️ **Never plug USB in while the BEC is powered, and never while the CAN pair
is spliced in** — see [Bike CAN bus](#bike-can-bus-talaria). Unplug the 2-pin
CAN connector or kill the BEC before flashing.

The two sketches keep duplicate copies of the packet structs, the CAN decode
and the UUIDs. That is deliberate — each is a standalone Arduino sketch — but
it means the two can drift. Anything touching the wire format must land in
both; this check should print nothing:

```sh
for f in tripper_puck tripper_light; do
  awk '/^struct __attribute__\(\(packed\)\) TelemetryPacket/,/^static_assert\(sizeof\(StatusPacket\)/' \
    hardware/firmware/$f/$f.ino | grep -E "^\s+(uint|int)[0-9_]+t\s" | sed 's|//.*||;s/[[:space:]]*$//' > /tmp/$f.f
done; diff /tmp/tripper_puck.f /tmp/tripper_light.f && echo "packets agree"
```

### Controls — Full build only

| Input | Action |
|---|---|
| Screen button (D1) click | Step to the next OLED screen |
| Screen button (D1) 3 s hold | Toggle auto-cycling (thin border = cycling) |
| Marker button (D2) click | Marker counter++ in telemetry · MARK splash |
| Marker button (D2) 10 s hold | Zero roll/pitch/yaw at current orientation (progress bar → ZEROED) |

The Light build has no buttons and no screen: markers arrive as control write
`0x01` and mount-zero as `0x02`, both from the app. Everything below describes
the Full build's display.

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
| 50 | u8 | canFlags | bit0 live · bit1 kickstand down · bits3:2 ride mode (1 Eco, 2 Sport) · bits6:4 regen level 1–4 (0 = unknown) |
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

`ver u8 · fix u8 · sats u8 · battPct u8 (0xFF = external supply) · hdop_c u16 ·
uptime_s u32 · temp_x10 i16 · marker u8 · caps u8`

The last byte was `reserved` (always 0) and now carries **capability bits**, so
the app can tell the builds apart — chiefly to know whether it must supply
position from the phone:

| Bit | Meaning | Light | Full |
|---|---|---|---|
| `0x01` | has GPS | — | ✅ |
| `0x02` | has OLED | — | ✅ |
| `0x04` | has buttons | — | ✅ |
| `0x08` | has CAN | ✅ | ✅ |

So Light sends `0x08` and Full sends `0x0F`. **A value of `0` means firmware
older than this field**, not "no capabilities" — which is why `has CAN` is an
explicit bit rather than assumed.

### Control opcodes

| Byte | Payload | Action |
|---|---|---|
| `0x01` | — | **Full:** marker ack — MARK splash on the OLED (the button already counted it). **Light:** originates the marker, incrementing the counter — there is no button |
| `0x02` | — | Zero roll/pitch/yaw at the current orientation — same as the 10 s button hold on Full, the only way to do it on Light. Saved to flash on both |
| `0x03` | — | Identify — **Full:** OLED inverts for 2 s. **Light:** accepted and logged, no indicator to flash |
| `0x04` | `active u8 · elapsed_s u32` | Ride state — **Full:** inverts the OLED and adds the trip-time screen, elapsed seeds the timer. **Light:** accepted and logged, no display. The app re-sends it on every reconnect |

Both builds accept all four opcodes, so the app never has to withhold a write.

## Bike CAN bus (Talaria)

The bike's own CAN bus carries battery and drivetrain data the puck's sensors
can't see — pack voltage, cell balance, motor current, state of charge. **Both
builds** read it listen-only and append it to the BLE telemetry packet — CAN is
in the rear box, so it is part of the Light build, not an upgrade;
[`tools/`](tools/) holds the host-side bring-up and reverse-engineering
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
and USB 5 V collides with the BEC's 5 V on VUSB. **Unplug the 4-pin bike
connector before you attach USB** — it carries +5 V, GND, CAN_H and CAN_L
together, so pulling it isolates both hazards at once. (It replaces the 2-pin
CAN-only break earlier revisions called for; one connector, one action, nothing
to forget.) Read telemetry over BLE instead.

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
| Regen level | `0x490` | byte 0 bits 2:0 | 1–4 |
| Ride mode echo | `0x490` | byte 0 bits 5:3 | 1 Eco, 2 Sport |
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
- **`0x490[0]` is not a temperature, and it is not one field.** Bits 5:3 echo
  the ride mode (r = 0.99, which is why the whole byte looked like mode alone)
  and bits 2:0 carry the regen level. Reading the low *nibble* as regen works
  only in Sport: in Eco bit 3 is set, so regen 1 reads `0x09` rather than
  `0x01`. Mask 3 bits, not 4.
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

What the **app** side has to do differently — the capability byte, the regen
level, gating on `canFlags`, and why `battPct` is a sentinel rather than a
reading — is in [`docs/app-integration.md`](docs/app-integration.md).
