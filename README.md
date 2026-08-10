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
| [`tripper_light`](hardware/firmware/tripper_light/) | **Light-build firmware** — IMU + baro + CAN → BLE. No GPS/OLED/button code at all; marker and mount-zero arrive as control writes. Same UUIDs and same 88-byte packet as the Full build |
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

USB is needed exactly twice in a puck's life — the first flash of an
OTA-capable firmware, and recovery if an update bricks it. After that the puck
raises its own WiFi network on request and can be flashed from a laptop *or
straight from a phone browser* with the bike untouched; the last built image is
committed at [`releases/`](releases/) so no toolchain is needed at all.
**[`docs/flashing.md`](docs/flashing.md) is the complete reference** —
networks, passwords, both command lines, making a `.bin`, and which of the nine
build artifacts is the one you flash.

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
| Marker button (D2) 10 s hold | Zero the mount at the current orientation (progress bar → ZEROED, or `ACC n/3` if the accelerometer isn't calibrated yet) |

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

Device name `Tripper-DL1`. One service, four characteristics:

| UUID | Char | Direction |
|---|---|---|
| `8E7C1A20-0F5A-4B9C-9C90-54B1D2A70001` | *service* | |
| `…0002` | telemetry | notify + read, 5 Hz, 92 B |
| `…0003` | status | notify + read, 1 Hz, 15 B |
| `…0004` | control | write / write-no-response |
| `…0005` | raw | notify, 10 Hz, 148 B — the un-averaged 100 Hz IMU stream |

### Telemetry packet (86 bytes, little-endian, packed)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | ver | `0x05` — byte-identical *layout* to `0x04`; the bump marks a semantics change (see `gyr`/`acc` below). `0x03` was the 78-byte packet with no raw accelerometer, `0x02` the 70-byte one with no IMU health, `0x01` the 50-byte pre-CAN one |
| 1 | u8 | flags | bit0 fix valid · bit1 time valid · bit2 IMU calibration usable |
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
| 70 | i16×3 | gyr x y z | gyro, deg/s × 16 (sensor frame). **`0x05`: the mean of the 200 ms window**, not the newest 100 Hz sample |
| 76 | u8 | calib | bits 7:6 sys · 5:4 gyro · 3:2 accel · 1:0 mag, each 0–3 |
| 77 | u8 | zeroCount | mount zeros the puck has **accepted** since boot |
| 78 | i16×3 | acc x y z | **raw** accelerometer, mg (sensor frame) — gravity included, pre-fusion. **`0x05`: window mean**, as `gyr` |
| 84 | u16 | quatRejects | non-unit `getQuat()` reads dropped since boot, saturating |
| 86 | u16 | accDev_mg | **`0x06`:** largest \|‖a‖ − 1 g\| seen at 100 Hz inside this window, mg — the peak the `acc` mean cannot show |
| 88 | u32 | canOdo_km | **`0x07`:** the bike's odometer, whole km, from CAN `0x402[2:4]`. 0 means not seen yet |

Bytes 50–69 are the bike's CAN bus, read listen-only from a SN65HVD230 on
D8/D9 (see [Bike CAN bus](#bike-can-bus-talaria)). **The whole block is zeroed
and `canFlags` bit0 is clear whenever no frame has arrived in the last 2 s** —
transceiver unplugged, bike asleep, or bus fault. Gate every CAN field on that
bit rather than trusting a zero speed, or a parked bike and a disconnected
cable look identical.

Note `speed_cmps` at offset 26 is GPS ground speed in cm/s; `canSpeed_dkph`
at 51 is the bike's own wheel speed in 0.1 km/h. They are independent
measurements and will disagree — wheelspin, GPS lag, tyre circumference.

Bytes 70–77 are IMU health, appended *after* the CAN block so every `0x02`
offset stays byte-identical and an older app keeps parsing. They exist because
raw gyro and calibration used to stay on the puck — which is why a lean fault
took a GPS cross-check to diagnose rather than a look at the recording. `mag` is
always 0: IMUPLUS never turns the magnetometer on. See
[IMU calibration](#imu-calibration) for what the numbers mean.

`zeroCount` exists because the mount zero is a fire-and-forget control write
that the puck is allowed to refuse — so "the rider tapped the button" and "the
puck re-referenced itself" are different facts, and on a Light build there is no
screen to tell them apart. It increments only on a zero the puck **accepted**.
Watch for a *change* against the value you held when you sent `0x02`, never for
a particular number: it wraps at 255 and restarts at 0 on reboot.

Bytes 78–85 are the **raw accelerometer** and the quaternion-reject count, and
they exist because everything above them that bears on attitude — the
quaternion, the linear accel, even the calibration byte — is a *product* of the
BNO055's fusion. When the fusion itself is the suspect, those fields agree with
each other by construction, whatever they say. A ride that read −10° of lean
down a dead-straight road ruled out the mount, the axes, the zero, gyro drift,
the bike's own acceleration and vibration, then ran out of evidence, because
every remaining witness was the accused.

`acc` is the pre-fusion measurement: gravity plus motion, straight off the
sensor, in the sensor frame and deliberately not rotated by anything.
**`acc − lin` is the fusion's own gravity vector** — compare its direction
against the quaternion, and against the low-passed direction of `acc` itself,
and "is the chip fed bad data, or mishandling good data" becomes answerable
from a recording. `quatRejects` should stay flat; a count that climbs mid-ride
means the attitude is being *held* across glitched I2C reads rather than
tracking the bike.

**Why `0x06` adds a peak beside the mean.** Averaging is what makes `acc`
usable as a direction, and it is also what makes it useless as a *warning*.
The app's estimator only takes its "down" reference from the accelerometer
while `‖a‖` is near 1 g — braking bumps and drops point it somewhere that
isn't down — but it was applying that test to the window mean, which is the
one number that cannot reveal a jolt inside its own window. The 100 Hz flight
recorder settled it on the 2026-08-09 rides: the real `‖a‖` was outside
0.8–1.2 g on **17–20% of moving samples**, while the means were outside on
**0.6–1.3%**. The gate had been tuned against a signal already smoothed into
compliance and was passing almost everything it exists to refuse. A mean
cannot be un-averaged, so the peak is taken here, where the samples are.

Note this is *not* `maxG_mg`, which latches the peak of the fusion's linear
acceleration — a product of the very fusion `acc` exists to cross-check.

**Why `0x05` averages.** Through `0x04` these two fields carried whichever
100 Hz sample happened to land when the packet was built — 1 in 20 reached the
app and the other 19 were discarded, so a single vibration spike could be the
one that got through. `0x05` sends the mean of all 20 instead. The layout did
not move a byte, which is why an app that ignores the version still parses it.

That fixed the spikes and introduced a subtler problem: a mean is a boxcar
filter, and a boxcar has a first sidelobe only 13 dB down. Suspension
resonance (2–5 Hz) and wheel hop (10–20 Hz) are above the 2.5 Hz Nyquist of a
5 Hz output, so they **alias** into the road-grade band rather than being
removed, and no amount of downstream filtering can separate them again. That
is what the raw characteristic below exists for.

### Status packet (15 bytes)

`ver u8 · fix u8 · sats u8 · battPct u8 (0xFF = external supply) · hdop_c u16 ·
uptime_s u32 · temp_x10 i16 · marker u8 · caps u8 · otaState u8`

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

`otaState` (byte 14) reports WiFi flashing mode as the puck actually has it,
not as the app last asked for it: `0` off · `1` network up with no client ·
`1+n` with n clients joined. The app's toggle reads this back so it shows the
truth after a reconnect, and the mode never survives a power cycle. Length-gate
it — a 14-byte status packet is older firmware, not `otaState = 0`.

### Raw stream (148 bytes per batch, little-endian, packed)

The flight recorder: the 100 Hz IMU samples themselves, no averaging. Ten per
notification, ten notifications a second — about 1.5 kB/s against the 185 B ATT
MTU iOS negotiates, so one batch is one notification.

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | ver | `0x01` |
| 1 | u8 | count | RawSamples following the header (10) |
| 2 | u16 | period_us100 | nominal sample period, units of 100 µs (`100` = 10 ms) |
| 4 | u32 | t0_ms | **puck** millis at the first sample in this batch |
| 8 | i16×4 | qw qx qy qz | the BNO055's own fusion quaternion, ×16384, **unzeroed** |
| 16 | i32 | press_pa | raw pressure — *not* converted altitude |
| 20 | i16 | temp_x10 | °C × 10 |
| 22 | u8×4 | calSys calGyr calAcc calMag | 0–3 each |
| 26 | u8×2 | pad | |
| 28 | i16×6 | gx gy gz ax ay az | ×10 samples: gyro deg/s × 100, accel mg |

**`t0_ms` is the whole point of the header.** BLE delivers in bursts, so the
phone's arrival time is not when the sample was taken. At 5 Hz nobody noticed;
at 100 Hz it would corrupt every derivative computed downstream. Reconstruct
the timeline from the puck's own counter and the nominal period, and use the
phone's clock only to anchor the start of the ride.

Everything here is **raw and un-decided**: pressure rather than altitude
because the conversion is lossy, the quaternion unzeroed because the mount
reference is an app-side convention that may be re-derived later, and samples
rather than means because a mean cannot be un-averaged. The chip's own fusion
is logged even though it was convicted (see
[`docs/data-quality-audit.md`](docs/data-quality-audit.md)) on the grounds that
a verdict nobody can re-test is an opinion.

The stream is **additive**: a phone that never subscribes sees the puck behave
exactly as it did before the characteristic existed.

### Control opcodes

| Byte | Payload | Action |
|---|---|---|
| `0x01` | — | **Full:** marker ack — MARK splash on the OLED (the button already counted it). **Light:** originates the marker, incrementing the counter — there is no button |
| `0x02` | — | Zero the mount at the current orientation — same as the 10 s button hold on Full, the only way to do it on Light. Saved to flash on both. Refused while accel calibration is under 3/3 |
| `0x03` | — | Identify — **Full:** OLED inverts for 2 s. **Light:** accepted and logged, no indicator to flash |
| `0x04` | `active u8 · elapsed_s u32` | Ride state — **Full:** inverts the OLED and adds the trip-time screen, elapsed seeds the timer. **Light:** accepted and logged, no display. The app re-sends it on every reconnect |
| `0x05` | `on u8` | WiFi flashing mode — raises the puck's own SoftAP for OTA and the diagnostics/`/update` web server. Never persists across a power cycle. Live state comes back in `otaState`. See [`docs/flashing.md`](docs/flashing.md) |
| `0x06` | `on u8` | Raw 100 Hz stream on/off. **Boots on** — every ride should be a regression test, and the phone decides whether to keep the bytes. The switch is there for the day a link problem needs the radio quiet |
| `0x07` | `mode u8` | **Ride-mode override** (ver `0x07`) — 0 releases · 1 Eco · 2 Sport. Held by answering each of the dash's `0x490` frames with the overridden value |
| `0x08` | `level u8` | **Regen override** (ver `0x07`) — 0 releases · 1–4 selects. Nothing can confirm it took, and the bike inhibits regen above ~90% SOC where it correctly does nothing |

Both builds accept all eight opcodes, so the app never has to withhold a write.

### Overriding the bike — what `0x07` and `0x08` actually do

Until ver `0x07` this puck could not transmit at all. It now does, and the two
rules that make that safe are worth stating outright.

**An override is a standing argument, not a command.** `0x490` is the dash's
mode/regen command to the motor controller, the controller obeys whichever copy
it heard *last*, and it never latches — so the dash's next frame, 200 ms later,
undoes anything sent once. The puck therefore answers **every** `0x490` the dash
sends, immediately, for as long as an override is held. Bench-measured
2026-08-10: replying on receipt held the controller on **297/297** of its own
frames, where a free-running 20 Hz injection lost 24%. Latency is the mechanism,
not rate.

**Every failure path releases.** BLE disconnect, CAN going stale, and the rider
pressing the handlebar button all drop the override, and the bike is back under
the dash's control within 100 ms of the last frame the puck sends. The
handlebar-button rule matters most: without it software could hold a mode the
rider is actively pressing to leave, and the only escape would be finding the
phone.

The puck reports what it is actually holding in the status packet's `ovrState`,
which is not an echo of the request — an override can end without the app doing
anything, so the app must render the report rather than its own intent.

**The bike's own screen will disagree.** The dash never listens to the bus; it
displays its own selection. While an override is held the display and the
machine genuinely differ, and no firmware change can fix that from this side.

### More than one app at a time

The puck accepts up to **3 simultaneous connections** (`CONFIG_BT_NIMBLE_MAX_CONNECTIONS`),
but until 2026-08-10 it served exactly one. A BLE peripheral stops advertising
the moment it accepts a connection, and while `advertiseOnDisconnect(true)`
brought it back when a device *left*, nothing ever restarted it on connect —
so a second app could never find the puck. `onConnect` now resumes advertising
while there is room.

The CCCD budget is 8. Tripper subscribes to 3 characteristics and the dashboard
app to 2 — which is exactly why the dashboard leaves the raw stream alone. Two
apps spend 5 of 8; a third full subscriber would reach the ceiling.

An override belongs to the **connection that requested it** and is released when
that connection drops, not when any client happens to disconnect. Otherwise one
app leaving would silently cancel another's override.

## IMU calibration

Both builds run the BNO055 in `OPERATION_MODE_IMUPLUS` — six-axis, no
magnetometer. That is the right choice on a motorcycle (a steel frame and a
motor make magnetometer calibration junk, and heading comes from GPS anyway),
but it has a consequence worth stating plainly:

> **In IMUPLUS the accelerometer is the only thing that knows where down is.**
> Every lean and slope number the puck produces is anchored to it. An
> uncalibrated accelerometer doesn't fail loudly — it biases attitude quietly,
> for the whole ride.

So the firmware now:

- calls `setExtCrystalUse(true)`. The Gravity board carries a 32.768 kHz
  crystal; without this the fusion runs on the internal RC oscillator, which
  Bosch does not consider adequate for the fusion modes
- reads `getCalibration()` at 1 Hz and puts it in the telemetry packet
- **refuses a mount zero while the IMU is uncalibrated** — the OLED shows
  `ACC n/3` instead of `ZEROED`, and the app disables its zero button and says
  why (a Light puck has no screen, so the app is the only place it can)
- saves the offsets to flash the first time calibration reaches 3/3 and restores
  them at every boot
- rejects any `getQuat()` that isn't a unit quaternion, counting the drops as
  `qrej` in the serial line — a climbing count means the I²C run is marginal

**Calibrating from cold:** leave the bike still and upright on level ground for
a few seconds. Accelerometer and gyro both settle without any waving about —
that dance is for the magnetometer, which this mode never uses. Watch `cal=` in
the serial debug line, or the app's *IMU calibration* row. It is a once-per-chip
chore, not once-per-ride, because the offsets persist.

> **`cal=` reads 0 for accel after every reboot, even on a calibrated chip.**
> The BNO055's `CALIB_STAT` reports the fusion's *live* confidence, not whether
> offsets are loaded — restoring them does not restore the status byte. So
> nothing gates on `cal` directly: the firmware tracks whether it restored
> offsets from flash, and publishes the verdict as **flags bit2**. That is what
> the app's zero button follows. Gating on `cal` alone would refuse a zero after
> every power-up, which on a Light puck (no buttons, no screen) would leave no
> way to zero at all.

**Reading the debug line** — `cal=321` is sys 3, gyro 2, accel 1. Only the last
two matter here; `sys` stays low in IMUPLUS and is not a fault. `gz=` is the raw
yaw rate: on a straight road it should sit near zero, and a steady non-zero
reading is gyro bias, which is what makes attitude wander.

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
| Ride mode (dash command) | `0x490` | byte 0 bits 5:3 | 1 Eco, 2 Sport |
| Odometer | `0x402` | 2–3 LE | 1 km |
| Throttle demand | `0x202` | 3–4 LE | units unconfirmed |

Speed and RPM hold a fixed 5.887 ratio at r = 0.999 across two independent
rides; integrating speed reproduces plausible ride distances. Cell high/low
averaged × 16 reproduces pack voltage to 0.02 V, confirming the 16S pack.

Traps worth knowing:

- **`0x103[0]` is not the battery percentage.** It reads a constant 100 while
  actual charge is elsewhere — almost certainly state of *health*.
- **`0x202[6:8]` is not an odometer.** It is monotonic, which is why it looks
  like one, but its rate has correlation −0.0005 with speed and its counts per
  km differ 34% between rides. **The odometer is `0x402[2:4]`**, whole km — see
  below.
- **The bus carries 15 IDs; this firmware kept 8.** "No odometer has been found
  on the bus" stood here for months and was false: it had been concluded from
  the eight frames the puck captures, and `0x402` was one of the seven nobody
  had ever decoded. Found 2026-08-10 by reading the odometer off the bike's own
  display (400 km) and looking for that number in a full-bus capture — `0x402
  [2:4]` read exactly 400. **Provisional** until a ride shows it incrementing:
  this bus carries round constants (1200 sits in both `0x101[4:6]` and
  `0x302[6:8]`), so one static match against a round number is suggestive, not
  proof. The lesson generalises — capture every ID, not the ones you decode,
  and when the vehicle already displays a number, use it as ground truth.
- **`0x490[0]` is not a temperature, and it is not one field.** Bits 5:3 carry
  the ride mode (r = 0.99, which is why the whole byte looked like mode alone)
  and bits 2:0 carry the regen level. Reading the low *nibble* as regen works
  only in Sport: in Eco bit 3 is set, so regen 1 reads `0x09` rather than
  `0x01`. Mask 3 bits, not 4.
- **`0x490` is a COMMAND from the dash, not an echo from the controller.** It
  was labelled an echo until 2026-08-10. Across five button presses `0x490[0]`
  moved *first* every time, with `0x202` following 20–27 ms later carrying both
  the new mode bits and the new demand floor (Sport 1100 / Eco 750). Injecting
  `0x490` with Eco, while the dash went on sending Sport at 5 Hz, held the
  controller at `demand=750` for the whole burst — so the controller obeys the
  bus, and `0x202` and `0x490` are different nodes. Consequences if you ever
  want to *control* one of these bikes: the controller follows the most recent
  `0x490` and never latches, so an override must out-transmit the dash
  continuously (free-running at 20 Hz still lost 24% of the time; replying the
  instant the dash speaks held 297/297); the dash never listens, so an
  overridden bike disagrees with its own screen; and regen can be commanded but
  **not verified from anywhere** — it moves nothing else on the bus, and regen
  current is unmeasurable because power and current are unsigned.
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

| Doc | What it covers |
|---|---|
| [`docs/flashing.md`](docs/flashing.md) | Flashing over USB, WiFi and from a phone; making a `.bin`; wireless diagnostics |
| [`docs/data-quality-audit.md`](docs/data-quality-audit.md) | What the recordings actually contain, and where they lie |
| [`docs/lean-investigation.md`](docs/lean-investigation.md) | Why the BNO055's fusion is not trusted for attitude |
| [`docs/direct-attitude-sensing.md`](docs/direct-attitude-sensing.md) | What a hardware answer to pitch/roll would cost |
| [`CLAUDE.md`](CLAUDE.md) | Handoff notes — conventions, verdicts, and what not to relitigate |

Replay tooling for `.trip` recordings is in [`tools/`](tools/): `triplib.py`
decodes the LZFSE container, `harness.py` re-runs an estimator over a recording
so a change can be scored against a ride instead of guessed at.
