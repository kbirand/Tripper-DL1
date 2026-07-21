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
- **Mount-zero calibration** — hold a button 10 s and the current orientation
  becomes 0/0/0 (reference quaternion, persisted to flash, survives reboots,
  applied to both display and telemetry)

## Hardware

| Part | Role |
|---|---|
| Seeed XIAO ESP32-S3 | MCU, BLE 5.0, USB-C power |
| DFRobot Gravity 10DOF (BNO055 + BMP280) | On-chip sensor fusion + barometer, I²C |
| u-blox NEO-M8 GPS (GY-GPSU3 carrier) | 5 Hz position/speed/time, UART @ 115200 |
| SSD1306 0.91" OLED 128×32 | Clock / live-data screens, I²C `0x3C` |
| WS2812 NeoPixel ×1 | Status LED |
| 12 mm button | Marker (glove-friendly) |
| HW-483 button | Screen hold / attitude zero |

### Pin map (XIAO ESP32-S3)

| Pin | Function |
|---|---|
| D0 | NeoPixel DIN |
| D1 | Marker button → GND (internal pull-up) |
| D2 | Hold/zero button → GND (internal pull-up) |
| D4 / D5 | I²C SDA / SCL — BNO055 `0x28`, BMP280 `0x76`, OLED `0x3C` @ 100 kHz |
| D6 / D7 | UART TX→GPS RX / RX←GPS TX @ 115200 |
| D3, D8–D10 | free |

The I²C bus **must run at 100 kHz** — the BNO055's clock-stretching upsets
ESP32 I²C at higher speeds (validated: 59,722 reads / 0 errors / 10 min).
The firmware bursts OLED frames at 400 kHz between sensor transactions.

## Firmware

Arduino sketches in [`hardware/firmware/`](hardware/firmware/):

| Sketch | Purpose |
|---|---|
| [`tripper_puck`](hardware/firmware/tripper_puck/) | **Production firmware** — sensors → BLE + OLED |
| [`i2c_gate`](hardware/firmware/i2c_gate/) | BNO055 clock-stretch stress test (the go/no-go gate) |
| [`i2c_diag`](hardware/firmware/i2c_diag/) | Wiring diagnostic — line states + normal/swapped bus scans |
| [`gps_config_5hz`](hardware/firmware/gps_config_5hz/) | One-time GPS config: 5 Hz + 115200, saved to BBR (production firmware re-applies at boot) |

### Build & flash

```sh
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit BNO055" "Adafruit BMP280 Library" \
  "Adafruit SSD1306" "Adafruit NeoPixel" "TinyGPSPlus" "NimBLE-Arduino"
cd hardware/firmware/tripper_puck
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 .
arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32S3 -p /dev/cu.usbmodem* .
```

### Controls & LED

| Input | Action |
|---|---|
| Marker button (D1) click | Marker counter++ in telemetry · blue flash · MARK splash |
| Hold button (D2) click | Pin / unpin the current OLED screen (border = pinned) |
| Hold button (D2) 10 s hold | Zero roll/pitch/yaw at current orientation (progress bar → ZEROED) |

| LED | Meaning |
|---|---|
| Amber blink | No GPS fix |
| Green solid | Fix, waiting for phone |
| Red slow pulse | Connected & streaming |
| Blue flash | Marker · Green flash: zeroed |

OLED alternates every 5 s between a GPS clock screen (UTC+3) and a data
screen (speed hero, roll/pitch, satellites, g, link state).

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

| Byte | Action |
|---|---|
| `0x01` | Marker ack — blue flash + MARK splash |
| `0x03` | Identify — LED rainbow + OLED invert, 2 s |

## Docs

The full build story — shopping list (Turkey), decision records, validation
results, pitfalls — lives in [`hardware/build-guide.html`](hardware/build-guide.html).
