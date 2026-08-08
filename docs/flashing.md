# Flashing a puck — USB and WiFi, complete reference

Verified working 2026-08-08 on the Light puck (macOS, arduino-cli, XIAO
ESP32S3). This is the single flashing document: networks, passwords, the
command lines for both paths, and the wireless diagnostics.

USB is needed exactly twice in a puck's life: the first flash of an
OTA-capable firmware, and recovery if an update ever bricks it. Everything
else is WiFi.

## Networks & passwords

The authoritative values are the `#define`s in the OTA block near the top of
each firmware file — if you change them there, update this table in the same
commit.

| | Light build (`tripper_light.ino`) | Full build (`tripper_puck.ino`) |
|---|---|---|
| WiFi network the puck raises (`OTA_AP_SSID`) | `Tripper-Light-OTA` | `Tripper-Puck-OTA` |
| WiFi password (`OTA_AP_PASS`) | `tripper-ota` | `tripper-ota` |
| Network-port name in Arduino IDE (`OTA_HOSTNAME`) | `tripper-light` | `tripper-puck` |
| Upload password (`OTA_PASSWORD`) | `tripper` | `tripper` |
| Puck's IP once joined | `192.168.4.1` | `192.168.4.1` |

Constraints when changing them: `OTA_AP_PASS` must be **at least 8
characters** (WPA2 rule — shorter and the AP silently fails to start), and
the SSIDs are deliberately different per build so two powered pucks never
raise the same network. These values are checked into the repo, so treat them
as labels, not secrets; the protection is that the mode is off unless toggled
on and never survives a power cycle.

## One-time setup

```bash
brew install arduino-cli
arduino-cli core install esp32:esp32
```

arduino-cli shares Arduino IDE's libraries folder (`~/Documents/Arduino`), so
the Adafruit/NimBLE libraries are already visible if the IDE ever built these
sketches. The first compile builds the whole ESP32 core and takes several
minutes **in silence** — it is not stuck; add `-v` to watch. Every compile
after that reuses the cache and takes seconds.

Both builds use the same FQBN, `esp32:esp32:XIAO_ESP32S3`, whose default
partition scheme already has the two app slots OTA needs. Paths below assume
the repo at `~/Documents/GitHub/Untitled`; swap `tripper_light` for
`tripper_puck` to flash a Full build.

## Flashing over USB

Find the port (the puck shows as "ESP32 Family Device" — generic USB
identity; the FQBN on the command line picks the real board):

```bash
arduino-cli board list
# -> /dev/cu.usbmodem101 (or similar)
```

Compile and upload in one step:

```bash
arduino-cli compile -u -p /dev/cu.usbmodem101 \
  --fqbn esp32:esp32:XIAO_ESP32S3 \
  ~/Documents/GitHub/Untitled/Tripper-DL1/hardware/firmware/tripper_light
```

Watch it boot:

```bash
arduino-cli monitor -p /dev/cu.usbmodem101 -c baudrate=115200
# Ctrl+C to exit
```

A healthy Light boot prints the LIGHT-build banner, `IMU ok | BMP280 ok`,
`CAN ok`, then a `[dbg]` line once per second. On v0x05 firmware, `gz` sits
within ~±0.2 deg/s at rest (window-averaged) and `baro` changes a little
every line (5 Hz readings).

## Flashing over WiFi

1. In the Tripper app: **Settings → Tripper Puck → WiFi flashing mode** on.
   The status line under the toggle confirms when the network is up.
2. On the Mac, join the puck's network from the WiFi menu (`Tripper-Light-OTA`,
   password `tripper-ota`). No internet while joined — irrelevant, nothing
   below needs it.
3. Compile, then upload to the puck's fixed address:

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 \
  ~/Documents/GitHub/Untitled/Tripper-DL1/hardware/firmware/tripper_light

arduino-cli upload -p 192.168.4.1 \
  --fqbn esp32:esp32:XIAO_ESP32S3 \
  --upload-field password=tripper \
  ~/Documents/GitHub/Untitled/Tripper-DL1/hardware/firmware/tripper_light
```

`password=` is `OTA_PASSWORD`, not the WiFi password. `--upload-field` needs
a recent arduino-cli (`brew upgrade arduino-cli` if it complains).

**What success looks like, in order:** upload progress completes → the puck
reboots into the new image → the OTA network *vanishes* (the mode never
survives a reboot, by design) → the Mac drops back to its normal WiFi → the
app's toggle snaps off by itself and the puck is back on BLE within seconds.

## Making a `.bin`

Needed for the phone route below, and for handing a build to someone who has
no toolchain. `--output-dir` is the only difference from a normal compile:

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 \
  --output-dir ~/Desktop/puck-bin \
  ~/Documents/GitHub/Untitled/Tripper-DL1/hardware/firmware/tripper_light
```

That directory then holds nine files and **only one of them is the firmware
you flash** (sizes from the 2026-08-08 Light build):

| File | ~size | What it is |
|---|---|---|
| **`tripper_light.ino.bin`** | **1.2 MB** | **The application image. This is the one.** |
| `tripper_light.ino.bootloader.bin` | 20 KB | Second-stage bootloader; written only on a first USB flash of a blank chip |
| `tripper_light.ino.partitions.bin` | 3 KB | Partition table; same — only on a blank chip |
| `tripper_light.ino.merged.bin` | 8 MB | All of the above glued into one whole-flash image, for factory programming |
| `*_flashed.bin` | — | Byte-identical copies arduino-cli leaves behind showing what it sent |
| `tripper_light.ino.elf` / `.map` | 19 MB each | Debug symbols. Not firmware. |

Two traps. `merged.bin` is the biggest file with the most reassuring name and
is **wrong** for OTA — it is a whole-flash image, not an app image. And an
app image starts with byte `0xE9` while `partitions.bin` starts `0xAA`, which
is the quickest way to tell you grabbed the wrong thing:

```bash
xxd -p -l1 ~/Desktop/puck-bin/tripper_light.ino.bin   # -> e9
```

Rule of thumb: the file ending in exactly `.ino.bin`, about 1.2 MB. Anything
8 MB is the wrong file. For reference the XIAO's default layout gives roughly
3.3 MB per app slot, so 1.2 MB leaves plenty of room.

## Flashing from a `.bin` — phone only, no laptop at the bike

The puck is bolted to the bike; this route needs nothing but the phone. It
works because the puck serves an ordinary HTML upload form at `/update` while
flashing mode is on — no arduino-cli, no espota, just a browser. (ArduinoOTA
speaks espota, which only the Arduino tooling implements, which is why the
laptop used to be mandatory.)

The last build is also checked in at **`releases/tripper_light.ino.bin`** in
the repo root, so you can grab it from GitHub on the phone directly and skip
the compile entirely. Rebuild it with the command above whenever the firmware
changes, and commit the new one in the same commit as the source.

AirDrop `tripper_light.ino.bin` to the phone once — it lands in Files — then:

1. Tripper app: **Settings → Tripper Puck → WiFi flashing mode** on.
2. Phone WiFi: join `Tripper-Light-OTA`, password `tripper-ota`.
3. Safari → `http://192.168.4.1/update`
4. **Choose File** → pick the `.bin` → **Upload**.

A percentage appears while it runs. Do not lock the phone, switch apps, or
reload the page mid-upload — a browser retry part-way through a write is how
a puck gets bricked. If the transfer does die, the firmware aborts the write
so the slot is clean; just start again. When it finishes the puck reboots
itself, the OTA network vanishes, and the app's toggle snaps off on its own.

The same endpoint works from the Mac when arduino-cli is not to hand:

```bash
curl -F "fw=@$HOME/Desktop/puck-bin/tripper_light.ino.bin" \
  http://192.168.4.1/update
```

Recovery is unchanged: USB always works and needs nothing from the running
firmware.

## Diagnostics over the same network

While the flashing network is up, the puck also serves what used to require
the USB cable:

```bash
curl http://192.168.4.1/        # one-page live status: build, uptime, reset
                                # reason, heap, cal, quatRejects, baro, CAN
curl http://192.168.4.1/log     # the last ~8 KB of debug lines, oldest first
curl http://192.168.4.1/update  # the upload form (open this one in a browser)
```

(Or open the same URLs in a browser.) The log ring lives in RAM **from
boot**, not from when the toggle went on — after something odd on the bench,
toggle WiFi flashing on and `/log` still holds the `[dbg]` lines from when it
happened. It opens with a `[boot]` line whose reset reason tells a power
cycle from a crash from an OTA reboot. The ring does not survive a power
cycle, and ride data is not here at all — rides live in the phone's `.trip`
files; the puck is deliberately stateless.

## Notes

- WiFi flashing mode is entered ONLY via the app's toggle (BLE control write
  `0x05` — the Light build has no buttons). Firmware never raises the radio
  on its own.
- The status packet's `otaState` byte carries the live state:
  `0` off · `1` network up, no client · `1+n` = n clients joined.

## If something goes wrong

- **Upload rejected / auth failed** — wrong `password=`; the *running*
  firmware does the checking, so use its `OTA_PASSWORD`.
- **No network appears after toggling** — the app's status line says what the
  puck reported; "no report" means firmware older than the feature (USB flash
  once). Also confirm the app shows Connected: the toggle is a BLE write.
- **Upload succeeds but the puck misbehaves** — flash back over USB; that
  path needs nothing from the firmware to work.
- **`arduino-cli` seems hung on compile** — first build of a core version;
  give it minutes, or re-run with `-v`.
- **`Can't open sketch: no such file or directory`** — the sketch path is a
  *folder* containing an `.ino` of the same name, and it is relative to where
  you are standing. From inside `tripper_light/`, the argument is `.`; from
  `hardware/firmware/`, it is `tripper_light`. The absolute path always works.
- **iOS says "no internet" and drops the OTA network** — the firmware answers
  Apple's captive-portal probe to prevent exactly this, so seeing it means
  firmware older than the `/update` feature. Tap "Use Without Internet" to
  finish this flash; the next build fixes it permanently.
- **Phone upload fails part-way** — nothing is half-written; the firmware
  aborts the slot on a broken transfer. Rejoin the network and start again.
  Do not reload the page while a percentage is still climbing.
- **Uploaded and the puck went quiet** — you probably sent `merged.bin`
  instead of `.ino.bin`. USB recovery, then re-read the table above.
