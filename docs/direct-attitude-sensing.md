# Measuring lean and slope directly, instead of deriving them

**Status:** scoping note. Nothing here is built, and no code was changed to
write it. It exists to answer one question properly — *is there a part we can
buy or a method we can integrate that measures lean and slope, rather than
inferring them?* — because the honest answer is "yes for one of them, and yes
for the other but not the way you'd expect", and that deserves more than a
conversation.

Read [`lean-investigation.md`](lean-investigation.md) first. This picks up where
that leaves off: that document establishes the BNO055's fusion is wrong, this
one asks what a *correct* instrument would look like.

---

## The short version

| | Can it be measured directly? | With what | Realistic accuracy |
|---|---|---|---|
| **Slope** | Yes, today, cheaply | Barometer over ground distance | ~1.7° |
| **Slope** | Yes, much better | RTK GNSS | ~0.1° over 10 m |
| **Lean** | **Not by any single sensor** | — | — |
| **Lean** | Yes, with velocity aiding | GNSS velocity + gyro | ~1–2° |
| **Lean** | Yes, bought outright | GNSS/INS module | ~0.1–0.5° |

The interesting entry is the third row, and it is not a budget problem.

---

## Why no accelerometer reads lean, at any price

An accelerometer does not measure gravity. It measures **specific force** —
gravity and the vehicle's own acceleration, summed into a single vector with no
way to tell which part is which.

In a steady corner those two components add up along the bike's own vertical
axis. That is not a coincidence or an artefact; it is the definition of a
balanced corner. If the resultant force were *not* through the bike's centre
line, the bike would be falling over.

So an accelerometer strapped to a motorcycle reads "upright" at 45° of lean,
and it reads that correctly. The app's own estimator records the measurement:
lateral g averages **0.015** across these rides while moving.

This is why the answer to "can we buy a better sensor" is no. A navigation-grade
accelerometer costing more than the bike has the identical blind spot, because
the blind spot is in the physics of the quantity being measured, not in the
quality of the measurement.

It is also why the current firmware runs the BNO055 in IMUPLUS: no amount of
sensor fusion over accelerometer and gyro alone can recover what neither sensor
observes.

---

## The thing that breaks the tie

To separate gravity from acceleration you need **an independent measurement of
the vehicle's own acceleration**. Then:

```
gravity = specific force − acceleration
```

and roll falls straight out of where gravity points relative to the bike.

The standard source for that independent measurement is **GNSS velocity**. This
is the architecture behind every serious attitude system, from agricultural
autosteer to the aircraft the question invoked. The aircraft's advantage is not
that its gyros are better — though they are. It is that satellite navigation is
continuously telling its inertial system what it is really doing, so the
inertial system never has to guess whether a force is gravity or motion.

### Why velocity and not position

A GNSS receiver derives velocity from the **Doppler shift of the carrier**, not
by differencing positions. These are separate measurements with very different
error characteristics: position may be good to metres while velocity is good to
a few centimetres per second. A receiver with a mediocre position fix can still
have excellent velocity.

Differentiating that velocity gives acceleration in earth coordinates, accurate
enough to subtract from the accelerometer and leave gravity behind. u-blox
receivers report a live accuracy estimate (`sAcc`) alongside, so the filter can
weight it honestly rather than trusting it blindly.

---

## What is already on the bench

This is the part that changes the calculus, and it was a surprise.

The Full build carries a **u-blox NEO-8M**, already running at 5 Hz and 115200
baud. And the firmware already speaks UBX in both directions:

- [`sendUBX()`](../hardware/firmware/tripper_puck/tripper_puck.ino) builds UBX
  frames with correct Fletcher checksums — used today for `CFG-PRT`,
  `CFG-RATE` and `CFG-CFG`.
- [`waitAck()`](../hardware/firmware/tripper_puck/tripper_puck.ino) is already a
  byte-level UBX receive state machine. It only recognises `ACK-ACK` today, but
  it is the same skeleton any UBX message parser needs: sync bytes, class, id,
  length, payload, checksum.

What is missing is narrow: **the receiver is configured over UBX but read over
NMEA.** `TinyGPSPlus` parses NMEA sentences, and NMEA carries 2D speed and
course — the 3D velocity vector never reaches the firmware, because the
sentence format has nowhere to put it.

`UBX-NAV-PVT` carries `velN`, `velE`, `velD` in mm/s plus `sAcc`, in one
message, at whatever rate the receiver is configured for.

**So on the Full build, the hardware for velocity-aided attitude is already
fitted and already talking the right protocol.** The gap is a message parser and
a filter, not a bill of materials.

The Light build has no receiver on the puck at all — position comes from the
phone at 1 Hz, already smoothed by CoreLocation. That is far too slow and too
filtered to observe corner dynamics, so velocity aiding would be a Full-build
capability unless the Light puck gained a receiver of its own.

---

## The routes, by effort

### 1. Velocity-aided attitude on the hardware we already have

Configure `UBX-NAV-PVT`, parse it, and run an error-state Kalman filter: the
gyro propagates attitude between fixes, and the GNSS velocity derivative
corrects the accumulated error and pins down which way gravity actually points.

- **Parts cost:** nothing. It is already on the bike.
- **Accuracy:** roughly 1–2° of roll, including the components the kinematic
  formula structurally cannot see.
- **Effort:** substantial, and this is the honest catch. A correct 15-state
  error-state filter with proper lever-arm handling and covariance tuning is
  not a weekend, and a badly tuned one is worse than the formula it replaces
  because it fails in ways that look plausible.
- **Rate:** the NEO-8M will do 10 Hz on a single constellation. At 5 Hz across
  multiple constellations the corner dynamics are marginal but usable.

### 2. Buy a module that has already done the work

GNSS/INS units — SBG Ellipse, VectorNav VN-200, Inertial Labs, Advanced
Navigation Spatial — contain the IMU, the receiver and the filter, and output
roll, pitch and heading already fused over serial.

- **Accuracy:** typically quoted 0.1–0.5° roll/pitch.
- **Effort:** read a serial protocol. No filter to write or tune.
- **Cost:** roughly $1k–4k depending on grade — worth checking current pricing,
  this moves.
- **Cost in other ways:** physically larger than the whole current puck, more
  power, its own antenna siting requirements.

This is what professional data loggers use, and it is the honest answer to
"can I just buy the thing".

### 3. RTK, for slope specifically

A ZED-F9P-class receiver with a correction stream gives centimetre-level
altitude. Over the 10 m window the app now fits slope across, that is roughly
**0.1°** — against the barometer's ~1.7°.

- Turkey has a national CORS network (**TUSAGA-Aktif**) that serves corrections
  over NTRIP; the phone already carries the mobile data to fetch them.
- It would improve the *distance* axis too — the quantity that turned out to be
  wrong when the wheel was measuring it.
- Needs sky view and a decent antenna, and corrections mean a live data link.

### 4. Dual-antenna GNSS

Two antennas on a fixed baseline give true heading and pitch from the carrier
phase difference, with no inertial sensor involved at all.

For **roll** you need lateral separation, and a motorcycle offers maybe 50–60 cm.
At ~1 cm baseline accuracy that is about 1° — real, but marginal, and it means
two rigidly mounted antennas that must not move relative to each other on a
vehicle designed to flex. Mechanically the least attractive option here.

---

## What will not work, and why

**A magnetometer.** In theory this is the clean answer: Earth's magnetic field
is a second reference direction, independent of gravity, and two non-parallel
reference vectors fully determine attitude. That is what 9-axis chips are for.

On this bike it is hopeless. The climb analysed in the trip data drew a measured
**105 A peak** through the motor cabling. The field from currents at that scale
swamps Earth's field entirely, and it varies with throttle — so the error would
be largest exactly when on the gas, which is exactly when the reading matters.
No calibration fixes a disturbance that changes with the throttle. The BNO055
already runs in IMUPLUS with the magnetometer disabled for this reason.

**A better accelerometer.** Covered above: the blind spot is in the quantity,
not the instrument.

**A better gyro.** The gyro's weakness is drift, not blindness, and drift is
already handled by referencing the kinematic term through a complementary
filter. A better gyro lengthens the interval over which integration can be
trusted; it does not tell you where gravity is.

---

## What each option actually buys

| | Lean accuracy | Sees rider lean-off | Sees camber | Slope | Parts |
|---|---|---|---|---|---|
| **Today** (gyro + wheel speed + baro) | good through corners | no | no | ~1.7° | fitted |
| **Velocity-aided** (existing NEO-8M) | ~1–2° | **yes** | **yes** | ~1.7° | fitted |
| **GNSS/INS module** | ~0.1–0.5° | yes | yes | ~1.7° | $1k–4k |
| **+ RTK** | as above | — | — | **~0.1°** | ~$300+ |

The honest note on the first row: `tan(lean) = v·ψ̇/g` is not a poor substitute
for a real measurement — it is what most professional motorcycle telemetry
quotes, including the lean angle behind cornering ABS. Bosch's system is a
6-axis IMU plus a vehicle model plus wheel speeds. Same physics, more modelling.

What the formula structurally cannot see is **the rider moving on the bike** and
**road camber**, because it describes the bike's balance rather than its
geometry. That gap is what velocity aiding closes, and it is the only thing it
closes.

---

## Recommendation for this bike

The riding this puck is built for is **technical climbing at low speed**, and
that reshapes the priorities:

- **Lean matters least there.** At 3 km/h in a rut there is barely a turn rate
  to derive lean from, and not much lean to measure. The kinematic term is
  weakest exactly where the riding is slowest — but so is the demand for it.
- **Slope is where the money pays.** Grade is the number that describes a
  technical climb, and it is currently good to ~1.7° with scatter from a
  barometer. RTK would take it to a tenth of a degree on every pitch, and would
  sharpen the ground-distance measurement at the same time.

So if one upgrade is bought, buy the one for slope. Velocity-aided lean is the
intellectually correct fix and the one that closes a real gap in what the
current method can observe — but for this riding it buys the least.

---

## If it were pursued, what else would move

Noted here so the scoping is honest, not as a plan:

- **The telemetry packet has room.** It is 78 bytes today against a negotiated
  ATT MTU of 247, so a 3D velocity vector and its accuracy estimate fit without
  restructuring. The `ver` byte already gates parsing, and the app's
  `>=`-based parse gates mean an older app reads a newer packet and drops the
  tail silently — which Settings now warns about.
- **`speed_cmps` and `course_cdeg` become redundant.** Both are 2D projections
  of the velocity vector that would replace them.
- **The Light/Full split widens.** Today the two builds differ in what they
  report but not in how good it is. Velocity aiding would make the Full build
  measurably more accurate rather than merely better equipped, which is a
  product decision as much as a technical one.
- **Nothing here helps the recorded rides.** As with the slope fix, only rides
  recorded afterwards would carry it.
