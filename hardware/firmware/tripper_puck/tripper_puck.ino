// ============================================================
// Tripper Puck — production firmware, e-bike / BLE edition
// ============================================================
// Stateless BLE sensor: boots with the bike, streams telemetry to the
// Tripper app. No SD, no battery management — the phone is the recorder.
//
//   100 Hz  BNO055 quaternion + linear accel (IMUPLUS), latch interval max-g,
//           gyro/raw-accel window sums (the packet sends the 200 ms mean)
//   5 Hz    GPS epochs (module pre-configured; re-configured at every boot)
//   5 Hz    BLE telemetry notify (86-byte packed sample) + BMP280 baro sample
//   1 Hz    BLE status notify, baro temperature, serial debug line
//   10 Hz   BLE raw notify — the 100 Hz IMU samples themselves, un-averaged,
//           batched ten at a time with the puck's own timestamp (see RawBatch)
//   2 Hz    OLED refresh — clock / live data / bike CAN (/ trip time)
//   ~92/s   Talaria CAN frames, listen-only (SN65HVD230 on D8/D9)
//
// Status lives on the OLED (fix dot, link state, MARK/ZEROED splashes) —
// no separate status LED.
// Button 1 (D1→GND): click = step to the next screen · 3 s hold = toggle
// auto-cycling (thin border = cycling). Screens do NOT rotate on their own by
// default — on a moving bike the screen you picked should stay put.
// Button 2 (D2→GND): click = marker, bumps a counter in the telemetry packet ·
// 10 s hold = zero the mount at the current orientation — reference quaternion
// saved to flash, survives reboots, and the BLE telemetry quaternion is
// re-referenced the same way. Refused until the BNO055 reports accelerometer
// calibration 3/3: in IMUPLUS the accelerometer is the only thing that knows
// where down is, so a zero taken against an uncalibrated one is silently wrong
// for the whole ride.
// Control writes: 0x01 = marker ack flash · 0x02 = zero the mount (same as the
// button 2 long-hold, and refused the same way) · 0x03 = identify (LED
// rainbow + OLED invert, for picking the right device in a scanner app) ·
// 0x06 + [on u8] = raw 100 Hz stream on/off (boots on; the phone decides
// whether to keep the bytes).
// 0x04 + [active u8][elapsed s u32 LE] = ride state — while active the OLED
// runs inverted and a third screen (trip time) joins the cycle. The app
// re-sends it on every reconnect, so a link flap or puck reboot mid-ride
// self-heals. · 0x05 + [on u8] = WiFi flashing mode (SoftAP + ArduinoOTA,
// see the OTA block below).

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <driver/twai.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Update.h>
#include <esp_system.h>

// ---------- OTA (flashing over WiFi) ----------
// The puck lives inside an enclosure on the bike, so every USB flash means
// dismounting it. Instead the puck raises its OWN network on demand: the app's
// Settings toggle (control write 0x05) starts a SoftAP; the computer joins it,
// and the puck appears as a NETWORK PORT in Arduino IDE (Tools -> Port ->
// "tripper-puck at 192.168.4.1"). Upload works from the desk with the puck
// still on the bike, anywhere — no home-network credentials in the firmware,
// nothing to configure per location.
//
// The mode is deliberately NOT persistent: a bike power cycle always boots
// into normal riding with the WiFi radio off, so a toggle forgotten overnight
// can't leave an open AP riding around. The status packet reports the live
// state (see StatusPacket::otaState) so the app's toggle shows the truth.
// Needs a partition scheme with two app slots — the XIAO ESP32S3 default
// (8MB with OTA) is one. USB remains the recovery path if an update bricks.
// Keep in step with tripper_light.ino ("Tripper-Light-OTA" / "tripper-light").
#define OTA_AP_SSID     "Tripper-Puck-OTA"
#define OTA_AP_PASS     "tripper-ota"   // WPA2 needs >= 8 chars
#define OTA_HOSTNAME    "tripper-puck"  // the network-port name in the IDE
#define OTA_PASSWORD    "tripper"       // IDE prompts for this before upload

bool otaActive = false;                 // SoftAP up, ArduinoOTA polled

// While the AP is up, the puck also serves diagnostics over HTTP — the same
// lines that go to USB serial, without the cable:
//   http://192.168.4.1/     one-page live status
//   http://192.168.4.1/log  the last ~8 KB of debug lines, oldest first
// The ring lives in RAM from boot, so lines logged long BEFORE the toggle
// are still there when the network comes up — turn it on after something
// odd and read what the puck said at the time. Lost on power cycle.
// Keep in step with tripper_light.ino.
WebServer httpd(80);
static char logRing[8192];              // zero-initialised; NULs are skipped
static size_t logHead = 0;

// printf-alike that mirrors every line to USB serial and the ring. The 1 Hz
// [dbg] line goes through here; one-off boot/event lines can too.
void logLine(const char *fmt, ...) {
  char line[320];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(line) - 1) n = sizeof(line) - 1;
  line[n++] = '\n';
  Serial.write((const uint8_t *)line, n);
  for (int i = 0; i < n; i++) {
    logRing[logHead] = line[i];
    logHead = (logHead + 1) % sizeof(logRing);
  }
}

// ---------- pins & constants ----------
// I2C on the XIAO's default pads. Stated explicitly rather than relying on a
// bare Wire.begin() because this project has already been bitten once: the
// first board's D4 pad was clamped low — it read 0 even against the internal
// pull-up with nothing attached — so SDA could never idle high and all four
// sensors looked dead. That board was replaced; if the symptom ever returns,
// bench_imu_can prints both line states at boot and i2c_diag localises it.
#define PIN_SDA      D4
#define PIN_SCL      D5
#define PIN_BUTTON   D1
#define PIN_BUTTON2  D2        // screen hold / attitude zero
// The OLED lives in the handlebar enclosure, a ~1 m cable away from the ESP32,
// so it gets its own I2C bus. Sharing one bus would drag the BNO055 — whose
// clock-stretching already forces the whole bus to 100 kHz — onto that long,
// capacitive, noisy run. Split, the worst a bad cable can do is glitch the
// screen; the sensor bus stays 15 cm long inside the rear enclosure.
#define PIN_OLED_SDA D3
#define PIN_OLED_SCL D10
#define OLED_BUS_HZ    100000  // idle speed on the long handlebar run
// Frames burst faster than the idle speed since nothing else shares Wire1.
// Drop this to OLED_BUS_HZ if a long or unshielded cable tears the display.
#define OLED_BURST_HZ  400000
#define CAN_TX_GPIO  GPIO_NUM_7   // D8 -> SN65HVD230 CTX
#define CAN_RX_GPIO  GPIO_NUM_8   // D9 <- SN65HVD230 CRX
#define CAN_STALE_MS 2000UL       // no frame for this long = bus considered dead
#define TZ_HOURS     3         // display offset for the clock screen (TRT, no DST)
#define CLICK_MAX_MS 600       // release under this = a click, not a hold
#define ZERO_SHOW_MS 800       // held past this = show the zero progress bar
#define ZERO_HOLD_MS 10000UL   // button 2 held to here = capture the mount reference
#define CYCLE_HOLD_MS 3000UL   // button 1 held to here = toggle auto-cycling
#define SCREEN_MS    5000UL    // dwell per screen while auto-cycling

// Capability bits, reported in StatusPacket::caps, so the app can tell which
// build it is talking to — a Light puck has no GPS and the phone must supply
// position. Zero means firmware older than the field, not "no capabilities",
// which is why CAP_CAN is a bit rather than assumed.
#define CAP_GPS      0x01
#define CAP_OLED     0x02
#define CAP_BUTTONS  0x04
#define CAP_CAN      0x08
#define BUILD_CAPS   (CAP_GPS | CAP_OLED | CAP_BUTTONS | CAP_CAN)

static const char *SVC_UUID  = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70001";
static const char *TELE_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70002";
static const char *STAT_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70003";
static const char *CTRL_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70004";
// Raw 100 Hz flight recorder. Additive: a phone that never subscribes sees the
// puck behave exactly as before.
static const char *RAW_UUID  = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70005";

// ---------- packets (little-endian, packed) ----------
struct __attribute__((packed)) TelemetryPacket {
  uint8_t  ver;          // 0x05 — byte-identical layout to 0x04; the bump marks
                         // a semantics change: gyr*/acc* below are 200 ms
                         // window MEANS, not the newest 100 Hz sample
  uint8_t  flags;        // bit0 fix valid · bit1 time valid · bit2 IMU
                         // calibration usable (live 3/3 or restored from flash)
  uint32_t gpsTimeMs;    // UTC ms of day, 0xFFFFFFFF if invalid
  int32_t  lat_e7;       // deg * 1e7
  int32_t  lon_e7;
  int32_t  alt_cm;       // GPS altitude
  int32_t  baroAlt_cm;   // BMP280, std-atmosphere reference
  uint32_t press_pa;
  uint16_t speed_cmps;
  uint16_t course_cdeg;  // deg * 100
  uint8_t  sats;
  uint16_t hdop_c;       // hdop * 100
  int16_t  qw, qx, qy, qz;              // quat * 16384
  int16_t  linx_mg, liny_mg, linz_mg;   // linear accel, mg
  int16_t  maxG_mg;      // interval max |lin|, reset each packet
  uint8_t  marker;       // increments on each button press
  // ---- bike CAN block (ver 0x02). Zeroed and canFlags bit0 clear when the
  // bus is absent or stale, so the app must gate all of it on that bit.
  uint8_t  canFlags;     // bit0 live · bit1 kickstand down · bits3:2 ride mode
                         // bits6:4 regen level 1..4 (0 = unknown/not live)
  uint16_t canSpeed_dkph;   // 0.1 km/h  (0x303[0:2])
  uint16_t canRpm;          // rpm       (0x203[0:2])
  uint16_t canPower_w;      // W         (0x203[2:4])
  uint16_t canCurrent_da;   // 0.1 A     (0x302[4:6])
  uint16_t canPack_dv;      // 0.1 V     (0x101[0:2])
  uint8_t  canSoc_pct;      // %         (0x401[0])
  uint16_t canDemand;       // throttle demand, units unconfirmed (0x202[3:5])
  uint16_t cellHi_mv;       // mV        (0x201[0:2])
  uint8_t  cellHi_idx;      // 1..16     (0x201[4])
  uint16_t cellLo_mv;       // mV        (0x201[2:4])
  uint8_t  cellLo_idx;      // 1..16     (0x201[5])
  // ---- IMU health (ver 0x03). Appended after the CAN block on purpose, so
  // every 0x02 offset above stays byte-identical and an older app keeps
  // parsing. Raw gyro and calibration used to stay on the puck, which is why
  // an attitude fault took a GPS cross-check to diagnose rather than a look at
  // the recording. Keep in step with tripper_light.ino.
  //
  // Since ver 0x05 these carry the MEAN over the 200 ms packet window (about
  // 20 samples at 100 Hz), not the single newest sample. The app integrates
  // this field for lean, and a mean is exactly the integral over the window
  // divided by its length — the instantaneous sample it replaced threw away
  // 19 of every 20 measurements and aliased engine vibration into lean.
  int16_t  gyrx_d16, gyry_d16, gyrz_d16;   // sensor-frame gyro, deg/s * 16
  uint8_t  calib;        // bits 7:6 sys · 5:4 gyro · 3:2 accel · 1:0 mag
  // Increments on every mount zero the puck actually ACCEPTS, so the app can
  // tell "captured" from "the write never landed" or "refused". Wraps at 255
  // and restarts at 0 on reboot; the app watches for *any* change against a
  // baseline it takes when it sends, so neither matters.
  uint8_t  zeroCount;
  // ---- raw accelerometer (ver 0x04). Everything above that bears on attitude
  // is a *product* of the BNO055's fusion — the quaternion, the linear accel,
  // even the calibration byte — so when the fusion itself is the suspect, the
  // packet holds nothing independent to convict it with. A ride that read -10°
  // of lean down a dead-straight road ruled out the mount, the axes, the zero,
  // gyro drift, the bike's own acceleration and vibration, and then ran out of
  // evidence, because every remaining witness was the accused.
  //
  // This is the pre-fusion measurement: gravity plus motion, straight off the
  // accelerometer. Subtract lin*_mg and what remains is the fusion's own idea
  // of down — which is what the whole attitude rests on. Keep in step with
  // tripper_light.ino.
  // Since ver 0x05: the 200 ms window mean, same reasoning as the gyro above.
  // The app low-passes this field anyway, so averaging on the puck only moves
  // the first (and heaviest) stage of that filter to where all the samples are.
  int16_t  accx_mg, accy_mg, accz_mg;   // sensor-frame accelerometer, mg
  // Non-unit getQuat() results dropped since boot, saturating. Printed to USB
  // since the first build and never recorded; a count that climbs mid-ride
  // means the attitude is being *held* across glitched I2C reads, not tracking.
  uint16_t quatRejects;
};
static_assert(sizeof(TelemetryPacket) == 86, "telemetry packet size drifted");

struct __attribute__((packed)) StatusPacket {
  uint8_t  ver;          // 0x01
  uint8_t  fix;
  uint8_t  sats;
  uint8_t  battPct;      // 0xFF = external supply (BEC in production, USB while flashing)
  uint16_t hdop_c;
  uint32_t uptime_s;
  int16_t  temp_x10;     // BMP280 °C * 10
  uint8_t  marker;
  uint8_t  caps;         // was `reserved` (always 0) — now the capability bits
  // WiFi flashing state, appended like the telemetry tail (ver stays 0x01,
  // the app length-gates): 0 = off · 1 = AP up, no client · 1+n = n clients
  // joined. The app's toggle is only a request; this is the confirmation.
  uint8_t  otaState;
};
static_assert(sizeof(StatusPacket) == 15, "status packet size drifted");

// ---------- raw flight recorder ----------
// The telemetry packet averages twenty 100 Hz samples into one number. That is
// right for a live display and wrong for everything else: fork and shock
// resonance sits at 2-5 Hz and wheel hop at 10-20 Hz, so at a 5 Hz output rate
// they alias into the road-grade band and can never be separated again. A
// boxcar mean is a poor anti-alias filter anyway — first sidelobe only 13 dB
// down. This characteristic ships the samples themselves so the filter can be
// designed offline, once, against real road data instead of guesswork.
//
// TIME IS THE PART THAT IS EASY TO GET WRONG. BLE delivers in bursts, so the
// phone's receive time is not when the sample was taken; at 5 Hz nobody
// noticed, at 100 Hz it would wreck every derivative computed downstream. Each
// batch therefore carries the PUCK's own millisecond counter for its first
// sample and the nominal period, and the true timeline is reconstructed from
// those. The phone's clock anchors the start of the ride and nothing else.
struct __attribute__((packed)) RawSample {
  int16_t gx, gy, gz;    // deg/s * 100  (BNO055 resolves 1/16 deg/s)
  int16_t ax, ay, az;    // milli-g      (BNO055 resolves ~1 mg)
};
static_assert(sizeof(RawSample) == 12, "raw sample size drifted");

struct __attribute__((packed)) RawBatch {
  uint8_t  ver;          // 0x01
  uint8_t  count;        // RawSamples following this header
  uint16_t period_us100; // nominal sample period, units of 100 us (100 = 10 ms)
  uint32_t t0_ms;        // puck millis at the FIRST sample in this batch
  // The chip's own fusion, UNZEROED and unprocessed. Convicted on 2026-08-08 of
  // inventing tilt its own raw accelerometer contradicts, and kept here anyway:
  // a verdict nobody can re-test is an opinion. Logging it costs 8 bytes.
  int16_t  qw, qx, qy, qz;
  int32_t  press_pa;     // raw pressure, NOT altitude — the conversion is lossy
  int16_t  temp_x10;
  uint8_t  calSys, calGyr, calAcc, calMag;
  uint8_t  pad[2];
};
static_assert(sizeof(RawBatch) == 28, "raw batch header size drifted");

// 10 samples per batch = 148 B on the wire at 10 batches/s. iOS negotiates a
// 185 B ATT MTU (182 B of payload), so one batch is one notification with room
// to spare, and 1.5 kB/s is a fraction of what the link carries.
#define RAW_BATCH_N 10
static RawSample rawBuf[RAW_BATCH_N];
static uint8_t   rawN = 0;
static uint32_t  rawT0 = 0;
static bool      rawEnabled = true;   // opcode 0x06 turns it off; boots on
static uint32_t  rawBatches = 0;

// ---------- devices ----------
Adafruit_BNO055   bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP280   bmp(&Wire);
Adafruit_SSD1306  oled(128, 32, &Wire1, -1);   // handlebar bus, see PIN_OLED_SDA
TinyGPSPlus       gps;
TinyGPSCustom     gsvGP(gps, "GPGSV", 3), gsvGL(gps, "GLGSV", 3), gsvGB(gps, "GBGSV", 3);

NimBLECharacteristic *chTele = nullptr, *chStat = nullptr, *chRaw = nullptr;
NimBLEServer *bleServer = nullptr;

bool imuOk = false, bmpOk = false, oledOk = false, canOk = false;

// ---------- bike CAN (Talaria, 250 kbit/s, listen-only) ----------
// Latest payload of each decoded message. Signal offsets and the evidence
// behind every one of them live in tools/talaria.dbc.
struct CanState {
  uint8_t  f101[8], f201[8], f202[8], f203[8], f302[8], f303[8], f401[8], f490[8];
  uint32_t lastRxMs = 0;
  uint32_t frames = 0;
} canS;

static inline uint16_t canU16(const uint8_t *d, int off) {
  return (uint16_t)d[off] | ((uint16_t)d[off + 1] << 8);
}

// ---------- live state ----------
volatile uint8_t markerCount = 0;
float maxG_g = 0;                       // latched between telemetry packets
imu::Quaternion lastQuat;
imu::Vector<3> lastLin;
imu::Vector<3> lastGyro;                // sensor frame, deg/s
imu::Vector<3> lastAcc;                 // sensor frame, m/s² — RAW, pre-fusion
// 200 ms window sums for the telemetry means (ver 0x05). Plain doubles rather
// than imu::Vector so the accumulate can't lose precision over the window.
double gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
double accSumX = 0, accSumY = 0, accSumZ = 0;
uint16_t imuWinN = 0;                   // samples in the sums (~20 per packet)
// BNO055 calibration, refreshed at 1 Hz. IMUPLUS has no magnetometer, so the
// accelerometer is the only thing that knows where down is: calAcc is what
// decides whether a mount zero means anything, and calMag stays 0 forever.
uint8_t calSys = 0, calGyro = 0, calAcc = 0, calMag = 0;
bool     calSaved = false;              // offsets already written to flash
// True once per-chip offsets have been loaded from flash. Needed because the
// BNO055's CALIB_STAT reports the fusion's *live* confidence, not whether
// offsets are applied — it reads 0 after every reset even on a chip whose
// calibration was saved. Gating the mount zero on calAcc alone would therefore
// refuse it after every power-up, which on a build with no buttons and no
// screen leaves the rider no way to zero at all.
bool     calRestored = false;
uint32_t quatRejects = 0;               // getQuat() results that weren't unit
uint8_t  zeroCount = 0;                 // successful mount zeros since boot
float lastPressPa = 0, lastTempC = 0, lastBaroAlt = 0;
uint32_t identifyUntil = 0, splashUntil = 0;
uint32_t tImu = 0, tTele = 0, tStatus = 0, tOled = 0;
uint32_t loopsPerSec = 0;               // loop() iterations between debug lines
uint32_t maxLoopGapMs = 0, lastLoopAt = 0;   // worst single-iteration stall

// Buttons are captured by pin-change ISRs so presses land even while the
// loop is stalled (I2C timeout etc.); the loop only consumes the results.
// Both buttons discriminate click from hold the same way: the ISR timestamps
// the press and only counts a click if the release lands inside CLICK_MAX_MS,
// so a long hold never also registers as a click.
volatile uint32_t b1FallAt = 0, b1EdgeAt = 0;
volatile bool     b1Down = false;
volatile uint8_t  b1Clicks = 0;         // button 1 short releases not yet consumed
volatile uint32_t b2FallAt = 0, b2EdgeAt = 0;
volatile bool     b2Down = false;
volatile uint8_t  b2Clicks = 0;         // button 2 short releases not yet consumed
volatile bool     bleZeroReq = false;   // 0x02 control write, consumed in loop()
volatile bool     bleRideMsg = false;   // 0x04 control write latched below
volatile uint8_t  bleRideActiveB = 0;
volatile uint32_t bleRideElapsedS = 0;
volatile bool     bleOtaMsg = false;    // 0x05 control write latched below
volatile uint8_t  bleOtaOnB = 0;

void IRAM_ATTR isrBtn1() {
  uint32_t t = millis();
  bool dn = digitalRead(PIN_BUTTON) == LOW;
  if (dn == b1Down || t - b1EdgeAt < 50) return;   // bounce
  b1EdgeAt = t;
  b1Down = dn;
  if (dn) b1FallAt = t;
  else if (t - b1FallAt < CLICK_MAX_MS) b1Clicks++;
}

void IRAM_ATTR isrBtn2() {
  uint32_t t = millis();
  bool dn = digitalRead(PIN_BUTTON2) == LOW;
  if (dn == b2Down || t - b2EdgeAt < 50) return;   // bounce
  b2EdgeAt = t;
  b2Down = dn;
  if (dn) b2FallAt = t;
  else if (t - b2FallAt < CLICK_MAX_MS) b2Clicks++;
}

// screens (button 1) + mount-zero (button 2)
Preferences prefs;
imu::Quaternion qRef;                   // mount reference; identity until zeroed
// Screens no longer rotate on their own. Button 1 steps through them; a 3 s
// hold hands the stepping back to a timer. Manual is the default because on a
// moving bike you want the screen you chose to stay put.
int  screenIdx = 0;
bool autoCycle = false;
uint32_t tCycle = 0;
bool cycleFired = false;
bool rideActive = false;                // phone is recording: invert + trip screen
uint32_t tripStartMs = 0;               // millis() epoch of the ride (elapsed-adjusted)
uint32_t zeroSplashUntil = 0;
bool zeroFired = false;
bool zeroRefused = false;               // last zero attempt blocked on accel cal

void saveQRef() {
  prefs.putFloat("qw", qRef.w()); prefs.putFloat("qx", qRef.x());
  prefs.putFloat("qy", qRef.y()); prefs.putFloat("qz", qRef.z());
}

void quatToEuler(const imu::Quaternion &q, float &rollDeg, float &pitchDeg, float &yawDeg) {
  double sinr = 2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  rollDeg = atan2(sinr, cosr) * 57.29578;
  double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  if (sinp > 1.0) sinp = 1.0;
  if (sinp < -1.0) sinp = -1.0;
  pitchDeg = asin(sinp) * 57.29578;
  double siny = 2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yawDeg = atan2(siny, cosy) * 57.29578;
}

// Strips yaw off a quaternion, keeping its tilt. IMUPLUS has no magnetometer,
// so yaw free-runs from wherever the puck happened to point at power-on: it is
// not a measurement and must never reach the mount reference.
imu::Quaternion tiltOnly(const imu::Quaternion &q) {
  float r, p, y;
  quatToEuler(q, r, p, y);
  const double H = 0.0087266462;        // deg -> rad, halved
  double cr = cos(r * H), sr = sin(r * H);
  double cp = cos(p * H), sp = sin(p * H);
  return imu::Quaternion(cr * cp, sr * cp, cr * sp, -sr * sp);   // ZYX, yaw = 0
}

// q ⊗ qRef⁻¹ — the mount correction applied on the BODY side (unit quat:
// conj == inverse). It has to be this way round. Yaw error is a rotation about
// the world vertical, so it enters on the left (q = qYaw(δ) ⊗ qTrue); a
// reference applied on the left too would sit the drift *between* itself and
// the attitude, and its tilt would no longer cancel the mount tilt — leaving a
// residual that rotates with δ and swamps the real lean. Applied on the right,
// the drift stays outermost, and a left-multiplied yaw leaves the ZYX roll and
// pitch untouched: lean and slope come out immune to both the boot origin and
// gyro drift.
imu::Quaternion qRel() {
  const imu::Quaternion &a = lastQuat;
  double bw = qRef.w(), bx = -qRef.x(), by = -qRef.y(), bz = -qRef.z();
  return imu::Quaternion(
      a.w() * bw - a.x() * bx - a.y() * by - a.z() * bz,
      a.w() * bx + a.x() * bw + a.y() * bz - a.z() * by,
      a.w() * by - a.x() * bz + a.y() * bw + a.z() * bx,
      a.w() * bz + a.x() * by - a.y() * bx + a.z() * bw);
}

// The single path for "this orientation is now zero", shared by the 10 s hold
// and the app's 0x02 write. Refused while the accelerometer is uncalibrated:
// in IMUPLUS it is the only thing that knows where down is, so a zero taken
// against a bad one is silently wrong for the whole ride. tripper_light gates
// the same way, and the app checks the calibration byte before it ever sends
// 0x02 — that matters on the Light build, which has no screen to refuse on.
void captureZero(uint32_t now, const char *source) {
  zeroSplashUntil = now + 1500;
  tOled = 0;
  if (calAcc < 3 && !calRestored) {
    zeroRefused = true;
    Serial.printf("[zero] refused (%s): accel calibration %d/3 — leave the bike "
                  "still and level for a few seconds\n", source, calAcc);
    return;
  }
  zeroRefused = false;
  qRef = tiltOnly(lastQuat);            // this orientation's TILT is the new zero
  saveQRef();
  zeroCount++;                          // the app's proof it landed
  Serial.printf("[zero] mount reference captured & saved (%s)\n", source);
}

// ---------- GPS bring-up (idempotent, runs every boot) ----------
void sendUBX(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  uint8_t hdr[4] = {cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  Serial1.write(0xB5); Serial1.write(0x62);
  for (int i = 0; i < 4; i++) { Serial1.write(hdr[i]); ckA += hdr[i]; ckB += ckA; }
  for (int i = 0; i < len; i++) { Serial1.write(payload[i]); ckA += payload[i]; ckB += ckA; }
  Serial1.write(ckA); Serial1.write(ckB);
}

bool nmeaAlive(uint32_t windowMs) {
  uint32_t t0 = millis();
  char w0 = 0, w1 = 0; int hits = 0;
  while (millis() - t0 < windowMs) {
    if (!Serial1.available()) continue;
    char c = Serial1.read();
    if (w0 == '$' && w1 == 'G' && (c == 'P' || c == 'N' || c == 'L' || c == 'B' || c == 'A'))
      if (++hits >= 2) return true;
    w0 = w1; w1 = c;
  }
  return false;
}

// Drop bytes buffered at the previous baud — a probe right after a baud
// switch can otherwise "pass" on stale NMEA.
void gpsFlushRx() {
  delay(50);
  while (Serial1.available()) Serial1.read();
}

// 1 = ACK, 0 = NAK, -1 = timeout
int waitAck(uint8_t cls, uint8_t id, uint32_t timeoutMs = 1500) {
  uint32_t t0 = millis();
  int st = 0; uint8_t ackId = 0, p0 = 0;
  while (millis() - t0 < timeoutMs) {
    if (!Serial1.available()) continue;
    uint8_t b = Serial1.read();
    switch (st) {
      case 0: st = (b == 0xB5) ? 1 : 0; break;
      case 1: st = (b == 0x62) ? 2 : 0; break;
      case 2: st = (b == 0x05) ? 3 : (b == 0xB5 ? 1 : 0); break;
      case 3: if (b == 0x01 || b == 0x00) { ackId = b; st = 4; } else st = 0; break;
      case 4: st = (b == 0x02) ? 5 : 0; break;
      case 5: st = (b == 0x00) ? 6 : 0; break;
      case 6: p0 = b; st = 7; break;
      case 7:
        if (p0 == cls && b == id) return ackId == 0x01 ? 1 : 0;
        st = 0; break;
    }
  }
  return -1;
}

bool gpsReconfigured = false;           // fallback path ran → BBR was lost

bool gpsBringup() {
  Serial1.setRxBufferSize(2048);        // survive OLED/BLE stalls at 115200
  Serial1.begin(115200, SERIAL_8N1, D7, D6);
  gpsFlushRx();
  if (nmeaAlive(1200)) return true;     // already configured (BBR intact)

  // BBR lost (dead backup cell + power cycle): module is back at 9600/1 Hz
  // factory defaults. Confirm it's alive at 9600 BEFORE sending config, wait
  // for the ACK, and retry — a blind one-shot send demonstrably fails here.
  const uint8_t rate[6] = {0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
  const uint8_t prt[20] = {0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                           0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00,
                           0x00, 0x00, 0x00, 0x00};
  for (int attempt = 0; attempt < 3; attempt++) {
    Serial1.updateBaudRate(9600);
    gpsFlushRx();
    if (!nmeaAlive(1500)) continue;     // not talking yet (still booting?)
    gpsReconfigured = true;
    // Only the baud switch happens on the flaky 9600 link (its ACK would
    // straddle the switch anyway); everything else runs at 115200 where
    // ACKs are reliable.
    sendUBX(0x06, 0x00, prt, 20);
    Serial1.flush();
    delay(200);
    Serial1.updateBaudRate(115200);
    gpsFlushRx();
    if (!nmeaAlive(1500)) continue;     // switch didn't take — retry from 9600
    int ack = -1;
    for (int r = 0; r < 3 && ack != 1; r++) {
      sendUBX(0x06, 0x08, rate, 6);
      ack = waitAck(0x06, 0x08);
    }
    Serial.printf("[gps] CFG-RATE 5Hz: %s\n",
                  ack == 1 ? "ACK" : "NO ACK — rate may still be 1 Hz");
    // Re-save to BBR/flash: with a healthy backup cell the next power-up
    // then takes the fast path — so the "BBR lost" warning only ever
    // fires when the cell genuinely failed to hold.
    const uint8_t save[13] = {0, 0, 0, 0, 0xFF, 0xFF, 0x00, 0x00, 0, 0, 0, 0, 0x03};
    sendUBX(0x06, 0x09, save, 13);
    int sa = waitAck(0x06, 0x09);
    Serial.printf("[gps] CFG-CFG save to BBR: %s\n",
                  sa == 1 ? "ACK" : sa == 0 ? "NAK" : "timeout");
    return true;
  }
  return false;
}

// ---------- BLE callbacks ----------
class SrvCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override { Serial.println("[ble] phone connected"); }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
    Serial.printf("[ble] disconnected (reason %d), advertising again\n", reason);
  }
};

class CtrlCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
    NimBLEAttValue v = c->getValue();
    if (v.size() < 1) return;
    switch (v.data()[0]) {
      case 0x01:                                                         // phone marker ack
        splashUntil = millis() + 800;
        tOled = 0;
        break;
      case 0x02: bleZeroReq = true; break;                              // zero pitch/roll
      case 0x03: identifyUntil = millis() + 2000; break;                // identify
      case 0x04:                                                        // ride state
        if (v.size() >= 6) {
          bleRideActiveB = v.data()[1];
          bleRideElapsedS = (uint32_t)v.data()[2] | ((uint32_t)v.data()[3] << 8) |
                            ((uint32_t)v.data()[4] << 16) | ((uint32_t)v.data()[5] << 24);
          bleRideMsg = true;
        }
        break;
      case 0x05:                                                        // WiFi flashing mode
        if (v.size() >= 2) { bleOtaOnB = v.data()[1]; bleOtaMsg = true; }
        break;
      // 0x06 + [on u8]: raw 100 Hz stream. Boots ON — every ride should be a
      // regression test, and the phone decides whether to keep the bytes. The
      // switch exists for the day a link problem needs the radio quiet.
      case 0x06:
        if (v.size() >= 2) {
          rawEnabled = v.data()[1] != 0;
          rawN = 0;
          Serial.printf("[ble] raw stream %s\n", rawEnabled ? "on" : "off");
        }
        break;
    }
  }
};

// ---------- helpers ----------
int satsInView() {
  int n = 0;
  if (gsvGP.isValid()) n += atoi(gsvGP.value());
  if (gsvGL.isValid()) n += atoi(gsvGL.value());
  if (gsvGB.isValid()) n += atoi(gsvGB.value());
  return n;
}

bool fixValid() { return gps.location.isValid() && gps.location.age() < 2000; }

// --- tiny glyphs ---
void drawBtRune(int x, int y) {         // 7x9 bluetooth rune
  oled.drawLine(x + 3, y, x + 3, y + 8, SSD1306_WHITE);
  oled.drawLine(x + 3, y, x + 6, y + 2, SSD1306_WHITE);
  oled.drawLine(x + 6, y + 2, x, y + 6, SSD1306_WHITE);
  oled.drawLine(x + 3, y + 8, x + 6, y + 6, SSD1306_WHITE);
  oled.drawLine(x + 6, y + 6, x, y + 2, SSD1306_WHITE);
}

void drawSatBars(int x, int yBase, int sats) {  // 4 rising signal bars
  int lit = sats >= 9 ? 4 : sats >= 6 ? 3 : sats >= 3 ? 2 : sats >= 1 ? 1 : 0;
  for (int i = 0; i < 4; i++) {
    int h = 2 + i * 2;
    if (i < lit) oled.fillRect(x + i * 3, yBase - h, 2, h, SSD1306_WHITE);
    else         oled.drawPixel(x + i * 3, yBase - 1, SSD1306_WHITE);
  }
}

void drawClockScreen(uint32_t now) {
  bool connected = bleServer && bleServer->getConnectedCount() > 0;
  oled.setTextSize(3);
  oled.setCursor(2, 5);
  if (gps.time.isValid()) {
    int h = (gps.time.hour() + TZ_HOURS) % 24;
    // colon blinks with the GPS seconds — the display's heartbeat
    oled.printf("%02d%c%02d", h, (gps.time.second() % 2) ? ':' : ' ', gps.time.minute());
    oled.setTextSize(1);
    oled.setCursor(102, 5);
    oled.printf("%02d", gps.time.second());
  } else {
    oled.print("--:--");
  }
  // right-edge status column: BT rune (blinks while only advertising) + sat bars
  if (connected || (now / 500) % 2) drawBtRune(101, 15);
  drawSatBars(114, 25, satsInView());
}

void drawDataScreen(uint32_t now) {
  bool connected = bleServer && bleServer->getConnectedCount() > 0;
  float rollD, pitchD, yawD;
  quatToEuler(qRel(), rollD, pitchD, yawD);
  float aMag = lastLin.magnitude() / 9.80665f;

  // status bar: rune · link state · sat bars+count · live g · fix dot
  if (connected || (now / 500) % 2) drawBtRune(1, 0);
  oled.setTextSize(1);
  oled.setCursor(11, 1);
  oled.print(connected ? "LINK" : "ADV");
  drawSatBars(52, 9, satsInView());
  oled.setCursor(66, 1);
  oled.printf("%d", satsInView());
  oled.setCursor(86, 1);
  oled.printf("%.1fg", aMag);
  if (fixValid()) oled.fillCircle(122, 4, 3, SSD1306_WHITE);
  else            oled.drawCircle(122, 4, 3, SSD1306_WHITE);
  oled.drawFastHLine(0, 11, 128, SSD1306_WHITE);

  // hero: speed, big
  oled.setTextSize(2);
  oled.setCursor(0, 15);
  oled.printf("%4.1f", gps.speed.isValid() ? gps.speed.kmph() : 0.0);
  oled.setTextSize(1);
  oled.setCursor(50, 22);
  oled.print("km/h");

  // right column: attitude
  oled.setCursor(84, 13);
  oled.printf("R%+6.1f", rollD);
  oled.setCursor(84, 23);
  oled.printf("P%+6.1f", pitchD);
}

void drawTripScreen(uint32_t now) {
  uint32_t s = rideActive ? (now - tripStartMs) / 1000UL : 0;
  uint32_t h = s / 3600, m = (s / 60) % 60, sec = s % 60;
  oled.setTextSize(3);
  if (h) {                              // 7 chars just fit the 128 px
    oled.setCursor(1, 5);
    oled.printf("%lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m, (unsigned long)sec);
  } else {
    oled.setCursor(19, 5);
    oled.printf("%02lu:%02lu", (unsigned long)m, (unsigned long)sec);
  }
}

bool canLive(uint32_t now) {
  return canOk && canS.lastRxMs && now - canS.lastRxMs < CAN_STALE_MS;
}

void drawCanScreen(uint32_t now) {
  if (!canLive(now)) {
    oled.setTextSize(1);
    oled.setCursor(22, 6);
    oled.print("BIKE CAN BUS");
    oled.setCursor(31, 18);
    oled.print(canOk ? "no data" : "off");
    return;
  }
  uint8_t st = canS.f202[0];
  uint8_t mode = (st >> 4) & 3;

  // top strip: ride mode left, kickstand right
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(mode == 1 ? "Eco" : mode == 2 ? "Sport" : "?");
  oled.setCursor(86, 0);
  oled.print((st >> 7) & 1 ? "KICK DN" : "KICK UP");
  oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);

  // hero: bike speed (not GPS — this is the wheel's own number)
  oled.setTextSize(2);
  oled.setCursor(0, 13);
  oled.printf("%4.1f", canU16(canS.f303, 0) / 10.0f);
  oled.setTextSize(1);
  oled.setCursor(50, 20);
  oled.print("km/h");

  // battery percentage, right
  oled.setTextSize(2);
  oled.setCursor(80, 13);
  oled.printf("%3u%%", canS.f401[0]);
}

void drawMarkerSplash() {
  oled.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(2);
  oled.setCursor(22, 9);
  oled.printf("MARK %d", markerCount);
}

void drawZeroProgress(uint32_t heldMs) {
  oled.setTextSize(1);
  oled.setCursor(16, 4);
  oled.print("hold to ZERO axes");
  long w = (long)(heldMs - ZERO_SHOW_MS) * 120 / (long)(ZERO_HOLD_MS - ZERO_SHOW_MS);
  if (w > 120) w = 120;
  oled.drawRect(4, 18, 120, 9, SSD1306_WHITE);
  if (w > 0) oled.fillRect(4, 18, (int)w, 9, SSD1306_WHITE);
}

void drawZeroedSplash() {
  oled.fillRect(0, 0, 128, 32, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  if (zeroRefused) {
    // Tell the rider what to do about it, not just that it failed.
    oled.setTextSize(2);
    oled.setCursor(10, 2);
    oled.printf("ACC %d/3", calAcc);
    oled.setTextSize(1);
    oled.setCursor(4, 22);
    oled.print("hold still, level");
    return;
  }
  oled.setTextSize(2);
  oled.setCursor(28, 9);
  oled.print("ZEROED");
}

// clock · data · CAN, plus trip time while the app says a ride is recording.
int screenCount() { return rideActive ? 4 : 3; }

void refreshOled(uint32_t now) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  uint32_t heldMs = (b2Down && !zeroFired) ? now - b2FallAt : 0;
  if (heldMs > ZERO_SHOW_MS)        drawZeroProgress(heldMs);
  else if (now < zeroSplashUntil)   drawZeroedSplash();
  else if (now < splashUntil)       drawMarkerSplash();
  else {
    int idx = screenIdx;
    if (idx >= screenCount()) idx = 0;  // trip screen vanished when the ride ended
    if (idx == 0)      drawClockScreen(now);
    else if (idx == 1) drawDataScreen(now);
    else if (idx == 2) drawCanScreen(now);
    else               drawTripScreen(now);
    // Border marks the unusual state: the screens are stepping on their own.
    if (autoCycle) oled.drawRect(0, 0, 128, 32, SSD1306_WHITE);
  }
  // recording inverts the whole display; identify blinks relative to that
  oled.invertDisplay(rideActive != (now < identifyUntil));
  Wire1.setClock(OLED_BURST_HZ);        // burst the frame out fast
  oled.display();
  Wire1.setClock(OLED_BUS_HZ);          // back to the long-cable-safe speed
}

// ---------- setup ----------
// Adafruit_SSD1306::begin() returns true as soon as its framebuffer mallocs —
// it never checks for an ACK — so an unplugged display still reported "ok" and
// then ate a NACKing 512-byte transfer on every refresh. Probe the bus first
// so oledOk means what it says.
bool i2cPresent(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

// Listen-only: the controller never transmits and never even ACKs, so the
// puck cannot influence the bike's bus whatever this firmware does. Failure
// here is non-fatal — the puck is a GPS/IMU logger first and must still boot
// and stream if the transceiver is unplugged.
bool canBringup() {
  twai_general_config_t g =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_len = 32;                  // ~92 frames/s; drained every loop pass
  g.alerts_enabled = TWAI_ALERT_NONE;   // nothing here reacts to alerts
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  return true;
}

// Drain whatever the controller has queued. Bounded so a burst can never
// stretch a loop iteration — the same reason the buttons sit on ISRs.
void canPoll() {
  if (!canOk) return;
  twai_message_t m;
  for (int i = 0; i < 16 && twai_receive(&m, 0) == ESP_OK; i++) {
    if (m.extd || m.rtr || m.data_length_code < 8) continue;
    switch (m.identifier) {
      case 0x101: memcpy(canS.f101, m.data, 8); break;
      case 0x201: memcpy(canS.f201, m.data, 8); break;
      case 0x202: memcpy(canS.f202, m.data, 8); break;
      case 0x203: memcpy(canS.f203, m.data, 8); break;
      case 0x302: memcpy(canS.f302, m.data, 8); break;
      case 0x303: memcpy(canS.f303, m.data, 8); break;
      case 0x401: memcpy(canS.f401, m.data, 8); break;
      case 0x490: memcpy(canS.f490, m.data, 8); break;
      default: continue;                // other IDs are undecoded, ignore
    }
    canS.lastRxMs = millis();
    canS.frames++;
  }
}

void setup() {
  Serial.begin(115200);
  // Never let debug prints block the loop: with a half-open USB CDC (host
  // opened the port but isn't draining it) each printf otherwise stalls for
  // its timeout — seconds-long loop freezes, dead buttons, laggy OLED.
  Serial.setTxTimeoutMs(0);
  delay(1500);
  Serial.println("\n=== Tripper Puck firmware (e-bike/BLE) ===");
  // Into the wireless ring too, so /log always opens with how this boot began
  // — reset reason distinguishes a power cycle from a crash from an OTA.
  logLine("[boot] FULL build %s %s, reset reason %d",
          __DATE__, __TIME__, (int)esp_reset_reason());

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), isrBtn1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON2), isrBtn2, CHANGE);

  prefs.begin("puck", false);           // load the mount reference, if ever zeroed
  qRef = imu::Quaternion(prefs.getFloat("qw", 1.0f), prefs.getFloat("qx", 0.0f),
                         prefs.getFloat("qy", 0.0f), prefs.getFloat("qz", 0.0f));
  // References saved by builds that stored yaw carry a yaw from a *previous*
  // boot's origin, which is meaningless now. The tilt half is still good, so
  // strip rather than discard: an already-zeroed puck needs no re-zero.
  qRef = tiltOnly(qRef);

  Wire.begin(PIN_SDA, PIN_SCL);         // rear enclosure: BNO055 + BMP280
  Wire.setClock(100000);
  Wire.setTimeOut(1000);
  Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);   // handlebar enclosure: OLED alone
  Wire1.setClock(OLED_BUS_HZ);
  Wire1.setTimeOut(1000);

  imuOk = bno.begin(OPERATION_MODE_IMUPLUS);
  if (imuOk) {
    // The Gravity board carries a 32.768 kHz crystal. Without this the fusion
    // runs off the internal RC oscillator, which Bosch does not consider good
    // enough for the fusion modes — and a wandering timebase shows up as
    // attitude drift over a ride rather than as an obvious failure.
    bno.setExtCrystalUse(true);
    // Restore calibration so a ride starts accurate instead of converging over
    // its first few minutes. Offsets are per-chip, so they live in flash next
    // to the mount reference.
    adafruit_bno055_offsets_t off;
    if (prefs.getBytes("bnooff", &off, sizeof(off)) == sizeof(off)) {
      bno.setSensorOffsets(off);
      calRestored = true;
      Serial.println("IMU calibration offsets restored from flash");
    }
    bno.getCalibration(&calSys, &calGyro, &calAcc, &calMag);
  }
  bmpOk = bmp.begin(0x76);
  if (bmpOk)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_63);
  uint8_t oledAddr = i2cPresent(Wire1, 0x3C) ? 0x3C : i2cPresent(Wire1, 0x3D) ? 0x3D : 0;
  oledOk = oledAddr && oled.begin(SSD1306_SWITCHCAPVCC, oledAddr);
  Serial.printf("IMU %s | BMP280 %s | OLED %s\n",
                imuOk ? "ok" : "FAIL", bmpOk ? "ok" : "FAIL",
                oledOk ? "ok" : oledAddr ? "FAIL" : "absent");

  bool gpsOk = gpsBringup();
  Serial.printf("GPS %s (115200/5Hz)%s\n", gpsOk ? "ok" : "NOT RESPONDING",
                gpsReconfigured ? " — was at 9600 factory: BBR lost, check backup cell" : "");

  canOk = canBringup();
  Serial.printf("CAN %s (250k listen-only, D8/D9)\n", canOk ? "ok" : "FAIL");

  NimBLEDevice::init("Tripper-DL1");
  // The telemetry packet is 70 B, so the link must land above the 23 B default
  // ATT MTU. iOS negotiates 185 B; this just states the requirement explicitly.
  NimBLEDevice::setMTU(247);
  // Full TX power: phone logs showed supervision timeouts every few minutes
  // at the default level. The link crosses a bike frame and a rider's body
  // to a pocketed phone — margin matters more than the ~20 mW it costs.
  NimBLEDevice::setPower(9);
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new SrvCB());
  bleServer->advertiseOnDisconnect(true);
  NimBLEService *svc = bleServer->createService(SVC_UUID);
  chTele = svc->createCharacteristic(TELE_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  chStat = svc->createCharacteristic(STAT_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  // Notify-only: nothing ever reads the raw stream on demand, and leaving READ
  // off keeps a curious client from pulling one stale batch out of context.
  chRaw  = svc->createCharacteristic(RAW_UUID, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic *ctrl = svc->createCharacteristic(
      CTRL_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  ctrl->setCallbacks(new CtrlCB());
  svc->start();
  // The 128-bit service UUID (18 B) and the name (13 B) don't both fit in the
  // 31-byte primary advertisement, and NimBLE silently drops the UUID — which
  // makes the puck invisible to Tripper's service-filtered scan (the only kind
  // iOS allows in the background). UUID goes in the primary packet, name in
  // the scan response; iOS merges the two, so scanners still show the name.
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setCompleteServices(NimBLEUUID(SVC_UUID));
  NimBLEAdvertisementData scanData;
  scanData.setName("Tripper-DL1");
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->start();
  Serial.println("[ble] advertising as Tripper-DL1 (UUID in adv, name in scan rsp)");

  if (oledOk) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 8);
    oled.println("   Tripper Puck");
    oled.setCursor(0, 20);
    oled.println("   BLE: Tripper-DL1");
    oled.display();
  }
}

// The single path in and out of WiFi flashing mode, driven by control write
// 0x05. On: raise the SoftAP and start ArduinoOTA — the puck becomes network
// "Tripper-Puck-OTA" at 192.168.4.1 and a network port in Arduino IDE for
// any computer that joins. Off: tear both down and kill the radio.
void otaSetMode(bool on) {
  if (on == otaActive) return;
  if (on) {
    WiFi.mode(WIFI_AP);
    // Two clients max: this is a flashing jig, not infrastructure.
    if (!WiFi.softAP(OTA_AP_SSID, OTA_AP_PASS, 6, 0, 2)) {
      logLine("[ota] softAP failed to start");
      WiFi.mode(WIFI_OFF);
      return;
    }
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() { logLine("[ota] update starting"); });
    ArduinoOTA.onEnd([]() { logLine("[ota] update done, rebooting"); });
    ArduinoOTA.onError([](ota_error_t e) { logLine("[ota] error %u", e); });
    ArduinoOTA.begin();
    // Diagnostics over the same network — status at /, recent log at /log.
    httpd.on("/", []() {
      char out[512];
      snprintf(out, sizeof(out),
               "Tripper puck - FULL build\n"
               "fw built %s %s (packet v0x05)\n"
               "uptime %lus, reset reason %d, free heap %u\n"
               "fix %d, sats %d\n"
               "cal sys/gyro/accel %d/%d/%d, quatRejects %lu\n"
               "baro %.1f m, temp %.1f C\n"
               "CAN %s, %lu frames\n\n"
               "/log     recent debug lines\n"
               "/update  flash new firmware from a phone browser\n",
               __DATE__, __TIME__, millis() / 1000UL, (int)esp_reset_reason(),
               (unsigned)ESP.getFreeHeap(), fixValid() ? 1 : 0,
               gps.satellites.isValid() ? (int)gps.satellites.value() : 0,
               calSys, calGyro, calAcc, (unsigned long)quatRejects,
               lastBaroAlt, lastTempC,
               canOk ? "ok" : "FAIL", (unsigned long)canS.frames);
      httpd.send(200, "text/plain", out);
    });
    httpd.on("/log", []() {
      String out;
      out.reserve(sizeof(logRing) + 16);
      for (size_t i = 0; i < sizeof(logRing); i++) {
        char c = logRing[(logHead + i) % sizeof(logRing)];
        if (c) out += c;               // NULs = never-written slots, skip
      }
      httpd.send(200, "text/plain", out);
    });

    // ---- flashing from a phone ----
    // ArduinoOTA speaks espota, which only the Arduino tooling implements —
    // fine from a laptop, useless from a phone. This is the same firmware
    // image pushed through an ordinary HTML file upload instead, so anything
    // with a browser can flash the puck while it stays bolted to the bike.
    // Safari's file picker reaches iCloud Drive and Files, so AirDrop the .bin
    // from the Mac once and the laptop never has to come near the bike again.
    httpd.on("/update", HTTP_GET, []() {
      httpd.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset=utf-8>"
        "<meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Flash puck</title><style>"
        "body{font:17px -apple-system,sans-serif;margin:0;padding:28px 22px;"
        "background:#faf9f7;color:#22201d}h1{font-size:19px;margin:0 0 6px}"
        "p{color:#7a736b;font-size:14px;margin:0 0 22px}"
        "input[type=file]{display:block;margin:0 0 18px;width:100%}"
        "button{font:600 17px -apple-system,sans-serif;background:#2a78d6;"
        "color:#fff;border:0;border-radius:9px;padding:14px 22px;width:100%}"
        "#s{margin-top:18px;font-size:15px}</style></head><body>"
        "<h1>Flash puck</h1><p>Pick the firmware .bin, then Upload. "
        "Do not lock the phone or leave this page while it runs. "
        "The puck reboots by itself when it finishes.</p>"
        "<form id=f method=POST action='/update' enctype='multipart/form-data'>"
        "<input type=file name=fw accept='.bin,application/octet-stream' required>"
        "<button type=submit>Upload</button></form><div id=s></div>"
        "<script>"
        // XHR rather than a plain submit so the phone shows real progress —
        // a 1.4 MB upload over SoftAP takes long enough that a blank white
        // page reads as a hang, and a reload mid-flash bricks the puck.
        "var f=document.getElementById('f'),s=document.getElementById('s');"
        "f.onsubmit=function(e){e.preventDefault();"
        "var x=new XMLHttpRequest();x.open('POST','/update');"
        "x.upload.onprogress=function(p){if(p.lengthComputable)"
        "s.textContent='Uploading '+Math.round(p.loaded/p.total*100)+'%'};"
        "x.onload=function(){s.textContent=x.responseText||'done'};"
        "x.onerror=function(){s.textContent='Upload failed - stay on the "
        "Tripper AP and try again'};"
        "s.textContent='Starting...';x.send(new FormData(f));return false};"
        "</script></body></html>");
    });
    // Two handlers: the second runs per chunk as the body streams in, the
    // first only once the whole body has been consumed.
    httpd.on("/update", HTTP_POST, []() {
      bool ok = !Update.hasError();
      httpd.sendHeader("Connection", "close");
      httpd.send(200, "text/plain",
                 ok ? "OK - puck rebooting into the new firmware"
                    : "FAILED - puck kept the old firmware");
      logLine("[ota] http update %s", ok ? "ok, rebooting" : "FAILED");
      if (ok) { delay(400); ESP.restart(); }
    }, []() {
      HTTPUpload &up = httpd.upload();
      if (up.status == UPLOAD_FILE_START) {
        logLine("[ota] http update starting: %s", up.filename.c_str());
        // UPDATE_SIZE_UNKNOWN: a browser upload has no reliable length up
        // front, so let the Update library size it against the free OTA slot.
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          logLine("[ota] Update.begin failed: %s", Update.errorString());
        }
      } else if (up.status == UPLOAD_FILE_WRITE) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize) {
          logLine("[ota] write failed: %s", Update.errorString());
        }
      } else if (up.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) {
          logLine("[ota] Update.end failed: %s", Update.errorString());
        }
      } else if (up.status == UPLOAD_FILE_ABORTED) {
        // Phone locked, browser backgrounded, or the user walked out of range.
        // Abort explicitly so the next attempt starts from a clean slot.
        Update.abort();
        logLine("[ota] http update aborted by client");
      }
    });

    // iOS probes this URL the moment it joins a network and expects Apple's
    // exact success body. Without it the phone decides there is no internet,
    // shows the captive-portal sheet, and can silently drop back to cellular —
    // at which point 192.168.4.1 is unreachable and the whole thing looks
    // broken for no visible reason. Android and Windows probe their own URLs;
    // onNotFound covers those and anything else with the same answer.
    httpd.on("/hotspot-detect.html", []() {
      httpd.send(200, "text/html",
                 "<HTML><HEAD><TITLE>Success</TITLE></HEAD>"
                 "<BODY>Success</BODY></HTML>");
    });
    httpd.onNotFound([]() {
      httpd.send(200, "text/html",
                 "<HTML><HEAD><TITLE>Success</TITLE></HEAD>"
                 "<BODY>Success</BODY></HTML>");
    });

    httpd.begin();
    otaActive = true;
    logLine("[ota] AP \"%s\" up at %s — IDE network port %s.local; "
            "http://192.168.4.1/ status, /log debug, /update flash from a phone",
            OTA_AP_SSID, WiFi.softAPIP().toString().c_str(), OTA_HOSTNAME);
  } else {
    httpd.stop();
    ArduinoOTA.end();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    otaActive = false;
    logLine("[ota] AP down, WiFi radio off");
  }
}

// ---------- main loop ----------
void loop() {
  uint32_t now = millis();
  loopsPerSec++;
  if (lastLoopAt && now - lastLoopAt > maxLoopGapMs) maxLoopGapMs = now - lastLoopAt;
  lastLoopAt = now;

  while (Serial1.available()) gps.encode(Serial1.read());
  canPoll();

  if (bleOtaMsg) {                      // 0x05: WiFi flashing mode toggle
    bleOtaMsg = false;
    otaSetMode(bleOtaOnB != 0);
  }
  if (otaActive) {
    ArduinoOTA.handle();
    httpd.handleClient();
  }

  // buttons: edges were latched by the ISRs — just consume them here
  // button 1: click steps the screen, 3 s hold toggles auto-cycling
  if (b1Clicks) {
    noInterrupts();
    uint8_t n = b1Clicks; b1Clicks = 0;
    interrupts();
    screenIdx = (screenIdx + n) % screenCount();
    tCycle = now;                       // a manual step earns a full dwell
    tOled = 0;
    Serial.printf("[screen] %d/%d\n", screenIdx, screenCount());
  }
  static bool b1Prev = false;
  if (b1Down && !b1Prev) cycleFired = false;       // new press = fresh hold
  b1Prev = b1Down;
  if (b1Down && !cycleFired && now - b1FallAt >= CYCLE_HOLD_MS) {
    cycleFired = true;
    autoCycle = !autoCycle;
    tCycle = now;
    tOled = 0;
    Serial.printf("[screen] auto-cycle %s\n", autoCycle ? "on" : "off");
  }
  if (autoCycle && now - tCycle >= SCREEN_MS) {
    tCycle = now;
    screenIdx = (screenIdx + 1) % screenCount();
    tOled = 0;
  }

  // button 2: click drops a marker, long hold zeroes the mount reference
  if (b2Clicks) {
    noInterrupts();
    uint8_t n = b2Clicks; b2Clicks = 0;
    interrupts();
    markerCount += n;
    splashUntil = now + 800;
    tOled = 0;                          // redraw immediately with the splash
    Serial.printf("[marker] #%d\n", markerCount);
  }
  static bool b2Prev = false;
  if (b2Down && !b2Prev) zeroFired = false;        // new press = fresh hold
  b2Prev = b2Down;
  if (b2Down && !zeroFired && now - b2FallAt >= ZERO_HOLD_MS) {
    zeroFired = true;
    captureZero(now, "button");
  }
  if (bleZeroReq) {                     // 0x02 from the app — same capture as the hold
    bleZeroReq = false;
    if (imuOk) captureZero(now, "app");
  }
  if (bleRideMsg) {                     // 0x04: ride state, elapsed is app-authoritative
    bleRideMsg = false;
    bool active = bleRideActiveB != 0;
    uint32_t elapsedS = bleRideElapsedS;
    if (active) tripStartMs = now - elapsedS * 1000UL;
    if (active != rideActive) { rideActive = active; tOled = 0; }
    Serial.printf("[ride] %s at %lus\n", active ? "recording" : "idle", (unsigned long)elapsedS);
  }

  // 100 Hz IMU
  if (imuOk && now - tImu >= 10) {
    tImu = now;
    // A glitched I2C read, or the chip briefly back in CONFIG mode, returns
    // something that isn't a rotation. Holding the previous sample costs one
    // 10 ms tick; letting it through corrupts the attitude outright.
    imu::Quaternion q = bno.getQuat();
    float n2 = q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
    if (n2 > 0.9f && n2 < 1.1f) lastQuat = q;
    else                        quatRejects++;
    lastLin  = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    lastGyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // Sampled in the same 10 ms tick as the quaternion and the linear accel, so
    // the three are one consistent snapshot — the comparison they exist for is
    // meaningless if they come from different instants.
    lastAcc  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // Accumulate for the packet means. Every 100 Hz sample now reaches the
    // app (as part of a mean) instead of 1 in 20 reaching it verbatim.
    gyroSumX += lastGyro.x(); gyroSumY += lastGyro.y(); gyroSumZ += lastGyro.z();
    accSumX  += lastAcc.x();  accSumY  += lastAcc.y();  accSumZ  += lastAcc.z();
    imuWinN++;
    float g = lastLin.magnitude() / 9.80665f;
    if (g > maxG_g) maxG_g = g;

    // ---- raw flight recorder ----
    // Same tick, same snapshot as the means above: the whole point is that this
    // is the un-averaged version of exactly what the packet carries.
    if (rawEnabled) {
      if (rawN == 0) rawT0 = now;
      RawSample &s = rawBuf[rawN];
      // Clamped rather than wrapped. A saturated sample is obvious in analysis;
      // a wrapped one reads as a violent rotation that never happened.
      auto cl16 = [](double v) -> int16_t {
        return (int16_t)(v > 32767.0 ? 32767.0 : (v < -32768.0 ? -32768.0 : v));
      };
      s.gx = cl16(lastGyro.x() * 100.0);
      s.gy = cl16(lastGyro.y() * 100.0);
      s.gz = cl16(lastGyro.z() * 100.0);
      s.ax = cl16(lastAcc.x() / 9.80665 * 1000.0);
      s.ay = cl16(lastAcc.y() / 9.80665 * 1000.0);
      s.az = cl16(lastAcc.z() / 9.80665 * 1000.0);
      rawN++;
      if (rawN >= RAW_BATCH_N) {
        if (bleServer->getConnectedCount() > 0) {
          uint8_t buf[sizeof(RawBatch) + sizeof(rawBuf)];
          RawBatch *h = (RawBatch *)buf;
          memset(h, 0, sizeof(RawBatch));
          h->ver = 0x01;
          h->count = rawN;
          h->period_us100 = 100;          // 10.000 ms nominal; t0_ms is the truth
          h->t0_ms = rawT0;
          // lastQuat, not qRel(): the mount zero is an app-side convention that
          // may be re-derived later, and a log that has already applied it can
          // never be un-applied.
          h->qw = (int16_t)(lastQuat.w() * 16384);
          h->qx = (int16_t)(lastQuat.x() * 16384);
          h->qy = (int16_t)(lastQuat.y() * 16384);
          h->qz = (int16_t)(lastQuat.z() * 16384);
          h->press_pa = (int32_t)lastPressPa;
          h->temp_x10 = (int16_t)(lastTempC * 10);
          // The globals the 1 Hz status block already refreshes. Re-reading the
          // chip ten times a second would spend I2C budget this bus does not
          // have — it runs at 100 kHz because the BNO055 stretches the clock —
          // on four numbers that change over tens of seconds.
          h->calSys = calSys; h->calGyr = calGyro;
          h->calAcc = calAcc; h->calMag = calMag;
          memcpy(buf + sizeof(RawBatch), rawBuf, sizeof(RawSample) * rawN);
          chRaw->setValue(buf, sizeof(RawBatch) + sizeof(RawSample) * rawN);
          chRaw->notify();
          rawBatches++;
        }
        rawN = 0;
      }
    }
  }

  // 5 Hz telemetry
  if (now - tTele >= 200) {
    tTele = now;
    // Read the barometer here, at packet rate, not in the 1 Hz status block
    // where it used to live. Slope is fitted over a 10 m window; at 30 km/h
    // that window passes in just over a second, and a 1 Hz altitude gave it
    // barely two distinct points to fit — the recording showed 0.99 altitude
    // changes per second across 5 Hz samples, a staircase. The BMP280 is
    // configured with 63 ms standby, so a fresh reading always exists.
    // Altitude is computed from the pressure just read rather than via
    // readAltitude(), which would issue a second I2C transaction for the
    // same number.
    if (bmpOk) {
      lastPressPa = bmp.readPressure();
      lastBaroAlt = 44330.0f * (1.0f - powf(lastPressPa / 101325.0f, 0.1903f));
    }
    TelemetryPacket p = {};
    // 0x01 was the 50-byte pre-CAN packet; 0x02 added CAN but carried no gyro
    // or calibration; 0x03 carried nothing the fusion hadn't already touched;
    // 0x04 added the raw accelerometer; 0x05 keeps its exact layout but sends
    // window means in the gyro/accel fields and reads the baro at 5 Hz.
    p.ver = 0x05;
    p.flags = (fixValid() ? 1 : 0) | (gps.time.isValid() ? 2 : 0)
              | ((calAcc >= 3 || calRestored) ? 4 : 0);
    p.gpsTimeMs = gps.time.isValid()
        ? (uint32_t)gps.time.hour() * 3600000UL + (uint32_t)gps.time.minute() * 60000UL +
          (uint32_t)gps.time.second() * 1000UL + (uint32_t)gps.time.centisecond() * 10UL
        : 0xFFFFFFFF;
    p.lat_e7 = (int32_t)(gps.location.lat() * 1e7);
    p.lon_e7 = (int32_t)(gps.location.lng() * 1e7);
    p.alt_cm = (int32_t)(gps.altitude.meters() * 100);
    p.baroAlt_cm = (int32_t)(lastBaroAlt * 100);
    p.press_pa = (uint32_t)lastPressPa;
    p.speed_cmps = gps.speed.isValid() ? (uint16_t)(gps.speed.mps() * 100) : 0;
    p.course_cdeg = gps.course.isValid() ? (uint16_t)(gps.course.deg() * 100) : 0;
    p.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    p.hdop_c = gps.hdop.isValid() ? (uint16_t)min(gps.hdop.hdop() * 100.0, 9999.0) : 9999;
    imu::Quaternion qz = qRel();        // mount-zeroed, same frame as the OLED
    p.qw = (int16_t)(qz.w() * 16384);
    p.qx = (int16_t)(qz.x() * 16384);
    p.qy = (int16_t)(qz.y() * 16384);
    p.qz = (int16_t)(qz.z() * 16384);
    p.linx_mg = (int16_t)(lastLin.x() / 9.80665f * 1000);
    p.liny_mg = (int16_t)(lastLin.y() / 9.80665f * 1000);
    p.linz_mg = (int16_t)(lastLin.z() / 9.80665f * 1000);
    p.maxG_mg = (int16_t)(maxG_g * 1000);
    p.marker = markerCount;
    // Window means (ver 0x05). Falls back to the newest sample on the packet
    // that races a boot — imuWinN can only be 0 before the first 10 ms tick.
    double invN = imuWinN ? 1.0 / imuWinN : 0.0;
    double mGx = imuWinN ? gyroSumX * invN : lastGyro.x();
    double mGy = imuWinN ? gyroSumY * invN : lastGyro.y();
    double mGz = imuWinN ? gyroSumZ * invN : lastGyro.z();
    double mAx = imuWinN ? accSumX * invN : lastAcc.x();
    double mAy = imuWinN ? accSumY * invN : lastAcc.y();
    double mAz = imuWinN ? accSumZ * invN : lastAcc.z();
    gyroSumX = gyroSumY = gyroSumZ = 0;
    accSumX = accSumY = accSumZ = 0;
    imuWinN = 0;
    p.gyrx_d16 = (int16_t)(mGx * 16);
    p.gyry_d16 = (int16_t)(mGy * 16);
    p.gyrz_d16 = (int16_t)(mGz * 16);
    p.calib = (uint8_t)((calSys << 6) | (calGyro << 4) | (calAcc << 2) | calMag);
    p.zeroCount = zeroCount;
    // Raw accel in mg. ±16 g would overflow an int16 in mg, but the BNO055's
    // accelerometer runs at ±4 g in fusion mode and clips there itself, so the
    // range can't be reached. Clamped anyway: a wrapped sign on the one channel
    // that exists to be trusted is worse than a saturated one.
    p.accx_mg = (int16_t)constrain(mAx / 9.80665 * 1000, -32000, 32000);
    p.accy_mg = (int16_t)constrain(mAy / 9.80665 * 1000, -32000, 32000);
    p.accz_mg = (int16_t)constrain(mAz / 9.80665 * 1000, -32000, 32000);
    p.quatRejects = (uint16_t)min(quatRejects, 65535UL);
    maxG_g = 0;

    // Bike CAN. The whole block stays zero unless a frame arrived recently, so
    // a disconnected transceiver or a sleeping bike reads as "absent" rather
    // than as a stale speed the app would happily plot.
    if (canOk && canS.lastRxMs && now - canS.lastRxMs < CAN_STALE_MS) {
      uint8_t st = canS.f202[0];
      // Regen is bits 2:0 of 0x490[0] — NOT the low nibble: bit 3 is the bottom
      // of the mode field there, so in Eco a nibble read returns 8+level. Only
      // ever 1..4, and the selector reverses at each end instead of wrapping,
      // so 0 never appears on a live bus and can mean "unknown".
      p.canFlags = 0x01 |                          // live
                   (((st >> 7) & 1) << 1) |        // kickstand down
                   (((st >> 4) & 3) << 2) |        // ride mode: 1 Eco, 2 Sport
                   ((canS.f490[0] & 0x07) << 4);   // regen level 1..4
      p.canSpeed_dkph = canU16(canS.f303, 0);
      p.canRpm        = canU16(canS.f203, 0);
      p.canPower_w    = canU16(canS.f203, 2);
      p.canCurrent_da = canU16(canS.f302, 4);
      p.canPack_dv    = canU16(canS.f101, 0);
      p.canSoc_pct    = canS.f401[0];
      p.canDemand     = canU16(canS.f202, 3);
      p.cellHi_mv     = canU16(canS.f201, 0);
      p.cellHi_idx    = canS.f201[4];
      p.cellLo_mv     = canU16(canS.f201, 2);
      p.cellLo_idx    = canS.f201[5];
    }

    chTele->setValue((uint8_t *)&p, sizeof(p));
    if (bleServer->getConnectedCount() > 0) chTele->notify();
  }

  // 1 Hz status + baro temperature + debug. Pressure and altitude moved to
  // the 5 Hz telemetry branch (ver 0x05) — temperature changes on weather
  // timescales and stays here.
  if (now - tStatus >= 1000) {
    tStatus = now;
    if (bmpOk) lastTempC = bmp.readTemperature();
    if (imuOk) {
      bno.getCalibration(&calSys, &calGyro, &calAcc, &calMag);
      // Persist once per boot, the first time the chip says it is there.
      // isFullyCalibrated() knows IMUPLUS wants accel and gyro but not mag.
      if (!calSaved && bno.isFullyCalibrated()) {
        adafruit_bno055_offsets_t off, prev;
        if (bno.getSensorOffsets(off)) {
          // Only write when they actually changed. Offsets are restored at
          // boot, so calibration reaches 3/3 within a second of every start —
          // writing unconditionally would burn an NVS cycle and stall the loop
          // ~115 ms on each power-up, which mid-ride costs telemetry packets.
          bool same = prefs.getBytes("bnooff", &prev, sizeof(prev)) == sizeof(prev)
                      && memcmp(&prev, &off, sizeof(off)) == 0;
          if (!same) {
            prefs.putBytes("bnooff", &off, sizeof(off));
            Serial.println("[cal] calibration reached 3/3 — offsets saved to flash");
          }
          calSaved = true;              // checked once per boot either way
        }
      }
    }
    StatusPacket s = {};
    s.ver = 0x01;
    s.fix = fixValid() ? 1 : 0;
    s.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    // There is no battery on this build and no fuel gauge, so this is a
    // constant sentinel, not a measurement. Production power is the BEC.
    s.battPct = 0xFF;                   // external supply
    s.hdop_c = gps.hdop.isValid() ? (uint16_t)min(gps.hdop.hdop() * 100.0, 9999.0) : 9999;
    s.uptime_s = now / 1000;
    s.temp_x10 = (int16_t)(lastTempC * 10);
    s.marker = markerCount;
    s.caps = BUILD_CAPS;                // tells the app this is the Full build
    s.otaState = otaActive ? (uint8_t)(1 + WiFi.softAPgetStationNum()) : 0;
    chStat->setValue((uint8_t *)&s, sizeof(s));
    if (bleServer->getConnectedCount() > 0) chStat->notify();
    float dbgR, dbgP, dbgY;
    quatToEuler(qRel(), dbgR, dbgP, dbgY);
    // btn: raw pin levels (1 = idle/high, 0 = pressed/low — 0 while unpressed
    // means the line is stuck) · lps: main-loop iterations since last line
    // (drops from tens of thousands to single digits when something blocks)
    bool canLive = canOk && canS.lastRxMs && now - canS.lastRxMs < CAN_STALE_MS;
    // cal: sys/gyro/accel — accel is the one that matters in IMUPLUS, and the
    // one that gates the zero. gz is raw yaw rate: on a straight road it should
    // sit near 0, and a steady non-zero reading is gyro bias, which is what
    // makes attitude wander. qrej counts non-unit quaternions dropped since
    // boot; a climbing number means the I2C run to the BNO055 is marginal.
    // Through logLine, so the last ~8 KB of these are readable at /log
    // whenever WiFi flashing mode is up — no USB cable needed.
    logLine("[dbg] fix=%d sats=%d view=%d conn=%d R=%+.1f P=%+.1f gz=%+.1f cal=%d%d%d qrej=%lu baro=%.1fm mark=%d btn=%d%d lps=%lu stall=%lu can=%s/%lu %u%% %.1fV",
            s.fix, s.sats, satsInView(), bleServer->getConnectedCount(),
            dbgR, dbgP, lastGyro.z(), calSys, calGyro, calAcc,
            (unsigned long)quatRejects, lastBaroAlt, markerCount,
            digitalRead(PIN_BUTTON), digitalRead(PIN_BUTTON2), loopsPerSec, maxLoopGapMs,
            canLive ? "live" : (canOk ? "idle" : "off"), (unsigned long)canS.frames,
            canLive ? canS.f401[0] : 0,
            canLive ? canU16(canS.f101, 0) / 10.0f : 0.0f);
    loopsPerSec = 0;
    maxLoopGapMs = 0;
  }

  // 2 Hz OLED
  if (oledOk && now - tOled >= 500) {
    tOled = now;
    refreshOled(now);
  }
}
