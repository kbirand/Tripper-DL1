// ============================================================
// Tripper Puck — LIGHT BUILD firmware
// ============================================================
// The rear unit only: IMU + barometer + bike CAN, streamed over BLE. No GPS,
// no OLED, no buttons — nothing on the handlebar. The phone is the recorder
// AND the position source; this firmware never claims a GPS fix.
//
//   100 Hz  BNO055 quaternion + linear accel (IMUPLUS), latch interval max-g,
//           gyro/raw-accel window sums (the packet sends the 200 ms mean)
//   5 Hz    BLE telemetry notify (88-byte packed sample, same layout as Full)
//           + BMP280 baro sample
//   1 Hz    BLE status notify, baro temperature, serial debug line
//   10 Hz   BLE raw notify — the 100 Hz IMU samples themselves, un-averaged,
//           batched ten at a time with the puck's own timestamp (see RawBatch)
//   ~92/s   Talaria CAN frames, listen-only (SN65HVD230 on D8/D9)
//
// WIRE-COMPATIBLE WITH THE FULL BUILD ON PURPOSE. Identical service UUIDs,
// identical 88-byte telemetry packet, identical 15-byte status packet, and the
// same 148-byte raw batch. (This paragraph said 78 and 14 for months after the
// packets grew — if you change a struct, change this line in the same commit.)
// The app needs no branch: the GPS block simply arrives zeroed with both flags
// bits clear, which is the same "no fix" state the Full build reports indoors, and
// the app already has to gate on those bits. Capability bits in the status
// packet say which build is on the other end (see StatusPacket::caps).
//
// What moves to the app in this build:
//   - position, speed, course      -> phone GPS
//   - marker                       -> control write 0x01 (no button here)
//   - mount zero                   -> control write 0x02 (no button here)
//   - all status//screens          -> the app's own UI (no OLED here)
//
// Keep this file's CAN decode, packet structs and UUIDs in step with
// tripper_puck.ino. Signal provenance lives in tools/talaria.dbc.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
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
// Settings toggle (control write 0x05 — this build has no buttons) starts a
// SoftAP; the computer joins it, and the puck appears as a NETWORK PORT in
// Arduino IDE (Tools -> Port -> "tripper-light at 192.168.4.1"). Upload works
// from the desk with the puck still on the bike, anywhere — no home-network
// credentials in the firmware, nothing to configure per location.
//
// The mode is deliberately NOT persistent: a bike power cycle always boots
// into normal riding with the WiFi radio off, so a toggle forgotten overnight
// can't leave an open AP riding around. The status packet reports the live
// state (see StatusPacket::otaState) so the app's toggle shows the truth.
// Needs a partition scheme with two app slots — the XIAO ESP32S3 default
// (8MB with OTA) is one. USB remains the recovery path if an update bricks.
#define OTA_AP_SSID     "Tripper-Light-OTA"
#define OTA_AP_PASS     "tripper-ota"   // WPA2 needs >= 8 chars
#define OTA_HOSTNAME    "tripper-light" // the network-port name in the IDE
#define OTA_PASSWORD    "tripper"       // IDE prompts for this before upload

bool otaActive = false;                 // SoftAP up, ArduinoOTA polled

// While the AP is up, the puck also serves diagnostics over HTTP — the same
// lines that go to USB serial, without the cable (this build is inside an
// enclosure on the bike; USB means dismounting):
//   http://192.168.4.1/     one-page live status
//   http://192.168.4.1/log  the last ~8 KB of debug lines, oldest first
// The ring lives in RAM from boot, so lines logged long BEFORE the toggle
// are still there when the network comes up — turn it on after something
// odd and read what the puck said at the time. Lost on power cycle.
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
// I2C on the XIAO's default pads, stated explicitly rather than relying on a
// bare Wire.begin(). Keep in step with tripper_puck.ino.
#define PIN_SDA      D4
#define PIN_SCL      D5
#define CAN_TX_GPIO  GPIO_NUM_7   // D8 -> SN65HVD230 CTX
#define CAN_RX_GPIO  GPIO_NUM_8   // D9 <- SN65HVD230 CRX
#define CAN_STALE_MS 2000UL       // no frame for this long = bus considered dead

static const char *SVC_UUID  = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70001";
static const char *TELE_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70002";
static const char *STAT_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70003";
static const char *CTRL_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70004";
// Raw 100 Hz flight recorder. Additive: a phone that never subscribes sees the
// puck behave exactly as before.
static const char *RAW_UUID  = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70005";

// Capability bits, reported in StatusPacket::caps. A Full build sends
// GPS|OLED|BUTTONS; this build sends CAN only. Zero would be an old firmware
// that predates the field, which is why HAS_CAN is a bit rather than assumed.
#define CAP_GPS      0x01
#define CAP_OLED     0x02
#define CAP_BUTTONS  0x04
#define CAP_CAN      0x08
#define BUILD_CAPS   (CAP_CAN)

// ---------- packets (little-endian, packed) ----------
// Byte-for-byte identical to tripper_puck.ino. Do not reorder either copy.
struct __attribute__((packed)) TelemetryPacket {
  uint8_t  ver;          // 0x06 — 0x05 plus accDev_mg on the tail. 0x05 was
                         // itself byte-identical to 0x04 and marked a semantics
                         // change: gyr*/acc* below are 200 ms window MEANS, not
                         // the newest 100 Hz sample
  uint8_t  flags;        // bit0 fix valid, bit1 time valid — both 0 in this
                         // build · bit2 IMU calibration usable
  uint32_t gpsTimeMs;    // always 0xFFFFFFFF here
  int32_t  lat_e7;       // always 0 here
  int32_t  lon_e7;       // always 0 here
  int32_t  alt_cm;       // always 0 here
  int32_t  baroAlt_cm;   // BMP280, std-atmosphere reference
  uint32_t press_pa;
  uint16_t speed_cmps;   // always 0 here (phone GPS supplies ground speed)
  uint16_t course_cdeg;  // always 0 here
  uint8_t  sats;         // always 0 here
  uint16_t hdop_c;       // always 9999 here
  int16_t  qw, qx, qy, qz;              // quat * 16384
  int16_t  linx_mg, liny_mg, linz_mg;   // linear accel, mg
  int16_t  maxG_mg;      // interval max |lin|, reset each packet
  uint8_t  marker;       // bumped by control write 0x01
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
  // the recording. Keep in step with tripper_puck.ino.
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
  // tripper_puck.ino.
  // Since ver 0x05: the 200 ms window mean, same reasoning as the gyro above.
  // The app low-passes this field anyway, so averaging on the puck only moves
  // the first (and heaviest) stage of that filter to where all the samples are.
  int16_t  accx_mg, accy_mg, accz_mg;   // sensor-frame accelerometer, mg
  // Non-unit getQuat() results dropped since boot, saturating. Printed to USB
  // since the first build and never recorded; a count that climbs mid-ride
  // means the attitude is being *held* across glitched I2C reads, not tracking.
  uint16_t quatRejects;
  // ---- accelerometer window PEAK (ver 0x06) --------------------------------
  // Largest |‖a‖ − 1 g| seen at 100 Hz across the window, in mg. The app's
  // estimator refuses to take its "down" reference from the accelerometer
  // while the magnitude is far from 1 g — braking bumps and drops point it
  // somewhere that isn't down. That test was being applied to the window MEAN,
  // which is the one number that cannot show it: the 100 Hz flight recorder
  // (2026-08-09 rides) measured |a| outside 0.8–1.2 g on 17–20% of moving
  // samples while the means were outside on 0.6–1.3%. The gate had been tuned
  // against a signal already smoothed into compliance, so it almost never
  // fired. A mean cannot be un-averaged; the peak has to be taken here.
  //
  // NOT the same as maxG_mg above, which latches the peak of the fusion's
  // LINEAR acceleration — a product of the very fusion this channel exists to
  // cross-check. This is the raw accelerometer, gravity included, untouched.
  uint16_t accDev_mg;
};
static_assert(sizeof(TelemetryPacket) == 88, "telemetry packet size drifted");

struct __attribute__((packed)) StatusPacket {
  uint8_t  ver;          // 0x01
  uint8_t  fix;          // always 0 here
  uint8_t  sats;         // always 0 here
  uint8_t  battPct;      // 0xFF = external supply (BEC or USB)
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
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP280 bmp(&Wire);

NimBLECharacteristic *chTele = nullptr, *chStat = nullptr, *chRaw = nullptr;
NimBLEServer *bleServer = nullptr;

bool imuOk = false, bmpOk = false, canOk = false;

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
// Window PEAK of |‖a‖ − 1 g| in g, latched at 100 Hz and reset each packet.
// The one thing about the window a mean is structurally unable to report.
double accDevMax = 0;
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
uint32_t tImu = 0, tTele = 0, tStatus = 0;
uint32_t loopsPerSec = 0;
uint32_t maxLoopGapMs = 0, lastLoopAt = 0;

volatile bool bleZeroReq = false;       // 0x02 control write, consumed in loop()
volatile bool bleMarkReq = false;       // 0x01 control write, consumed in loop()
volatile bool bleOtaMsg = false;        // 0x05 control write latched below
volatile uint8_t bleOtaOnB = 0;

Preferences prefs;
imu::Quaternion qRef;                   // mount reference; identity until zeroed

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

// ---------- BLE callbacks ----------
class SrvCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
    Serial.println("[ble] phone connected");
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
    Serial.printf("[ble] disconnected (reason %d), advertising again\n", reason);
  }
};

class CtrlCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
    NimBLEAttValue v = c->getValue();
    if (v.size() < 1) return;
    switch (v.data()[0]) {
      // In the Full build this is an ack for a marker the button already
      // counted. With no button here it is the only way a marker can happen,
      // so it originates one.
      case 0x01: bleMarkReq = true; break;
      case 0x02: bleZeroReq = true; break;                 // zero pitch/roll
      case 0x03: Serial.println("[ble] identify (no indicator on this build)"); break;
      // Accepted and logged so the app's unconditional write succeeds; there
      // is no OLED to invert and no trip screen to add.
      case 0x04: Serial.println("[ble] ride state (no display on this build)"); break;
      // 0x05 + [on u8]: WiFi flashing mode. No buttons on this build, so the
      // app's Settings toggle is the only way in and out — and a power cycle,
      // which always lands back in "off".
      case 0x05:
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

// ---------- CAN ----------
// Listen-only: the controller never transmits and never even ACKs, so the puck
// cannot influence the bike's bus whatever this firmware does. Failure here is
// non-fatal — the puck is an IMU logger first and must still boot and stream
// if the transceiver is unplugged.
bool canBringup() {
  twai_general_config_t g =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_len = 32;                  // ~92 frames/s; drained every loop pass
  g.alerts_enabled = TWAI_ALERT_NONE;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  return true;
}

// Drain whatever the controller has queued, bounded so a burst can never
// stretch a loop iteration.
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

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  // Never let debug prints block the loop: with a half-open USB CDC (host
  // opened the port but isn't draining it) each printf otherwise stalls for
  // its timeout.
  // If the console ever goes silent, replug the USB cable — the host CDC attach
  // state can get stuck (toggling DTR/RTS from a host script will do it) and no
  // firmware change fixes that. Measured stall with this at 0: 1 ms.
  Serial.setTxTimeoutMs(0);
  delay(1500);
  Serial.println("\n=== Tripper Puck firmware — LIGHT build (IMU + baro + CAN) ===");
  Serial.println("no GPS, no OLED, no buttons — phone supplies position and UI");
  // Into the wireless ring too, so /log always opens with how this boot began
  // — reset reason distinguishes a power cycle from a crash from an OTA.
  logLine("[boot] LIGHT build %s %s, reset reason %d",
          __DATE__, __TIME__, (int)esp_reset_reason());

  prefs.begin("puck", false);           // load the mount reference, if ever zeroed
  qRef = imu::Quaternion(prefs.getFloat("qw", 1.0f), prefs.getFloat("qx", 0.0f),
                         prefs.getFloat("qy", 0.0f), prefs.getFloat("qz", 0.0f));
  // References saved by builds that stored yaw carry a yaw from a *previous*
  // boot's origin, which is meaningless now. The tilt half is still good, so
  // strip rather than discard: an already-zeroed puck needs no re-zero.
  qRef = tiltOnly(qRef);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);                // BNO055 clock-stretching needs 100 kHz
  Wire.setTimeOut(1000);

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
  Serial.printf("IMU %s | BMP280 %s\n", imuOk ? "ok" : "FAIL", bmpOk ? "ok" : "FAIL");

  canOk = canBringup();
  Serial.printf("CAN %s (250k listen-only, D8/D9)\n", canOk ? "ok" : "FAIL");

  NimBLEDevice::init("Tripper-DL1");
  // The telemetry packet is 70 B, so the link must land above the 23 B default
  // ATT MTU. iOS negotiates 185 B; this states the requirement explicitly.
  NimBLEDevice::setMTU(247);
  // Full TX power: the link crosses a bike frame and a rider's body to a
  // pocketed phone — margin matters more than the ~20 mW it costs.
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
}

// The single path in and out of WiFi flashing mode, driven by control write
// 0x05. On: raise the SoftAP and start ArduinoOTA — the puck becomes network
// "Tripper-Light-OTA" at 192.168.4.1 and a network port in Arduino IDE for
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
               "Tripper puck - LIGHT build\n"
               "fw built %s %s (packet v0x05)\n"
               "uptime %lus, reset reason %d, free heap %u\n"
               "cal sys/gyro/accel %d/%d/%d, quatRejects %lu\n"
               "baro %.1f m, temp %.1f C\n"
               "CAN %s, %lu frames\n\n"
               "/log     recent debug lines\n"
               "/update  flash new firmware from a phone browser\n",
               __DATE__, __TIME__, millis() / 1000UL, (int)esp_reset_reason(),
               (unsigned)ESP.getFreeHeap(), calSys, calGyro, calAcc,
               (unsigned long)quatRejects, lastBaroAlt, lastTempC,
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

  canPoll();

  if (bleOtaMsg) {                      // 0x05: WiFi flashing mode toggle
    bleOtaMsg = false;
    otaSetMode(bleOtaOnB != 0);
  }
  if (otaActive) {
    ArduinoOTA.handle();
    httpd.handleClient();
  }

  if (bleMarkReq) {
    bleMarkReq = false;
    markerCount++;
    Serial.printf("[marker] #%d (app)\n", markerCount);
  }
  if (bleZeroReq) {
    bleZeroReq = false;
    if (imuOk) {
      // Refuse while the accelerometer is uncalibrated. In IMUPLUS it is the
      // only thing that knows where down is, so a zero taken against a bad one
      // is silently wrong for the whole ride. This build has no screen and no
      // buttons, so the app is the only place the rider can be told — it
      // checks the same calibration byte before it ever sends 0x02, and this
      // is the backstop behind that check.
      if (calAcc < 3 && !calRestored) {
        Serial.printf("[zero] refused: accel calibration %d/3 — leave the bike "
                      "still and level for a few seconds\n", calAcc);
      } else {
        qRef = tiltOnly(lastQuat);      // this orientation's TILT is the new zero
        saveQRef();
        zeroCount++;                    // the app's proof it landed
        Serial.println("[zero] mount reference captured & saved (app)");
      }
    }
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
    // Peak departure from 1 g over the window — see accDev_mg. Latched from the
    // same snapshot as the sums, so the mean and its peak describe one window.
    double accDev = fabs(lastAcc.magnitude() / 9.80665 - 1.0);
    if (accDev > accDevMax) accDevMax = accDev;
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
    // barely two distinct points to fit. The BMP280 runs 63 ms standby, so a
    // fresh reading always exists. Altitude comes from the pressure just read
    // rather than readAltitude(), which would issue a second I2C transaction.
    if (bmpOk) {
      lastPressPa = bmp.readPressure();
      lastBaroAlt = 44330.0f * (1.0f - powf(lastPressPa / 101325.0f, 0.1903f));
    }
    TelemetryPacket p = {};
    p.ver = 0x06;                       // 0x05 + the accelerometer window peak
    // No GPS in this build. flags stays 0 and the position block stays zeroed,
    // which is the same state the Full build reports before it gets a fix — so
    // the app's existing gating covers this without a special case.
    p.flags = (calAcc >= 3 || calRestored) ? 4 : 0;
    p.gpsTimeMs = 0xFFFFFFFF;
    p.hdop_c = 9999;
    p.baroAlt_cm = (int32_t)(lastBaroAlt * 100);
    p.press_pa = (uint32_t)lastPressPa;
    imu::Quaternion qz = qRel();        // mount-zeroed, same frame as the Full build
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
    // Saturates at 65 g of departure, which the ±4 g sensor cannot reach.
    p.accDev_mg = (uint16_t)constrain(accDevMax * 1000.0, 0, 65535);
    accDevMax = 0;
    maxG_g = 0;

    // Bike CAN. The whole block stays zero unless a frame arrived recently, so
    // a disconnected transceiver or a sleeping bike reads as "absent" rather
    // than as a stale speed the app would happily plot.
    if (canOk && canS.lastRxMs && now - canS.lastRxMs < CAN_STALE_MS) {
      uint8_t st = canS.f202[0];
      // Regen is bits 2:0 of 0x490[0] — NOT the low nibble: bit 3 is the bottom
      // of the mode field there, so in Eco a nibble read returns 8+level.
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
    s.fix = 0;                          // no GPS on this build
    s.sats = 0;
    s.battPct = 0xFF;                   // external supply
    s.hdop_c = 9999;
    s.uptime_s = now / 1000;
    s.temp_x10 = (int16_t)(lastTempC * 10);
    s.marker = markerCount;
    s.caps = BUILD_CAPS;                // tells the app this is the Light build
    s.otaState = otaActive ? (uint8_t)(1 + WiFi.softAPgetStationNum()) : 0;
    chStat->setValue((uint8_t *)&s, sizeof(s));
    if (bleServer->getConnectedCount() > 0) chStat->notify();

    float dbgR, dbgP, dbgY;
    quatToEuler(qRel(), dbgR, dbgP, dbgY);
    bool canLive = canOk && canS.lastRxMs && now - canS.lastRxMs < CAN_STALE_MS;
    // cal: sys/gyro/accel — accel is the one that matters in IMUPLUS, and the
    // one that gates the zero. gz is raw yaw rate: on a straight road it should
    // sit near 0, and a steady non-zero reading is gyro bias, which is what
    // makes attitude wander. qrej counts non-unit quaternions dropped since
    // boot; a climbing number means the I2C run to the BNO055 is marginal.
    // Through logLine, so the last ~8 KB of these are readable at /log
    // whenever WiFi flashing mode is up — no USB cable, no dismount.
    logLine("[dbg] conn=%d R=%+.1f P=%+.1f gz=%+.1f cal=%d%d%d qrej=%lu "
            "baro=%.1fm mark=%d "
            "lps=%lu stall=%lu can=%s/%lu %u%% %.1fV regen=%u",
            bleServer->getConnectedCount(), dbgR, dbgP, lastGyro.z(),
            calSys, calGyro, calAcc, (unsigned long)quatRejects,
            lastBaroAlt, markerCount, loopsPerSec, maxLoopGapMs,
            canLive ? "live" : (canOk ? "idle" : "off"),
            (unsigned long)canS.frames,
            canLive ? canS.f401[0] : 0,
            canLive ? canU16(canS.f101, 0) / 10.0f : 0.0f,
            canLive ? (canS.f490[0] & 0x07) : 0);
    loopsPerSec = 0;
    maxLoopGapMs = 0;
  }
}
