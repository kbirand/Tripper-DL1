// ============================================================
// Tripper Puck — LIGHT BUILD firmware
// ============================================================
// The rear unit only: IMU + barometer + bike CAN, streamed over BLE. No GPS,
// no OLED, no buttons — nothing on the handlebar. The phone is the recorder
// AND the position source; this firmware never claims a GPS fix.
//
//   100 Hz  BNO055 quaternion + linear accel (IMUPLUS), latch interval max-g
//   5 Hz    BLE telemetry notify (78-byte packed sample, same layout as Full)
//   1 Hz    BLE status notify, baro sample, serial debug line
//   ~92/s   Talaria CAN frames, listen-only (SN65HVD230 on D8/D9)
//
// WIRE-COMPATIBLE WITH THE FULL BUILD ON PURPOSE. Identical service UUIDs,
// identical 78-byte telemetry packet, identical 14-byte status packet. The app
// needs no branch: the GPS block simply arrives zeroed with both flags bits
// clear, which is the same "no fix" state the Full build reports indoors, and
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
  uint8_t  ver;          // 0x03
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
  int16_t  gyrx_d16, gyry_d16, gyrz_d16;   // sensor-frame gyro, deg/s * 16
  uint8_t  calib;        // bits 7:6 sys · 5:4 gyro · 3:2 accel · 1:0 mag
  // Increments on every mount zero the puck actually ACCEPTS, so the app can
  // tell "captured" from "the write never landed" or "refused". Wraps at 255
  // and restarts at 0 on reboot; the app watches for *any* change against a
  // baseline it takes when it sends, so neither matters.
  uint8_t  zeroCount;
};
static_assert(sizeof(TelemetryPacket) == 78, "telemetry packet size drifted");

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
};
static_assert(sizeof(StatusPacket) == 14, "status packet size drifted");

// ---------- devices ----------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP280 bmp(&Wire);

NimBLECharacteristic *chTele = nullptr, *chStat = nullptr;
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

// ---------- main loop ----------
void loop() {
  uint32_t now = millis();
  loopsPerSec++;
  if (lastLoopAt && now - lastLoopAt > maxLoopGapMs) maxLoopGapMs = now - lastLoopAt;
  lastLoopAt = now;

  canPoll();

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
    float g = lastLin.magnitude() / 9.80665f;
    if (g > maxG_g) maxG_g = g;
  }

  // 5 Hz telemetry
  if (now - tTele >= 200) {
    tTele = now;
    TelemetryPacket p = {};
    p.ver = 0x03;                       // 0x02 carried no gyro or calibration
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
    p.gyrx_d16 = (int16_t)(lastGyro.x() * 16);
    p.gyry_d16 = (int16_t)(lastGyro.y() * 16);
    p.gyrz_d16 = (int16_t)(lastGyro.z() * 16);
    p.calib = (uint8_t)((calSys << 6) | (calGyro << 4) | (calAcc << 2) | calMag);
    p.zeroCount = zeroCount;
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

  // 1 Hz status + baro + debug
  if (now - tStatus >= 1000) {
    tStatus = now;
    if (bmpOk) {
      lastPressPa = bmp.readPressure();
      lastTempC = bmp.readTemperature();
      lastBaroAlt = bmp.readAltitude(1013.25f);
    }
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
    Serial.printf("[dbg] conn=%d R=%+.1f P=%+.1f gz=%+.1f cal=%d%d%d qrej=%lu "
                  "baro=%.1fm mark=%d "
                  "lps=%lu stall=%lu can=%s/%lu %u%% %.1fV regen=%u\n",
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
