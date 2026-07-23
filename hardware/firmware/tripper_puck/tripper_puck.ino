// ============================================================
// Tripper Puck — production firmware, e-bike / BLE edition
// ============================================================
// Stateless BLE sensor: boots with the bike, streams telemetry to the
// Tripper app. No SD, no battery management — the phone is the recorder.
//
//   100 Hz  BNO055 quaternion + linear accel (IMUPLUS), latch interval max-g
//   5 Hz    GPS epochs (module pre-configured; re-configured at every boot)
//   5 Hz    BLE telemetry notify (70-byte packed sample)
//   1 Hz    BLE status notify, baro sample, serial debug line
//   2 Hz    OLED refresh — clock / live data / bike CAN (/ trip time)
//   ~92/s   Talaria CAN frames, listen-only (SN65HVD230 on D8/D9)
//
// Status lives on the OLED (fix dot, link state, MARK/ZEROED splashes) —
// no separate status LED.
// Button 1 (D1→GND): click = step to the next screen · 3 s hold = toggle
// auto-cycling (thin border = cycling). Screens do NOT rotate on their own by
// default — on a moving bike the screen you picked should stay put.
// Button 2 (D2→GND): click = marker, bumps a counter in the telemetry packet ·
// 10 s hold = zero roll/pitch/yaw at the mounted orientation — reference
// quaternion saved to flash, survives reboots, and the BLE telemetry
// quaternion is re-referenced the same way.
// Control writes: 0x01 = marker ack flash · 0x02 = zero roll/pitch/yaw (same
// as the button 2 long-hold, triggered from the app) · 0x03 = identify (LED
// rainbow + OLED invert, for picking the right device in a scanner app) ·
// 0x04 + [active u8][elapsed s u32 LE] = ride state — while active the OLED
// runs inverted and a third screen (trip time) joins the cycle. The app
// re-sends it on every reconnect, so a link flap or puck reboot mid-ride
// self-heals.

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

// ---------- pins & constants ----------
#define PIN_BUTTON   D1
#define PIN_BUTTON2  D2        // screen hold / attitude zero
#define CAN_TX_GPIO  GPIO_NUM_7   // D8 -> SN65HVD230 CTX
#define CAN_RX_GPIO  GPIO_NUM_8   // D9 <- SN65HVD230 CRX
#define CAN_STALE_MS 2000UL       // no frame for this long = bus considered dead
#define TZ_HOURS     3         // display offset for the clock screen (TRT, no DST)
#define CLICK_MAX_MS 600       // release under this = a click, not a hold
#define ZERO_SHOW_MS 800       // held past this = show the zero progress bar
#define ZERO_HOLD_MS 10000UL   // button 2 held to here = capture the mount reference
#define CYCLE_HOLD_MS 3000UL   // button 1 held to here = toggle auto-cycling
#define SCREEN_MS    5000UL    // dwell per screen while auto-cycling

static const char *SVC_UUID  = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70001";
static const char *TELE_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70002";
static const char *STAT_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70003";
static const char *CTRL_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70004";

// ---------- packets (little-endian, packed) ----------
struct __attribute__((packed)) TelemetryPacket {
  uint8_t  ver;          // 0x01
  uint8_t  flags;        // bit0 fix valid, bit1 time valid
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
};
static_assert(sizeof(TelemetryPacket) == 70, "telemetry packet size drifted");

struct __attribute__((packed)) StatusPacket {
  uint8_t  ver;          // 0x01
  uint8_t  fix;
  uint8_t  sats;
  uint8_t  battPct;      // 0xFF = external USB power
  uint16_t hdop_c;
  uint32_t uptime_s;
  int16_t  temp_x10;     // BMP280 °C * 10
  uint8_t  marker;
  uint8_t  reserved;
};
static_assert(sizeof(StatusPacket) == 14, "status packet size drifted");

// ---------- devices ----------
Adafruit_BNO055   bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP280   bmp(&Wire);
Adafruit_SSD1306  oled(128, 32, &Wire, -1);
TinyGPSPlus       gps;
TinyGPSCustom     gsvGP(gps, "GPGSV", 3), gsvGL(gps, "GLGSV", 3), gsvGB(gps, "GBGSV", 3);

NimBLECharacteristic *chTele = nullptr, *chStat = nullptr;
NimBLEServer *bleServer = nullptr;

bool imuOk = false, bmpOk = false, oledOk = false, canOk = false;

// ---------- bike CAN (Talaria, 250 kbit/s, listen-only) ----------
// Latest payload of each decoded message. Signal offsets and the evidence
// behind every one of them live in tools/talaria.dbc.
struct CanState {
  uint8_t  f101[8], f201[8], f202[8], f203[8], f302[8], f303[8], f401[8];
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

void saveQRef() {
  prefs.putFloat("qw", qRef.w()); prefs.putFloat("qx", qRef.x());
  prefs.putFloat("qy", qRef.y()); prefs.putFloat("qz", qRef.z());
}

imu::Quaternion qRel() {                // qRef⁻¹ ⊗ q (unit quat: conj == inverse)
  double rw = qRef.w(), rx = -qRef.x(), ry = -qRef.y(), rz = -qRef.z();
  const imu::Quaternion &q = lastQuat;
  return imu::Quaternion(
      rw * q.w() - rx * q.x() - ry * q.y() - rz * q.z(),
      rw * q.x() + rx * q.w() + ry * q.z() - rz * q.y(),
      rw * q.y() - rx * q.z() + ry * q.w() + rz * q.x(),
      rw * q.z() + rx * q.y() - ry * q.x() + rz * q.w());
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
  Wire.setClock(400000);                // burst the frame out fast (OLED-only transfer)
  oled.display();
  Wire.setClock(100000);                // back to the BNO055-safe speed
}

// ---------- setup ----------
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

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), isrBtn1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON2), isrBtn2, CHANGE);

  prefs.begin("puck", false);           // load the mount reference, if ever zeroed
  qRef = imu::Quaternion(prefs.getFloat("qw", 1.0f), prefs.getFloat("qx", 0.0f),
                         prefs.getFloat("qy", 0.0f), prefs.getFloat("qz", 0.0f));

  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeOut(1000);

  imuOk = bno.begin(OPERATION_MODE_IMUPLUS);
  bmpOk = bmp.begin(0x76);
  if (bmpOk)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_63);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C) || oled.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  Serial.printf("IMU %s | BMP280 %s | OLED %s\n",
                imuOk ? "ok" : "FAIL", bmpOk ? "ok" : "FAIL", oledOk ? "ok" : "FAIL");

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

// ---------- main loop ----------
void loop() {
  uint32_t now = millis();
  loopsPerSec++;
  if (lastLoopAt && now - lastLoopAt > maxLoopGapMs) maxLoopGapMs = now - lastLoopAt;
  lastLoopAt = now;

  while (Serial1.available()) gps.encode(Serial1.read());
  canPoll();

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
    qRef = lastQuat;                    // this orientation is the new zero
    saveQRef();
    zeroSplashUntil = now + 1500;
    tOled = 0;
    Serial.println("[zero] mount reference captured & saved");
  }
  if (bleZeroReq) {                     // 0x02 from the app — same capture as the hold
    bleZeroReq = false;
    if (imuOk) {
      qRef = lastQuat;
      saveQRef();
      zeroSplashUntil = now + 1500;
      tOled = 0;
      Serial.println("[zero] mount reference captured & saved (app)");
    }
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
    lastQuat = bno.getQuat();
    lastLin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    float g = lastLin.magnitude() / 9.80665f;
    if (g > maxG_g) maxG_g = g;
  }

  // 5 Hz telemetry
  if (now - tTele >= 200) {
    tTele = now;
    TelemetryPacket p = {};
    p.ver = 0x02;                       // 0x01 was the 50-byte, pre-CAN packet
    p.flags = (fixValid() ? 1 : 0) | (gps.time.isValid() ? 2 : 0);
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
    maxG_g = 0;

    // Bike CAN. The whole block stays zero unless a frame arrived recently, so
    // a disconnected transceiver or a sleeping bike reads as "absent" rather
    // than as a stale speed the app would happily plot.
    if (canOk && canS.lastRxMs && now - canS.lastRxMs < CAN_STALE_MS) {
      uint8_t st = canS.f202[0];
      p.canFlags = 0x01 |                          // live
                   (((st >> 7) & 1) << 1) |        // kickstand down
                   (((st >> 4) & 3) << 2);         // ride mode: 1 Eco, 2 Sport
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
    StatusPacket s = {};
    s.ver = 0x01;
    s.fix = fixValid() ? 1 : 0;
    s.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    s.battPct = 0xFF;                   // external USB power
    s.hdop_c = gps.hdop.isValid() ? (uint16_t)min(gps.hdop.hdop() * 100.0, 9999.0) : 9999;
    s.uptime_s = now / 1000;
    s.temp_x10 = (int16_t)(lastTempC * 10);
    s.marker = markerCount;
    chStat->setValue((uint8_t *)&s, sizeof(s));
    if (bleServer->getConnectedCount() > 0) chStat->notify();
    float dbgR, dbgP, dbgY;
    quatToEuler(qRel(), dbgR, dbgP, dbgY);
    // btn: raw pin levels (1 = idle/high, 0 = pressed/low — 0 while unpressed
    // means the line is stuck) · lps: main-loop iterations since last line
    // (drops from tens of thousands to single digits when something blocks)
    bool canLive = canOk && canS.lastRxMs && now - canS.lastRxMs < CAN_STALE_MS;
    Serial.printf("[dbg] fix=%d sats=%d view=%d conn=%d R=%+.1f P=%+.1f qW=%.3f baro=%.1fm mark=%d btn=%d%d lps=%lu stall=%lu can=%s/%lu %u%% %.1fV\n",
                  s.fix, s.sats, satsInView(), bleServer->getConnectedCount(),
                  dbgR, dbgP, lastQuat.w(), lastBaroAlt, markerCount,
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
