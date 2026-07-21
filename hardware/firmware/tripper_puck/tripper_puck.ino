// ============================================================
// Tripper Puck — production firmware, e-bike / BLE edition
// ============================================================
// Stateless BLE sensor: boots with the bike, streams telemetry to the
// Tripper app. No SD, no battery management — the phone is the recorder.
//
//   100 Hz  BNO055 quaternion + linear accel (IMUPLUS), latch interval max-g
//   5 Hz    GPS epochs (module pre-configured; re-configured at every boot)
//   5 Hz    BLE telemetry notify (50-byte packed sample)
//   1 Hz    BLE status notify, baro sample, serial debug line
//   2 Hz    OLED refresh — alternates every 5 s: GPS clock / live data
//
// Status lives on the OLED (fix dot, link state, MARK/ZEROED splashes) —
// no separate status LED.
// Button (D1→GND): marker — bumps a counter in the telemetry packet.
// Button 2 (D2→GND): click = hold/release the screen cycle (thin border =
// held) · long-hold = zero roll/pitch/yaw at the mounted orientation —
// reference quaternion saved to flash, survives reboots, and the BLE
// telemetry quaternion is re-referenced the same way.
// Control writes: 0x01 = marker ack flash · 0x03 = identify (LED rainbow +
// OLED invert, for picking the right device in a scanner app).

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <NimBLEDevice.h>
#include <Preferences.h>

// ---------- pins & constants ----------
#define PIN_BUTTON   D1
#define PIN_BUTTON2  D2        // screen hold / attitude zero
#define TZ_HOURS     3         // display offset for the clock screen (TRT, no DST)
#define CLICK_MAX_MS 600       // release under this = a click (toggle hold)
#define ZERO_SHOW_MS 800       // held past this = show the zero progress bar
#define ZERO_HOLD_MS 10000UL   // held to here = capture the mount reference

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
};
static_assert(sizeof(TelemetryPacket) == 50, "telemetry packet size drifted");

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

bool imuOk = false, bmpOk = false, oledOk = false;

// ---------- live state ----------
volatile uint8_t markerCount = 0;
float maxG_g = 0;                       // latched between telemetry packets
imu::Quaternion lastQuat;
imu::Vector<3> lastLin;
float lastPressPa = 0, lastTempC = 0, lastBaroAlt = 0;
uint32_t identifyUntil = 0, splashUntil = 0;
uint32_t tImu = 0, tTele = 0, tStatus = 0, tOled = 0;
uint32_t btnLastEdge = 0;
bool btnWasDown = false;

// screen hold + mount-zero (button 2)
Preferences prefs;
imu::Quaternion qRef;                   // mount reference; identity until zeroed
bool screenHold = false;
int  heldScreen = 0;
uint32_t zeroSplashUntil = 0;
uint32_t btn2DownAt = 0, btn2LastEdge = 0;
bool btn2WasDown = false, zeroFired = false;

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

bool gpsBringup() {
  Serial1.setRxBufferSize(2048);        // survive OLED/BLE stalls at 115200
  Serial1.begin(115200, SERIAL_8N1, D7, D6);
  if (nmeaAlive(1200)) return true;     // already configured (BBR intact)
  Serial1.end();
  Serial1.begin(9600, SERIAL_8N1, D7, D6);
  const uint8_t rate[6] = {0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
  sendUBX(0x06, 0x08, rate, 6);
  delay(150);
  const uint8_t prt[20] = {0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                           0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00,
                           0x00, 0x00, 0x00, 0x00};
  sendUBX(0x06, 0x00, prt, 20);
  Serial1.flush();
  delay(250);
  Serial1.end();
  Serial1.begin(115200, SERIAL_8N1, D7, D6);
  return nmeaAlive(1500);
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
      case 0x03: identifyUntil = millis() + 2000; break;                // identify
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

void refreshOled(uint32_t now) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  uint32_t heldMs = (btn2WasDown && !zeroFired) ? now - btn2DownAt : 0;
  if (heldMs > ZERO_SHOW_MS)        drawZeroProgress(heldMs);
  else if (now < zeroSplashUntil)   drawZeroedSplash();
  else if (now < splashUntil)       drawMarkerSplash();
  else {
    int idx = screenHold ? heldScreen : (int)((now / 5000) % 2);  // 5 s clock / 5 s data
    if (idx == 0) drawClockScreen(now); else drawDataScreen(now);
    if (screenHold) oled.drawRect(0, 0, 128, 32, SSD1306_WHITE);  // border = pinned
  }
  oled.invertDisplay(now < identifyUntil);
  Wire.setClock(400000);                // burst the frame out fast (OLED-only transfer)
  oled.display();
  Wire.setClock(100000);                // back to the BNO055-safe speed
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== Tripper Puck firmware (e-bike/BLE) ===");

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);

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
  Serial.printf("GPS %s (115200/5Hz)\n", gpsOk ? "ok" : "NOT RESPONDING");

  NimBLEDevice::init("Tripper-DL1");
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
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->setName("Tripper-DL1");
  adv->addServiceUUID(SVC_UUID);
  adv->start();
  Serial.println("[ble] advertising as Tripper-DL1");

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

  while (Serial1.available()) gps.encode(Serial1.read());

  // marker button, 50 ms debounce
  bool down = digitalRead(PIN_BUTTON) == LOW;
  if (down != btnWasDown && now - btnLastEdge > 50) {
    btnLastEdge = now;
    btnWasDown = down;
    if (down) {
      markerCount++;
      splashUntil = now + 800;
      tOled = 0;                        // redraw immediately with the splash
      Serial.printf("[marker] #%d\n", markerCount);
    }
  }

  // button 2: click = hold/release screen, long-hold = zero attitude
  bool down2 = digitalRead(PIN_BUTTON2) == LOW;
  if (down2 && !btn2WasDown && now - btn2LastEdge > 50) {
    btn2LastEdge = now;
    btn2WasDown = true;
    btn2DownAt = now;
    zeroFired = false;
  }
  if (down2 && btn2WasDown && !zeroFired && now - btn2DownAt >= ZERO_HOLD_MS) {
    zeroFired = true;
    qRef = lastQuat;                    // this orientation is the new zero
    saveQRef();
    zeroSplashUntil = now + 1500;
    tOled = 0;
    Serial.println("[zero] mount reference captured & saved");
  }
  if (!down2 && btn2WasDown && now - btn2LastEdge > 50) {
    btn2LastEdge = now;
    btn2WasDown = false;
    if (!zeroFired && now - btn2DownAt < CLICK_MAX_MS) {
      screenHold = !screenHold;
      if (screenHold) heldScreen = (int)((now / 5000) % 2);
      tOled = 0;
      Serial.printf("[screen] %s\n", screenHold ? "held" : "cycling");
    }
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
    p.ver = 0x01;
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
    Serial.printf("[dbg] fix=%d sats=%d view=%d conn=%d R=%+.1f P=%+.1f qW=%.3f baro=%.1fm mark=%d\n",
                  s.fix, s.sats, satsInView(), bleServer->getConnectedCount(),
                  dbgR, dbgP, lastQuat.w(), lastBaroAlt, markerCount);
  }

  // 2 Hz OLED
  if (oledOk && now - tOled >= 500) {
    tOled = now;
    refreshOled(now);
  }
}
