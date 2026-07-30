// ============================================================
// Tripper Puck — bench bring-up rig: IMU + baro + bike CAN only
// ============================================================
// The production firmware assumes the whole harness is present. This one is
// for the half-built puck on the bench: no OLED, no buttons, no GPS. What it
// keeps is everything needed to prove the sensor and CAN halves are soldered
// correctly, plus the same BLE contract as production so the phone app still
// works once USB has to come out (see "two-stage test" below).
//
//   100 Hz  BNO055 quaternion + linear accel (IMUPLUS), interval max-g latch
//   1 Hz    serial health report, BMP280 sample, BLE status notify
//   5 Hz    BLE telemetry notify — same 70-byte v0x02 packet as production
//   ~92/s   Talaria CAN frames, listen-only (SN65HVD230 on D8/D9)
//
// Deliberate differences from tripper_puck.ino, all in service of bring-up:
//
//   * The BNO055 is driven directly over Wire instead of through
//     Adafruit_BNO055. On a bare bus the ESP32's weak internal pull-ups let
//     the controller sample a phantom ACK, and a library begin() that only
//     checks "did something answer" reports success against a chip that
//     isn't there. Talking to the registers ourselves means every claim in
//     the report below is backed by a byte that actually came off the bus.
//   * Every 100 Hz read is checked and counted. A cold solder joint shows up
//     as a climbing err= instead of quietly corrupt quaternions.
//   * Missing sensors are retried every 3 s, so you can solder a wire with
//     the rig running and watch it come alive without a reboot.
//   * Boot scans the I2C bus and prints what it finds. More than a handful of
//     addresses answering means the bus is floating, not populated.
//
// Two-stage test — the bike's CAN ground reaches the puck through the BEC,
// and USB 5 V collides with the BEC's 5 V on VUSB, so these cannot both be
// connected (README "Reading the bike's CAN bus"):
//
//   Stage 1  USB, CANH/CANL unplugged from the bike. Proves the IMU, the
//            baro and that the TWAI driver installs. Watch it over serial.
//   Stage 2  BEC on, USB out, bike live. The only way to prove frames are
//            actually arriving. Serial is gone, so read it on the phone —
//            the BLE service, UUIDs and packet layout are byte-identical to
//            production, so the Tripper app connects to this build unchanged.
//            GPS fields stay zeroed with the fix bit clear.

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <driver/twai.h>

// ---------- pins & constants ----------
// I2C on the XIAO's default pads. SDA lived on D3 for a while because the
// first board's D4 pad was clamped low — it read 0 even against the internal
// pull-up with nothing attached, so SDA could never idle high and every sensor
// looked dead. That board was replaced and D4 is back in use.
//
// The pad sweep below is what diagnosed it, and it still runs on every boot: a
// pad the chip controls follows both internal pulls, so a clamped one stands
// out against its neighbours. Keeping the sweep is the point of this sketch —
// if the symptom ever comes back, it names the pin in one line.
#define PIN_SDA      D4
#define PIN_SCL      D5
#define CAN_TX_GPIO  GPIO_NUM_7   // D8 -> SN65HVD230 CTX
#define CAN_RX_GPIO  GPIO_NUM_8   // D9 <- SN65HVD230 CRX
#define CAN_STALE_MS 2000UL       // no frame for this long = bus considered dead
#define RETRY_MS     3000UL       // re-probe a missing sensor this often
#define BNO_ADDR     0x28
#define BMP_ADDR     0x76

// BNO055 registers (page 0)
#define BNO_CHIP_ID     0x00
#define BNO_PAGE_ID     0x07
#define BNO_QUA_W_LSB   0x20      // 0x20..0x2D is quat(8) then linear accel(6)
#define BNO_TEMP        0x34
#define BNO_CALIB_STAT  0x35
#define BNO_ST_RESULT   0x36
#define BNO_SYS_STATUS  0x39
#define BNO_SYS_ERR     0x3A
#define BNO_UNIT_SEL    0x3B
#define BNO_OPR_MODE    0x3D
#define BNO_PWR_MODE    0x3E
#define BNO_SYS_TRIGGER 0x3F
#define BNO_ID_EXPECT   0xA0
#define BNO_MODE_CONFIG 0x00
#define BNO_MODE_IMU    0x08      // IMUPLUS: 6-axis, no magnetometer

static const char *SVC_UUID  = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70001";
static const char *TELE_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70002";
static const char *STAT_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70003";
static const char *CTRL_UUID = "8E7C1A20-0F5A-4B9C-9C90-54B1D2A70004";

// ---------- packets ----------
// Byte-identical to tripper_puck.ino. Keep them that way: the phone app
// parses by offset, and stage 2 is the only way to see CAN frames arrive.
struct __attribute__((packed)) TelemetryPacket {
  uint8_t  ver;
  uint8_t  flags;        // bit0 fix valid, bit1 time valid — both always 0 here
  uint32_t gpsTimeMs;
  int32_t  lat_e7;
  int32_t  lon_e7;
  int32_t  alt_cm;
  int32_t  baroAlt_cm;
  uint32_t press_pa;
  uint16_t speed_cmps;
  uint16_t course_cdeg;
  uint8_t  sats;
  uint16_t hdop_c;
  int16_t  qw, qx, qy, qz;
  int16_t  linx_mg, liny_mg, linz_mg;
  int16_t  maxG_mg;
  uint8_t  marker;
  uint8_t  canFlags;
  uint16_t canSpeed_dkph;
  uint16_t canRpm;
  uint16_t canPower_w;
  uint16_t canCurrent_da;
  uint16_t canPack_dv;
  uint8_t  canSoc_pct;
  uint16_t canDemand;
  uint16_t cellHi_mv;
  uint8_t  cellHi_idx;
  uint16_t cellLo_mv;
  uint8_t  cellLo_idx;
};
static_assert(sizeof(TelemetryPacket) == 70, "telemetry packet size drifted");

struct __attribute__((packed)) StatusPacket {
  uint8_t  ver;
  uint8_t  fix;
  uint8_t  sats;
  uint8_t  battPct;
  uint16_t hdop_c;
  uint32_t uptime_s;
  int16_t  temp_x10;
  uint8_t  marker;
  uint8_t  reserved;
};
static_assert(sizeof(StatusPacket) == 14, "status packet size drifted");

// ---------- devices ----------
Adafruit_BMP280 bmp(&Wire);
NimBLECharacteristic *chTele = nullptr, *chStat = nullptr;
NimBLEServer *bleServer = nullptr;
Preferences prefs;

bool imuOk = false, bmpOk = false, canOk = false;

// ---------- quaternion ----------
// Adafruit_BNO055 is not linked here, so imu::Quaternion comes with it. This
// is the same math the production firmware runs, spelled out locally.
struct Quat {
  float w = 1, x = 0, y = 0, z = 0;
};

Quat qRef;                              // mount reference; identity until zeroed
Quat lastQuat;
float linX = 0, linY = 0, linZ = 0;     // linear accel, m/s^2
float maxG_g = 0;                       // latched between telemetry packets

Quat qRel() {                           // qRef^-1 (x) q  (unit quat: conj == inverse)
  const float rw = qRef.w, rx = -qRef.x, ry = -qRef.y, rz = -qRef.z;
  const Quat &q = lastQuat;
  Quat o;
  o.w = rw * q.w - rx * q.x - ry * q.y - rz * q.z;
  o.x = rw * q.x + rx * q.w + ry * q.z - rz * q.y;
  o.y = rw * q.y - rx * q.z + ry * q.w + rz * q.x;
  o.z = rw * q.z + rx * q.y - ry * q.x + rz * q.w;
  return o;
}

void quatToEuler(const Quat &q, float &rollDeg, float &pitchDeg, float &yawDeg) {
  double sinr = 2.0 * (q.w * q.x + q.y * q.z);
  double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  rollDeg = atan2(sinr, cosr) * 57.29578;
  double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  if (sinp > 1.0) sinp = 1.0;
  if (sinp < -1.0) sinp = -1.0;
  pitchDeg = asin(sinp) * 57.29578;
  double siny = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yawDeg = atan2(siny, cosy) * 57.29578;
}

void saveQRef() {
  prefs.putFloat("qw", qRef.w); prefs.putFloat("qx", qRef.x);
  prefs.putFloat("qy", qRef.y); prefs.putFloat("qz", qRef.z);
}

// ---------- raw I2C ----------
bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Look at SDA/SCL as plain inputs before asking the I2C driver to touch them.
//
// This is not belt-and-braces: probing a bus that nothing is driving walks the
// IDF master driver into s_i2c_hw_fsm_reset() -> s_i2c_master_clear_bus(),
// which races its own ISR and panics the core (StoreProhibited inside
// i2c_isr_receive_handler). One boot scan survives it; a retry every 3 s with
// BLE running does not. So the retry only touches Wire when something is
// actually holding the bus up.
//
// A powered module drives both lines high through its own pull-ups, so the
// line states also say *why* the bus is dead, which is the diagnostic
// i2c_diag exists to give. Side effect worth having: Wire is torn down and
// re-begun each call, which clears any half-stuck FSM from the last attempt.
bool busLinesHigh(bool verbose) {
  Wire.end();
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delayMicroseconds(200);
  int sdaF = digitalRead(PIN_SDA), sclF = digitalRead(PIN_SCL);
  if (verbose) {
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
    delayMicroseconds(200);
    int sdaP = digitalRead(PIN_SDA), sclP = digitalRead(PIN_SCL);
    Serial.printf("[i2c] lines floating SDA=%d SCL=%d | pulled up SDA=%d SCL=%d\n",
                  sdaF, sclF, sdaP, sclP);
    if (sdaF && sclF)
      Serial.println("[i2c] both float high — a powered module is driving the bus");
    else if (!sdaP || !sclP)
      Serial.println("[i2c] a line stays low even pulled up — clamped: shorted, "
                     "or a module with no 3V3");
    else
      Serial.println("[i2c] lines float low — module not connected or unpowered "
                     "(check the 3V3 rail first)");
  }
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(1000);
  return sdaF && sclF;
}

// ---------- IMU read stats ----------
uint32_t imuReads = 0, imuErrs = 0, imuMaxUs = 0;
uint32_t imuReadsWin = 0, imuErrsWin = 0;   // reset each report

// Bring the BNO055 up from scratch and prove it answered. Returns false
// without leaving state behind, so the 3 s retry can just call it again.
bool bnoBringup() {
  uint8_t id = 0;
  if (!readRegs(BNO_ADDR, BNO_CHIP_ID, &id, 1)) return false;
  if (id != BNO_ID_EXPECT) {
    Serial.printf("[imu] chip ID 0x%02X, expected 0x%02X — wrong device or noise\n",
                  id, BNO_ID_EXPECT);
    return false;
  }
  writeReg(BNO_ADDR, BNO_OPR_MODE, BNO_MODE_CONFIG);
  delay(25);
  writeReg(BNO_ADDR, BNO_SYS_TRIGGER, 0x20);            // reset
  delay(700);
  // The chip drops off the bus across the reset; wait for it to answer again.
  for (int i = 0; i < 10 && !(readRegs(BNO_ADDR, BNO_CHIP_ID, &id, 1) && id == BNO_ID_EXPECT); i++)
    delay(20);
  if (id != BNO_ID_EXPECT) return false;
  writeReg(BNO_ADDR, BNO_PWR_MODE, 0x00);               // normal
  delay(10);
  writeReg(BNO_ADDR, BNO_PAGE_ID, 0x00);
  writeReg(BNO_ADDR, BNO_UNIT_SEL, 0x00);               // m/s^2, deg, degC
  writeReg(BNO_ADDR, BNO_SYS_TRIGGER, 0x00);
  delay(10);
  if (!writeReg(BNO_ADDR, BNO_OPR_MODE, BNO_MODE_IMU)) return false;
  delay(20);
  return true;
}

// Quaternion and linear accel are contiguous (0x20..0x2D), so one 14-byte
// burst gets both — half the transactions, half the chances to fail.
bool bnoRead() {
  uint8_t b[14];
  uint32_t t0 = micros();
  bool ok = readRegs(BNO_ADDR, BNO_QUA_W_LSB, b, 14);
  uint32_t dt = micros() - t0;
  if (dt > imuMaxUs) imuMaxUs = dt;
  imuReads++; imuReadsWin++;
  if (!ok) { imuErrs++; imuErrsWin++; return false; }

  auto i16 = [&](int i) { return (int16_t)((uint16_t)b[i] | ((uint16_t)b[i + 1] << 8)); };
  const float qs = 1.0f / 16384.0f;
  Quat q;
  q.w = i16(0) * qs; q.x = i16(2) * qs; q.y = i16(4) * qs; q.z = i16(6) * qs;

  // An all-zero quaternion is what a NACKed-but-unreported read looks like;
  // a non-unit one means corrupted bytes. Neither is a valid orientation, so
  // count it as a bus error rather than feeding it to the app.
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 0.9f || n > 1.1f) { imuErrs++; imuErrsWin++; return false; }

  lastQuat = q;
  const float as = 1.0f / 100.0f;       // BNO055 LSB = 1/100 m/s^2
  linX = i16(8) * as; linY = i16(10) * as; linZ = i16(12) * as;
  float g = sqrtf(linX * linX + linY * linY + linZ * linZ) / 9.80665f;
  if (g > maxG_g) maxG_g = g;
  return true;
}

// ---------- bike CAN (Talaria, 250 kbit/s, listen-only) ----------
struct CanState {
  uint8_t  f101[8], f201[8], f202[8], f203[8], f302[8], f303[8], f401[8];
  uint32_t lastRxMs = 0;
  uint32_t frames = 0, framesWin = 0;
} canS;

// Every ID the bus emits, not just the seven we decode — if the wiring is
// right but the DBC is stale, this is where that shows up.
#define CENSUS_MAX 24
struct { uint32_t id, count; } census[CENSUS_MAX];
uint8_t censusN = 0;

void censusAdd(uint32_t id) {
  for (uint8_t i = 0; i < censusN; i++)
    if (census[i].id == id) { census[i].count++; return; }
  if (censusN < CENSUS_MAX) { census[censusN].id = id; census[censusN].count = 1; censusN++; }
}

static inline uint16_t canU16(const uint8_t *d, int off) {
  return (uint16_t)d[off] | ((uint16_t)d[off + 1] << 8);
}

bool canBringup() {
  twai_general_config_t g =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_len = 32;
  g.alerts_enabled = TWAI_ALERT_NONE;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  return true;
}

void canPoll() {
  if (!canOk) return;
  twai_message_t m;
  for (int i = 0; i < 16 && twai_receive(&m, 0) == ESP_OK; i++) {
    if (m.extd || m.rtr || m.data_length_code < 8) continue;
    censusAdd(m.identifier);
    switch (m.identifier) {
      case 0x101: memcpy(canS.f101, m.data, 8); break;
      case 0x201: memcpy(canS.f201, m.data, 8); break;
      case 0x202: memcpy(canS.f202, m.data, 8); break;
      case 0x203: memcpy(canS.f203, m.data, 8); break;
      case 0x302: memcpy(canS.f302, m.data, 8); break;
      case 0x303: memcpy(canS.f303, m.data, 8); break;
      case 0x401: memcpy(canS.f401, m.data, 8); break;
      default: break;                   // undecoded, but still counted above
    }
    canS.lastRxMs = millis();
    canS.frames++; canS.framesWin++;
  }
}

bool canLive() {
  return canOk && canS.lastRxMs && millis() - canS.lastRxMs < CAN_STALE_MS;
}

// ---------- BLE ----------
volatile bool bleZeroReq = false;

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
    // 0x02 (zero) is the only opcode with meaning here — 0x01/0x03/0x04 all
    // drive the OLED, which this build does not have.
    if (v.data()[0] == 0x02) bleZeroReq = true;
  }
};

// ---------- state ----------
float lastPressPa = 0, lastTempC = 0, lastBaroAlt = 0;
int8_t bnoTempC = 0;
uint32_t tImu = 0, tTele = 0, tStatus = 0, tRetry = 0;
uint32_t loopsPerSec = 0, maxLoopGapMs = 0, lastLoopAt = 0;

void i2cScan() {
  Serial.println("[i2c] scanning at 100 kHz...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (!i2cPresent(a)) continue;
    const char *who = a == BNO_ADDR ? " (BNO055)"
                    : a == BMP_ADDR ? " (BMP280)"
                    : (a == 0x3C || a == 0x3D) ? " (OLED — not used by this build)"
                    : "";
    Serial.printf("[i2c]   0x%02X%s\n", a, who);
    found++;
  }
  if (found == 0)
    Serial.println("[i2c] nothing answered — check the SDA/SCL wires, 3V3 and GND");
  else if (found > 8)
    Serial.printf("[i2c] %d addresses answered — that is a floating bus, not %d\n"
                  "[i2c] devices. No pull-ups: check the 3V3 rail and the harness.\n",
                  found, found);
  else
    Serial.printf("[i2c] %d device(s)\n", found);
}

void reportBnoHealth() {
  uint8_t st = 0, sys = 0, err = 0, cal = 0;
  bool ok = readRegs(BNO_ADDR, BNO_ST_RESULT, &st, 1) &&
            readRegs(BNO_ADDR, BNO_SYS_STATUS, &sys, 1) &&
            readRegs(BNO_ADDR, BNO_SYS_ERR, &err, 1) &&
            readRegs(BNO_ADDR, BNO_CALIB_STAT, &cal, 1);
  if (!ok) { Serial.println("[imu] health registers unreadable"); return; }
  // sys: 5 = fusion running. err is only meaningful when sys == 1.
  Serial.printf("[imu] self-test acc=%d gyr=%d mag=%d mcu=%d | sys=%d err=0x%02X\n",
                (st >> 0) & 1, (st >> 2) & 1, (st >> 1) & 1, (st >> 3) & 1, sys, err);
  // Self-test is power-on, so all four bits read 1 even in IMUPLUS — the mag
  // is tested before the mode is chosen. It's the *calibration* that stays 0,
  // for sys and mag both, because fusion never uses the magnetometer here.
  Serial.println("[imu] mode IMUPLUS — sys and mag calibration stay 0 by design");
  if (sys == 1) Serial.println("[imu] SYS ERROR — see BNO055 datasheet table 4-4");
}

// Pull each pad up, then down, and read it back. A pad the chip still controls
// follows the pull both ways and reads 1/0. Anything else is tied by something
// outside the GPIO: 0/0 is held to ground, 1/1 is held high — which is what a
// populated I2C bus looks like, and why D4/D5 are in the sweep as controls.
//
// Sweeping the whole row at once is the point. A single pin reading oddly is a
// story about a broken test; the same test passing on six neighbours and
// failing on one is a story about the pin. Runs before Wire.begin() so it
// never fights the I2C driver for the bus pins.
void padSweep() {
  const struct { uint8_t pin; const char *name; } pads[] = {
    {D0, "D0"}, {D1, "D1"}, {D2, "D2"}, {D3, "D3"},
    {D4, "D4"}, {D5, "D5"}, {D10, "D10"},
    {D8, "D8"}, {D9, "D9"},
  };
  // D9 is the interesting one for CAN: a powered SN65HVD230 drives its RXD
  // output high while the bus idles recessive, so a healthy CRX link reads
  // "held HIGH". If D9 follows the pull instead, nothing is driving it — the
  // transceiver has no power, or CRX never reaches the pin. Runs before
  // canBringup() so the TWAI driver doesn't own the pins yet.
  Serial.println("[pad] pull-up/pull-down readback — free pad = 1/0, driven pad = 1/1");
  for (auto &p : pads) {
    pinMode(p.pin, INPUT_PULLUP);
    delay(2);
    int up = digitalRead(p.pin);
    pinMode(p.pin, INPUT_PULLDOWN);
    delay(2);
    int dn = digitalRead(p.pin);
    pinMode(p.pin, INPUT);
    const char *note = (up == 1 && dn == 0) ? "follows both — pad is free"
                     : (up == 1 && dn == 1) ? "held HIGH — pull-up on the line"
                     : (up == 0 && dn == 0) ? "held LOW  <-- tied to ground"
                                            : "inverted?? — suspect the test";
    Serial.printf("[pad]   %-3s up=%d dn=%d  %s\n", p.name, up, dn, note);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);             // never let a half-open CDC stall the loop
  delay(1500);
  Serial.println("\n=== Tripper Puck — bench rig (IMU + CAN, no OLED/buttons/GPS) ===");

  padSweep();

  prefs.begin("puck", false);           // shares the production mount reference
  qRef.w = prefs.getFloat("qw", 1.0f);
  qRef.x = prefs.getFloat("qx", 0.0f);
  qRef.y = prefs.getFloat("qy", 0.0f);
  qRef.z = prefs.getFloat("qz", 0.0f);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);                // BNO055 clock-stretch safe
  Wire.setTimeOut(1000);

  Serial.printf("[i2c] SDA=GPIO%d (D4)  SCL=GPIO%d (D5)\n", PIN_SDA, PIN_SCL);
  // The separate "old SDA pad" watch is gone: SDA is back on D4, so padSweep()
  // and busLinesHigh() already report that pin every boot.

  // Electrical state first, then who answers. On a clamped bus every probe
  // burns the full Wire timeout, so scanning all 126 addresses costs over two
  // minutes and tells you nothing the line test hasn't already said.
  if (busLinesHigh(true)) {
    i2cScan();
    imuOk = bnoBringup();
    bmpOk = bmp.begin(BMP_ADDR);
    if (bmpOk)
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                      Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X4,
                      Adafruit_BMP280::STANDBY_MS_63);
  } else {
    Serial.println("[i2c] skipping scan and sensor init — fix the lines first");
  }
  Serial.printf("IMU %s | BMP280 %s\n", imuOk ? "ok" : "FAIL", bmpOk ? "ok" : "FAIL");
  if (imuOk) reportBnoHealth();
  if (!imuOk || !bmpOk) Serial.printf("[retry] missing sensors re-probed every %lu ms\n",
                                      (unsigned long)RETRY_MS);

  // Hold D9 under a pull-down for 30 s with a meter-readable window, before the
  // TWAI driver claims the pin and puts its own pull-up on it. With the ESP32
  // pull-up off, the only thing that can hold CRX high is the transceiver's
  // output transistor — so this is the one measurement that separates "RXD is
  // driving" from "RXD is high-impedance and the ESP32 was holding the net".
  //
  // Probe the module's CRX pad and the chip's pin 4 during the countdown:
  //   both stay ~3.3 V  -> receiver is alive, look elsewhere
  //   both near 0 V     -> RXD output is dead
  //   pin 4 high, pad 0 -> internal trace open, bodge pin 4 to the pad
  pinMode(D9, INPUT_PULLDOWN);
  Serial.println("\n[diag] D9 pulled DOWN for 30 s — ESP32 pull-up is OFF.");
  Serial.println("[diag] measure module CRX pad and chip pin 4 against GND now.");
  for (int s = 30; s > 0; s--) {
    Serial.printf("[diag] %2ds  D9 reads %d %s\n", s, digitalRead(D9),
                  digitalRead(D9) ? "<-- something is driving it HIGH" : "(low)");
    delay(1000);
  }
  pinMode(D9, INPUT);
  Serial.println("[diag] done — starting CAN normally\n");

  canOk = canBringup();
  Serial.printf("CAN %s (250k listen-only, D8/D9)\n", canOk ? "ok" : "FAIL");
  Serial.println("[can] driver state only — frames need the bike powered and "
                 "CANH/CANL connected (stage 2, USB out)");

  NimBLEDevice::init("Tripper-DL1");
  NimBLEDevice::setMTU(247);
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
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setCompleteServices(NimBLEUUID(SVC_UUID));
  NimBLEAdvertisementData scanData;
  scanData.setName("Tripper-DL1");
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->start();
  Serial.println("[ble] advertising as Tripper-DL1 (same UUIDs/packets as production)");
}

void loop() {
  uint32_t now = millis();
  loopsPerSec++;
  if (lastLoopAt && now - lastLoopAt > maxLoopGapMs) maxLoopGapMs = now - lastLoopAt;
  lastLoopAt = now;

  canPoll();

  if (bleZeroReq) {
    bleZeroReq = false;
    if (imuOk) {
      qRef = lastQuat;
      saveQRef();
      Serial.println("[zero] mount reference captured & saved (app)");
    } else {
      Serial.println("[zero] ignored — no IMU");
    }
  }

  // Hot-plug: re-probe whatever is missing so a wire soldered mid-run comes
  // up on its own. Only runs while something is absent, so a healthy rig
  // never pays for it.
  if ((!imuOk || !bmpOk) && now - tRetry >= RETRY_MS) {
    tRetry = now;
    // If one sensor is already answering, the bus is demonstrably alive and the
    // driver-panic path isn't in play — probe straight away rather than tearing
    // Wire down under a running 100 Hz read. Only check the lines when nothing
    // at all has come up.
    if (!imuOk && !bmpOk && !busLinesHigh(false)) {
      Serial.println("[retry] bus lines still low — nothing powered on D4/D5");
      return;                           // never probe a bus nothing is driving
    }
    if (!imuOk && i2cPresent(BNO_ADDR) && bnoBringup()) {
      imuOk = true;
      Serial.println("[retry] IMU came up");
      reportBnoHealth();
    }
    if (!bmpOk && i2cPresent(BMP_ADDR) && bmp.begin(BMP_ADDR)) {
      bmpOk = true;
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                      Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X4,
                      Adafruit_BMP280::STANDBY_MS_63);
      Serial.println("[retry] BMP280 came up");
    }
  }

  // 100 Hz IMU
  if (imuOk && now - tImu >= 10) {
    tImu = now;
    bnoRead();
  }

  // 5 Hz telemetry — same packet the production build sends
  if (now - tTele >= 200) {
    tTele = now;
    TelemetryPacket p = {};
    p.ver = 0x02;
    p.flags = 0;                        // no GPS in this build
    p.gpsTimeMs = 0xFFFFFFFF;
    p.hdop_c = 9999;
    p.baroAlt_cm = (int32_t)(lastBaroAlt * 100);
    p.press_pa = (uint32_t)lastPressPa;
    Quat qz = qRel();
    p.qw = (int16_t)(qz.w * 16384);
    p.qx = (int16_t)(qz.x * 16384);
    p.qy = (int16_t)(qz.y * 16384);
    p.qz = (int16_t)(qz.z * 16384);
    p.linx_mg = (int16_t)(linX / 9.80665f * 1000);
    p.liny_mg = (int16_t)(linY / 9.80665f * 1000);
    p.linz_mg = (int16_t)(linZ / 9.80665f * 1000);
    p.maxG_mg = (int16_t)(maxG_g * 1000);
    maxG_g = 0;

    if (canLive()) {
      uint8_t st = canS.f202[0];
      p.canFlags = 0x01 | (((st >> 7) & 1) << 1) | (((st >> 4) & 3) << 2);
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

  // 1 Hz: baro sample, BLE status, and the report this rig exists for
  if (now - tStatus >= 1000) {
    tStatus = now;
    if (bmpOk) {
      lastPressPa = bmp.readPressure();
      lastTempC = bmp.readTemperature();
      lastBaroAlt = bmp.readAltitude(1013.25f);
    }
    StatusPacket s = {};
    s.ver = 0x01;
    s.fix = 0;
    s.battPct = 0xFF;
    s.hdop_c = 9999;
    s.uptime_s = now / 1000;
    s.temp_x10 = (int16_t)(lastTempC * 10);
    chStat->setValue((uint8_t *)&s, sizeof(s));
    if (bleServer->getConnectedCount() > 0) chStat->notify();

    if (imuOk) {
      float r, p, y;
      Quat qz = qRel();
      quatToEuler(qz, r, p, y);
      uint8_t cal = 0;
      readRegs(BNO_ADDR, BNO_CALIB_STAT, &cal, 1);
      readRegs(BNO_ADDR, BNO_TEMP, (uint8_t *)&bnoTempC, 1);
      Serial.printf("[imu] q=%+.3f,%+.3f,%+.3f,%+.3f R=%+6.1f P=%+6.1f Y=%+6.1f | "
                    "lin=%+5.2f,%+5.2f,%+5.2f m/s2 | rd=%lu err=%lu (+%lu/%lu) "
                    "max=%luus cal S/G/A=%d/%d/%d T=%dC\n",
                    qz.w, qz.x, qz.y, qz.z, r, p, y, linX, linY, linZ,
                    (unsigned long)imuReads, (unsigned long)imuErrs,
                    (unsigned long)imuErrsWin, (unsigned long)imuReadsWin,
                    (unsigned long)imuMaxUs,
                    (cal >> 6) & 3, (cal >> 4) & 3, (cal >> 2) & 3, bnoTempC);
    } else {
      Serial.println("[imu] absent");
    }
    imuReadsWin = imuErrsWin = 0;

    if (bmpOk)
      Serial.printf("[baro] %.2f hPa  %.2f C  alt %.1f m\n",
                    lastPressPa / 100.0f, lastTempC, lastBaroAlt);
    else
      Serial.println("[baro] absent");

    Serial.printf("[can] %s frames=%lu (+%lu/s) ids=%d",
                  canOk ? (canLive() ? "live" : "idle") : "off",
                  (unsigned long)canS.frames, (unsigned long)canS.framesWin, censusN);
    for (uint8_t i = 0; i < censusN; i++)
      Serial.printf(" %03X:%lu", (unsigned)census[i].id, (unsigned long)census[i].count);
    Serial.println();

    // The controller's own error counters are what separate "wired wrong" from
    // "nothing there". A bus running at the wrong bit rate still puts edges on
    // RX, so the controller sees frames it cannot parse and busErr climbs fast.
    // A silent RX line produces no frames AND no errors, because from the
    // controller's side an idle bus and a disconnected one look identical.
    twai_status_info_t st;
    if (canOk && twai_get_status_info(&st) == ESP_OK) {
      const char *sn = st.state == TWAI_STATE_RUNNING    ? "running"
                     : st.state == TWAI_STATE_BUS_OFF    ? "BUS-OFF"
                     : st.state == TWAI_STATE_RECOVERING ? "recovering"
                                                         : "stopped";
      Serial.printf("[can] ctrl %s rxErr=%lu txErr=%lu busErr=%lu missed=%lu "
                    "overrun=%lu arbLost=%lu\n",
                    sn, (unsigned long)st.rx_error_counter,
                    (unsigned long)st.tx_error_counter,
                    (unsigned long)st.bus_error_count,
                    (unsigned long)st.rx_missed_count,
                    (unsigned long)st.rx_overrun_count,
                    (unsigned long)st.arb_lost_count);
      if (canS.frames == 0 && st.bus_error_count == 0)
        Serial.println("[can] 0 frames and 0 bus errors — RX never leaves recessive. "
                       "Bike asleep, CANH/CANL open, transceiver unpowered, or its "
                       "Rs pin not tied low. A wrong bit rate would show busErr instead.");
      else if (canS.frames == 0 && st.bus_error_count > 0)
        Serial.println("[can] bus errors but no frames — edges are arriving but "
                       "unparseable: wrong bit rate, or CANH/CANL swapped.");
    }
    if (canLive())
      Serial.printf("[bike] soc=%u%% pack=%.1fV spd=%.1fkm/h rpm=%u pwr=%uW "
                    "cell hi=%umV[%u] lo=%umV[%u]\n",
                    canS.f401[0], canU16(canS.f101, 0) / 10.0f,
                    canU16(canS.f303, 0) / 10.0f, canU16(canS.f203, 0),
                    canU16(canS.f203, 2), canU16(canS.f201, 0), canS.f201[4],
                    canU16(canS.f201, 2), canS.f201[5]);
    canS.framesWin = 0;

    Serial.printf("[sys] up=%lus lps=%lu stall=%lums ble=%d\n\n",
                  (unsigned long)(now / 1000), (unsigned long)loopsPerSec,
                  (unsigned long)maxLoopGapMs, bleServer->getConnectedCount());
    loopsPerSec = 0;
    maxLoopGapMs = 0;
  }
}
