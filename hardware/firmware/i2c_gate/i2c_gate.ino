// Tripper Puck — build step 2: the go/no-go gate.
// Scans the I2C bus at 100 kHz (expect BNO055 at 0x28, BMP280 at 0x76),
// puts the BNO055 into IMUPLUS fusion mode, then hammers quaternion reads
// for 10 minutes counting every bus error. Zero errors = the clock-stretch
// risk is retired and the Gravity board is a keeper.

#include <Wire.h>

const uint8_t BNO_ADDR = 0x28;
const uint8_t BMP_ADDR = 0x76;
const uint32_t TEST_MS = 10UL * 60UL * 1000UL;  // 10 minutes

uint32_t reads = 0, errors = 0, maxReadUs = 0;
uint32_t testStart = 0, lastReport = 0;
bool testDone = false;

bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Returns bytes actually read; expected count means success.
int readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;  // repeated start
  uint8_t got = Wire.requestFrom(addr, len);
  for (uint8_t i = 0; i < got; i++) buf[i] = Wire.read();
  return got;
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // give the monitor time to attach

  Wire.begin();               // XIAO ESP32-S3 defaults: SDA=D4/GPIO5, SCL=D5/GPIO6
  Wire.setClock(100000);      // 100 kHz — clock-stretch mitigation #1
  Wire.setTimeOut(1000);      // generous timeout (ms) — mitigation #2

  // --- Phase 1: bus scan ---
  Serial.println("\n=== Tripper Puck I2C gate test ===");
  Serial.println("Scanning bus at 100 kHz...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X %s\n", a,
                    a == BNO_ADDR ? "(BNO055 ✓)" :
                    a == BMP_ADDR ? "(BMP280 ✓)" : "(unexpected!)");
      found++;
    }
  }
  if (found == 0) {
    Serial.println("NO DEVICES FOUND — check wiring (SDA→D4, SCL→D5, 3V3, GND). Halting.");
    while (true) delay(1000);
  }

  // --- Phase 2: identify chips ---
  uint8_t id;
  if (readRegs(BNO_ADDR, 0x00, &id, 1) == 1)
    Serial.printf("BNO055 chip ID: 0x%02X %s\n", id, id == 0xA0 ? "(correct)" : "(WRONG)");
  else
    Serial.println("BNO055 chip ID read FAILED");
  if (readRegs(BMP_ADDR, 0xD0, &id, 1) == 1)
    Serial.printf("BMP280 chip ID: 0x%02X %s\n", id, id == 0x58 ? "(correct)" : "(WRONG)");
  else
    Serial.println("BMP280 chip ID read FAILED");

  // --- Phase 3: start fusion so the chip clock-stretches like it will in real use ---
  writeReg(BNO_ADDR, 0x3D, 0x00);  // OPR_MODE = CONFIG
  delay(25);
  writeReg(BNO_ADDR, 0x3F, 0x20);  // SYS_TRIGGER = reset
  delay(700);
  writeReg(BNO_ADDR, 0x3D, 0x08);  // OPR_MODE = IMUPLUS (6-axis, no magnetometer)
  delay(20);
  Serial.println("BNO055 in IMUPLUS mode. Hammering quaternion reads for 10 minutes...");
  Serial.println("(a clean PASS = zero errors)\n");

  testStart = lastReport = millis();
}

void loop() {
  if (testDone) return;

  uint8_t q[8];
  uint32_t t0 = micros();
  int got = readRegs(BNO_ADDR, 0x20, q, 8);  // QUA_DATA_W_LSB..Z_MSB
  uint32_t dt = micros() - t0;

  reads++;
  if (got != 8) errors++;
  if (dt > maxReadUs) maxReadUs = dt;

  uint32_t now = millis();
  if (now - lastReport >= 10000) {
    int16_t w = (int16_t)(q[1] << 8 | q[0]);
    Serial.printf("[%3lus] reads=%lu errors=%lu maxRead=%luus quatW=%.4f\n",
                  (now - testStart) / 1000, reads, errors, maxReadUs, w / 16384.0);
    lastReport = now;
  }

  if (now - testStart >= TEST_MS) {
    testDone = true;
    Serial.println("\n=== RESULT ===");
    Serial.printf("%lu reads, %lu errors in 10 minutes.\n", reads, errors);
    Serial.println(errors == 0
      ? "PASS — bus is solid. Proceed to build step 3."
      : "FAIL — clock-stretch trouble. Do NOT solder; re-check pull-ups/wire length, then reconsider the board.");
  }

  delay(8);  // ~100 Hz, matching the production read rate
}
