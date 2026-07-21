// Tripper Puck — one-time GPS config: 5 Hz measurement rate + 115200 baud.
// Idempotent: detects the module's current baud (9600 factory or 115200 if
// already configured), applies CFG-RATE + CFG-PRT, verifies by read-back,
// saves to BBR/flash, then live-counts GGA epochs so the 5 Hz is proven,
// not assumed. Production firmware repeats this config at every boot anyway —
// if the backup battery ever drains, the module silently reverts to defaults.

#include <Arduino.h>

void sendUBX(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  uint8_t hdr[4] = {cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  Serial1.write(0xB5); Serial1.write(0x62);
  for (int i = 0; i < 4; i++) { Serial1.write(hdr[i]); ckA += hdr[i]; ckB += ckA; }
  for (int i = 0; i < len; i++) { Serial1.write(payload[i]); ckA += payload[i]; ckB += ckA; }
  Serial1.write(ckA); Serial1.write(ckB);
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

// Poll CFG-RATE and return measRate in ms, or -1 on timeout.
int pollMeasRate(uint32_t timeoutMs = 1500) {
  while (Serial1.available()) Serial1.read();
  sendUBX(0x06, 0x08, nullptr, 0);
  uint32_t t0 = millis();
  int st = 0; uint8_t pay[6]; int got = 0;
  while (millis() - t0 < timeoutMs) {
    if (!Serial1.available()) continue;
    uint8_t b = Serial1.read();
    switch (st) {
      case 0: st = (b == 0xB5) ? 1 : 0; break;
      case 1: st = (b == 0x62) ? 2 : 0; break;
      case 2: st = (b == 0x06) ? 3 : (b == 0xB5 ? 1 : 0); break;
      case 3: st = (b == 0x08) ? 4 : 0; break;
      case 4: st = (b == 0x06) ? 5 : 0; break;   // len LSB = 6
      case 5: st = (b == 0x00) ? 6 : 0; break;   // len MSB = 0
      case 6:
        pay[got++] = b;
        if (got == 6) return pay[0] | (pay[1] << 8);
        break;
    }
  }
  return -1;
}

bool nmeaAlive(uint32_t windowMs = 1500) {
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

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== GPS config: 5 Hz + 115200 ===");

  Serial1.begin(9600, SERIAL_8N1, D7, D6);
  bool at9600 = nmeaAlive();
  if (!at9600) {
    Serial1.end();
    Serial1.begin(115200, SERIAL_8N1, D7, D6);
    if (nmeaAlive()) Serial.println("Module already at 115200.");
    else { Serial.println("NO NMEA at 9600 or 115200 — check wiring. Halting."); while (true) delay(1000); }
  } else {
    Serial.println("Module detected at 9600 (factory default).");
  }

  // CFG-RATE: measRate 200 ms, navRate 1, timeRef GPS
  const uint8_t rate[6] = {0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
  sendUBX(0x06, 0x08, rate, 6);
  int a = waitAck(0x06, 0x08);
  Serial.printf("CFG-RATE 5 Hz: %s\n", a == 1 ? "ACK" : a == 0 ? "NAK!" : "timeout!");

  // CFG-PRT: UART1, 8N1, 115200, UBX+NMEA in/out
  const uint8_t prt[20] = {0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                           0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00,
                           0x00, 0x00, 0x00, 0x00};
  sendUBX(0x06, 0x00, prt, 20);
  Serial1.flush();
  delay(200);
  if (at9600) {
    Serial1.end();
    Serial1.begin(115200, SERIAL_8N1, D7, D6);
    Serial.println("Switched host side to 115200.");
  }
  delay(200);

  int mr = pollMeasRate();
  Serial.printf("Read-back measRate: %d ms %s\n", mr,
                mr == 200 ? "= 5 Hz ✓" : "(expected 200!)");

  // CFG-CFG: save current config to BBR + flash
  const uint8_t save[13] = {0, 0, 0, 0, 0xFF, 0xFF, 0x00, 0x00, 0, 0, 0, 0, 0x03};
  sendUBX(0x06, 0x09, save, 13);
  a = waitAck(0x06, 0x09);
  Serial.printf("CFG-CFG save to BBR: %s\n", a == 1 ? "ACK" : a == 0 ? "NAK!" : "timeout!");
  Serial.println("Now measuring real epoch rate (GGA sentences per second):");
}

void loop() {
  static char w0, w1, w2;
  static int gga = 0;
  static uint32_t t0 = millis();
  while (Serial1.available()) {
    char c = Serial1.read();
    if (w0 == 'G' && w1 == 'G' && w2 == 'A' && c == ',') gga++;
    w0 = w1; w1 = w2; w2 = c;
  }
  if (millis() - t0 >= 2000) {
    Serial.printf("[rate] %.1f GGA/s (target 5.0)\n", gga / 2.0);
    gga = 0;
    t0 = millis();
  }
}
