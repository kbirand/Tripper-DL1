// Tripper Puck — GPS revert-to-factory test harness.
// Simulates what a power cycle with a dead backup cell does to the NEO-M8:
// clears the stored config (CFG-CFG clear+load, BBR+flash) so the module
// drops back to 9600 baud / 1 Hz factory defaults. The nav database is NOT
// cleared — the module keeps its fix, which is exactly the real-world state
// this reproduces (module tracking happily, host deaf at the wrong baud).
//
// Use: flash this, watch serial for PASS, then flash tripper_puck and check
// its bring-up recovers ("GPS ok"). Repeatable RED state for the bring-up path.

#include <Arduino.h>

void sendUBX(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  uint8_t hdr[4] = {cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  Serial1.write(0xB5); Serial1.write(0x62);
  for (int i = 0; i < 4; i++) { Serial1.write(hdr[i]); ckA += hdr[i]; ckB += ckA; }
  for (int i = 0; i < len; i++) { Serial1.write(payload[i]); ckA += payload[i]; ckB += ckA; }
  Serial1.write(ckA); Serial1.write(ckB);
}

// Drop bytes buffered at the previous baud — without this a probe can "pass"
// on stale NMEA received before the switch.
void flushRx() {
  delay(50);
  while (Serial1.available()) Serial1.read();
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
  Serial.println("\n=== GPS revert to factory (simulate dead backup cell) ===");

  // Find the module at whichever baud it's on now.
  Serial1.begin(115200, SERIAL_8N1, D7, D6);
  flushRx();
  uint32_t foundAt = 0;
  if (nmeaAlive()) foundAt = 115200;
  else {
    Serial1.updateBaudRate(9600);
    flushRx();
    if (nmeaAlive()) foundAt = 9600;
  }
  if (!foundAt) { Serial.println("NO NMEA at 115200 or 9600 — check wiring. Halting."); while (true) delay(1000); }
  Serial.printf("Module found at %lu.\n", foundAt);

  // CFG-CFG: clearMask 0xFFFF (wipe stored config, BBR+flash), loadMask 0xFFFF
  // (activate the now-default stored config). Module reverts to 9600/1 Hz.
  const uint8_t revert[13] = {0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0xFF, 0xFF, 0x00, 0x00, 0x03};
  sendUBX(0x06, 0x09, revert, 13);
  Serial1.flush();
  delay(500);

  Serial1.updateBaudRate(9600);
  flushRx();
  bool at9600 = nmeaAlive(3000);
  Serial1.updateBaudRate(115200);
  flushRx();
  bool at115200 = nmeaAlive(2000);

  Serial.printf("After revert: NMEA at 9600 %s, at 115200 %s\n",
                at9600 ? "YES" : "no", at115200 ? "YES" : "no");
  Serial.println(at9600 && !at115200
                     ? "PASS — module is back to factory 9600. Now flash tripper_puck."
                     : "FAIL — unexpected state, revert didn't take.");
}

void loop() { delay(1000); }
