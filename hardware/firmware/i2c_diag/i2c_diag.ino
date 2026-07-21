// Tripper Puck — I2C line diagnostic.
// Reads the electrical state of D4/D5, then scans the bus normally AND with
// SDA/SCL swapped. Interpretation:
//   floating reads 1,1  -> Gravity board is powered (its onboard pull-ups
//                          are driving the lines) and wires reach the XIAO
//   floating 0, pullup 1 -> wire present but Gravity unpowered (dead pull-ups)
//                          or line not actually reaching the board
//   pullup reads 0       -> line clamped: module unpowered or shorted
//   devices on swapped scan -> blue/green wires are crossed

#include <Wire.h>

int scanBus(const char *label, int sda, int scl) {
  Wire.begin(sda, scl);
  Wire.setClock(100000);
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  %s: device at 0x%02X\n", label, a);
      found++;
    }
  }
  if (!found) Serial.printf("  %s: nothing\n", label);
  Wire.end();
  return found;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== I2C line diagnostic ===");

  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  delay(20);
  Serial.printf("Floating:  D4(SDA)=%d  D5(SCL)=%d   (1,1 = Gravity powered & connected)\n",
                digitalRead(D4), digitalRead(D5));

  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);
  delay(20);
  Serial.printf("Pulled up: D4(SDA)=%d  D5(SCL)=%d   (any 0 = line clamped low)\n",
                digitalRead(D4), digitalRead(D5));

  Serial.println("Scanning...");
  scanBus("normal  SDA=D4 SCL=D5", D4, D5);
  scanBus("swapped SDA=D5 SCL=D4", D5, D4);
  Serial.println("=== done ===");
}

void loop() { delay(1000); }
