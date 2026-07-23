// Tripper Puck — Talaria CAN probe (SN65HVD230 on D8/D9).
// Brings up the ESP32-S3's TWAI controller at 250 kbit/s in LISTEN-ONLY mode:
// the peripheral never transmits and never even ACKs, so nothing here can
// disturb the bike's controller no matter what the firmware does.
//
// Reports over BLE and the OLED, NOT over USB serial. That is deliberate.
// Once CANH/CANL are spliced into the bike, the puck's ground IS the bike's
// ground. Adding a USB cable to a mains-earthed desktop would tie the bike's
// battery negative to protective earth through the Mac, and feeding the 5V pin
// while USB is attached back-feeds the bike's 5V into the host port (on the
// XIAO the 5V pin is USB VBUS). So: power from the bike, read wirelessly.
// Serial still prints, and is safe ONLY when the CAN wires are unplugged.
//
// Reference capture to check against (USB-CAN-A, 2026-07-23, bike parked):
//   81 %  ·  64.3 V  ·  16S, cell high #12 ≈ 4.028 V, low #9 ≈ 4.010 V
//   kickstand DOWN  ·  Sport  ·  speed/rpm/power/current all zero
//
// Signal map and its provenance: tools/talaria.dbc
// Wiring: module 3V3->3V3, GND->GND, CTX->D8, CRX->D9, CANH/CANL to the bike.
// The module's onboard 120R terminator MUST be removed — the bike's bus is
// already terminated at both ends (60 R) and a third resistor makes it 40 R.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include <driver/twai.h>

static const gpio_num_t CAN_TX = GPIO_NUM_7;   // D8
static const gpio_num_t CAN_RX = GPIO_NUM_8;   // D9

// Nordic UART Service — nRF Connect and LightBlue both render this as a
// scrolling terminal with no app-side work.
#define NUS_SVC "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static Adafruit_SSD1306 oled(128, 32, &Wire, -1);
static bool haveOled = false;
static NimBLECharacteristic *chTx = nullptr;
static bool phoneConnected = false;

struct Frame {
  uint8_t  data[8] = {0};
  uint32_t count   = 0;
};
static Frame f101, f201, f202, f203, f302, f303, f401;
static uint32_t totalFrames = 0, otherFrames = 0;
static uint32_t lastPrint = 0, startMs = 0;
static char dash[640];

static inline uint16_t u16(const uint8_t *d, int off) {
  return (uint16_t)d[off] | ((uint16_t)d[off + 1] << 8);
}

static void store(Frame &f, const twai_message_t &m) {
  memcpy(f.data, m.data, 8);
  f.count++;
}

class SrvCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override { phoneConnected = true; }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override { phoneConnected = false; }
};

void setup() {
  Serial.begin(115200);
  startMs = millis();

  // OLED-only bus here, so 400 kHz is safe: the BNO055 whose clock-stretching
  // forces 100 kHz in the production firmware is never addressed by this sketch.
  Wire.begin(D4, D5);
  Wire.setClock(400000);
  haveOled = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (haveOled) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println("CAN probe 250k");
    oled.println("listen-only");
    oled.display();
  }

  NimBLEDevice::init("Tripper-CAN");
  NimBLEDevice::setPower(9);
  NimBLEDevice::setMTU(247);
  NimBLEServer *srv = NimBLEDevice::createServer();
  srv->setCallbacks(new SrvCB());
  srv->advertiseOnDisconnect(true);
  NimBLEService *svc = srv->createService(NUS_SVC);
  chTx = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SVC);
  adv->setName("Tripper-CAN");
  adv->start();

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_len = 32;                       // ~92 frames/s, plenty of headroom
  g.alerts_enabled = TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL |
                     TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t fl = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  bool ok = twai_driver_install(&g, &t, &fl) == ESP_OK && twai_start() == ESP_OK;
  Serial.println(ok ? "\n=== CAN probe: bus up, listen-only ===" : "\nTWAI bring-up FAILED");
  if (!ok && haveOled) {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("TWAI INIT FAILED");
    oled.display();
    while (true) delay(1000);
  }
}

// Listen-only cannot cause bus errors, so any alert points at the wiring.
static const char *alertText() {
  uint32_t a = 0;
  if (twai_read_alerts(&a, 0) != ESP_OK || !a) return nullptr;
  if (a & TWAI_ALERT_BUS_ERROR)     return "bus error - bitrate or CANH/CANL swapped";
  if (a & TWAI_ALERT_BUS_OFF)       return "bus off";
  if (a & TWAI_ALERT_ERR_PASS)      return "error passive";
  if (a & TWAI_ALERT_RX_QUEUE_FULL) return "rx queue full";
  return nullptr;
}

static void buildDash(const char *alert) {
  float secs = (millis() - startMs) / 1000.0f;
  int n = snprintf(dash, sizeof dash,
                   "--- %.1fs  %lu frames  %lu unmapped ---\n", secs,
                   (unsigned long)totalFrames, (unsigned long)otherFrames);

  if (alert) n += snprintf(dash + n, sizeof dash - n, "!! %s\n", alert);

  if (!totalFrames) {
    snprintf(dash + n, sizeof dash - n,
             "nothing received:\n"
             " 1 bike ON and awake?\n"
             " 2 CANH/CANL swapped? (harmless)\n"
             " 3 CTX/CRX straight through, not crossed\n"
             " 4 module 120R terminator removed?\n");
    return;
  }

  uint8_t st = f202.data[0];
  const char *mode = ((st >> 4) & 3) == 1 ? "Eco" : ((st >> 4) & 3) == 2 ? "Sport" : "?";
  n += snprintf(dash + n, sizeof dash - n,
                "Speed %.1f km/h  Motor %u rpm\nPower %u W  Current %.1f A\n"
                "Pack %.1f V  Charge %u %%  Demand %u\n"
                "Kickstand %s  Mode %s  State 0x%02X\n",
                u16(f303.data, 0) / 10.0f, u16(f203.data, 0), u16(f203.data, 2),
                u16(f302.data, 4) / 10.0f, u16(f101.data, 0) / 10.0f,
                f401.data[0], u16(f202.data, 3),
                (st >> 7) & 1 ? "DOWN" : "up", mode, st);

  if (f201.count) {
    uint16_t hi = u16(f201.data, 0), lo = u16(f201.data, 2);
    snprintf(dash + n, sizeof dash - n,
             "Cells hi #%u %.3f  lo #%u %.3f  d%u mV  pack~%.2f V\n",
             f201.data[4], hi / 1000.0f, f201.data[5], lo / 1000.0f,
             (unsigned)(hi - lo), (hi + lo) * 8.0f / 1000.0f);
  }
}

static void notifyDash() {
  if (!phoneConnected || !chTx) return;
  // Chunk to fit the negotiated MTU rather than assuming a big one.
  const size_t chunk = 180;
  for (size_t i = 0, len = strlen(dash); i < len; i += chunk) {
    size_t take = len - i < chunk ? len - i : chunk;
    chTx->setValue((uint8_t *)(dash + i), take);
    chTx->notify();
    delay(12);                               // let the stack drain the queue
  }
}

static void drawOled() {
  if (!haveOled) return;
  oled.clearDisplay();
  oled.setCursor(0, 0);
  if (!totalFrames) {
    oled.printf("CAN 250k  NO DATA\ncheck H/L swap,\n120R, CTX/CRX\n%lus",
                (unsigned long)((millis() - startMs) / 1000));
  } else {
    uint8_t st = f202.data[0];
    uint16_t hi = u16(f201.data, 0), lo = u16(f201.data, 2);
    oled.printf("CAN OK %lu fr\n", (unsigned long)totalFrames);
    oled.printf("%u%%  %.1fV  %.0fkmh\n", f401.data[0], u16(f101.data, 0) / 10.0f,
                u16(f303.data, 0) / 10.0f);
    oled.printf("c%u %.3f c%u %.3f\n", f201.data[4], hi / 1000.0f,
                f201.data[5], lo / 1000.0f);
    oled.printf("KICK %s  %s", (st >> 7) & 1 ? "DN" : "UP",
                ((st >> 4) & 3) == 1 ? "Eco" : ((st >> 4) & 3) == 2 ? "Sport" : "?");
  }
  oled.display();
}

void loop() {
  twai_message_t m;
  // Drain everything queued before reporting, so the dashboard is a coherent
  // snapshot rather than a mix of old and new frames.
  while (twai_receive(&m, 0) == ESP_OK) {
    totalFrames++;
    if (m.extd || m.rtr) { otherFrames++; continue; }
    switch (m.identifier) {
      case 0x101: store(f101, m); break;
      case 0x201: store(f201, m); break;
      case 0x202: store(f202, m); break;
      case 0x203: store(f203, m); break;
      case 0x302: store(f302, m); break;
      case 0x303: store(f303, m); break;
      case 0x401: store(f401, m); break;
      default: otherFrames++; break;
    }
  }

  const char *alert = alertText();

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    buildDash(alert);
    Serial.print(dash);
    Serial.println();
    notifyDash();
    drawOled();
  }
}
