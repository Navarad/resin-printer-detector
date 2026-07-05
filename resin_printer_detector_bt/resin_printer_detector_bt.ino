/*
 * Resin Printer Detector - Direct BLE control of Shelly Plug S Gen3
 *
 * ESP32 acts as BLE Client, connects directly to Shelly, and writes
 * JSON-RPC commands to switch the plug on/off. Based on:
 *   https://www.instructables.com/Controlling-Shelly-Home-Automation-Relays-From-an-/
 *
 * NO Shelly script, NO scenes, NO BTHome setup required.
 * Just: Shelly Settings -> Bluetooth -> enable Bluetooth + RPC.
 *
 * Hardware:
 *   - LilyGo T-Display ESP32
 *   - MQ-135 gas sensor - AO -> GPIO 32, VCC -> 3V3, GND -> GND
 *
 * BLE authentication is NOT implemented - by default Shelly accepts
 * commands from any BLE client. To enable auth, see article link above.
 *
 * FIRST-TIME SETUP:
 *   1. Upload this firmware.
 *   2. Open Serial Monitor at 115200 baud.
 *   3. On boot you will see all nearby BLE devices printed.
 *   4. Find your Shelly - look for a line like:
 *        BLE OTHER Device found=Name: ShellyPlugSG3-XXXXXXXXXX, Address: cc:xx:xx:xx:xx:xx, ...
 *   5. Copy the MAC address (cc:xx:...) OR the device name
 *   6. Paste it below into TARGET_DEVICE and re-upload.
 */

#include <Wire.h>
#include <TFT_eSPI.h>
#include <Adafruit_BME280.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ==================== CONFIG ====================

// Set this to your Shelly's MAC address (recommended) or a name prefix
// like "ShellyPlugSG3". You'll find it in Serial Monitor on first boot.
constexpr char TARGET_DEVICE[] = "ShellyPlugSG3";
// constexpr char TARGET_DEVICE[] = "cc:8d:a2:04:12:ab";   // example MAC

// ================================================

// ---------- Thresholds ----------
constexpr int32_t  DELTA_PRINTING = 50;
constexpr int32_t  DELTA_DANGER   = 150;
constexpr int32_t  DELTA_IDLE     = 25;
constexpr uint32_t PRINTING_CONFIRM_MS = 60UL * 1000UL;
constexpr uint32_t FINISHED_CONFIRM_MS = 5UL * 60UL * 1000UL;
constexpr uint32_t FINISHED_SHOW_MS    = 30UL * 1000UL;
constexpr uint32_t FAN_OFF_DELAY_MS    = 10UL * 60UL * 1000UL;

constexpr uint32_t BASELINE_UPDATE_MS  = 5UL * 1000UL;
constexpr float    BASELINE_ALPHA_UP   = 0.001f;
constexpr float    BASELINE_ALPHA_DOWN = 0.05f;

// ---------- Shelly BLE UUIDs (from Instructables article + Shelly source) ----------
static BLEUUID SHELLY_SVC_UUID ("5f6d4f53-5f52-5043-5f53-56435f49445f");
static BLEUUID SHELLY_TX_UUID  ("5f6d4f53-5f52-5043-5f74-785f63746c5f");  // write - size prefix
static BLEUUID SHELLY_DATA_UUID("5f6d4f53-5f52-5043-5f64-6174615f5f5f");  // read/write - JSON

// ---------- Pins ----------
constexpr uint8_t PIN_BTN_LEFT  = 0;
constexpr uint8_t PIN_BTN_RIGHT = 35;
constexpr uint8_t PIN_MQ135_AO  = 32;
constexpr uint8_t PIN_SDA       = 21;   // BME280 I2C data
constexpr uint8_t PIN_SCL       = 22;   // BME280 I2C clock

// ---------- Globals ----------
TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);
Adafruit_BME280 bme;
bool        bmeOk = false;

enum State : uint8_t { IDLE, PRINTING, DANGER, FINISHED };
State state = IDLE;

uint32_t aboveThresholdAt = 0;
uint32_t belowThresholdAt = 0;
uint32_t printStartAt     = 0;
uint32_t lastPrintMs      = 0;
uint32_t stateEnteredAt   = 0;

int32_t mqRaw     = 0;
float   mqBaseline = 500.0f;
int32_t mqDelta   = 0;
float   temp      = 0;
float   humidity  = 0;

// Fan control
bool     fanShouldBeOn   = false;
bool     fanIsOn         = false;
bool     fanCmdPending   = false;    // queued command waiting for BLE connect
uint32_t finishedOffAt    = 0;

// BLE state
enum BleState : uint8_t {
  BLE_STARTUP,    // init phase
  BLE_SCANNING,   // scanning for Shelly
  BLE_CONNECTING, // found, connecting
  BLE_CONNECTED,  // ready to send commands
  BLE_ERROR       // discovery failed, will retry
};
BleState bleState = BLE_STARTUP;

static BLEClient                *bleClient          = nullptr;
static BLERemoteCharacteristic  *charTx             = nullptr;
static BLERemoteCharacteristic  *charData           = nullptr;
static BLEAdvertisedDevice      *foundDevice        = nullptr;
static BLEScan                  *bleScanner         = nullptr;

uint32_t bleLastAttemptMs = 0;
uint32_t bleCmdCounter    = 0;   // 0..9 wrap-around, used in JSON id

// ---------- BLE client callbacks ----------
class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *) override {
    Serial.println("[BLE] onConnect");
  }
  void onDisconnect(BLEClient *) override {
    Serial.println("[BLE] onDisconnect");
    bleState = BLE_SCANNING;
    bleLastAttemptMs = 0;   // retry immediately
    charTx = nullptr;
    charData = nullptr;
  }
};

class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice adv) override {
    if (foundDevice != nullptr) return;   // already found one
    String desc = adv.toString().c_str();
    if (desc.indexOf(TARGET_DEVICE) >= 0) {
      Serial.printf("[BLE] MATCH: %s\n", desc.c_str());
      foundDevice = new BLEAdvertisedDevice(adv);
      bleScanner->stop();
      // Directly transition state — the async scan-complete callback is
      // unreliable when we stop the scan early from within onResult.
      bleState = BLE_CONNECTING;
    } else {
      // Only print unknown devices in early phase so user can find their Shelly
      static uint32_t seen = 0;
      if (seen < 20) {
        seen++;
        Serial.printf("[BLE] other: %s\n", desc.c_str());
      }
    }
  }
};

// ---------- BLE control ----------
bool bleConnectToServer() {
  if (foundDevice == nullptr) return false;
  Serial.printf("[BLE] Connecting to %s\n", foundDevice->getAddress().toString().c_str());
  bleClient = BLEDevice::createClient();
  bleClient->setClientCallbacks(new ClientCallbacks());
  if (!bleClient->connect(foundDevice)) {
    Serial.println("[BLE] connect failed");
    return false;
  }
  bleClient->setMTU(517);

  BLERemoteService *svc = bleClient->getService(SHELLY_SVC_UUID);
  if (!svc) {
    Serial.println("[BLE] Shelly RPC service not found");
    bleClient->disconnect();
    return false;
  }
  charTx   = svc->getCharacteristic(SHELLY_TX_UUID);
  charData = svc->getCharacteristic(SHELLY_DATA_UUID);
  if (!charTx || !charData) {
    Serial.println("[BLE] Shelly RPC characteristics not found");
    bleClient->disconnect();
    return false;
  }
  Serial.println("[BLE] CONNECTED and READY");
  return true;
}

bool bleSendSwitch(bool on) {
  if (bleState != BLE_CONNECTED || !charTx || !charData) return false;

  // Build JSON: {"id":N,"src":"esp32","method":"Switch.Set","params":{"id":0,"on":true }}}
  // Note: trailing space padding on "true "/"false" keeps the JSON length constant
  // so we can send a fixed 4-byte length header. The extra "}" at the end is a quirk
  // from the original article - Shelly parses it OK; keeping identical to proven code.
  char json[128];
  int n = snprintf(json, sizeof(json),
    "{\"id\":%u,\"src\":\"esp32\",\"method\":\"Switch.Set\",\"params\":{\"id\":0,\"on\":%s}}}",
    (unsigned)(bleCmdCounter % 10), on ? "true " : "false");
  bleCmdCounter++;

  // 4-byte big-endian size header
  uint8_t sizeHdr[4];
  sizeHdr[0] = 0;
  sizeHdr[1] = 0;
  sizeHdr[2] = (n >> 8) & 0xFF;
  sizeHdr[3] = n & 0xFF;

  charTx->writeValue(sizeHdr, 4);
  charData->writeValue((uint8_t *)json, n);
  Serial.printf("[BLE] Sent %s: %s\n", on ? "ON" : "OFF", json);
  return true;
}

void bleStartScan() {
  if (foundDevice) {
    delete foundDevice;
    foundDevice = nullptr;
  }
  Serial.println("[BLE] Starting scan...");
  bleScanner->start(6, [](BLEScanResults) {
    // async scan complete
    if (foundDevice) {
      bleState = BLE_CONNECTING;
    } else {
      Serial.println("[BLE] Scan finished, no target found - will retry");
      bleState = BLE_SCANNING;
      bleLastAttemptMs = millis();
    }
  }, false);
  bleState = BLE_SCANNING;
}

void bleTask() {
  switch (bleState) {
    case BLE_SCANNING:
      // Restart scan every 10s if idle
      if (millis() - bleLastAttemptMs >= 10000 && foundDevice == nullptr) {
        bleLastAttemptMs = millis();
        bleStartScan();
      }
      break;

    case BLE_CONNECTING:
      if (bleConnectToServer()) {
        bleState = BLE_CONNECTED;
        // If we had a pending command, send it now
        if (fanCmdPending) {
          if (bleSendSwitch(fanShouldBeOn)) {
            fanIsOn = fanShouldBeOn;
            fanCmdPending = false;
          }
        }
      } else {
        bleState = BLE_SCANNING;
        bleLastAttemptMs = millis();
        if (foundDevice) { delete foundDevice; foundDevice = nullptr; }
      }
      break;

    case BLE_CONNECTED:
      if (fanCmdPending) {
        if (bleSendSwitch(fanShouldBeOn)) {
          fanIsOn = fanShouldBeOn;
          fanCmdPending = false;
        }
      }
      break;

    default: break;
  }
}

// ---------- Fan request ----------
void requestFan(bool on) {
  if (fanShouldBeOn == on && !fanCmdPending) return;
  fanShouldBeOn = on;
  fanCmdPending = true;
  Serial.printf("[FAN] request: %s (pending)\n", on ? "ON" : "OFF");
}

// ---------- Sensors ----------
void readSensors() {
  int64_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PIN_MQ135_AO);
  mqRaw = sum / 16;
  mqDelta = mqRaw - static_cast<int32_t>(mqBaseline);

  if (bmeOk) {
    temp     = bme.readTemperature();
    humidity = bme.readHumidity();
  }
}

void updateBaseline() {
  if (state != IDLE) return;
  static uint32_t lastMs = 0;
  if (millis() - lastMs < BASELINE_UPDATE_MS) return;
  lastMs = millis();
  float r = static_cast<float>(mqRaw);
  if (r < mqBaseline) mqBaseline += (r - mqBaseline) * BASELINE_ALPHA_DOWN;
  else                mqBaseline += (r - mqBaseline) * BASELINE_ALPHA_UP;
}

// ---------- State machine ----------
void enterState(State next) {
  state = next;
  stateEnteredAt = millis();
}

void updateState() {
  const uint32_t now = millis();
  switch (state) {
    case IDLE:
      if (mqDelta >= DELTA_PRINTING) {
        if (aboveThresholdAt == 0) aboveThresholdAt = now;
        if (now - aboveThresholdAt >= PRINTING_CONFIRM_MS) {
          printStartAt = aboveThresholdAt;
          enterState(PRINTING);
          belowThresholdAt = 0;
          requestFan(true);
        }
      } else {
        aboveThresholdAt = 0;
      }
      if (fanShouldBeOn && finishedOffAt && (int32_t)(now - finishedOffAt) >= 0) {
        requestFan(false);
        finishedOffAt = 0;
      }
      break;
    case PRINTING:
      if (mqDelta >= DELTA_DANGER) enterState(DANGER);
      else if (mqDelta < DELTA_IDLE) {
        if (belowThresholdAt == 0) belowThresholdAt = now;
        if (now - belowThresholdAt >= FINISHED_CONFIRM_MS) {
          lastPrintMs = belowThresholdAt - printStartAt;
          enterState(FINISHED);
          finishedOffAt = now + FAN_OFF_DELAY_MS;
        }
      } else {
        belowThresholdAt = 0;
      }
      break;
    case DANGER:
      if (mqDelta < DELTA_PRINTING) {
        enterState(PRINTING);
        belowThresholdAt = 0;
      }
      break;
    case FINISHED:
      if (now - stateEnteredAt >= FINISHED_SHOW_MS) {
        aboveThresholdAt = 0;
        belowThresholdAt = 0;
        enterState(IDLE);
      }
      break;
  }
}

// ---------- Buttons ----------
void handleButtons() {
  static uint32_t lastBtnMs = 0;
  if (millis() - lastBtnMs < 250) return;

  if (digitalRead(PIN_BTN_LEFT) == LOW) {
    // Toggle fan - press to switch ON/OFF
    bool newState = !fanShouldBeOn;
    Serial.printf("[BTN] LEFT: toggle FAN %s\n", newState ? "ON" : "OFF");
    requestFan(newState);
    lastBtnMs = millis();
    drawDisplay();
  } else if (digitalRead(PIN_BTN_RIGHT) == LOW) {
    Serial.printf("[BTN] RIGHT: snap baseline %.0f -> %ld + FAN OFF\n", mqBaseline, (long)mqRaw);
    mqBaseline = static_cast<float>(mqRaw);
    mqDelta = 0;
    aboveThresholdAt = 0;
    belowThresholdAt = 0;
    finishedOffAt = 0;
    enterState(IDLE);
    requestFan(false);
    lastBtnMs = millis();
    drawDisplay();
  }
}

// ---------- Display ----------
void formatDuration(uint32_t ms, char *buf, size_t len) {
  uint32_t s = ms / 1000;
  uint32_t h = s / 3600, m = (s % 3600) / 60, sec = s % 60;
  if (h > 0) snprintf(buf, len, "%luh %02lum", h, m);
  else       snprintf(buf, len, "%lum %02lus", m, sec);
}

const char *bleStateLabel() {
  switch (bleState) {
    case BLE_STARTUP:    return "BLE:init";
    case BLE_SCANNING:   return "BLE:scan";
    case BLE_CONNECTING: return "BLE:conn";
    case BLE_CONNECTED:  return "BLE:OK";
    case BLE_ERROR:      return "BLE:err";
  }
  return "?";
}

void drawDisplay() {
  uint16_t bg;
  const char *label;
  switch (state) {
    case IDLE:     bg = TFT_DARKGREEN; label = "IDLE";       break;
    case PRINTING: bg = TFT_ORANGE;    label = "PRINTING";   break;
    case DANGER:   bg = TFT_RED;       label = "VENTILATE!"; break;
    case FINISHED: bg = TFT_BLUE;      label = "FINISHED";   break;
  }
  canvas.fillSprite(bg);
  canvas.setTextColor(TFT_WHITE, bg);
  canvas.setTextSize(3);
  canvas.drawString(label, 8, 6);

  canvas.setTextSize(2);
  char line[40];
  snprintf(line, sizeof(line), "MQ %ld  d%+ld", (long)mqRaw, (long)mqDelta);
  canvas.drawString(line, 8, 42);

  // Temperature / humidity if BME280 present, otherwise baseline
  if (bmeOk) {
    snprintf(line, sizeof(line), "%.1fC  %.0f%%", temp, humidity);
  } else {
    snprintf(line, sizeof(line), "base %.0f", mqBaseline);
  }
  canvas.drawString(line, 8, 66);

  // Duration
  char dur[24] = "";
  const char *prefix = nullptr;
  if (state == PRINTING || state == DANGER) {
    formatDuration(millis() - printStartAt, dur, sizeof(dur));
    prefix = "Time";
  } else if (state == FINISHED) {
    formatDuration(lastPrintMs, dur, sizeof(dur));
    prefix = "Done";
  } else if (lastPrintMs > 0) {
    formatDuration(lastPrintMs, dur, sizeof(dur));
    prefix = "Last";
  }
  if (prefix) {
    snprintf(line, sizeof(line), "%s %s", prefix, dur);
    canvas.drawString(line, 8, 95);
  }

  // Fan status (right side)
  const char *fanLabel;
  uint16_t fanColor;
  if (fanCmdPending)  { fanLabel = "FAN ?";   fanColor = TFT_YELLOW; }
  else if (fanIsOn)   { fanLabel = "FAN ON";  fanColor = TFT_WHITE; }
  else                { fanLabel = "FAN OFF"; fanColor = TFT_LIGHTGREY; }
  canvas.setTextColor(fanColor, bg);
  canvas.drawString(fanLabel, 145, 95);

  // BLE status (bottom)
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_LIGHTGREY, bg);
  canvas.drawString(bleStateLabel(), 8, 122);
  canvas.drawString(TARGET_DEVICE, 60, 122);

  canvas.pushSprite(0, 0);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Resin Printer Detector (BLE direct) ===");
  Serial.printf("Looking for: \"%s\"\n", TARGET_DEVICE);

  pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ135_AO, ADC_11db);

  // BME280 (I2C at 0x76 with SDO->GND, or 0x77 with SDO->3V3)
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  bmeOk = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
  Serial.printf("BME280: %s\n", bmeOk ? "OK" : "FAIL (check CSB->3V3, SDO->GND)");

  // Initial baseline
  delay(100);
  int64_t sum = 0;
  for (int i = 0; i < 32; i++) { sum += analogRead(PIN_MQ135_AO); delay(5); }
  mqBaseline = static_cast<float>(sum / 32);
  Serial.printf("MQ initial: %.0f\n", mqBaseline);

  // BLE init
  BLEDevice::init("resin-detector");
  bleScanner = BLEDevice::getScan();
  bleScanner->setAdvertisedDeviceCallbacks(new ScanCallbacks());
  bleScanner->setInterval(1349);
  bleScanner->setWindow(449);
  bleScanner->setActiveScan(true);
  bleStartScan();

  tft.init();
  tft.setRotation(1);
  canvas.createSprite(240, 135);
  canvas.setTextDatum(TL_DATUM);

  stateEnteredAt = millis();
  drawDisplay();
}

// ---------- Main loop ----------
void loop() {
  static uint32_t lastTickMs = 0;
  handleButtons();
  bleTask();

  if (millis() - lastTickMs >= 1000) {
    lastTickMs = millis();
    readSensors();
    updateBaseline();
    updateState();
    drawDisplay();

    Serial.printf("state=%u  MQ=%ld  base=%.0f  delta=%+ld  T=%.1f  RH=%.1f  fan=%d  ble=%s  pending=%d\n",
                  state, (long)mqRaw, mqBaseline, (long)mqDelta,
                  temp, humidity,
                  fanIsOn ? 1 : 0, bleStateLabel(), fanCmdPending ? 1 : 0);
  }
}
