/*
 * Resin Printer Detector
 *
 * Detects when a resin 3D printer starts and finishes by watching the
 * VOC spike from curing resin. Runs offline on a LilyGo T-Display ESP32.
 *
 * Hardware:
 *   - LilyGo T-Display ESP32 (1.14" TFT, 240x135)
 *   - Sensirion SGP40 (VOC)     - I2C 0x59
 *   - Bosch BME280  (T/RH/P)    - I2C 0x76 or 0x77
 *
 * Wiring (both sensors share the I2C bus):
 *   ESP32 GPIO 21 (SDA) -> SDA on BME280 + SGP40
 *   ESP32 GPIO 22 (SCL) -> SCL on BME280 + SGP40
 *   3.3V                -> VIN/VCC on both
 *   GND                 -> GND  on both
 *
 * Buttons (on T-Display):
 *   GPIO 0  (left)  -> force back to IDLE / clear summary
 *   GPIO 35 (right) -> reset VOC algorithm baseline (use after airing the room)
 *
 * Before compiling:
 *   1. Install via Library Manager:
 *        - TFT_eSPI by Bodmer
 *        - Adafruit BME280 Library  (pulls in Adafruit Unified Sensor, BusIO)
 *        - Sensirion I2C SGP40
 *        - Sensirion Gas Index Algorithm
 *   2. In TFT_eSPI/User_Setup_Select.h: enable Setup25_TTGO_T_Display.h
 *   3. Board: "ESP32 Dev Module", PSRAM disabled, Flash 4MB.
 *
 * Note on warm-up:
 *   The Sensirion VOC Index algorithm needs ~10 s to start producing
 *   meaningful values and learns the room baseline over a few hours.
 *   Leave the device running idle for a while before first real use.
 */

#include <new>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TFT_eSPI.h>
#include <Adafruit_BME280.h>
#include <SensirionI2CSgp40.h>
#include <VOCGasIndexAlgorithm.h>
#include "secrets.h"   // provides SECRET_AP_SSID, SECRET_AP_PASSWORD, SECRET_SHELLY_IP

// ---------- Thresholds (tune to your room) ----------
constexpr int32_t  VOC_PRINTING_THRESHOLD = 200;             // > this for a while -> PRINTING
constexpr int32_t  VOC_DANGER_THRESHOLD   = 400;             // > this -> VENTILATE
constexpr int32_t  VOC_IDLE_THRESHOLD     = 150;             // < this for a while -> back to IDLE
constexpr uint32_t PRINTING_CONFIRM_MS    = 60UL * 1000UL;   // 1 min above threshold to confirm
constexpr uint32_t FINISHED_CONFIRM_MS    = 5UL * 60UL * 1000UL;  // 5 min below threshold
constexpr uint32_t FINISHED_SHOW_MS       = 30UL * 1000UL;   // show summary for 30 s

// ---------- Wi-Fi AP (T-Display is the access point, Shelly is the client) ----------
constexpr char     AP_SSID[]       = SECRET_AP_SSID;
constexpr char     AP_PASSWORD[]   = SECRET_AP_PASSWORD;   // min 8 chars
constexpr char     SHELLY_IP[]     = SECRET_SHELLY_IP;     // static IP set in Shelly app
constexpr uint32_t FAN_OFF_DELAY_MS = 10UL * 60UL * 1000UL;  // keep fan on 10 min after FINISHED
constexpr uint32_t SHELLY_RETRY_MS  = 5UL * 1000UL;          // retry failed Shelly call every 5 s

// ---------- Pins ----------
constexpr uint8_t PIN_SDA       = 21;
constexpr uint8_t PIN_SCL       = 22;
constexpr uint8_t PIN_BTN_LEFT  = 0;
constexpr uint8_t PIN_BTN_RIGHT = 35;

// ---------- Globals ----------
TFT_eSPI  tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);   // off-screen buffer, flicker-free
Adafruit_BME280 bme;
SensirionI2CSgp40 sgp40;
VOCGasIndexAlgorithm vocAlgorithm;

enum State : uint8_t { IDLE, PRINTING, DANGER, FINISHED };
State state = IDLE;

uint32_t aboveThresholdAt = 0;     // when VOC first crossed up
uint32_t belowThresholdAt = 0;     // when VOC first dropped
uint32_t printStartAt     = 0;
uint32_t lastPrintMs      = 0;
uint32_t stateEnteredAt   = 0;

float   temp = 0, humidity = 0, pressure = 0;
int32_t vocIndex = 0;
bool    sensorsOk = false;

// Fan control
bool     fanShouldBeOn   = false;     // desired state derived from state machine
bool     fanIsOn         = false;     // last confirmed state on Shelly
bool     fanCmdPending   = false;     // current desired differs from confirmed
uint32_t fanLastAttemptMs = 0;
uint32_t finishedOffAt    = 0;        // millis() when fan should turn off after FINISHED
bool     shellyOnline     = false;    // last Shelly call succeeded

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Resin Printer Detector ===");

  pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT);   // GPIO 35 is input-only; T-Display has external pull-up

  // ---- I2C bus recovery (unstick any device holding SDA low) ----
  // Manually clock SCL 9x while SDA is released, then send a START/STOP.
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_SDA, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SDA, LOW);  delayMicroseconds(5);   // START
  digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_SDA, HIGH); delayMicroseconds(5);   // STOP

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(50000);          // 50 kHz — safer with Wi-Fi AP running
  Wire.setTimeOut(200);          // ms — wait longer for SGP40 to ACK

  // General-call reset (0x00 with 0x06) — hard-resets Sensirion sensors.
  Wire.beginTransmission(0x00);
  Wire.write(0x06);
  Wire.endTransmission();
  delay(50);

  // I2C bus scan – tells us exactly which devices respond
  Serial.println("Scanning I2C bus...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found device at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  NO DEVICES FOUND - check VCC/GND/SDA/SCL wiring and power");
  }

  // BME280 (expected at 0x76 with SDO->GND, or 0x77 with SDO->3V3)
  bool bmeOk = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
  Serial.printf("BME280: %s\n", bmeOk ? "OK" : "FAIL (check CSB->3V3, SDO->GND, VCC->3V3)");

  // SGP40 (fixed address 0x59)
  sgp40.begin(Wire);
  uint16_t sn[3] = {0};
  uint16_t sgpErr = sgp40.getSerialNumber(sn, 3);
  bool sgpOk = (sgpErr == 0);
  if (sgpOk) {
    Serial.printf("SGP40: OK (S/N %04X%04X%04X)\n", sn[0], sn[1], sn[2]);
  } else {
    Serial.printf("SGP40: FAIL (error 0x%X - check VIN->3V3, SDA/SCL)\n", sgpErr);
  }

  sensorsOk = bmeOk && sgpOk;

  // Wi-Fi AP for Shelly Plug
  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("Wi-Fi AP \"%s\": %s, IP=%s\n",
                AP_SSID, apOk ? "OK" : "FAIL",
                WiFi.softAPIP().toString().c_str());

  tft.init();
  tft.setRotation(1);
  canvas.createSprite(240, 135);
  canvas.setTextDatum(TL_DATUM);

  stateEnteredAt = millis();
  drawDisplay();
}

// ---------- Sensors ----------
uint16_t lastSgpErr = 0;
uint16_t lastSraw   = 0;

void readSensors() {
  temp     = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0f;

  // Compensation ticks per Sensirion datasheet
  uint16_t rhTicks = static_cast<uint16_t>(humidity * 65535.0f / 100.0f);
  uint16_t tTicks  = static_cast<uint16_t>((temp + 45.0f) * 65535.0f / 175.0f);

  uint16_t srawVoc = 0;
  lastSgpErr = sgp40.measureRawSignal(rhTicks, tTicks, srawVoc);
  if (lastSgpErr != 0) {                  // one retry after brief settle
    delay(40);
    lastSgpErr = sgp40.measureRawSignal(rhTicks, tTicks, srawVoc);
  }
  if (lastSgpErr == 0) {
    lastSraw = srawVoc;
    vocIndex = vocAlgorithm.process(srawVoc);
  } else {
    // On error: check if SGP40 is still on the I2C bus at all
    static uint32_t lastScanMs = 0;
    if (millis() - lastScanMs > 5000) {   // don't spam
      lastScanMs = millis();
      Wire.beginTransmission(0x59);
      uint8_t r = Wire.endTransmission();
      Wire.beginTransmission(0x76);
      uint8_t rB = Wire.endTransmission();
      Serial.printf("  I2C probe: SGP40 0x59 r=%u (%s), BME280 0x76 r=%u (%s)\n",
                    r,  r  == 0 ? "PRESENT" : "MISSING",
                    rB, rB == 0 ? "PRESENT" : "MISSING");
    }
  }
}

// ---------- Shelly control ----------
// Returns true on HTTP 200, false otherwise. Non-blocking enough at 2s timeout.
bool shellySetSwitch(bool on) {
  HTTPClient http;
  String url = String("http://") + SHELLY_IP +
               "/rpc/Switch.Set?id=0&on=" + (on ? "true" : "false");
  http.setConnectTimeout(2000);
  http.begin(url);
  int code = http.GET();
  http.end();
  Serial.printf("Shelly %s -> HTTP %d\n", on ? "ON" : "OFF", code);
  return code == 200;
}

void syncFan() {
  if (!fanCmdPending) return;
  if (millis() - fanLastAttemptMs < SHELLY_RETRY_MS) return;
  fanLastAttemptMs = millis();
  bool ok = shellySetSwitch(fanShouldBeOn);
  shellyOnline = ok;
  if (ok) {
    fanIsOn = fanShouldBeOn;
    fanCmdPending = false;
  }
}

void requestFan(bool on) {
  if (fanShouldBeOn == on && !fanCmdPending) return;
  fanShouldBeOn = on;
  fanCmdPending = true;
  fanLastAttemptMs = 0;   // try immediately
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
      if (vocIndex >= VOC_PRINTING_THRESHOLD) {
        if (aboveThresholdAt == 0) aboveThresholdAt = now;
        if (now - aboveThresholdAt >= PRINTING_CONFIRM_MS) {
          printStartAt = aboveThresholdAt;   // backdate to when VOC actually rose
          enterState(PRINTING);
          belowThresholdAt = 0;
          requestFan(true);
        }
      } else {
        aboveThresholdAt = 0;
      }
      // turn fan off once the post-print airing period is over
      if (fanShouldBeOn && finishedOffAt && (int32_t)(now - finishedOffAt) >= 0) {
        requestFan(false);
        finishedOffAt = 0;
      }
      break;

    case PRINTING:
      requestFan(true);
      if (vocIndex >= VOC_DANGER_THRESHOLD) {
        enterState(DANGER);
      } else if (vocIndex < VOC_IDLE_THRESHOLD) {
        if (belowThresholdAt == 0) belowThresholdAt = now;
        if (now - belowThresholdAt >= FINISHED_CONFIRM_MS) {
          lastPrintMs = belowThresholdAt - printStartAt;
          enterState(FINISHED);
          finishedOffAt = now + FAN_OFF_DELAY_MS;   // keep airing for 10 min
        }
      } else {
        belowThresholdAt = 0;
      }
      break;

    case DANGER:
      requestFan(true);
      if (vocIndex < VOC_PRINTING_THRESHOLD) {
        enterState(PRINTING);
        belowThresholdAt = 0;
      }
      break;

    case FINISHED:
      // fan keeps running through this state; will turn off in IDLE when finishedOffAt elapses
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
    aboveThresholdAt = 0;
    belowThresholdAt = 0;
    enterState(IDLE);
    lastBtnMs = millis();
  } else if (digitalRead(PIN_BTN_RIGHT) == LOW) {
    vocAlgorithm.~VOCGasIndexAlgorithm();              // reset baseline learning
    new (&vocAlgorithm) VOCGasIndexAlgorithm();
    lastBtnMs = millis();
  }
}

// ---------- Display ----------
void formatDuration(uint32_t ms, char *buf, size_t len) {
  uint32_t s = ms / 1000;
  uint32_t h = s / 3600;
  uint32_t m = (s % 3600) / 60;
  uint32_t sec = s % 60;
  if (h > 0) snprintf(buf, len, "%luh %02lum",  h, m);
  else       snprintf(buf, len, "%lum %02lus", m, sec);
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
  snprintf(line, sizeof(line), "VOC %ld", (long)vocIndex);
  canvas.drawString(line, 8, 42);

  snprintf(line, sizeof(line), "%.1fC  %.0f%%", temp, humidity);
  canvas.drawString(line, 8, 66);

  // Duration line
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

  // Fan status (bottom right)
  canvas.setTextSize(2);
  uint16_t fanColor;
  const char *fanLabel;
  if (fanCmdPending && !shellyOnline)  { fanColor = TFT_YELLOW; fanLabel = "FAN ?"; }
  else if (fanIsOn)                    { fanColor = TFT_WHITE;  fanLabel = "FAN ON"; }
  else                                 { fanColor = TFT_LIGHTGREY; fanLabel = "FAN OFF"; }
  canvas.setTextColor(fanColor, bg);
  canvas.drawString(fanLabel, 145, 95);

  // Client count (bottom right small)
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_LIGHTGREY, bg);
  char wifiLine[20];
  snprintf(wifiLine, sizeof(wifiLine), "wifi: %d client",
           WiFi.softAPgetStationNum());
  canvas.drawString(wifiLine, 145, 118);

  if (!sensorsOk) {
    canvas.setTextColor(TFT_YELLOW, bg);
    canvas.setTextSize(1);
    canvas.drawString("sensor err - check wiring", 8, 122);
  }

  canvas.pushSprite(0, 0);
}

// ---------- Main loop ----------
void loop() {
  static uint32_t lastTickMs = 0;
  handleButtons();
  syncFan();   // pushes commands to Shelly; retries on failure

  // SGP40 algorithm expects exactly 1 Hz sampling
  if (millis() - lastTickMs >= 1000) {
    lastTickMs = millis();
    if (sensorsOk) {
      readSensors();
      updateState();
    }
    drawDisplay();

    Serial.printf("state=%u  VOC=%ld  sraw=%u  sgpErr=0x%X  T=%.1f  RH=%.1f  fan=%d clients=%d\n",
                  state, (long)vocIndex, lastSraw, lastSgpErr,
                  temp, humidity,
                  fanIsOn ? 1 : 0, WiFi.softAPgetStationNum());
  }
}
