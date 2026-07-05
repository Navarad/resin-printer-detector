/*
 * Resin Printer Detector - SIMPLIFIED DEBUG VERSION
 *
 * Stripped down to focus on Wi-Fi AP reliability + MQ-135 reading.
 * BME280 and Shelly HTTP calls are DISABLED for now.
 *
 * Shows on display:
 *   - MQ raw / baseline / delta
 *   - State (IDLE / PRINTING / DANGER / FINISHED)
 *   - Connected Wi-Fi clients with IP + MAC
 *
 * Buttons:
 *   GPIO 0  (left)  short  -> reset state to IDLE
 *                   long   -> restart Wi-Fi AP (2s hold)
 *   GPIO 35 (right)        -> snap baseline + reset state
 */

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <DNSServer.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <tcpip_adapter.h>
#include <TFT_eSPI.h>
#include "secrets.h"   // provides SECRET_AP_SSID, SECRET_AP_PASSWORD, SECRET_SHELLY_IP

// ---------- Thresholds ----------
constexpr int32_t  DELTA_PRINTING = 50;
constexpr int32_t  DELTA_DANGER   = 150;
constexpr int32_t  DELTA_IDLE     = 25;
constexpr uint32_t PRINTING_CONFIRM_MS = 60UL * 1000UL;
constexpr uint32_t FINISHED_CONFIRM_MS = 5UL * 60UL * 1000UL;
constexpr uint32_t FINISHED_SHOW_MS    = 30UL * 1000UL;

constexpr uint32_t BASELINE_UPDATE_MS = 5UL * 1000UL;
constexpr float    BASELINE_ALPHA_UP   = 0.001f;
constexpr float    BASELINE_ALPHA_DOWN = 0.05f;

// ---------- Wi-Fi AP ----------
constexpr char     AP_SSID[]     = SECRET_AP_SSID;
constexpr char     AP_PASSWORD[] = SECRET_AP_PASSWORD;

// ---------- Pins ----------
constexpr uint8_t PIN_BTN_LEFT  = 0;
constexpr uint8_t PIN_BTN_RIGHT = 35;
constexpr uint8_t PIN_MQ135_AO  = 32;

// ---------- Globals ----------
TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);

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

// Connected client info (only tracks one - first client)
String   clientIP  = "";
String   clientMAC = "";
uint32_t clientConnectedAt = 0;      // millis() when client first appeared

// Wi-Fi AP watchdog
uint32_t lastClientSeenMs = 0;
constexpr uint32_t AP_RESTART_TIMEOUT_MS = 45UL * 1000UL;
uint32_t apRestartCount = 0;

// DNS server - responds to all DNS queries with 192.168.4.1 (this AP).
// Prevents Shelly from timing out on internet connectivity checks.
DNSServer dnsServer;

// Shelly HTTP control
bool     fanShouldBeOn   = false;
bool     fanIsOn         = false;
bool     fanCmdPending   = false;
uint32_t fanLastAttemptMs = 0;
uint32_t finishedOffAt    = 0;
constexpr uint32_t FAN_OFF_DELAY_MS = 10UL * 60UL * 1000UL;
constexpr uint32_t SHELLY_RETRY_MS  = 3UL * 1000UL;

// ---------- Wi-Fi AP setup + restart ----------
void startAP() {
  WiFi.softAPdisconnect(true);
  delay(100);
  IPAddress apIP(192, 168, 4, 1);
  IPAddress apNet(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apIP, apNet);
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, 11, false, 4);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.printf("Wi-Fi AP \"%s\": %s, IP=%s, ch=11\n",
                AP_SSID, ok ? "OK" : "FAIL",
                WiFi.softAPIP().toString().c_str());
}

void restartAP() {
  apRestartCount++;
  Serial.printf("\n!!! AP restart #%lu (uptime %lus, %lus since last client) !!!\n",
                (unsigned long)apRestartCount,
                (unsigned long)(millis() / 1000),
                (unsigned long)((millis() - lastClientSeenMs) / 1000));
  // Softer restart - avoids the esp_wifi_get_mac error we saw before
  WiFi.softAPdisconnect(true);
  delay(300);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  startAP();
  lastClientSeenMs = millis();
  clientIP  = "";
  clientMAC = "";
  clientConnectedAt = 0;
}

void refreshClientInfo() {
  wifi_sta_list_t sta;
  esp_wifi_ap_get_sta_list(&sta);
  tcpip_adapter_sta_list_t staIp;
  tcpip_adapter_get_sta_list(&sta, &staIp);

  if (staIp.num > 0 && staIp.sta[0].ip.addr != 0) {
    String newIP = IPAddress(staIp.sta[0].ip.addr).toString();
    char macBuf[18];
    snprintf(macBuf, sizeof(macBuf), "%02X:%02X:%02X:%02X:%02X:%02X",
             staIp.sta[0].mac[0], staIp.sta[0].mac[1], staIp.sta[0].mac[2],
             staIp.sta[0].mac[3], staIp.sta[0].mac[4], staIp.sta[0].mac[5]);
    String newMAC(macBuf);

    if (newIP != clientIP || newMAC != clientMAC) {
      Serial.printf("Client info updated: MAC=%s IP=%s\n",
                    newMAC.c_str(), newIP.c_str());
      clientIP  = newIP;
      clientMAC = newMAC;
    }
  } else if (clientIP.length() > 0) {
    Serial.println("Client disconnected - clearing info");
    clientIP  = "";
    clientMAC = "";
  }
}

void apWatchdog() {
  if (WiFi.softAPgetStationNum() > 0) {
    lastClientSeenMs = millis();
    return;
  }
  if (millis() - lastClientSeenMs >= AP_RESTART_TIMEOUT_MS) {
    restartAP();
  }
}

// ---------- Shelly HTTP control ----------
bool shellySetSwitch(bool on) {
  if (clientIP.length() == 0) {
    Serial.printf("Shelly %s -> skipped (no client)\n", on ? "ON" : "OFF");
    return false;
  }
  HTTPClient http;
  String url = String("http://") + clientIP +
               "/rpc/Switch.Set?id=0&on=" + (on ? "true" : "false");
  http.setConnectTimeout(2000);
  http.begin(url);
  int code = http.GET();
  http.end();
  Serial.printf("Shelly (%s) %s -> HTTP %d\n", clientIP.c_str(), on ? "ON" : "OFF", code);
  return code == 200;
}

void syncFan() {
  if (!fanCmdPending) return;
  if (millis() - fanLastAttemptMs < SHELLY_RETRY_MS) return;
  fanLastAttemptMs = millis();
  if (shellySetSwitch(fanShouldBeOn)) {
    fanIsOn = fanShouldBeOn;
    fanCmdPending = false;
  }
}

void requestFan(bool on) {
  if (fanShouldBeOn == on) return;
  fanShouldBeOn = on;
  fanCmdPending = true;
  fanLastAttemptMs = 0;
}

// Guess device type from MAC OUI (first 3 bytes)
const char* guessDevice(const String& mac) {
  if (mac.length() < 8) return "?";
  String oui = mac.substring(0, 8);
  if (oui == "D0:CF:13") return "Shelly";
  if (oui.startsWith("EA:") || oui.startsWith("DE:") ||
      oui.startsWith("F2:") || oui.startsWith("C2:")) return "Phone(random)";
  if (oui == "A0:DD:6C" || oui == "AC:0B:FB" ||
      oui == "A4:83:E7" || oui == "94:E9:79") return "Apple";
  return "Unknown";
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Resin Printer Detector (DEBUG) ===");

  // Silence harmless ESP-IDF warning during AP restart transitions
  esp_log_level_set("wifi_init_default", ESP_LOG_NONE);
  esp_log_level_set("wifi", ESP_LOG_WARN);

  pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ135_AO, ADC_11db);

  // BME280 disabled for debugging - uncomment when ready
  // Wire.begin(21, 22);
  // ...

  // Initial MQ baseline
  delay(100);
  int64_t sum = 0;
  for (int i = 0; i < 32; i++) { sum += analogRead(PIN_MQ135_AO); delay(5); }
  mqBaseline = static_cast<float>(sum / 32);
  Serial.printf("MQ-135 initial: %.0f\n", mqBaseline);

  // Wi-Fi event handlers - detailed logging
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
      case ARDUINO_EVENT_WIFI_AP_STACONNECTED: {
        clientConnectedAt = millis();
        auto &e = info.wifi_ap_staconnected;
        char mac[18];
        snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 e.mac[0], e.mac[1], e.mac[2], e.mac[3], e.mac[4], e.mac[5]);
        Serial.printf(">>> CONNECTED  MAC=%s  AID=%u  t=%lus\n",
                      mac, e.aid, (unsigned long)(millis()/1000));
        break;
      }
      case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: {
        auto &e = info.wifi_ap_stadisconnected;
        char mac[18];
        snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 e.mac[0], e.mac[1], e.mac[2], e.mac[3], e.mac[4], e.mac[5]);
        uint32_t sessionSec = 0;
        if (clientConnectedAt > 0) sessionSec = (millis() - clientConnectedAt) / 1000;
        Serial.printf("<<< DISCONNECTED  MAC=%s  AID=%u  session=%lus  t=%lus\n",
                      mac, e.aid, (unsigned long)sessionSec,
                      (unsigned long)(millis()/1000));
        clientConnectedAt = 0;
        break;
      }
      case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED: {
        auto &e = info.wifi_ap_staipassigned;
        Serial.printf("    IP assigned: %d.%d.%d.%d\n",
                      (int)((e.ip.addr >> 0) & 0xFF),
                      (int)((e.ip.addr >> 8) & 0xFF),
                      (int)((e.ip.addr >> 16) & 0xFF),
                      (int)((e.ip.addr >> 24) & 0xFF));
        break;
      }
      case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED: {
        // Fired when a device sees our AP and asks about it (before connecting)
        // Useful to detect "someone is trying but not connecting"
        static uint32_t lastProbe = 0;
        if (millis() - lastProbe > 5000) {   // log at most every 5s to avoid spam
          lastProbe = millis();
          Serial.println("    probe request received (someone scanning)");
        }
        break;
      }
      default: break;
    }
  });

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  startAP();

  // DNS server catches all DNS queries -> answers with our AP IP (192.168.4.1).
  // Purpose: prevent Shelly from timing out on internet connectivity checks.
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));
  Serial.println("DNS server started (captures all queries)");

  tft.init();
  tft.setRotation(1);
  canvas.createSprite(240, 135);
  canvas.setTextDatum(TL_DATUM);

  stateEnteredAt = millis();
  lastClientSeenMs = millis();
  drawDisplay();
}

// ---------- Sensors ----------
void readSensors() {
  int64_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PIN_MQ135_AO);
  mqRaw = sum / 16;
  mqDelta = mqRaw - static_cast<int32_t>(mqBaseline);
}

void updateBaseline() {
  if (state != IDLE) return;
  static uint32_t lastMs = 0;
  if (millis() - lastMs < BASELINE_UPDATE_MS) return;
  lastMs = millis();

  float r = static_cast<float>(mqRaw);
  if (r < mqBaseline) {
    mqBaseline += (r - mqBaseline) * BASELINE_ALPHA_DOWN;
  } else {
    mqBaseline += (r - mqBaseline) * BASELINE_ALPHA_UP;
  }
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
  static uint32_t leftBtnPressStart = 0;

  // Long-press LEFT (>2s) -> AP restart
  if (digitalRead(PIN_BTN_LEFT) == LOW) {
    if (leftBtnPressStart == 0) leftBtnPressStart = millis();
    if (millis() - leftBtnPressStart > 2000 && millis() - lastBtnMs > 250) {
      Serial.println("LEFT long-press: AP restart");
      restartAP();
      lastBtnMs = millis();
      leftBtnPressStart = 0;
      drawDisplay();
      return;
    }
  } else {
    leftBtnPressStart = 0;
  }

  if (millis() - lastBtnMs < 250) return;

  if (digitalRead(PIN_BTN_LEFT) == LOW) {
    Serial.println("LEFT: reset state -> IDLE");
    aboveThresholdAt = 0;
    belowThresholdAt = 0;
    enterState(IDLE);
    lastBtnMs = millis();
    drawDisplay();
  } else if (digitalRead(PIN_BTN_RIGHT) == LOW) {
    Serial.printf("RIGHT: snap baseline %.0f -> %ld\n", mqBaseline, (long)mqRaw);
    mqBaseline = static_cast<float>(mqRaw);
    mqDelta = 0;
    aboveThresholdAt = 0;
    belowThresholdAt = 0;
    enterState(IDLE);
    lastBtnMs = millis();
    drawDisplay();
  }
}

// ---------- Display ----------
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

  // Top: state label
  canvas.setTextSize(3);
  canvas.drawString(label, 8, 6);

  // MQ readings
  canvas.setTextSize(2);
  char line[40];
  snprintf(line, sizeof(line), "MQ %ld  d%+ld", (long)mqRaw, (long)mqDelta);
  canvas.drawString(line, 8, 42);

  snprintf(line, sizeof(line), "base %.0f", mqBaseline);
  canvas.drawString(line, 8, 66);

  // Wi-Fi status - prominently shown
  canvas.setTextSize(1);
  uint8_t clientCount = WiFi.softAPgetStationNum();
  if (clientCount > 0 && clientIP.length() > 0) {
    canvas.setTextColor(TFT_GREENYELLOW, bg);
    snprintf(line, sizeof(line), "%s (%s)", guessDevice(clientMAC), clientIP.c_str());
    canvas.drawString(line, 8, 95);
    canvas.drawString(clientMAC.c_str(), 8, 108);
  } else {
    canvas.setTextColor(TFT_YELLOW, bg);
    uint32_t secSinceClient = (millis() - lastClientSeenMs) / 1000;
    snprintf(line, sizeof(line), "no client (%lus)", (unsigned long)secSinceClient);
    canvas.drawString(line, 8, 95);
    canvas.drawString(WiFi.softAPIP().toString().c_str(), 8, 108);
  }

  // Bottom status line - AP name + fan status
  canvas.setTextColor(TFT_LIGHTGREY, bg);
  const char *fanLabel;
  if (fanCmdPending)     fanLabel = "FAN ?";
  else if (fanIsOn)      fanLabel = "FAN ON";
  else                   fanLabel = "FAN OFF";
  snprintf(line, sizeof(line), "%s  %s", AP_SSID, fanLabel);
  canvas.drawString(line, 8, 122);

  canvas.pushSprite(0, 0);
}

// ---------- Main loop ----------
void loop() {
  static uint32_t lastTickMs = 0;
  handleButtons();
  dnsServer.processNextRequest();   // answer DNS queries (Shelly connectivity check)
  syncFan();                         // push pending Shelly command with retry

  if (millis() - lastTickMs >= 1000) {
    lastTickMs = millis();
    readSensors();
    updateBaseline();
    updateState();
    refreshClientInfo();
    apWatchdog();
    drawDisplay();

    Serial.printf("state=%u  MQ=%ld  base=%.0f  delta=%+ld  clients=%d  IP=%s  fan=%d\n",
                  state, (long)mqRaw, mqBaseline, (long)mqDelta,
                  WiFi.softAPgetStationNum(),
                  clientIP.length() > 0 ? clientIP.c_str() : "-",
                  fanIsOn ? 1 : 0);
  }
}
