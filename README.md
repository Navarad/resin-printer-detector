# Resin Printer Detector

Detects when a resin 3D printer starts and finishes by watching the VOC spike
from curing resin, then automatically switches a Shelly Plug S Gen3 relay to
control an exhaust fan. Runs entirely on a LilyGo T-Display ESP32.

Three variants in this repo track how the solution evolved. Use the **BLE
variant** for new builds - it is the most reliable of the three.

## Hardware

- LilyGo T-Display ESP32 (1.14" TFT, 240x135)
- One of: Sensirion SGP40 / SGP41 (VOC, I2C) **or** MQ-135 (analog)
- Bosch BME280 (temperature / humidity / pressure, I2C) - optional
- Shelly Plus Plug S MTR Gen3 (or any BLE-capable Shelly Plug S / 1 Mini Gen3+)
- 5V exhaust fan plugged into the Shelly

Wiring:

```
BME280           LilyGo T-Display
─────────────────────────────────
VCC         →    3V3
GND         →    GND
SCL / SCK   →    GPIO 22
SDA         →    GPIO 21
CSB         →    3V3    (forces I2C mode)
SDO         →    GND    (I2C address 0x76)

MQ-135 (or SGP40) LilyGo
─────────────────────────
VCC          →    3V3       (5V for MQ-135 needs a voltage divider on AO)
GND          →    GND
AO / SDA     →    GPIO 32 for MQ-135, GPIO 21 for SGP40
SCL          →    GPIO 22 (SGP40 only)
```

## Variants

### `resin_printer_detector_bt/` - direct BLE control (recommended)

The T-Display connects to Shelly via BLE and writes JSON-RPC directly to the
Shelly RPC service. No Wi-Fi, no Cloud, no Shelly scripts required.

Shelly setup: **Settings → Bluetooth → Enable + RPC**. That's it.

Also included is a `shelly_script.js` for the BTHome broadcast approach - kept
for reference, not required by the current `.ino`.

### `resin_printer_detector_mq135/` - Wi-Fi AP + HTTP RPC (MQ-135)

The T-Display creates its own Wi-Fi access point (`ResinFanCtl`) and Shelly
connects to it as a client. The T-Display then sends HTTP RPC to Shelly.

Includes a DNS server on the ESP32 that answers all queries with its own IP,
which prevents Shelly's connectivity watchdog from timing out and dropping
the connection.

Fill in `secrets.h` (see `secrets.h.example`) before compiling.

### `resin_printer_detector/` - Wi-Fi AP + HTTP RPC (SGP40)

Same architecture as the MQ-135 variant but with a Sensirion SGP40 (I2C VOC
sensor). Left in the repo mainly for archaeological reasons - Wi-Fi AP mode on
ESP32 turned out to be unreliable enough that the BLE variant replaced it.

Fill in `secrets.h` before compiling.

## Setup and flashing

Arduino IDE 2.x with the following libraries:

- `TFT_eSPI` (Bodmer) - configure `User_Setup_Select.h` to enable
  `Setup25_TTGO_T_Display.h`
- `Adafruit BME280 Library`
- `Sensirion I2C SGP40` and `Sensirion Gas Index Algorithm` (SGP40 variant only)
- BLE, WiFi, HTTPClient, DNSServer - bundled with the ESP32 Arduino core

Board: **ESP32 Dev Module**, PSRAM disabled, flash 4MB, partition scheme
"Default 4MB with spiffs", upload speed 460800.

For Wi-Fi variants, copy `secrets.h.example` to `secrets.h` and set your AP
SSID and password. `secrets.h` is git-ignored.

## Detection tuning

The MQ-135 delta thresholds in the code are calibrated for VCC=3V3. Bump them
if you power the MQ-135 with 5V through a voltage divider (roughly 3-5x higher
readings). Watch the serial output during your first real print and set
`DELTA_PRINTING` to about half the observed peak `delta` value.

Baseline is learned in the background while the state is `IDLE` (rolling
minimum-tracking filter). Snap it manually to the current reading by pressing
the right button on the T-Display.

## State machine

```
IDLE ─delta ≥ PRINTING for 60s─→ PRINTING ─delta ≥ DANGER─→ DANGER
                                     │                        │
                                     │ delta < IDLE for 5min  │
                                     ↓                        │
                                 FINISHED ←─delta < PRINTING──┘
                                     │
                                     │ show summary 30s
                                     ↓
                                   IDLE
```

Fan turns ON on entry to `PRINTING`, keeps running through `DANGER`, and stays
on for 10 minutes after entering `FINISHED` to air out remaining resin fumes.

## References

The BLE direct-control approach is based on Rob__S's Instructables article:
[Controlling Shelly Relays From an ESP32 Using BLE](https://www.instructables.com/Controlling-Shelly-Home-Automation-Relays-From-an-/).

The Shelly BLE RPC service and characteristic UUIDs used here come from the
Shelly mongoose-OS RPC-over-BLE implementation.
