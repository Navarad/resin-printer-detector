# Resin Printer Detector

Detects when a resin 3D printer starts and finishes by watching the VOC spike
from curing resin, then automatically toggles a Shelly Plug S Gen3 relay to
control an exhaust fan. Runs entirely on a LilyGo T-Display ESP32.

Communication with the Shelly is done over **direct BLE RPC** - no Wi-Fi, no
router, no cloud, no Shelly script or scene required. The ESP32 acts as a BLE
central, connects to the Shelly's built-in RPC service, and writes JSON-RPC
commands to switch the relay.

## Hardware

- LilyGo T-Display ESP32 (1.14" TFT, 240x135)
- MQ-135 gas sensor (analog VOC / air quality)
- Bosch BME280 (temperature / humidity / pressure over I2C) - optional but
  the display expects it
- Shelly Plus Plug S MTR Gen3 (or any BLE-enabled Shelly relay with firmware
  supporting RPC over BLE)
- 5V exhaust fan plugged into the Shelly

### Wiring

```
BME280           LilyGo T-Display
─────────────────────────────────
VCC         →    3V3
GND         →    GND
SCL / SCK   →    GPIO 22
SDA         →    GPIO 21
CSB         →    3V3    (forces I2C mode)
SDO         →    GND    (I2C address 0x76)

MQ-135           LilyGo T-Display
─────────────────────────────────
VCC         →    3V3    (avoids the 5V-on-3.3V-ADC issue)
GND         →    GND
AO          →    GPIO 32
DO          →    unused
```

At VCC=3V3 the MQ-135 heater runs cooler, sensitivity is lower and the raw
ADC baseline sits around 250-400 counts. That's fine for detecting resin
printers because they emit strong VOC. If you want more headroom, power the
MQ-135 with 5V and add a 10k+10k voltage divider on AO before wiring it to
GPIO 32.

## Software

Arduino IDE 2.x with:

- **ESP32 by Espressif Systems** - board package (tested with 2.0.17)
- **TFT_eSPI** (Bodmer) - display driver. In
  `libraries/TFT_eSPI/User_Setup_Select.h` comment out the default
  `#include <User_Setup.h>` and enable `#include <User_Setups/Setup25_TTGO_T_Display.h>`
- **Adafruit BME280 Library** (pulls in `Adafruit Unified Sensor` and
  `Adafruit BusIO`)
- BLE, Wire, and other core libraries - bundled with the ESP32 core

Board settings: **ESP32 Dev Module**, PSRAM disabled, flash 4MB, partition
scheme "Default 4MB with spiffs", upload speed 460800 (higher speeds tend to
fail on the T-Display's USB-serial bridge).

## Setup

1. Flash the sketch in `resin_printer_detector_bt/`.
2. On boot the T-Display will print all nearby BLE devices to the serial
   monitor.
3. Find your Shelly in the list - look for a line like
   `[BLE] other: Name: ShellyPlugSG3-D0CF13DA7744, Address: d0:cf:13:da:77:46, ...`
4. Copy the MAC address (or the device name).
5. Paste it into `TARGET_DEVICE` at the top of the sketch:
   ```cpp
   constexpr char TARGET_DEVICE[] = "d0:cf:13:da:77:46";  // or "ShellyPlugSG3-D0CF13DA7744"
   ```
   The MAC form is more reliable - it survives firmware quirks where the
   device name occasionally arrives corrupted.
6. Re-flash. Within a few seconds the display should show `BLE:OK` at the
   bottom and the ESP32 is connected to your Shelly.

On the Shelly side, only two settings matter:

- **Settings → Bluetooth → Enable** ✅
- **Settings → Bluetooth → RPC** ✅

No scenes, actions, scripts or authentication are required.

## Detection logic

The sketch runs a small state machine driven by the MQ-135 `delta`, i.e. the
ADC reading minus a rolling baseline learned during idle periods:

```
IDLE ─delta ≥ DELTA_PRINTING for 60s─→ PRINTING ─delta ≥ DELTA_DANGER──→ DANGER
                                          │                                │
                                          │ delta < DELTA_IDLE for 5min    │
                                          ↓                                │
                                      FINISHED ←────delta < PRINTING──────┘
                                          │
                                          │ show summary 30s
                                          ↓
                                        IDLE
```

The fan turns ON on entry to `PRINTING`, keeps running through `DANGER`, and
stays on for 10 minutes after entering `FINISHED` to air out remaining resin
fumes.

Default thresholds (at VCC=3V3):

```cpp
DELTA_PRINTING = 50    // above baseline -> assume printer running
DELTA_DANGER   = 150   // above baseline -> ventilate warning
DELTA_IDLE     = 25    // below this -> heading back to idle
```

Bump these 3-5x if you drive the MQ-135 at 5V.

## Display

```
┌────────────────────────────┐
│ PRINTING                   │  ← state (color coded)
│ MQ 412  d+120              │  ← current ADC, delta vs baseline
│ 28.5C  42%                 │  ← temperature / humidity from BME280
│ Time 12m 34s     FAN ON    │  ← print duration, fan state
│ BLE:OK  ShellyPlugSG3      │  ← BLE connection status + target
└────────────────────────────┘
```

State colors: green (idle), orange (printing), red (danger), blue (finished).

## Buttons

- **Left (GPIO 0)** - toggle fan on/off manually. Handy for testing without
  waiting for the state machine.
- **Right (GPIO 35)** - snap the baseline to the current MQ reading and
  reset the state machine to `IDLE`. Also turns the fan off. Use this after
  airing out the room or when the reading has drifted.

## Calibration

The MQ-135 needs a **long burn-in period** the first time you power it:

- **24-48 hours** continuous running when brand new. Readings drift heavily
  until then.
- **15-30 minutes** every subsequent power-on before the baseline settles.

During the burn-in, the baseline learner (rolling minimum) tracks the
readings down. Once stable, keep the ESP32 running continuously.

For real thresholds, watch the serial output during your first real print:

```
state=1  MQ=537  base=280  delta=+257  T=29.1  RH=41  fan=1  ble=BLE:OK
```

Set `DELTA_PRINTING` to roughly half the peak delta you observed. Too low
gives false triggers on breathing / cooking / perfume; too high delays
detection.

## Repository layout

```
resin_printer_detector_bt/         ← the working project
├── resin_printer_detector_bt.ino  ← main sketch
└── shelly_script.js               ← unused; kept from a BTHome experiment

_archive/                          ← unused prototypes (see below)
├── resin_printer_detector/
└── resin_printer_detector_mq135/
```

### About the `_archive/` folder

Two earlier sketches that took a Wi-Fi-based approach:

- `_archive/resin_printer_detector/` - Sensirion SGP40 (I2C VOC sensor) with
  Wi-Fi AP + HTTP RPC. The SGP40 died during development so this variant was
  never end-to-end tested for reliability.
- `_archive/resin_printer_detector_mq135/` - MQ-135 with Wi-Fi AP + HTTP RPC.
  Includes a captive DNS server hack that stops Shelly's connectivity
  watchdog from dropping the AP connection.

Both **do actually work** but Wi-Fi AP on ESP32 turned out to be flaky enough
in practice (association drops, retry loops, 25-second reconnect cycles) that
the BLE approach replaced them. They are kept here as reference in case
someone wants to explore that direction. Neither has been re-tested against
recent Shelly firmware. Each has its own git-ignored `secrets.h` for AP
credentials.

## Attribution

The direct BLE RPC approach - service and characteristic UUIDs, JSON framing
with a 4-byte big-endian length header, MAC-based device matching - is based
on [Rob__S's Instructables writeup](https://www.instructables.com/Controlling-Shelly-Home-Automation-Relays-From-an-/).
