// Shelly Gen3 script - listens for ResinFanBT BLE beacon and toggles Switch 0
//
// Setup:
//   1. In Shelly UI open Scripts menu
//   2. Click "New" (or "Add script")
//   3. Paste this ENTIRE file into the editor
//   4. Save
//   5. Click "Start" (or the play icon) to run
//   6. Enable "Run on startup" so it survives reboots
//
// What it does:
//   - Scans BLE continuously (low power, passive scan)
//   - Filters for our device name "ResinFanBT"
//   - Parses BTHome v2 service data (UUID 0xFCD2)
//   - When it sees button press (0x01)      -> turns Switch 0 ON
//   - When it sees button long press (0x04) -> turns Switch 0 OFF
//   - Debounces repeat events (won't fire twice within 3 seconds)

let CONFIG = {
  targetName: "ResinFanBT",   // must match BLE_DEVICE_NAME in ESP32 firmware
  bthomeUuid: 0xFCD2,
  debounceMs: 3000,
  logRssi: false              // set true to log signal strength for debugging
};

let state = {
  lastPress: 0,
  lastActionMs: 0
};

function handleScan(ev, res) {
  if (ev !== BLE.Scanner.SCAN_RESULT) return;
  if (!res || res.local_name !== CONFIG.targetName) return;

  if (CONFIG.logRssi) {
    print("Beacon seen: RSSI=" + res.rssi + " addr=" + res.addr);
  }

  // Match BTHome service data UUID
  if (res.service_data_uuid16 !== CONFIG.bthomeUuid) return;

  let sd = res.service_data;
  if (!sd || sd.length < 3) return;

  // BTHome v2 payload: [info_byte, property_id, value]
  // property 0x3A = button event
  if (sd.charCodeAt(1) !== 0x3A) return;

  let pressType = sd.charCodeAt(2);

  // Only react on transitions (ignore repeated broadcasts of same value)
  if (pressType === state.lastPress) return;

  // Debounce - ignore new events too quickly after previous action
  let now = Date.now();
  if (state.lastActionMs > 0 && now - state.lastActionMs < CONFIG.debounceMs) return;

  state.lastPress = pressType;

  if (pressType === 0x01) {
    print("Detected PRESS -> Switch ON");
    state.lastActionMs = now;
    Shelly.call("Switch.Set", { id: 0, on: true });
  } else if (pressType === 0x04) {
    print("Detected LONG PRESS -> Switch OFF");
    state.lastActionMs = now;
    Shelly.call("Switch.Set", { id: 0, on: false });
  }
  // Other press values (0x00 = idle, 0x02/03/05) are ignored
}

// Start scanning
BLE.Scanner.Subscribe(handleScan);
BLE.Scanner.Start({ duration_ms: BLE.Scanner.INFINITE_SCAN, active: false });

print("ResinFanBT listener started, waiting for " + CONFIG.targetName + "...");
