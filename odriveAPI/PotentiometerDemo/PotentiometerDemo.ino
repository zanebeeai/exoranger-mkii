/* =========================================
   Section C: PotentiometerDemo.ino (example)
   - Replicates your original behavior:
     analog pin -> 0..1 rev -> multiply by ratio R -> position command
   ========================================= */

// If you split into files, include the header like this:
#include "ODriveArduino.h"

// If you kept everything in ONE file, this section can be your .ino code directly.

using namespace odrv;

static constexpr uint32_t ODRIVE_BAUD = 115200;

// --- Hardware choices (match your prior setup) ---
ODriveUART odrive(Serial8);

// Pot input
static constexpr int   ANALOG_PIN = 27;   // per your request
static constexpr float R_TURNS    = 4.0f; // ratio R (set as desired)

// Logging
static constexpr uint32_t LOG_PERIOD_MS = 100;
static uint32_t lastLogMs = 0;

// Axis configuration (incremental encoder example, same as your working script)
AxisConfig axis0_cfg = {
  .axis = 0,
  .pole_pairs = 20,
  .current_lim = 10.0f,
  .vel_limit = 10.0f,
  .control_mode = CONTROL_MODE_POSITION_CONTROL,
  .input_mode = INPUT_MODE_PASSTHROUGH,
  .encoder_type = EncoderType::Incremental,
  .encoder_cpr = 4000,
  .encoder_mode_raw = -1,           // not used for incremental
  .calib_timeout_ms = 30000,
  .poll_ms = 250
};

ODriveAxis axis0(odrive, axis0_cfg);

static void setupAnalogPot() {
  pinMode(ANALOG_PIN, INPUT);

  // These APIs are available on Teensy; on some Arduino boards they may not exist.
  // If your board doesn’t support them, comment them out.
  analogReadResolution(12);
  analogReadAveraging(16);
}

static void updateMotorFromPot(bool log = true) {
  int raw = analogRead(ANALOG_PIN);
  float u = (float)raw / 4095.0f;  // 0..1 for 12-bit
  float base_turns = u * 1.0f;     // 0..1 rotation
  float pos_turns  = base_turns * R_TURNS;

  axis0.setPosition(pos_turns);

  if (log) {
    uint32_t now = millis();
    if (now - lastLogMs >= LOG_PERIOD_MS) {
      lastLogMs = now;
      Serial.print("analog_raw=");
      Serial.print(raw);
      Serial.print(" u=");
      Serial.print(u, 4);
      Serial.print(" desired_turns=");
      Serial.println(pos_turns, 4);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {}

  Serial.println("Teensy 4.1 -> ODrive UART (Serial8 TX=35 RX=34)");
  Serial.println("Analog pin 27 -> desired position (0..1 rev) * R");

  // UART pin mapping (Teensy-specific)
  Serial8.setTX(35);
  Serial8.setRX(34);
  odrive.begin(ODRIVE_BAUD);
  delay(200);

  setupAnalogPot();

  // General bringup sequence using the new API:
  Serial.println("Bringup: vbus/state/errors");
  Serial.print("vbus_voltage=");
  Serial.println(axis0.vbusVoltage(), 3);

  axis0.logErrors(Serial, "Boot");
  Serial.println("Bringup: clear_errors");
  axis0.clearErrors();
  axis0.logErrors(Serial, "After clear");

  Serial.println("Bringup: apply config");
  axis0.applyConfig(false);

  Serial.print("pre_calibrated motor=");
  Serial.print(axis0.isMotorPreCalibrated() ? 1 : 0);
  Serial.print(" encoder=");
  Serial.println(axis0.isEncoderPreCalibrated() ? 1 : 0);

  Serial.println("Bringup: calibrateIfNeeded + enterClosedLoop");
  bool ok = axis0.ensureClosedLoop(true);
  Serial.print("Closed-loop ready? ");
  Serial.println(ok ? "YES" : "NO");
  axis0.logErrors(Serial, "After closed-loop");

  Serial.println("Streaming commands...");
}

void loop() {
  updateMotorFromPot(true);
  delay(10);
}
