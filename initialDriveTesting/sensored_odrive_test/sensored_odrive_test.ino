#include <Arduino.h>
#include <stdarg.h>
#include <climits>

static constexpr uint32_t ODRIVE_BAUD = 115200;

enum AxisState : int {
  AXIS_STATE_IDLE = 1,
  AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
  AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
};

enum ControlMode : int { CONTROL_MODE_POSITION_CONTROL = 3 };
enum InputMode   : int { INPUT_MODE_PASSTHROUGH = 1 };

class ODriveUART {
public:
  explicit ODriveUART(HardwareSerial& s) : ser(s) {}
  void begin(uint32_t baud = ODRIVE_BAUD) { ser.begin(baud); }

  void writelnf(const char* fmt, ...) {
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ser.print(buf);
    ser.print('\n');
  }

  bool readLine(char* out, size_t out_sz, uint32_t timeout_ms) {
    uint32_t t0 = millis();
    size_t n = 0;
    while (millis() - t0 < timeout_ms) {
      while (ser.available()) {
        char c = (char)ser.read();
        if (c == '\r') continue;
        if (c == '\n') { out[n] = '\0'; return true; }
        if (n + 1 < out_sz) out[n++] = c;
      }
    }
    if (out_sz) out[0] = '\0';
    return false;
  }

  long readLong(uint32_t timeout_ms = 120) {
    char line[96];
    if (!readLine(line, sizeof(line), timeout_ms)) return LONG_MIN;
    return strtol(line, nullptr, 10);
  }

  float readFloat(uint32_t timeout_ms = 120) {
    char line[96];
    if (!readLine(line, sizeof(line), timeout_ms)) return NAN;
    return strtof(line, nullptr);
  }

  void set(const char* path, int v)   { writelnf("w %s %d", path, v); }
  void set(const char* path, float v) { writelnf("w %s %.8f", path, v); }

  void setAxisInt(int axis, const char* suffix, int v) {
    char p[96];
    snprintf(p, sizeof(p), "axis%d.%s", axis, suffix);
    set(p, v);
  }
  void setAxisFloat(int axis, const char* suffix, float v) {
    char p[96];
    snprintf(p, sizeof(p), "axis%d.%s", axis, suffix);
    set(p, v);
  }
  void setAxisBool(int axis, const char* suffix, bool v) {
    char p[96];
    snprintf(p, sizeof(p), "axis%d.%s", axis, suffix);
    set(p, v ? 1 : 0);
  }

  long getInt(const char* expr) {
    writelnf("%s", expr);
    return readLong();
  }

  float getFloat(const char* expr) {
    writelnf("%s", expr);
    return readFloat();
  }

  void requestState(int axis, int state) {
    writelnf("w axis%d.requested_state %d", axis, state);
  }

  void setPosition(int axis, float pos_turns, float vel_ff = 0.0f, float torque_ff = 0.0f) {
    writelnf("p %d %.6f %.6f %.6f", axis, pos_turns, vel_ff, torque_ff);
  }

private:
  HardwareSerial& ser;
};

struct AxisBringupConfig {
  int axis = 0;
  int pole_pairs = 20;
  int encoder_cpr = 4000;
  float current_lim = 10.0f;
  float vel_limit = 10.0f;
  uint32_t calib_timeout_ms = 30000;
};

class Motor {
public:
  Motor(ODriveUART& od, AxisBringupConfig cfg, int analog_pin, float ratio_turns)
    : odrv(od), c(cfg), aPin(analog_pin), R(ratio_turns) {}

  void beginAnalog() {
    pinMode(aPin, INPUT);
    analogReadResolution(12);
    analogReadAveraging(16);
  }

  void bringup() {
    Serial.println("Bringup: vbus/state/errors");
    float vbus = odrv.getFloat("r vbus_voltage");
    Serial.print("vbus_voltage="); Serial.println(vbus, 3);

    logErrors("Boot");

    Serial.println("Bringup: clear_errors");
    clearErrors();
    logErrors("After clear");

    Serial.println("Bringup: set params + controller modes");
    odrv.setAxisInt(c.axis, "motor.config.pole_pairs", c.pole_pairs);
    odrv.setAxisInt(c.axis, "encoder.config.cpr", c.encoder_cpr);
    odrv.setAxisFloat(c.axis, "motor.config.current_lim", c.current_lim);
    odrv.setAxisFloat(c.axis, "controller.config.vel_limit", c.vel_limit);
    odrv.setAxisInt(c.axis, "controller.config.control_mode", CONTROL_MODE_POSITION_CONTROL);
    odrv.setAxisInt(c.axis, "controller.config.input_mode", INPUT_MODE_PASSTHROUGH);

    bool motor_pre = getBoolAxis("motor.config.pre_calibrated");
    bool enc_pre   = getBoolAxis("encoder.config.pre_calibrated");

    Serial.print("pre_calibrated motor="); Serial.print(motor_pre ? 1 : 0);
    Serial.print(" encoder="); Serial.println(enc_pre ? 1 : 0);

    if (!(motor_pre && enc_pre)) {
      Serial.println("Bringup: FULL_CALIBRATION_SEQUENCE (await IDLE)");
      requestAndWaitIdle("FULL_CALIBRATION_SEQUENCE", AXIS_STATE_FULL_CALIBRATION_SEQUENCE, c.calib_timeout_ms);

      Serial.println("Bringup: mark pre_calibrated true");
      odrv.setAxisBool(c.axis, "motor.config.pre_calibrated", true);
      odrv.setAxisBool(c.axis, "encoder.config.pre_calibrated", true);
      delay(50);
    } else {
      Serial.println("Bringup: skipping calibration");
    }

    Serial.println("Bringup: CLOSED_LOOP_CONTROL");
    odrv.requestState(c.axis, AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(250);
    logErrors("After closed-loop");
  }

  void updateFromAnalog_0_to_1rev_then_ratio(bool log = true) {
    int raw = analogRead(aPin);
    float u = (float)raw / 4095.0f;     // 0..1
    float base_turns = u * 1.0f;        // 0..1 rotation
    float pos_turns = base_turns * R;   // apply ratio R

    odrv.setPosition(c.axis, pos_turns);

    if (log) {
      uint32_t now = millis();
      if (now - lastLogMs >= logPeriodMs) {
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

  void setLogPeriodMs(uint32_t ms) { logPeriodMs = ms; }

private:
  ODriveUART& odrv;
  AxisBringupConfig c;
  int aPin;
  float R;

  uint32_t lastLogMs = 0;
  uint32_t logPeriodMs = 100;

  long getState() {
    char q[64];
    snprintf(q, sizeof(q), "r axis%d.current_state", c.axis);
    return odrv.getInt(q);
  }

  bool getBoolAxis(const char* suffix) {
    char q[96];
    snprintf(q, sizeof(q), "r axis%d.%s", c.axis, suffix);
    long v = odrv.getInt(q);
    return (v == 1);
  }

  void logErrors(const char* tag) {
    char q1[64], q2[64], q3[64], q4[64];
    snprintf(q1, sizeof(q1), "r axis%d.error", c.axis);
    snprintf(q2, sizeof(q2), "r axis%d.motor.error", c.axis);
    snprintf(q3, sizeof(q3), "r axis%d.encoder.error", c.axis);
    snprintf(q4, sizeof(q4), "r axis%d.controller.error", c.axis);

    long ax  = odrv.getInt(q1);
    long mot = odrv.getInt(q2);
    long enc = odrv.getInt(q3);
    long ctl = odrv.getInt(q4);
    long st  = getState();

    Serial.print(tag);
    Serial.print(" | state="); Serial.print(st);
    Serial.print(" | err axis="); Serial.print(ax);
    Serial.print(" motor="); Serial.print(mot);
    Serial.print(" enc="); Serial.print(enc);
    Serial.print(" ctl="); Serial.println(ctl);
  }

  void clearErrors() {
    char p1[64], p2[64], p3[64], p4[64];
    snprintf(p1, sizeof(p1), "axis%d.error", c.axis);
    snprintf(p2, sizeof(p2), "axis%d.motor.error", c.axis);
    snprintf(p3, sizeof(p3), "axis%d.encoder.error", c.axis);
    snprintf(p4, sizeof(p4), "axis%d.controller.error", c.axis);

    odrv.set(p1, 0);
    odrv.set(p2, 0);
    odrv.set(p3, 0);
    odrv.set(p4, 0);
    delay(20);
  }

  bool awaitIdle(uint32_t timeout_ms, uint32_t poll_ms = 250) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
      long st = getState();
      if (st == AXIS_STATE_IDLE) return true;
      delay(poll_ms);
    }
    return false;
  }

  void requestAndWaitIdle(const char* label, int state, uint32_t timeout_ms) {
    Serial.print("Request: "); Serial.println(label);
    odrv.requestState(c.axis, state);

    bool ok = awaitIdle(timeout_ms);
    Serial.print("Finished (IDLE reached)? "); Serial.println(ok ? "YES" : "NO");
    logErrors("Post request");
  }
};

ODriveUART odrv(Serial8);

static constexpr int ANALOG_PIN = 27;  // per your request
static constexpr float R_TURNS = 4.0f; // ratio R

AxisBringupConfig cfg = {
  .axis = 0,
  .pole_pairs = 20,
  .encoder_cpr = 4000,
  .current_lim = 10.0f,
  .vel_limit = 10.0f,
  .calib_timeout_ms = 30000
};

Motor m0(odrv, cfg, ANALOG_PIN, R_TURNS);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {}

  Serial.println("Teensy 4.1 -> ODrive UART (Serial8 TX=35 RX=34)");
  Serial.println("Analog pin 27 -> desired position (0..1 rev) * R");

  Serial8.setTX(35);
  Serial8.setRX(34);
  odrv.begin(ODRIVE_BAUD);
  delay(200);

  m0.beginAnalog();
  m0.setLogPeriodMs(100);
  m0.bringup();

  Serial.println("Streaming commands...");
}

void loop() {
  m0.updateFromAnalog_0_to_1rev_then_ratio(true);
  delay(10);
}
