
/* ==========================
   Section B: ODriveArduino.cpp
   ========================== */
#ifdef ARDUINO
// If you put this in a separate .cpp, keep it there.
// If you paste everything into ONE .ino, you can keep this section below the header section.
#endif

namespace odrv {

ODriveUART::ODriveUART(HardwareSerial& s) : ser(s) {}

void ODriveUART::begin(uint32_t baud) { ser.begin(baud); }

void ODriveUART::writelnf(const char* fmt, ...) {
  char buf[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  ser.print(buf);
  ser.print('\n');
}

bool ODriveUART::readLine(char* out, size_t out_sz, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  size_t n = 0;
  while (millis() - t0 < timeout_ms) {
    while (ser.available()) {
      char c = (char)ser.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (out_sz) out[n] = '\0';
        return true;
      }
      if (n + 1 < out_sz) out[n++] = c;
    }
  }
  if (out_sz) out[0] = '\0';
  return false;
}

long ODriveUART::readLong(uint32_t timeout_ms) {
  char line[96];
  if (!readLine(line, sizeof(line), timeout_ms)) return LONG_MIN;
  return strtol(line, nullptr, 10);
}

float ODriveUART::readFloat(uint32_t timeout_ms) {
  char line[96];
  if (!readLine(line, sizeof(line), timeout_ms)) return NAN;
  return strtof(line, nullptr);
}

void ODriveUART::set(const char* path, int v)   { writelnf("w %s %d", path, v); }
void ODriveUART::set(const char* path, float v) { writelnf("w %s %.8f", path, v); }

void ODriveUART::setAxisInt(int axis, const char* suffix, int v) {
  char p[128];
  snprintf(p, sizeof(p), "axis%d.%s", axis, suffix);
  set(p, v);
}
void ODriveUART::setAxisFloat(int axis, const char* suffix, float v) {
  char p[128];
  snprintf(p, sizeof(p), "axis%d.%s", axis, suffix);
  set(p, v);
}
void ODriveUART::setAxisBool(int axis, const char* suffix, bool v) {
  char p[128];
  snprintf(p, sizeof(p), "axis%d.%s", axis, suffix);
  set(p, v ? 1 : 0);
}

long ODriveUART::getInt(const char* expr) {
  writelnf("%s", expr);
  return readLong();
}

float ODriveUART::getFloat(const char* expr) {
  writelnf("%s", expr);
  return readFloat();
}

void ODriveUART::requestState(int axis, int state) {
  writelnf("w axis%d.requested_state %d", axis, state);
}

void ODriveUART::setPosition(int axis, float pos_turns, float vel_ff, float torque_ff) {
  writelnf("p %d %.6f %.6f %.6f", axis, pos_turns, vel_ff, torque_ff);
}

void ODriveUART::setVelocity(int axis, float vel_turns_s, float torque_ff) {
  writelnf("v %d %.6f %.6f", axis, vel_turns_s, torque_ff);
}

void ODriveUART::setTorque(int axis, float torque_nm) {
  writelnf("c %d %.6f", axis, torque_nm);
}


// ---------------- ODriveAxis ----------------
ODriveAxis::ODriveAxis(ODriveUART& uart, const AxisConfig& cfg) : od(uart), c(cfg) {}

float ODriveAxis::vbusVoltage() {
  return od.getFloat("r vbus_voltage");
}

long ODriveAxis::currentState() {
  char q[64];
  snprintf(q, sizeof(q), "r axis%d.current_state", c.axis);
  return od.getInt(q);
}

long ODriveAxis::axisError() {
  char q[64];
  snprintf(q, sizeof(q), "r axis%d.error", c.axis);
  return od.getInt(q);
}
long ODriveAxis::motorError() {
  char q[80];
  snprintf(q, sizeof(q), "r axis%d.motor.error", c.axis);
  return od.getInt(q);
}
long ODriveAxis::encoderError() {
  char q[88];
  snprintf(q, sizeof(q), "r axis%d.encoder.error", c.axis);
  return od.getInt(q);
}
long ODriveAxis::controllerError() {
  char q[96];
  snprintf(q, sizeof(q), "r axis%d.controller.error", c.axis);
  return od.getInt(q);
}

void ODriveAxis::clearErrors() {
  // Clear all relevant error fields
  char p1[64], p2[80], p3[88], p4[96];
  snprintf(p1, sizeof(p1), "axis%d.error", c.axis);
  snprintf(p2, sizeof(p2), "axis%d.motor.error", c.axis);
  snprintf(p3, sizeof(p3), "axis%d.encoder.error", c.axis);
  snprintf(p4, sizeof(p4), "axis%d.controller.error", c.axis);

  od.set(p1, 0);
  od.set(p2, 0);
  od.set(p3, 0);
  od.set(p4, 0);
  delay(20);
}

void ODriveAxis::logErrors(Stream& out, const char* tag) {
  long st  = currentState();
  long ax  = axisError();
  long mot = motorError();
  long enc = encoderError();
  long ctl = controllerError();

  out.print(tag);
  out.print(" | state=");
  out.print(st);
  out.print(" | err axis=");
  out.print(ax);
  out.print(" motor=");
  out.print(mot);
  out.print(" enc=");
  out.print(enc);
  out.print(" ctl=");
  out.println(ctl);
}

bool ODriveAxis::getBoolR(const char* suffix) {
  char q[128];
  snprintf(q, sizeof(q), "r axis%d.%s", c.axis, suffix);
  long v = od.getInt(q);
  return (v == 1);
}

bool ODriveAxis::isMotorPreCalibrated()  { return getBoolR("motor.config.pre_calibrated"); }
bool ODriveAxis::isEncoderPreCalibrated(){ return getBoolR("encoder.config.pre_calibrated"); }
bool ODriveAxis::isPreCalibrated()       { return isMotorPreCalibrated() && isEncoderPreCalibrated(); }

void ODriveAxis::setPreCalibrated(bool motor_pre, bool enc_pre) {
  od.setAxisBool(c.axis, "motor.config.pre_calibrated", motor_pre);
  od.setAxisBool(c.axis, "encoder.config.pre_calibrated", enc_pre);
  delay(20);
}

void ODriveAxis::requestState(AxisState s) {
  od.requestState(c.axis, (int)s);
}

bool ODriveAxis::waitForState(AxisState target, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    long st = currentState();
    if (st == (long)target) return true;
    delay(c.poll_ms);
  }
  return false;
}

bool ODriveAxis::waitForIdle(uint32_t timeout_ms) {
  return waitForState(AXIS_STATE_IDLE, timeout_ms);
}

void ODriveAxis::applyEncoderConfig() {
  if (c.encoder_type == EncoderType::Incremental) {
    od.setAxisInt(c.axis, "encoder.config.cpr", c.encoder_cpr);
    // For incremental, many firmwares use encoder.config.mode = 0 by default.
    // We intentionally do NOT force a mode here, because it varies by firmware.
  } else {
    // Absolute: user must supply the correct mode integer for their firmware/encoder.
    if (c.encoder_mode_raw >= 0) {
      od.setAxisInt(c.axis, "encoder.config.mode", c.encoder_mode_raw);
    }
    // Absolute encoders often have additional fields (SPI pins, error rate, etc.)
    // Set those using setInt/setFloat escape hatch from your sketch if needed.
  }
}

void ODriveAxis::applyConfig(bool also_save_precal_flags) {
  // Motor/controller essentials
  od.setAxisInt (c.axis, "motor.config.pole_pairs", c.pole_pairs);
  od.setAxisFloat(c.axis, "motor.config.current_lim", c.current_lim);
  od.setAxisFloat(c.axis, "controller.config.vel_limit", c.vel_limit);

  od.setAxisInt(c.axis, "controller.config.control_mode", (int)c.control_mode);
  od.setAxisInt(c.axis, "controller.config.input_mode",   (int)c.input_mode);

  applyEncoderConfig();

  if (also_save_precal_flags) {
    // Optional: if you know you are pre-calibrated and want to persist it.
    // Generally you should set these only after successful calibration.
    setPreCalibrated(true, true);
  }
}

bool ODriveAxis::calibrateFull(uint32_t timeout_ms) {
  uint32_t to = (timeout_ms == 0) ? c.calib_timeout_ms : timeout_ms;

  requestState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
  bool ok = waitForIdle(to);

  if (ok) {
    // Mark as pre-calibrated so future boots can skip (matches your original behavior)
    setPreCalibrated(true, true);
  }
  return ok;
}

bool ODriveAxis::calibrateIfNeeded(uint32_t timeout_ms) {
  if (isPreCalibrated()) return true;
  return calibrateFull(timeout_ms);
}

bool ODriveAxis::enterClosedLoop(uint32_t settle_ms) {
  requestState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  delay(settle_ms);
  return (currentState() == AXIS_STATE_CLOSED_LOOP_CONTROL);
}

bool ODriveAxis::ensureClosedLoop(bool calibrate_if_needed) {
  if (calibrate_if_needed) {
    if (!calibrateIfNeeded()) return false;
  }
  return enterClosedLoop();
}

void ODriveAxis::setControlMode(ControlMode m) {
  c.control_mode = m;
  od.setAxisInt(c.axis, "controller.config.control_mode", (int)m);
}
void ODriveAxis::setInputMode(InputMode m) {
  c.input_mode = m;
  od.setAxisInt(c.axis, "controller.config.input_mode", (int)m);
}

void ODriveAxis::setPosition(float pos_turns, float vel_ff, float torque_ff) {
  od.setPosition(c.axis, pos_turns, vel_ff, torque_ff);
}
void ODriveAxis::setVelocity(float vel_turns_s, float torque_ff) {
  od.setVelocity(c.axis, vel_turns_s, torque_ff);
}
void ODriveAxis::setTorque(float torque_nm) {
  od.setTorque(c.axis, torque_nm);
}

void ODriveAxis::setInt(const char* suffix, int v) {
  od.setAxisInt(c.axis, suffix, v);
}
void ODriveAxis::setFloat(const char* suffix, float v) {
  od.setAxisFloat(c.axis, suffix, v);
}
long ODriveAxis::getIntR(const char* suffix) {
  char q[128];
  snprintf(q, sizeof(q), "r axis%d.%s", c.axis, suffix);
  return od.getInt(q);
}
float ODriveAxis::getFloatR(const char* suffix) {
  char q[128];
  snprintf(q, sizeof(q), "r axis%d.%s", c.axis, suffix);
  return od.getFloat(q);
}

} // namespace odrv

