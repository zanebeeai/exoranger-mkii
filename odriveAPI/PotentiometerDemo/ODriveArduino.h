/* =========================
   Section A: ODriveArduino.h
   ========================= */
#pragma once
#include <Arduino.h>
#include <stdarg.h>
#include <climits>
#include <math.h>

namespace odrv {

// Common ODrive axis states (ASCII protocol)
enum AxisState : int {
  AXIS_STATE_UNDEFINED                 = 0,
  AXIS_STATE_IDLE                      = 1,
  AXIS_STATE_STARTUP_SEQUENCE          = 2,
  AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
  AXIS_STATE_MOTOR_CALIBRATION         = 4,
  AXIS_STATE_ENCODER_INDEX_SEARCH      = 6,
  AXIS_STATE_ENCODER_OFFSET_CALIBRATION= 7,
  AXIS_STATE_CLOSED_LOOP_CONTROL       = 8,
};

// Common controller modes / input modes (ASCII protocol)
enum ControlMode : int {
  CONTROL_MODE_VOLTAGE_CONTROL   = 0,
  CONTROL_MODE_TORQUE_CONTROL    = 1,
  CONTROL_MODE_VELOCITY_CONTROL  = 2,
  CONTROL_MODE_POSITION_CONTROL  = 3,
};

enum InputMode : int {
  INPUT_MODE_INACTIVE      = 0,
  INPUT_MODE_PASSTHROUGH   = 1,
  INPUT_MODE_VEL_RAMP      = 2,
  INPUT_MODE_POS_FILTER    = 3,
  INPUT_MODE_TRAP_TRAJ     = 5,
};

// Encoder “type” is for your convenience.
// Incremental ABI usually uses CPR.
// Absolute varies; you must provide encoder_mode_raw that matches your ODrive firmware.
enum class EncoderType : uint8_t {
  Incremental = 0,
  Absolute    = 1,
};

// ---------- Low-level UART wrapper ----------
class ODriveUART {
public:
  explicit ODriveUART(HardwareSerial& s);

  void begin(uint32_t baud = 115200);

  // Formatted write + newline
  void writelnf(const char* fmt, ...);

  // Reads a line up to '\n' (strips '\r'), returns true if line complete before timeout.
  bool readLine(char* out, size_t out_sz, uint32_t timeout_ms);

  long  readLong(uint32_t timeout_ms = 120);
  float readFloat(uint32_t timeout_ms = 120);

  // Generic set/get helpers
  void  set(const char* path, int v);
  void  set(const char* path, float v);
  long  getInt(const char* expr);   // expr is a full ASCII command like: "r vbus_voltage"
  float getFloat(const char* expr); // same

  // Convenience for axis paths: suffix like "motor.config.current_lim"
  void setAxisInt(int axis, const char* suffix, int v);
  void setAxisFloat(int axis, const char* suffix, float v);
  void setAxisBool(int axis, const char* suffix, bool v);

  // State + setpoint helpers
  void requestState(int axis, int state);
  void setPosition(int axis, float pos_turns, float vel_ff = 0.0f, float torque_ff = 0.0f);
  void setVelocity(int axis, float vel_turns_s, float torque_ff = 0.0f);
  void setTorque(int axis, float torque_nm);

private:
  HardwareSerial& ser;
};


// ---------- High-level axis configuration ----------
struct AxisConfig {
  int axis = 0;

  // Motor basics
  int   pole_pairs   = 20;
  float current_lim  = 10.0f;
  float vel_limit    = 10.0f;

  // Controller defaults (you can change later)
  ControlMode control_mode = CONTROL_MODE_POSITION_CONTROL;
  InputMode   input_mode   = INPUT_MODE_PASSTHROUGH;

  // Encoder selection
  EncoderType encoder_type = EncoderType::Incremental;

  // Incremental:
  int encoder_cpr = 4000;

  // Absolute:
  // Provide the integer your ODrive firmware expects for axis#.encoder.config.mode.
  // (Leave at -1 unless you know what to set.)
  int encoder_mode_raw = -1;

  // Calibration behavior
  uint32_t calib_timeout_ms = 30000;
  uint32_t poll_ms          = 250;
};


// ---------- High-level axis API ----------
class ODriveAxis {
public:
  ODriveAxis(ODriveUART& uart, const AxisConfig& cfg);

  // Apply config parameters (motor/controller/encoder config).
  // Does NOT enter closed loop or calibrate by itself.
  void applyConfig(bool also_save_precal_flags = false);

  // Read status
  float vbusVoltage();
  long  currentState();
  long  axisError();
  long  motorError();
  long  encoderError();
  long  controllerError();

  // Errors
  void  clearErrors();
  void  logErrors(Stream& out, const char* tag);

  // Calibration flags
  bool  isMotorPreCalibrated();
  bool  isEncoderPreCalibrated();
  bool  isPreCalibrated();
  void  setPreCalibrated(bool motor_pre, bool enc_pre);

  // State helpers
  void  requestState(AxisState s);
  bool  waitForState(AxisState target, uint32_t timeout_ms);
  bool  waitForIdle(uint32_t timeout_ms);

  // Calibration
  bool  calibrateFull(uint32_t timeout_ms = 0);     // full calibration sequence
  bool  calibrateIfNeeded(uint32_t timeout_ms = 0); // only if not pre_calibrated

  // Closed loop
  bool  enterClosedLoop(uint32_t settle_ms = 250);
  bool  ensureClosedLoop(bool calibrate_if_needed = true);

  // Setpoints (work in the selected control mode; you can switch modes any time)
  void  setControlMode(ControlMode m);
  void  setInputMode(InputMode m);

  void  setPosition(float pos_turns, float vel_ff = 0.0f, float torque_ff = 0.0f);
  void  setVelocity(float vel_turns_s, float torque_ff = 0.0f);
  void  setTorque(float torque_nm);

  // Direct path access (escape hatch)
  void  setInt(const char* suffix, int v);
  void  setFloat(const char* suffix, float v);
  long  getIntR(const char* suffix);   // reads: r axis#.suffix
  float getFloatR(const char* suffix); // reads: r axis#.suffix

private:
  ODriveUART& od;
  AxisConfig  c;

  void applyEncoderConfig();
  bool getBoolR(const char* suffix);
};

} // namespace odrv
