// Teensy 4.1 <-> ODrive 3.6 UART
// Closed-loop (magnetic encoder) + 2R planar IK streaming
//
// Wiring (ODrive UART0 default):
//   Teensy TX -> ODrive GPIO2 (RX)
//   Teensy RX -> ODrive GPIO1 (TX)
//   GND <-> GND
//
// Command protocol over USB Serial (115200):
//   EE x y v
// Example:
//   EE 0.25 0.10 0.05
//
// Notes:
// - This is a "runtime streamer": ODrive should be pre-configured and calibrated.
// - Units are your choice; keep them consistent (L1,L2,x,y).

#include <Arduino.h>
#include <math.h>

// ---------------- UART selection ----------------
HardwareSerial &odrv = Serial8;   // match your current setup
// HardwareSerial &odrv = Serial1; // if you switch back

// ---------------- ODrive constants (ASCII protocol) ----------------

static const int AXIS_STATE_IDLE                 = 1;
static const int AXIS_STATE_CLOSED_LOOP_CONTROL  = 8;

static const int CONTROL_MODE_POSITION_CONTROL   = 3;
static const int INPUT_MODE_PASSTHROUGH          = 1;

// ---------------- Robot parameters (EDIT THESE) ----------------
static const float L1 = 0.200f;   // link 1 length
static const float L2 = 0.200f;   // link 2 length

// Convert joint radians -> ODrive "turns" setpoint.
// If motor turns == joint turns (direct drive): turns_per_rad = 1/(2*pi)
// gear ratio G = motor_turns / joint_turns, then:
// axis_turns_per_rad = G / (2*pi)
static const float axis0_turns_per_rad = 1.0f / (2.0f * (float)M_PI);
static const float axis1_turns_per_rad = 1.0f / (2.0f * (float)M_PI);

// Optional: joint direction flips if needed (set to -1.0f to invert)
static const float axis0_dir = +1.0f;
static const float axis1_dir = +1.0f;

// ---------------- Control loop ----------------
static const float ctrl_hz = 200.0f;
static const uint32_t ctrl_period_us = (uint32_t)(1e6f / ctrl_hz);

// End-effector state (tracked)
static float ee_x = 0.0f;
static float ee_y = 0.0f;

// End-effector command target
static volatile float ee_x_cmd = 0.20f;
static volatile float ee_y_cmd = 0.00f;
static volatile float ee_v_cmd = 0.05f;   // end-effector speed limit (units/s)

// IK solution (tracked)
static float q0 = 0.0f, q1 = 0.0f;       // joint angles [rad]
static float q0_prev = 0.0f, q1_prev = 0.0f;

// ---------------- UART helpers ----------------
static void sendCmd(const char *cmd) {
  odrv.print(cmd);
  odrv.print('\n');
  odrv.flush();
  delayMicroseconds(200);
  while (odrv.available()) { (void)odrv.read(); } // discard echoes/replies (simple)
}

static void sendCmdf(const char *fmt, float v) {
  char buf[128];
  snprintf(buf, sizeof(buf), fmt, v);
  sendCmd(buf);
}

static void sendCmdi(const char *fmt, int v) {
  char buf[128];
  snprintf(buf, sizeof(buf), fmt, v);
  sendCmd(buf);
}

static void axisEnterClosedLoop(int axis) {
  char buf[128];

  // Clear axis errors (best-effort)
  snprintf(buf, sizeof(buf), "w axis%d.error 0", axis); sendCmd(buf);
  snprintf(buf, sizeof(buf), "w axis%d.motor.error 0", axis); sendCmd(buf);
  snprintf(buf, sizeof(buf), "w axis%d.encoder.error 0", axis); sendCmd(buf);
  delay(50);

  // Set controller modes
  snprintf(buf, sizeof(buf), "w axis%d.controller.config.control_mode %d", axis, CONTROL_MODE_POSITION_CONTROL);
  sendCmd(buf);
  snprintf(buf, sizeof(buf), "w axis%d.controller.config.input_mode %d", axis, INPUT_MODE_PASSTHROUGH);
  sendCmd(buf);

  // Enter closed-loop control
  snprintf(buf, sizeof(buf), "w axis%d.requested_state %d", axis, AXIS_STATE_CLOSED_LOOP_CONTROL);
  sendCmd(buf);
  delay(200);
}

// ---------------- 2R kinematics ----------------
static bool ik_2r(float x, float y, bool elbow_up, float *out_q0, float *out_q1) {
  // Standard planar 2R analytic IK.
  // Returns false if unreachable.
  const float r2 = x*x + y*y;
  const float c2 = (r2 - L1*L1 - L2*L2) / (2.0f * L1 * L2);

  if (c2 < -1.0f || c2 > 1.0f) return false;

  float s2 = sqrtf(fmaxf(0.0f, 1.0f - c2*c2));
  if (!elbow_up) s2 = -s2;

  const float q1_local = atan2f(s2, c2);

  const float k1 = L1 + L2 * c2;
  const float k2 = L2 * s2;
  const float q0_local = atan2f(y, x) - atan2f(k2, k1);

  *out_q0 = q0_local;
  *out_q1 = q1_local;
  return true;
}

static void fk_2r(float q0_in, float q1_in, float *out_x, float *out_y) {
  const float c0 = cosf(q0_in), s0 = sinf(q0_in);
  const float c01 = cosf(q0_in + q1_in), s01 = sinf(q0_in + q1_in);
  *out_x = L1*c0 + L2*c01;
  *out_y = L1*s0 + L2*s01;
}

// Simple end-effector "speed-limited step" toward target (no accel shaping yet)
static void ee_step_toward(float dt) {
  const float dx = ee_x_cmd - ee_x;
  const float dy = ee_y_cmd - ee_y;
  const float dist = sqrtf(dx*dx + dy*dy);
  if (dist < 1e-6f) return;

  const float v = fmaxf(0.0f, ee_v_cmd);
  const float step = fminf(dist, v * dt);

  ee_x += (dx / dist) * step;
  ee_y += (dy / dist) * step;
}

// ---------------- Serial command parsing ----------------
static void pollUsbCommands() {
  static char line[128];
  static size_t n = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line[n] = '\0';
      n = 0;

      // Expected: EE x y v
      if (strncmp(line, "EE", 2) == 0) {
        float x, y, v;
        if (sscanf(line + 2, "%f %f %f", &x, &y, &v) == 3) {
          ee_x_cmd = x;
          ee_y_cmd = y;
          ee_v_cmd = v;
          Serial.println("OK");
        } else {
          Serial.println("ERR parse");
        }
      } else if (strncmp(line, "HOME", 4) == 0) {
        // Optional: set command to current FK point (no motion)
        fk_2r(q0, q1, &ee_x, &ee_y);
        ee_x_cmd = ee_x;
        ee_y_cmd = ee_y;
        Serial.println("OK");
      } else {
        Serial.println("ERR cmd");
      }

    } else {
      if (n + 1 < sizeof(line)) line[n++] = c;
    }
  }
}

// ---------------- ODrive setpoint streaming ----------------
static void odriveSetPosVelFF(int axis, float pos_turns, float vel_turns_s) {
  // Writes:
  //   axisX.controller.input_pos
  //   axisX.controller.input_vel (used as velocity feedforward in some configs)
  char buf[128];

  snprintf(buf, sizeof(buf), "w axis%d.controller.input_pos %f", axis, pos_turns);
  sendCmd(buf);

  snprintf(buf, sizeof(buf), "w axis%d.controller.input_vel %f", axis, vel_turns_s);
  sendCmd(buf);
}

void setup() {
  Serial.begin(115200);     // USB serial to PC
  delay(500);

  // UART to ODrive
  odrv.begin(115200);
  delay(500);

  // Put both axes into closed loop (assumes already configured/calibrated)
  axisEnterClosedLoop(0);
  axisEnterClosedLoop(1);

  // Initialize EE state from current q (assumed 0,0 at boot unless you read encoders)
  fk_2r(q0, q1, &ee_x, &ee_y);
  ee_x_cmd = ee_x;
  ee_y_cmd = ee_y;

  Serial.println("READY");
}

void loop() {
  static uint32_t t_last = micros();
  const uint32_t t_now = micros();

  pollUsbCommands();

  if ((uint32_t)(t_now - t_last) >= ctrl_period_us) {
    t_last += ctrl_period_us;
    const float dt = 1.0f / ctrl_hz;

    // 1) Step EE point toward command with speed limit
    ee_step_toward(dt);

    // 2) IK solve for current EE waypoint
    const bool elbow_up = true;
    float q0_new, q1_new;
    if (!ik_2r(ee_x, ee_y, elbow_up, &q0_new, &q1_new)) {
      // Unreachable: do nothing (or clamp, or back off)
      // In production: report fault and stop streaming motion.
      return;
    }

    // 3) Estimate joint velocities (simple finite-difference)
    q0_prev = q0; q1_prev = q1;
    q0 = q0_new; q1 = q1_new;

    const float q0d = (q0 - q0_prev) / dt;  // rad/s
    const float q1d = (q1 - q1_prev) / dt;  // rad/s

    // 4) Convert to ODrive turns
    const float axis0_pos_turns = axis0_dir * (q0 * axis0_turns_per_rad);
    const float axis1_pos_turns = axis1_dir * (q1 * axis1_turns_per_rad);

    const float axis0_vel_turns_s = axis0_dir * (q0d * axis0_turns_per_rad);
    const float axis1_vel_turns_s = axis1_dir * (q1d * axis1_turns_per_rad);

    // 5) Stream setpoints
    odriveSetPosVelFF(0, axis0_pos_turns, axis0_vel_turns_s);
    odriveSetPosVelFF(1, axis1_pos_turns, axis1_vel_turns_s);
  }
}
