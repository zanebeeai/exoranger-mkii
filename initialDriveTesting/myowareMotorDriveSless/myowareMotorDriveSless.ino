// Teensy 4.1 + MyoWare EMG (A5) + ODrive UART (Serial8 + isolator)
// Sensorless EMG-controlled forward/backward velocity with safety timeout
// Includes: startup spin, LED warning blinks, EMG plotting, 25s auto-stop

#include <Arduino.h>

HardwareSerial &odrv = Serial8;

const int emgPin       = A5;
const int avgCount     = 8;

const int flexThreshold = 850;

const float fwdVel  = 0.40f;
const float backVel = -0.40f;

float virtualPos = 0.0f;
const float posMax = 100.0f;
const float posMin = 0.0f;
const float posStep = 0.5f;

unsigned long startTime;
const unsigned long maxRuntime = 25000;   // 25 seconds

// ODrive states/modes
const int AXIS_STATE_IDLE               = 1;
const int AXIS_STATE_SENSORLESS_CONTROL = 5;
const int CONTROL_MODE_VELOCITY_CONTROL = 2;
const int INPUT_MODE_VEL_RAMP           = 2;

// ------------------------------------------------------
// UART helper
// ------------------------------------------------------
void sendCmd(const char *cmd) {
    odrv.print(cmd);
    odrv.print('\n');
    odrv.flush();
    delay(3);

    while (odrv.available()) {
        Serial.write(odrv.read());
    }
}

// ------------------------------------------------------
// ODrive init
// ------------------------------------------------------
void odriveInit() {
    char buf[96];

    sendCmd("w axis0.requested_state 1");
    sendCmd("w axis0.error 0");
    sendCmd("w axis0.motor.error 0");
    sendCmd("w axis0.encoder.error 0");

    snprintf(buf, sizeof(buf), "w axis0.motor.config.current_lim %f", 5.0f);
    sendCmd(buf);

    sendCmd("w axis0.controller.config.control_mode 2");   // VELOCITY_CONTROL
    sendCmd("w axis0.controller.config.input_mode 2");     // VEL_RAMP
    sendCmd("w axis0.controller.config.vel_limit 8.0");
    sendCmd("w axis0.controller.config.vel_ramp_rate 20.0");

    delay(200);
    sendCmd("w axis0.requested_state 5");   // SENSORLESS_CONTROL
    delay(300);
    sendCmd("w axis0.controller.input_vel 0");
}

// ------------------------------------------------------
// Startup spin (same as original)
// ------------------------------------------------------
void startupSpin() {
    char buf[96];

    float target_vel_turns_s = 0.25f;
    float vel_limit = fabsf(target_vel_turns_s) * 2.0f + 5.0f;
    float vel_ramp  = fabsf(target_vel_turns_s) * 5.0f + 5.0f;

    snprintf(buf, sizeof(buf), "w axis0.motor.config.current_lim %f", 5.0f);
    sendCmd(buf);

    snprintf(buf, sizeof(buf), "w axis0.controller.config.vel_limit %f", vel_limit);
    sendCmd(buf);

    snprintf(buf, sizeof(buf), "w axis0.controller.config.vel_ramp_rate %f", vel_ramp);
    sendCmd(buf);

    sendCmd("w axis0.requested_state 1");
    sendCmd("w axis0.error 0");
    sendCmd("w axis0.motor.error 0");
    sendCmd("w axis0.encoder.error 0");
    delay(150);

    sendCmd("w axis0.requested_state 5");   // SENSORLESS_CONTROL
    delay(250);

    sendCmd("w axis0.controller.config.control_mode 2");
    sendCmd("w axis0.controller.config.input_mode 2");

    snprintf(buf, sizeof(buf), "w axis0.controller.input_vel %f", target_vel_turns_s);
    sendCmd(buf);

    delay(1000);

    sendCmd("w axis0.controller.input_vel 0");
    delay(400);
    sendCmd("w axis0.requested_state 1");
    delay(300);
}

// ------------------------------------------------------
void odriveVel(float v) {
    char buf[64];
    snprintf(buf, sizeof(buf),
         "w axis0.motor.config.direction %d",
         v >= 0 ? 1 : -1);
    snprintf(buf, sizeof(buf), "w axis0.controller.input_vel %f", v);
    sendCmd(buf);
}

// ------------------------------------------------------
// Setup
// ------------------------------------------------------
void setup() {
    pinMode(34, INPUT);
    pinMode(35, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    odrv.begin(115200);

    delay(800);

    // Initial spin
    startupSpin();

    // Warning LED (2 blinks)
    delay(1000);
    for (int i = 0; i < 2; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }

    // Enter continuous sensorless mode
    odriveInit();

    startTime = millis();
}

// ------------------------------------------------------
// Main loop
// ------------------------------------------------------
void loop() {

    // ----- AUTO SHUTDOWN -----
    if (millis() - startTime > maxRuntime) {
        odriveVel(0);
        sendCmd("w axis0.requested_state 1");   // idle
        Serial.println("STOP: 25 second timeout");
        while (1) { delay(1000); }   // freeze program safely
    }

    // ----- EMG SMOOTHING -----
    uint32_t sum = 0;
    for (int i = 0; i < avgCount; i++) {
        sum += analogRead(emgPin);
    }
    int emgRaw = sum / avgCount;

    // ----- SERIAL PLOTTER OUTPUT -----
    // Format: EMG, VirtualPos
    Serial.print(emgRaw);
    Serial.print("\t");
    Serial.println(virtualPos);

    // ----- CONTROL LOGIC -----
    if (emgRaw > flexThreshold) {
        // FLEX -> forward
        if (virtualPos < posMax) {
            virtualPos += posStep;
            odriveVel(fwdVel);
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            odriveVel(0);
            digitalWrite(LED_BUILTIN, LOW);
        }
    } else {
        // RELAX -> backward
        if (virtualPos > posMin) {
            virtualPos -= posStep;
            odriveVel(backVel);
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            odriveVel(0);
            digitalWrite(LED_BUILTIN, LOW);
        }
    }

    delay(20);
}
