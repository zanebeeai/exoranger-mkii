// Teensy 4.1 + MyoWare EMG (A5) + ODrive UART (Serial8 + isolator)
// Sensorless EMG-controlled bidirectional velocity using spinSensorless()

#include <Arduino.h>

HardwareSerial &odrv = Serial8;

// ------------------------------------------------------
// Constants
// ------------------------------------------------------
const int emgPin        = A5;
const int avgCount      = 8;
const int flexThreshold = 850;

const float fwdVel  = 0.40f;
const float backVel = -0.40f;

const float posMax  = 100.0f;
const float posMin  = 0.0f;
const float posStep = 0.5f;

float virtualPos = 0.0f;

unsigned long startTime;
const unsigned long maxRuntime = 25000;   // 25 seconds

// Short execution window for EMG updates
const float controlBurstTime = 0.20f;     // 200 ms
const float currentLimit     = 5.0f;

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
// Sensorless velocity primitive (your verified function)
// ------------------------------------------------------
void spinSensorless(float target_vel_turns_s,
                    float run_time_s,
                    float current_limit) {

    char buf[96];

    float vel_limit = fabsf(target_vel_turns_s) * 2.0f + 5.0f;
    float vel_ramp  = fabsf(target_vel_turns_s) * 5.0f + 5.0f;

    snprintf(buf, sizeof(buf),
             "w axis0.motor.config.current_lim %f", current_limit);
    sendCmd(buf);

    snprintf(buf, sizeof(buf),
             "w axis0.controller.config.vel_limit %f", vel_limit);
    sendCmd(buf);

    snprintf(buf, sizeof(buf),
             "w axis0.controller.config.vel_ramp_rate %f", vel_ramp);
    sendCmd(buf);

    sendCmd("w axis0.requested_state 1");
    sendCmd("w axis0.error 0");
    sendCmd("w axis0.motor.error 0");
    sendCmd("w axis0.encoder.error 0");
    delay(200);

    snprintf(buf, sizeof(buf),
             "w axis0.motor.config.direction %d",
             target_vel_turns_s >= 0 ? 1 : -1);
    sendCmd(buf);

    sendCmd("w axis0.requested_state 5");   // SENSORLESS_CONTROL
    delay(300);

    sendCmd("w axis0.controller.config.control_mode 2");
    sendCmd("w axis0.controller.config.input_mode 2");

    snprintf(buf, sizeof(buf),
             "w axis0.controller.input_vel %f",
             target_vel_turns_s);
    sendCmd(buf);

    unsigned long t0 = millis();
    unsigned long hold_ms = (unsigned long)(run_time_s * 1000.0f);
    while (millis() - t0 < hold_ms) {}

    sendCmd("w axis0.controller.input_vel 0");
    delay(200);
    sendCmd("w axis0.requested_state 1");
}

// ------------------------------------------------------
// Setup
// ------------------------------------------------------
void setup() {
    pinMode(emgPin, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    odrv.begin(115200);
    delay(800);

    // Startup calibration spin
    spinSensorless(0.25f, 1.0f, currentLimit);

    // Warning LED
    delay(1000);
    for (int i = 0; i < 2; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }

    startTime = millis();
}

// ------------------------------------------------------
// Main loop
// ------------------------------------------------------
void loop() {

    // ----- AUTO SHUTDOWN -----
    if (millis() - startTime > maxRuntime) {
        spinSensorless(0.0f, 0.1f, currentLimit);
        Serial.println("STOP: 25 second timeout");
        while (1) delay(1000);
    }

    // ----- EMG SMOOTHING -----
    uint32_t sum = 0;
    for (int i = 0; i < avgCount; i++) {
        sum += analogRead(emgPin);
    }
    int emgRaw = sum / avgCount;

    // ----- SERIAL PLOTTER -----
    Serial.print(emgRaw);
    Serial.print('\t');
    Serial.println(virtualPos);

    // ----- CONTROL LOGIC -----
    if (emgRaw > flexThreshold) {
        if (virtualPos < posMax) {
            virtualPos += posStep;
            spinSensorless(fwdVel, controlBurstTime, currentLimit);
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
        }
    } else {
        if (virtualPos > posMin) {
            virtualPos -= posStep;
            spinSensorless(backVel, controlBurstTime, currentLimit);
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
        }
    }

    delay(20);
}
