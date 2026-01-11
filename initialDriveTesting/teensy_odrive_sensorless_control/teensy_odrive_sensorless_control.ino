// Teensy 4.1  <->  ODrive UART sensorless spin test
// Wiring (default ODrive UART0):
//   Teensy TX1 (pin 1/35) -> ODrive GPIO2 (RX)
//   Teensy RX1 (pin 0/34) -> ODrive GPIO1 (TX)
//   GND <-> GND
// Both at 115200 baud.

#include <Arduino.h>

// HardwareSerial &odrv = Serial1;
HardwareSerial &odrv = Serial8; //switched serial; teensy tx8 (35) -> odrive gpio2 (rx); teensy rx8 (34) -> odrive gpio1 (tx)

const int AXIS_STATE_IDLE               = 1;
const int AXIS_STATE_SENSORLESS_CONTROL = 5;  // old-style sensorless
const int CONTROL_MODE_VELOCITY_CONTROL = 2;
const int INPUT_MODE_VEL_RAMP           = 2;

void sendCmd(const char *cmd) {
    odrv.print(cmd);
    odrv.print('\n');
    odrv.flush();
    delay(4);                     // tiny pause so ODrive can echo
    while (odrv.available()) {    // forward replies to USB serial (optional)
        Serial.write(odrv.read());
    }
}
void spinSensorless(float target_vel_turns_s,
                    float run_time_s,
                    float current_limit) {

    char buf[96];

    // match your python math
    float vel_limit = fabsf(target_vel_turns_s) * 2.0f + 5.0f;
    float vel_ramp  = fabsf(target_vel_turns_s) * 5.0f + 5.0f;

    snprintf(buf, sizeof(buf), "w axis0.motor.config.current_lim %f", current_limit);
    sendCmd(buf);

    snprintf(buf, sizeof(buf), "w axis0.controller.config.vel_limit %f", vel_limit);
    sendCmd(buf);

    snprintf(buf, sizeof(buf), "w axis0.controller.config.vel_ramp_rate %f", vel_ramp);
    sendCmd(buf);

    sendCmd("w axis0.requested_state 1");     // idle
    sendCmd("w axis0.error 0");
    sendCmd("w axis0.motor.error 0");
    sendCmd("w axis0.encoder.error 0");
    delay(200);

    //manually edits buffer to allow for ccw rotatino
    snprintf(buf, sizeof(buf),
         "w axis0.motor.config.direction %d",
         target_vel_turns_s >= 0 ? 1 : -1);
    sendCmd(buf);

    sendCmd("w axis0.requested_state 5");     // SENSORLESS_CONTROL
    delay(300);

    snprintf(buf, sizeof(buf), "w axis0.controller.config.control_mode %d", CONTROL_MODE_VELOCITY_CONTROL);
    sendCmd(buf);
    snprintf(buf, sizeof(buf), "w axis0.controller.config.input_mode %d", INPUT_MODE_VEL_RAMP);
    sendCmd(buf);

    snprintf(buf, sizeof(buf), "w axis0.controller.input_vel %f", target_vel_turns_s);
    sendCmd(buf);

    unsigned long t0 = millis();
    unsigned long hold_ms = (unsigned long)(run_time_s * 1000.0f);
    while (millis() - t0 < hold_ms) {
        // could poll axis0.current_state here if you want
    }

    sendCmd("w axis0.controller.input_vel 0");
    delay(500);
    sendCmd("w axis0.requested_state 1");
}

void setup() {
    pinMode(34, INPUT);
    pinMode(35, OUTPUT);
    Serial.begin(115200);    
     // USB to PC (just to see echoes)
    odrv.begin(115200);       // UART to ODrive
    // Serial.println(odrv);
    delay(1000);

    Serial.println("Staring to connect");
    // same numbers you used in python:
    odrv.print("r axis0.error\n");

    Serial.println("Starting to spin forward");
    
    spinSensorless(1.0f, 2.0f, 5.0f);
    // delay(1000);
    spinSensorless(-0.25f, 1.0f, 5.0f);
}

void loop() {
    // nothing
}
