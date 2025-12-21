// Teensy 4.1 + Two MyoWare 2.0 EMG viewers (Serial Plotter)

const int emgPin1 = A5;  // MyoWare #1 signal (board pin 19)
const int emgPin2 = A4;  // MyoWare #2 signal (board pin 18)
const int avgCount = 8;  // simple smoothing

void setup() {
  Serial.begin(115200);
  // analogReadResolution(12);  // Teensy 4.1 → 0..4095
}

void loop() {
  uint32_t sum1 = 0, sum2 = 0;
  // uint32_t sum1 = 0;

  for (int i = 0; i < avgCount; i++) {
    sum1 += analogRead(emgPin1);
    sum2 += analogRead(emgPin2);
  }

  float emg1 = (sum1 / (float)avgCount) * (3.3 / 4095.0);
  float emg2 = (sum2 / (float)avgCount) * (3.3 / 4095.0);
  // analogReadResolution(12);
  int v1 = analogRead(A5);
  int v2 = analogRead(A4);

  // float emg2 = (sum2 / (float)avgCount) * (3.3 / 4095.0);

  // Print both on one line so Serial Plotter shows two traces
  Serial.print(0.0); Serial.print("\t");
  Serial.print(3.4); Serial.print("\t");
  // Serial.print(v); Serial.print("\t");
  Serial.print(emg1);
  Serial.print("\t");
  Serial.println(emg2);

  delay(25); // ~500 Hz update rate
}
