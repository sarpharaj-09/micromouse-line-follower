// ESP32 DevKit V1 - Read 8 analog IR sensors (with IR control)
// Wiring assumed:
// D1->32, D2->33, D3->34, D4->35, D5->36 (VP), D6->39 (VN), D7->25, D8->26
// IR -> GPIO4
// Make sure to use resistor divider on each sensor output to keep V <= ~3.0V

#define IR_PIN 4

int sensorPins[8] = {32, 33, 34, 35, 36, 39, 25, 26};

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(IR_PIN, OUTPUT);
  digitalWrite(IR_PIN, HIGH); // turn IR LEDs ON

  // Set ADC resolution to 12-bit (0..4095)
  analogReadResolution(12);

  // Set ADC attenuation (optional but helpful)
  // ADC_11db allows full-scale ~0-3.6V (note: still keep divider in place)
  for (int i = 0; i < 8; ++i) {
    // analogSetPinAttenuation is available in Arduino core for ESP32
    analogSetPinAttenuation(sensorPins[i], ADC_11db);
  }

  Serial.println("Starting 8-channel analog IR read...");
}

void loop() {
  int values[8];

  for (int i = 0; i < 8; ++i) {
    values[i] = analogRead(sensorPins[i]);
  }

  // Print tab-separated values
  for (int i = 0; i < 8; ++i) {
    Serial.print(values[i]);
    if (i < 7) Serial.print('\t');
  }
  Serial.println();

  delay(30); // ~33 Hz sample rate. Reduce delay for faster sampling.
}
