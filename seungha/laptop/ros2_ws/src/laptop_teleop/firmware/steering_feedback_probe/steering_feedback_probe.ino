// Read-only analog probe. It never configures or drives ESC/steering outputs.
// Disconnect the drive battery or lift all wheels before uploading any sketch.

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.print(F("ANALOG_CHANNELS="));
  Serial.println(NUM_ANALOG_INPUTS);
  Serial.println(F("Move the steering slowly by hand; find the A-pin that changes."));
}

void loop() {
  for (uint8_t channel = 0; channel < NUM_ANALOG_INPUTS; ++channel) {
    if (channel > 0) Serial.print(',');
    Serial.print('A');
    Serial.print(channel);
    Serial.print('=');
    Serial.print(analogRead(channel));
  }
  Serial.println();
  delay(200);
}

