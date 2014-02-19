void setup() {
  Serial.begin(9600);
}

void loop() {
  sendWhatReceive();
}

void sendWhatReceive() {
  if (Serial.available()) {
    Serial.println(Serial.read());
  }
}
