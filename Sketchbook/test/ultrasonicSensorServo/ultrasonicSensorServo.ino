// got some strange characters and

void setup() {
  Serial.begin(9600);
}

void loop() {
  for (int i = 10; i < 20; i++) {
    Serial.print("degree(approx.): ");
    Serial.print(i * 3);
    Serial.print(", distance: ");
    Serial.print(disAt(i));
    Serial.println("cm");
    delay(1000);
  }
}

void sendCmd(uint8_t* cmd) {
  for (int i = 0; i < 4; i++)
    Serial.write(cmd[i]);
}

int readDis() {
  int res[4];
  int distance ;
  if (Serial.available() > 4) {
    for (int i = 0; i < 4; i++)
      res[i] = Serial.read();
    
    //=== result will be wrong without this
    for (int i = 0; i < 4; i++) {
      Serial.print(res[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    //=== weird
    
    distance = res[1] * 255 + res[2];
  }
  return distance;
}

int disAt(int degree) {
  uint8_t cmd[4] = {0x22, degree, 0x00, 0x22+degree};
  sendCmd(cmd);
  return readDis();
}
  
