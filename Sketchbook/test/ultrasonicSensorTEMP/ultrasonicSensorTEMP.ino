int total = 0;
int result[4];
int i;
uint8_t DMCmd[4] ={0x11, 0x00, 0x00, 0x11};

void setup() {
    Serial.begin(9600);
    delay(200);
}

void loop() {
    for (i = 0; i < 4; i++)
      Serial.write(DMCmd[i]);
    
    if (Serial.available() > 0) {
      for (i = 0; i < 4; i++)
        result[i] = Serial.read();
      total = (result[1] & 0x0f) * 255 + result[2];
      
      for (i = 0; i < 4; i++) {
        Serial.print(result[i], BIN);
        Serial.print(" ");
      }
      Serial.print("result[1]&0xF0 = ");
      Serial.print(result[1] & 0xf0);
      Serial.print(", total=");
      Serial.println(total);
      
      Serial.print("      temperature=");
      if (result[1] & 0xf0 < 0x01) // wrong result here
        Serial.print("+");
      else
        Serial.print("-");
      Serial.println(total/10.0);
      Serial.println();
    }
    delay(500);
}
