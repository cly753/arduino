int dis21;
int dis02;

int disus;
uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
int trigger = 3;
int urpwm = 5;

float getDis21() {
    return 12343.85 * pow(analogRead(A0),-1.15);
}

float getDis02() {
    return 30431 * pow(analogRead(A1), -1.169);
}

float getDisus() {
    digitalWrite(trigger, LOW);
    digitalWrite(trigger, HIGH);

    unsigned long width = pulseIn(urpwm, LOW);

    return width / 50;
}

void setup() {
    Serial.begin(9600);

    for(int i=0;i<4;i++)
      Serial.write(EnPwmCmd[i]);
    pinMode(trigger, OUTPUT);
    digitalWrite(trigger, HIGH);
}

void loop() {
    Serial.print("ir02: ");
    Serial.print(getDis02());
    Serial.print("cm\nir21: ");
    Serial.print(getDis21());
    Serial.print("cm\n");
    Serial.print("us: ");
    Serial.print(getDisus());
    Serial.print("cm\n\n");

    delay(500);
}

