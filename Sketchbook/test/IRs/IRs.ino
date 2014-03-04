int dis21;
int dis02;

int disus;
uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
int trigger = 3;
int urpwm = 5;

float getDis21() {
    return 12343.85 * pow(analogRead(16),-1.15);
}

float getDis21x() {
    return 6787 / (analogRead(19) - 3) - 4;
}

float getDis02() {
    return 30431 * pow(analogRead(A0), -1.169);
}

// float getDisus() {
//     digitalWrite(trigger, LOW);
//     digitalWrite(trigger, HIGH);

//     unsigned long width = pulseIn(urpwm, LOW);

//     return width / 50;
// }

void setup() {
    Serial.begin(9600);

    // for(int i=0;i<4;i++)
    //   Serial.write(EnPwmCmd[i]);
    // pinMode(trigger, OUTPUT);
    // digitalWrite(trigger, HIGH);
}

void loop() {
    Serial.print("ir02 : ");
    Serial.print(getDis02());
    Serial.println("cm");

    // Serial.print("\nir21 : ");
    // Serial.print(getDis21());
    // Serial.println("cm");
    // Serial.print("ir21x: ");
    // Serial.print(getDis21x());
    // Serial.println("cm");

    // Serial.print("us   : ");
    // Serial.print(getDisus());
    // Serial.println("cm\n");

    delay(500);
}

