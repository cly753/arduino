#define leftHeadPin 17
#define leftFrontPin 16
#define rightFrontPin 15
#define rearPin 14

int dis21;
int dis02;

int disus;
uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
int trigger = 5;
int urpwm = 13;

float getDis21(int pin) {
    return 12343.85 * pow(analogRead(pin),-1.15);
}

float getDis21x(int pin) {
    return 6787 / (analogRead(pin) - 3) - 4;
}

float getDis02(int pin) {
    return 30431 * pow(analogRead(pin), -1.169);
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
    // Serial.println();
    // Serial.print("left  : ");
    // Serial.print(getDis21(leftFrontPin) - 10);
    // Serial.println("cm");
    // Serial.print("right : ");
    // Serial.print(getDis21(rightFrontPin) - 10);
    // Serial.println("cm");

    // Serial.print("\nir21 : ");
    // Serial.print(getDis21(leftHeadPin));
    // Serial.println("cm");
    // Serial.print("ir21x: ");
    // Serial.print(getDis21x());
    // Serial.println("cm");

    Serial.print("center: ");
    Serial.print(getDisus());
    Serial.println("cm\n");
    delay(500);
}

