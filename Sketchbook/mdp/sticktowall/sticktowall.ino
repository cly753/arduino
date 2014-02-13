#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield md;

const int urPWM = 3; // PWM Output 0－25000US，Every 50US represent 1cm
const int urTRIG=5; // PWM trigger pin
const int leftHeadPin = 14; // A0
const int leftTailPin = 15; // A1
int front;
int leftHead;
int leftTail;

float getDis21(int pin) {
  Serial.print("from ");
  Serial.print(pin);
  Serial.println(": ");
  Serial.print(12343.85 * pow(analogRead(pin),-1.15));
  Serial.println("cm");
  return 12343.85 * pow(analogRead(pin),-1.15);
}

void PWM_Mode_Setup() {
  pinMode(urTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(urTRIG,HIGH);                  // Set to HIGH
  
  pinMode(urPWM, INPUT);                      // Sending Enable PWM mode command
  
  uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
  for(int i=0;i<4;i++)
      Serial.write(EnPwmCmd[i]);
}
int PWM_Mode_getDis() {                              // a low pull on pin COMP/TRIG  triggering a sensor reading
    digitalWrite(urTRIG, LOW);
    digitalWrite(urTRIG, HIGH);               // reading Pin PWM will output pulses
    
    return pulseIn(urPWM, LOW) / 50;
}

void rotateLeft(int degree) { // require md, encoder left, encoder righ, 
    const int one360 = 1650;
    int need = degree / 360 * one360;

    if (degree < 0) {
        need *= -1;
        md.setSpeeds(200, -200);
    } else {
        md.setSpeeds(200, -200);
    }

    while (need) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need--;
    }
    md.setBrakes(400, 400);
}
void goAhead(int grid) {
    const int oneGrid = 562 * 10 / 20;
    int need = grid * oneGrid;

    md.setSpeeds(200, 200);
    while (need) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need--;
    }
    md.setBrakes(400, 400);
}

void go() {
  Serial.println("11111");
  while (1) {
    Serial.println("2222");

    front = (int)PWM_Mode_getDis();
    leftHead = getDis21(leftHeadPin) - 8;
    leftTail = getDis21(leftTailPin) - 8;

    Serial.print("front............................... ");
    Serial.println(front);
    Serial.print("leftHead............................ ");
    Serial.println(leftHead);
    Serial.print("leftTail............................ ");
    Serial.println(leftTail);

    if (leftTail > 15) {
      if (leftHead > 15) {
        rotateLeft(90);
        goAhead(1);
      } else if (front > 10) {
        goAhead(1);
      } else {
        rotateLeft(-90);
      }
    } else if (front > 10) {
      goAhead(1);
    } else {
      rotateLeft(-90);
    }
  }
}

void setup() {
    Serial.begin(9600);
    PWM_Mode_Setup();

    md.init();

    go();
}

void loop() {

}

