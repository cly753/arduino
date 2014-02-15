// testSelfAdjust.ino
#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield md;

const int one360 = 1650;
const int rd = 562;

const int leftHeadPin = 19; // A5
const int leftTailPin = 18; // A4
int front;
int leftHead;
int leftTail;
int enRight;
int enLeft = 11;

void PWM_Mode_Setup() {
  pinMode(urTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(urTRIG,HIGH);                  // Set to HIGH
  pinMode(urPWM, INPUT);                      // Sending Enable PWM mode command
  uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
  for(int i=0;i<4;i++)
      Serial.write(EnPwmCmd[i]);
}
int PWM_Mode_getDis() { // a low pull on pin COMP/TRIG  triggering a sensor reading
    digitalWrite(urTRIG, LOW);
    digitalWrite(urTRIG, HIGH);               // reading Pin PWM will output pulses
    return pulseIn(urPWM, LOW) / 50;
}

void rotateLeft(int degree) {
    int need = degree / 360.0 * one360;

    Serial.println("rotate1 ");
    Serial.println(degree);
    Serial.println(", need: ");
    Serial.println(need);

    if (degree < 0) {
        need *= -1;
        md.setSpeeds(-150, 150);
    } else {
        md.setSpeeds(150, -150);
    }

    Serial.println("rotate2 ");

    while (need) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need--;
    }

    Serial.println("rotate4");

    md.setBrakes(400, 400);
}
void goAhead(int grid) {
    int oneGrid = rd * 10.0 / 20.0;
    int need = grid * oneGrid;

    Serial.print("go1, grid: ");
    Serial.println(grid);
    Serial.print("go1, oneGrid: ");
    Serial.println(oneGrid);
    Serial.print("go1, need: ");
    Serial.println(need);

    if (grid < 0) {
      need *= -1;
      md.setSpeeds(-200, -200);
    } else {
      md.setSpeeds(150, 150);
    }

    while (need) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need--;

        // Serial.println(need);
    }
    md.setBrakes(400, 400);
    Serial.println("go3");
}
void shiftLeft(float grid) {
    rotateLeft(45);
    goAhead(1.414 * grid);
    rotateLeft(-45);
    goAhead(-1.414 * grid);
}

void selfAdjust(float leftHead, float leftTail) {
    if (14 < leftHead && 14 < leftTail) {

    } else if (10 < leftHead && leftHead < 14 && 10 < leftTail && leftTail < 14) {
        shiftLeft(1.5);
    } else if (7 < leftHead && leftHead < 10 && 7 < leftTail && leftTail < 10) {
        shiftLeft(1);
    } else if (leftHead < 4 && leftTail < 4) {
        shiftLeft(-1);
    } else if (4 < leftHead && leftHead < 7 && 7 < leftTail) {
        rotateLeft(-10);
    } else if (7 < leftHead && 4 < leftTail && leftTail < 7) {
        rotateLeft(10);
    }
}

void setup() {
    Serial.begin(9600);
    PWM_Mode_Setup();
    md.init();

    while (1) {
        selfAdjust();
        delay(1000);
    }
}

void loop() {

}

