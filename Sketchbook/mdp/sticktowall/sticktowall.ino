#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>

#define rd 562.0
#define oneGrid 281.0
#define one360 1650.0

DualVNH5019MotorShield md;
HMC5883L compass;

const int urPWM = 3; // PWM Output 0－25000US，Every 50US represent 1cm
const int urTRIG=5; // PWM trigger pin
const int leftHeadPin = 17; // A5
const int leftTailPin = 16; // A4
int front;
int leftHead;
int leftTail;
int enRight;
int enLeft = 11;

float getDis21(int pin) {
  return 12343.85 * pow(analogRead(pin),-1.15);
}
void setCompass() {
  compass = HMC5883L();
  compass.SetMeasurementMode(Measurement_Continuous);
}
float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  if (heading < 0)
    heading += 2 * PI;

  return heading * 180.0 / M_PI;
}
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

    Serial.print("rotate1 ");
    Serial.print(degree);
    Serial.print(", need: ");
    Serial.println(need);

    if (degree < 0) {
        need *= -1;
        md.setSpeeds(-200, 200);
    } else {
        md.setSpeeds(200, -200);
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
void goAhead(int grid) { // integrated with adjustHeading()
    delay(100);
    float st = getHeading();
    float error;
    
    int need = grid * oneGrid;
    
    if (grid < 0) {
      need *= -1;
      md.setSpeeds(-200, -200);
    } else {
      md.setSpeeds(200, 200);
    }

    while (need) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need--;
        // Serial.println(need);
    }
    md.setBrakes(400, 400);
    
    delay(100);
    error = getHeading() - st;
    error = -0.3 * error;
    Serial.print("error: ");
    Serial.print(error);
    rotateLeft(error);
    Serial.println("go3");
}
void shiftLeft(float grid) {
    rotateLeft(45);
    goAhead(1.414 * grid);
    rotateLeft(-45);
    goAhead(-1.414 * grid);
}


void go() {
  while (1) {
    front = (int)PWM_Mode_getDis();
    leftHead = getDis21(leftHeadPin) - 8;
    leftTail = getDis21(leftTailPin) - 8;

    Serial.print("front..............................: ");
    Serial.println(front);
    Serial.print("leftHead...........................: ");
    Serial.println(leftHead);
    Serial.print("leftTail...........................: ");
    Serial.println(leftTail);

    if (leftTail > 7) {
      Serial.println("here1");
      if (leftHead > 7) {
        Serial.println("here2");
        rotateLeft(90);
        goAhead(1);
      } else if (front > 7) {
        Serial.println("here3");
        goAhead(1);
      } else {
        Serial.println("here4");
        rotateLeft(-90);
      }
    } else if (front > 7) {
      Serial.println("here5");
      goAhead(1);
    } else {
      Serial.println("here6");
      rotateLeft(-90);
    }

    Serial.println("here");

    delay(100);
  }
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    PWM_Mode_Setup();
    setCompass();
    md.init();

    delay(1000);
    go();
}

void loop() {}
