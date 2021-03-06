#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <PID_v1.h>

#define rd 562.0
#define oneGrid 281.0
#define one360 1650.0

DualVNH5019MotorShield md;
HMC5883L compass;

float leftHeadWindow[3] = {0, 0, 0};
float leftTailWindow[3] = {0, 0, 0};
float leftHeadSum = 0;
float leftTailSum = 0;
int leftHeadMarker = 0;
int leftTailMarker = 0;

const int urPWM = 3; // PWM Output 0－25000US，Every 50US represent 1cm
const int urTRIG=5; // PWM trigger pin
const int leftHeadPin = 17; // A5
const int leftTailPin = 16; // A4
int front;
float leftHead;
float leftTail;
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
  float heading = atan2(scaled.YAxis, scaled.XAxis + 120);
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

void rotateLeft(int degree) { // require md, encoder left, encoder righ, 
  float neg = 1.0;
  if (degree < 0) neg = -1.0;
  int need = degree / 360.0 * one360 * neg;

  // int now = getHeading();
  // target = now + degree;
  // int pidTime = 10;
  // if (target > 360)
  //   target -= 360;
  // if (target < 0)
  //   target += 360;

  md.setSpeeds(-150 * neg, 150 * neg);

  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }
  md.setSpeeds(100 * neg, -100 * neg); // brake compensate // try
  delay(50);

  md.setBrakes(400, 400);

  // while (pidTime--) {
  //   input = getHeading();
  //   pid.Compute();
  //   md.setSpeeds(- (int)output, (int)output);

  //   Serial.print("\nheading: ");
  //   Serial.println(input);
  //   Serial.print("output: ");
  //   Serial.println(output);
  //   delay(100);
  // }
}
void goAhead(float grid) {
  float neg = 1.0;
  if (grid < 0) neg = -1.0;
  int need = grid * oneGrid * neg;

  md.setSpeeds(200 * neg, 200 * neg);

  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  md.setBrakes(400, 400);
}
void shiftLeft(float grid) {
    rotateLeft(90);
    goAhead(grid);
    rotateLeft(-90);
}
int selfAdjust(float leftHead, float leftTail, int S, int M, int L) {
  if (L < leftHead && L < leftTail) {
    Serial.println("do nothing");
    return 1; // reutn 1 if done
  } else if (M < leftHead && leftHead < L && M < leftTail && leftTail < L) {
    Serial.println("shift left");
    shiftLeft(0.1);
  } else if (leftHead < S && leftTail < S) {
    Serial.println("shift right");
    shiftLeft(-0.1);
  } else if (leftHead < M && M < leftTail) {
    Serial.println("turn right");
    rotateLeft(-5);
  } else if (M < leftHead && leftTail < M) {
    Serial.println("turn left");
    rotateLeft(5);
  } else {
    return 1;
  }
    
  return 0; // return 0 if not done
}

float smoothOutput(float output, float window[], float smoothSum, int marker) {
  if (output < smoothSum / 3.0 - 0.5 || output > smoothSum / 3.0 + 0.5) {
    for (int i = 0; i < 3; i++)
      window[i] = output;
    smoothSum = output * 3;
  } else {
    smoothSum += output;
    smoothSum -= window[marker];
    window[marker] = output;
    marker = (marker + 1) % 3;
  }
  return smoothSum / 3.0;
}

void go() {
  while (1) {
    front = PWM_Mode_getDis();
//    leftHead = getDis21(leftHeadPin) - 8;
//    leftTail = getDis21(leftTailPin) - 8;
    leftHead = smoothOutput(getDis21(leftHeadPin) - 8, leftHeadWindow, leftHeadSum, leftHeadMarker);
    leftTail = smoothOutput(getDis21(leftTailPin) - 8, leftTailWindow, leftTailSum, leftTailMarker);

    Serial.print("front..............................: ");
    Serial.println(front);
    Serial.print("leftHead...........................: ");
    Serial.println(leftHead);
    Serial.print("leftTail...........................: ");
    Serial.println(leftTail);

    if (leftHead < 10 && leftTail < 10) {
      while (1) {
//        leftHead = getDis21(leftHeadPin) - 8;
//        leftTail = getDis21(leftTailPin) - 8;
        leftHead = smoothOutput(getDis21(leftHeadPin) - 8, leftHeadWindow, leftHeadSum, leftHeadMarker);
        leftTail = smoothOutput(getDis21(leftTailPin) - 8, leftTailWindow, leftTailSum, leftTailMarker);
        Serial.print("front..............................: ");
        Serial.println(front);
        Serial.print("leftHead...........................: ");
        Serial.println(leftHead);
        Serial.print("leftTail...........................: ");
        Serial.println(leftTail);
        
        int re = selfAdjust(leftHead, leftTail, 4, 6, 10);
        Serial.print("re: ");
        Serial.println(re);
        if (re == 1) {
          Serial.println("done");
          break;
        }
        delay(100);
      }
    }

    if (leftTail > 10) {
      Serial.println("here1");
      if (leftHead > 10) {
        Serial.println("here2");
        rotateLeft(90);
        goAhead(1);
      } else if (front > 10) {
        Serial.println("here3");
        goAhead(1);
      } else {
        Serial.println("here4");
        rotateLeft(-90);
      }
    } else if (front > 10) {
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
