// add PID
// add Compass (goAhead, rotateLeft)

#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <PID_v1.h>

#define rd 562.0
#define oneGrid 281.0
#define one360 1650.0

DualVNH5019MotorShield md;
HMC5883L compass;

double input, output, target;
float inputWindow[3] = {0, 0, 0};
float inputSum = 0;
int inputMarker = 0;
PID pid(&input, &output, &target, 10, 3, 0, DIRECT);

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
int frontLeft;
int frontRight;
float left;
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
void setPID() {
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-100, 100);
  pid.SetSampleTime(200); // ???
}
float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  float heading = atan2(scaled.YAxis - 10, scaled.XAxis + 125);
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

void rotateLeft2(int degree) {
  float neg = 1.0;
  if (degree < 0) neg = -1.0;
  float now = getHeading();
  int des = ((int)now - degree) % 360;

  md.setSpeeds(-200 * neg, 200 * neg);
  while (des - now > 2 || des - now < -2) {
    now = getHeading();
  }
  md.setBrakes(400, 400);
}
void goAhead2(float grid) {
  // float st = getHeading();
  // float now;
  
  float neg = 1.0;
  if (grid < 0) neg = -1.0;
  int need = grid * oneGrid * neg;
  int spe = 150 * neg;

  int a = need / 50;
  int b = need % 50; // need = a * 100 + b

  int integrate = 0;
  int error;
  
  float target2 = getHeading();
  delay(200);
  target = (getHeading() + target2) / 2.0;
  delay(200);

  md.setSpeeds(spe, spe);

  for (int i = 0; i < a; i++) {
    need = 50;
    while (need--) {
      while (digitalRead(enLeft));
      while (!digitalRead(enLeft));
    }

    // now = getHeading();
    // integrate += now - st;
    // error = 2 * (now - st) + 1 * integrate;
    
    // input = getHeading();
    input = smoothOutput(getHeading(), inputWindow, inputSum, inputMarker);
    pid.Compute();
    output *= neg;
    md.setSpeeds(spe + output, spe - output);
  }

  while (b--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  // rotateLeft2(getHeading() - st);
  md.setBrakes(400, 400);
}
void shiftLeft(float grid) {
    rotateLeft2(90);
    goAhead2(grid);
    rotateLeft2(-90);
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
    rotateLeft2(-5);
  } else if (M < leftHead && leftTail < M) {
    Serial.println("turn left");
    rotateLeft2(5);
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

    left = 15;
    if (leftHead < 10 || leftTail < 10)
      left = 5;

    // if (leftTail > 10) {
    //   Serial.println("here1");
    //   if (leftHead > 10) {
    //     Serial.println("here2");
    //     rotateLeft2(90);
    //     goAhead2(1);
    //   } else if (front > 10) {
    //     Serial.println("here3");
    //     goAhead2(1);
    //   } else {
    //     Serial.println("here4");
    //     rotateLeft2(-90);
    //   }
    // } else if (front > 10) {
    //   Serial.println("here5");
    //   goAhead2(1);
    // } else {
    //   Serial.println("here6");
    //   rotateLeft2(-90);
    // }

    if (left > 10) {
      rotateLeft2(90);
      pid.Reset(); // <---
      goAhead2(1);
    } else if (front > 10) {
      pid.Reset(); // <---
      goAhead2(1);
    } else {
      rotateLeft2(-90);
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
    setPID();

    delay(1000);
    rotateLeft2(90);
    // go();
}

void loop() {}
