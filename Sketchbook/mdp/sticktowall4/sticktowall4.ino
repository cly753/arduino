// add servo
//
#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <PID_v1.h>
#include <Servo.h>

#define rd 562.0
#define oneGrid 281.0
#define one360 1650.0

DualVNH5019MotorShield md;
HMC5883L compass;
double input, output, target;
PID pid(&input, &output, &target, 0, 0, 0, DIRECT);
Servo sv;

float inputWindow[3] = {0, 0, 0};
float inputSum = 0;
int inputMarker = 0;
float leftHeadWindow[3] = {0, 0, 0};
float leftTailWindow[3] = {0, 0, 0};
float leftHeadSum = 0;
float leftTailSum = 0;
int leftHeadMarker = 0;
int leftTailMarker = 0;

const int urPWM = 3; // PWM Output 0－25000US，Every 50US represent 1cm
const int urTRIG = 5; // PWM trigger pin
const int leftHeadPin = 17; // A5
const int leftTailPin = 16; // A4
int front;
int frontLeft;
int frontRight;
float left;
float leftHead;
float leftTail;
int enLeft = 11;

float N[8];
int Nnow;

void go() {
  while (1) {
    // sv.write(25);
    // delay(250);
    rotateLeft3(1);
    frontRight = PWM_Mode_getDis();

    rotateLeft3(-1);
    // sv.write(40);
    // delay(250);
    frontLeft = PWM_Mode_getDis();
    Serial.print("right: ");
    Serial.println(frontRight);

    leftHead = smoothOutput(getDis21(leftHeadPin) - 8, leftHeadWindow, leftHeadSum, leftHeadMarker);
    leftTail = smoothOutput(getDis21(leftTailPin) - 8, leftTailWindow, leftTailSum, leftTailMarker);

   // Serial.print("front..............................: ");
   // Serial.println(front);
   // Serial.print("leftHead...........................: ");
   // Serial.println(leftHead);
   // Serial.print("leftTail...........................: ");
   // Serial.println(leftTail);

    if (leftHead < 10 && leftTail < 10) {
      while (1) {
        leftHead = smoothOutput(getDis21(leftHeadPin) - 8, leftHeadWindow, leftHeadSum, leftHeadMarker);
        leftTail = smoothOutput(getDis21(leftTailPin) - 8, leftTailWindow, leftTailSum, leftTailMarker);
       // Serial.print("front..............................: ");
       // Serial.println(front);
       // Serial.print("leftHead...........................: ");
       // Serial.println(leftHead);
       // Serial.print("leftTail...........................: ");
       // Serial.println(leftTail);
        
        int re = selfAdjust(leftHead, leftTail, 4, 6, 10);
        
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
      
    front = 15;
    if (frontLeft < 10 || frontRight < 10)
      front = 10;

    if (left > 10) {
      rotateLeft3(2);
      goAhead3(1);
    } else if (front > 10) {
      goAhead3(1);
    } else {
      rotateLeft3(-2);
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
    storeDirection();
//    sv.attach(13);
//
//    delay(1000);
    go();
//    sv.write(145);
//    delay(500);
//    sv.write(25);
//    delay(1000);
//    sv.detach();
    delay(1000);
    // md.setSpeeds(200, 200);
}

void loop() {}

void storeDirection() {
  delay(100);
  float now = getHeading();
  delay(100);
  now = (now + getHeading()) / 2.0;
  for (int i = 0; i < 8; i++)
    N[i] = ((int)now + i * 45) % 360;
}
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
  float heading = atan2(scaled.YAxis + 70, scaled.XAxis + 140);
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
    digitalWrite(urTRIG, HIGH);
    return pulseIn(urPWM, LOW) / 50;
}

void rotateLeft(int degree) { // require md, encoder left, encoder righ, 
  float neg = 1.0;
  if (degree < 0) neg = -1.0;
  int need = degree / 360.0 * one360 * neg;

  md.setSpeeds(-150 * neg, 150 * neg);

  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  md.setSpeeds(100 * neg, -100 * neg); // brake compensate // try
  delay(75);

  md.setBrakes(400, 400);
}
void rotateLeft3(int quarter) {
  float neg = 1.0;
  float now = N[Nnow];
  if (quarter < 0) neg = -1.0;
  Nnow = (Nnow - quarter + 8) % 8;
  int des = N[Nnow];
  
  md.setSpeeds(-150 * neg, 150 * neg);
  while (des - now > 15 || des - now < -15)
    now = getHeading();
  md.setSpeeds(50 * neg, 50 * neg);
  while (des - now > 3 || des - now < -3)
    now = getHeading();
  md.setBrakes(400, 400);
}
void goAhead3(float grid) {
  float neg = 1.0;
  if (grid < 0) neg = -1.0;
  int need = grid * oneGrid * neg;
  int spe = 150 * neg;

  int a = need / 50;
  int b = need % 50; // need = a * 50 + b
  
  target = N[Nnow];

  md.setSpeeds(spe, spe);

  for (int i = 0; i < a; i++) {
    need = 50;
    while (need--) {
      while (digitalRead(enLeft));
      while (!digitalRead(enLeft));
    }

    input = smoothOutput(getHeading(), inputWindow, inputSum, inputMarker);
    pid.Compute();
    output *= neg;
    md.setSpeeds(spe + output, spe - output);
  }

  while (b--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }
  md.setBrakes(400, 400);
}
void shiftLeft(float grid) {
  rotateLeft3(2);
  goAhead3(grid);
  rotateLeft3(-2);
}
int selfAdjust(float leftHead, float leftTail, int S, int M, int L) {
  if (L < leftHead && L < leftTail) {
    // Serial.println("do nothing");
    return 1; // reutn 1 if done
  } else if (M < leftHead && leftHead < L && M < leftTail && leftTail < L) {
    // Serial.println("shift left");
    shiftLeft(0.1);
  } else if (leftHead < S && leftTail < S) {
    // Serial.println("shift right");
    shiftLeft(-0.1);
  } else if (leftHead < M && M < leftTail) {
    // Serial.println("turn right");
    rotateLeft(-5);
  } else if (M < leftHead && leftTail < M) {
    // Serial.println("turn left");
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
