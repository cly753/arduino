#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>

#define rd 562.0
#define oneGrid 281.0
#define one360 1450.0

DualVNH5019MotorShield md;
HMC5883L compass;

volatile int m1;
volatile int m2;
const int enLeft = 11;
const int leftHeadPin = 17; // A5
const int leftTailPin = 16; // A4

float leftHeadWindow[3] = {0, 0, 0};
float leftTailWindow[3] = {0, 0, 0};
float leftHeadSum = 0;
float leftTailSum = 0;
int leftHeadMarker = 0;
int leftTailMarker = 0;

void testPulse() {
  long pulse;
  int spe;

  Serial.println("start.\n");

  for (spe = 30; spe < 400; spe += 10) {
    md.setSpeeds(spe, spe);
    delay(200);

    pulse = pulseIn(enLeft, HIGH);

    Serial.print(spe);
    Serial.print(" --> ");
    Serial.println(pulse);
  }

  md.setBrakes(400, 400);
  Serial.println("\nstop.");
}

void testTurn() {
  int need = one360 * 10;
  int need2 = one360;

  md.setSpeeds(400, -400);
  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }
  md.setSpeeds(100, -100);
  while (need2--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }
  md.setBrakes(400, 400);
}

void testTurn2() {
  int need;
  for (int i = 1; i < 9; i++) {
    need = one360 / 4;

    md.setSpeeds(100, -100);
    while (need--) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
    }
    md.setSpeeds(-100, 100);
    delay(50);
    md.setBrakes(400, 400);

    delay(1000);
  }
}

void testSelfAdjust() {
  float leftHead;
  float leftTail;
  
  while (1) {
    leftHead = smoothOutput(getDis21(leftHeadPin) - 8, leftHeadWindow, leftHeadSum, leftHeadMarker);
    leftTail = smoothOutput(getDis21(leftTailPin) - 8, leftTailWindow, leftTailSum, leftTailMarker);
    Serial.print("leftHead: ");
    Serial.print(leftHead);
    Serial.print(" leftTail: ");
    Serial.println(leftTail);
    selfAdjust(leftHead, leftTail, 5, 7, 30);
    delay(500);
  }
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

void setup() {
  Serial.begin(9600);
  Wire.begin();
  md.init();
  setCompass();

  pinMode(enLeft, INPUT);
  pinMode(leftHeadPin, INPUT);
  pinMode(leftTailPin, INPUT);

  //testSelfAdjust();
  // goAhead2(25);
  md.setSpeeds(400,400);
}

void loop() {}

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
void rotateLeft(int degree) { // require md, encoder left, encoder righ, 
  float neg = 1.0;
  if (degree < 0) neg = -1.0;
  int need = degree / 360.0 * one360 * neg;
  
  md.setSpeeds(150 * neg, -150 * neg);

  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  md.setBrakes(400, 400);
}
void rotateLeft2(int degree) {
  float neg = 1.0;
  if (degree < 0) neg = -1.0;
  float now = getHeading();
  int des = ((int)now + degree) % 360;

  md.setSpeeds(-100 * neg, 100 * neg);
  while (des - now > 1.5 || des - now < -1.5)
    now = getHeading();
  md.setBrakes(400, 400);
}
void goAhead(float grid) {
  float neg = 1.0;
  if (grid < 0)
    neg = -1.0;
  int need = grid * oneGrid * neg;

  md.setSpeeds(200 * neg, 200 * neg);

  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  md.setBrakes(400, 400);
}
void goAhead2(float grid) {
  float st = getHeading();
  float now;

  float neg = 1.0;
  if (grid < 0) neg = -1.0;
  int need = grid * oneGrid * neg;
  int spe = 200 * neg;

  int a = need / 100;
  int b = need % 100; // need = a * 100 + b

  md.setSpeeds(spe, spe);

  for (int i = 0; i < a; i++) {
    need = 100;
    while (need--) {
      while (digitalRead(enLeft));
      while (!digitalRead(enLeft));
    }

    now = getHeading();
    md.setSpeeds(spe + now - st, spe - now + st);
  }

  while (b--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  rotateLeft2(getHeading() - st);
  md.setBrakes(400, 400);
}
void selfAdjust(float leftHead, float leftTail, int S, int M, int L) {
  if (L < leftHead && L < leftTail) {
    Serial.println("do nothing");
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
  }
}
void shiftLeft(float grid) {
  rotateLeft(90);
  goAhead(grid);
  rotateLeft(-90);
}