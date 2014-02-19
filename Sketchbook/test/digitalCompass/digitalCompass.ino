#include <Wire.h>
#include <HMC5883L.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>

#define oneGrid 281.0

HMC5883L compass;
DualVNH5019MotorShield md;
double input, output, target;
PID pid(&input, &output, &target, 10, 3, 0, DIRECT);

float inputWindow[3] = {0, 0, 0};
float inputSum = 0;
int inputMarker = 0;

const int rd = 562;
const int one360 = 1650;
int enLeft = 11;

float N[8]; // N NW W SW S ES E NE
int Nnow;

void testCompass() {
  int need = rd * 5;
  float st;
  float error;

  delay(500);
  st = getHeading();

  md.setSpeeds(200, 200);
  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }
  md.setBrakes(400, 400);
  delay(200);
  
  error = getHeading() - st;
  rotateLeft(-1 * error);
  Serial.print("1 * error: ");
  Serial.println(-1 * error);
}
void testCompass2() {
  while (1) {
    // Serial.print(" heading: ");
    // Serial.println(getHeading());
    getHeading();
    delay(250);
  }
}
void testRotate90() {
  int st = getHeading();
  int des = (st + 90) % 360;
  float now = st;

  md.setSpeeds(200, -200);
  while (des - now > 1.5 || des - now < -1.5)
    now = getHeading();
  // while (des > now)
  //   now = getHeading();
  md.setBrakes(400, 400);
}
void testNormalizeDirection() {
  for (int i = 1; i < 9; i *= 2) {
    rotateLeft3(i);
    delay(500);
    rotateLeft3(-1 * i);
    delay(500);
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass = HMC5883L();
  compass.SetMeasurementMode(Measurement_Continuous);
  md.init();
  storeDirection();

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-100, 100);

  // goAhead2(12);
  // md.setSpeeds(400, 400);
  testNormalizeDirection();
}

void loop() {}

float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  MagnetometerRaw raw = compass.ReadRawAxis();
  Serial.print(raw.XAxis + 125);
  Serial.print(" ");
  Serial.println(raw.YAxis - 10);
  
  float heading = atan2(raw.YAxis - 10, raw.XAxis + 125);
  if (heading < 0)
    heading += 2 * PI;
    
  // Serial.print(" heading: ");
  // Serial.println(heading * 180 / M_PI);
    
  return heading * 180 / M_PI;
}
void rotateLeft(int degree) {
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
void rotateLeft3(int quarter) {
  float neg = 1.0;
  float now = N[Nnow];
  if (quarter < 0) neg = -1.0;
  int des = getTargetDirection(quarter);

  md.setSpeeds(-100 * neg, 100 * neg);
  while (des - now > 1 || des - now < -1)
    now = getHeading();
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
void goAhead3(float grid) {
  // float st = getHeading();
  // float now;
  
  float neg = 1.0;
  if (grid < 0) neg = -1.0;
  int need = grid * oneGrid * neg;
  int spe = 150 * neg;

  int a = need / 50;
  int b = need % 50; // need = a * 100 + b

  // int integrate = 0;
  // int error;

  target = getTargetDirection(0);

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
void storeDirection() {
  delay(100);
  float now = getHeading();
  delay(100);
  now = (now + getHeading()) / 2.0;
  for (int i = 0; i < 7; i++)
    N[i] = ((int)now + i * 45) % 360;
}
float getTargetDirection(int quarter) {
  Nnow = (Nnow + quarter) % 8;
  return N[Nnow + quarter];
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