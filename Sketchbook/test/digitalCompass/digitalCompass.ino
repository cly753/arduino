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

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass = HMC5883L();
  compass.SetMeasurementMode(Measurement_Continuous);
  md.init();
  storeDirection();

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-100, 100);

  // md.setSpeeds(200, 200);
}

void loop() {
  getHeading();
}

float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  MagnetometerRaw raw = compass.ReadRawAxis();

  float heading = atan2(raw.YAxis + 35, raw.XAxis + 55);
  if (heading < 0)
    heading += 2 * PI;

  Serial.print(raw.XAxis + 55);
  Serial.print(" ");
  Serial.println(raw.YAxis + 35);
  
  // Serial.print(" heading: ");
  // Serial.println(heading * 180 / M_PI);
    
  return heading * 180 / M_PI;
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
  delay(50);

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
  Nnow = (Nnow - quarter + 8) % 8;
  int des = N[Nnow];

  md.setSpeeds(-150 * neg, 150 * neg);
  while (des - now > 2 || des - now < -2)
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
void storeDirection() {
  delay(100);
  float now = getHeading();
  delay(100);
  now = (now + getHeading()) / 2;
  for (int i = 0; i < 8; i++)
    N[i] = (now + i * 45) % 360;
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
