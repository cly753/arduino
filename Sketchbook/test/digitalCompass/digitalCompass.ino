#include <Wire.h>
#include <HMC5883L.h>
#include <DualVNH5019MotorShield.h>

HMC5883L compass;
DualVNH5019MotorShield md;

const int rd = 562;
const int one360 = 1650;
int enLeft = 11;
int enRight = 17;

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
  rotateLeft(-0.5 * error);
  Serial.print("0.5 * error: ");
  Serial.println(-0.5 * error);
}

void testCompass2() {
  while (1) {
    //Serial.print("heading: ");
    //Serial.println(getHeading());
    getHeading();
    delay(1000);
  }
}

void rotateLeft(int degree) {
    int need = degree / 360.0 * one360;

    if (degree < 0) {
        need *= -1;
        md.setSpeeds(200, -200);
    } else {
        md.setSpeeds(-200, 200);
    }

    while (need) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need--;
    }

    md.setBrakes(400, 400);
}

float getHeading() {
//  MagnetometerScaled scaled = compass.ReadScaledAxis();
  MagnetometerRaw raw = compass.ReadRawAxis();
//  float heading = atan2(scaled.YAxis, scaled.XAxis);
  Serial.print("rawX ");
  Serial.print(raw.XAxis - 20);
  Serial.print(" raxY ");
  Serial.println(raw.YAxis + 90);
  float heading = atan2(raw.YAxis + 90, raw.XAxis - 20);
  if (heading < 0)
    heading += 2 * PI;

  return heading * 180 / M_PI;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass = HMC5883L();
  compass.SetMeasurementMode(Measurement_Continuous);
  md.init();
}

void loop() {
  testCompass2();
  delay(5000);
}
