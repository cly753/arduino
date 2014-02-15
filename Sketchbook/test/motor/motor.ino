#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>

#define rd 562.0
#define oneGrid 281.0
#define one360 1633.0

DualVNH5019MotorShield md;
HMC5883L compass;

volatile int m1;
volatile int m2;
int enLeft = 11;
int enRight = 17;

void setCompass() {
  compass = HMC5883L();
  compass.SetMeasurementMode(Measurement_Continuous);
}
float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  if (heading < 0)
    heading += 2 * PI;
  return heading * 180 / M_PI;
}

void rotateLeft(int degree) {
    int need = degree / 360.0 * one360;

    if (degree < 0) {
        need *= -1;
        md.setSpeeds(-200, 200);
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
void goAhead(float grid) {
    int need = grid * oneGrid;

    if (need < 0) {
        need *= -1;
        md.setSpeeds(-200, -200);
    } else {
        md.setSpeeds(200, 200);
    }

    while (need--) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
    }

    md.setBrakes(400, 400);
}

void test1() {
  int res1 = 3;
  int res2 = 5;
  int count;
  int count2;

  long pulse;

  int xxx = 0;
  int target = 562 * 5;
  int target2 = 562 * 5;

  int spe;

  Serial.println("start.\n");

  count = 0;
  count2 = 0;

  for (spe = 30; spe < 400; spe += 10) {
    md.setSpeeds(spe, spe);
    delay(200);

    pulse = pulseIn(3, HIGH);

    //Serial.print("speed: ");
    Serial.print(spe);
    Serial.print(".");
    Serial.print(pulse);
    // Serial.print(". pulse2: ");
    // Serial.print(pulse2);
    // Serial.print(". pulse3: ");
    // Serial.print(pulse3);
    Serial.println();
  }

  md.setBrakes(400, 400);

  // Serial.print("count: ");
  // Serial.println(count);
  Serial.println("\nstop.");

  delay(200);
  md.setBrakes(0, 0);
}

void testCompass() {
  int need = rd * 5;
  int st;
  int error;

  delay(500);
  st = getHeading();

  md.setSpeeds(200, 200);

  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  error = getHeading() - st;
  rotateLeft(error);

  md.setBrakes(400, 400);
}

void adjustSpeed(int spe) {
    int left = 0;
    int right = 0;
    int leftTime;
    int rightTime;
    int count = 200;

    leftTime = millis();
    while (count--) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
    }
    leftTime = millis() - leftTime;

    count = 200;

    rightTime = millis();
    while (count--) {
        while (digitalRead(enRight));
        while (!digitalRead(enRight));
    }
    rightTime = millis() - rightTime;

    Serial.print("leftTime - rightTime: ");
    Serial.println(leftTime - rightTime);

    md.setSpeeds(spe - (leftTime - rightTime) * 1, spe + (leftTime - rightTime) * 1);
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
    delay(20);
    md.setBrakes(400, 400);

    delay(1000);
  }
}

void testTurn3() { 
  for (int i = 0; i < 4; i++) {
    int need = one360 / 4;
    int need2 = rd * 2;

    md.setSpeeds(100, 100);
    while (need2) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need2--;
    }
    md.setBrakes(400, 400);
    delay(100);

    md.setSpeeds(100, -100);
    while (need) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need--;
    }
    md.setBrakes(400, 400);
    delay(100);
  }
}

void testShift() {
  while (1) {
    shiftLeft(1.0);
    delay(1000);
  }
}
void shiftLeft(float grid) {
    rotateLeft(35);
    goAhead(1.414 * grid);
    rotateLeft(-35);
    goAhead(-1.414 * grid);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  md.init();
  setCompass();

  pinMode(enLeft, INPUT);
  pinMode(enRight, INPUT);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);

  testShift();
}

void loop() {}
