// complete sticktowall
//

#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <PID_v1.h>

#define goalX 20
#define goalY 15

#define rd 562.0
#define oneGrid 290.0
#define one360speed150 1600.0
#define one90speed150 390
#define one90speed200 394
#define one90speed300 390 // ? 
#define one90speed400 380
#define leftCompensate350 18 // ?
#define leftCompensate250 5
#define leftCompensate150 5

#define urPWM 3
#define urTRIG 5
#define leftHeadPin 17
// #define leftTailPin 16
#define leftFrontPin 16
#define enLeft 11

DualVNH5019MotorShield md;
HMC5883L compass;
double input, output, target;
PID pid(&input, &output, &target, 4, 1, 0, DIRECT);
int distanceNow;

int curPos[2] = {2, 2};
int leftEmpty;

float inputWindow[3] = {0, 0, 0};
float inputSum = 0;
int inputMarker = 0;
float leftHeadWindow[3] = {0, 0, 0};
float leftTailWindow[3] = {0, 0, 0};
float leftHeadSum = 0;
float leftTailSum = 0;
int leftHeadMarker = 0;
int leftTailMarker = 0;

int disFL;
int disFR;
int disL;

int N[8];
int Nnow;
int tenMove;

void go() {
  leftEmpty = 0;
  // tenMove = 10;
  while (1) {
    // if (tenMove) {
    //   tenMove--;
    // } else {
    //   correct();
    //   tenMove = 10;
    // }
    delay(300);

    disL = getDis21(leftHeadPin) - 9;

    if (disL > 10) {
      leftEmpty++;
    } else {
      leftEmpty = 0;
    }

    if (leftEmpty == 3) {
      leftEmpty = 0;
      rotateLeft90speedX00(1);
      delay(300);
      goAhead3(1);
      continue;
    } 

    disFL = getDis21(leftFrontPin) - 9;
    disFR = PWM_Mode_getDis();
    if (disFL < 10 || disFR < 10) {
      rotateLeft90speedX00(-1);
      if (disFR > 10) {
        leftEmpty = 1;
      } else {
        leftEmpty = 0;
      }
      continue;
    }
    goAhead3(1);
  }
}

void correct() {
  md.setSpeeds(100, -100);
  int des = N[Nnow];
  int now = getHeading();
  int delta;
  int temp;
  int temp2;

  if (now < des)
    md.setSpeeds(150, -150);
  else
    md.setSpeeds(-150, 150);
  temp  = des - now;
  delta = abs(temp);
  temp2 = 360 - delta;
  while (min(delta, temp2) > 3) {
    now = getHeading();
    temp  = des - now;
    delta = abs(temp);
    temp2 = 360 - delta;
  }
  md.setBrakes(400, 400);
}

void setup() {
  delay(1000);
  Serial.begin(9600);
  Wire.begin();
  PWM_Mode_Setup();
  setCompass();
  md.init();
  setPID();
  storeDirection();
  pinMode(enLeft, INPUT);
  delay(500);

  go();
}

void loop() {
}

void storeDirection() {
  delay(100);
  int now = getHeading();
  delay(100);
  now = (now + getHeading()) / 2;
  for (int i = 0; i < 8; i++)
    N[i] = (now + i * 45) % 360;
  Nnow = 0;
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
float getDis21(int pin) {

  return 12343.85 * pow(analogRead(pin),-1.15);
}
float getDis02(int pin) { // big

    return 30431 * pow(analogRead(pin), -1.169);
}
float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  float heading = atan2(scaled.YAxis + 24, scaled.XAxis + 81);
  if (heading < 0)
    heading += 2 * PI;
  return heading * 180.0 / M_PI;
}

void rotateLeft360(int times) { // require md, encoder left, encoder righ, 
  float neg = 1.0;
  if (times < 0) neg = -1.0;
  int need = times * one360speed150 * neg;

  md.setSpeeds(-150 * neg, 150 * neg);
  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  md.setSpeeds(100 * neg, -100 * neg); // brake compensate // try
  delay(50);
  md.setBrakes(400, 400);
}
void rotateLeft90(int times) {
  float neg = 1.0;
  if (times < 0) neg = -1.0;
  int need = times * one90speed150 * neg;

  md.setSpeeds(-150 * neg, 150 * neg);
  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  // md.setSpeeds(100 * neg, -100 * neg); // brake compensate // try
  // delay(50);
  md.setBrakes(400, 400);
}
void rotateLeft90speedX00(int times) {
  float neg = 1.0;
  if (times < 0) neg = -1.0;
  int need = times * one90speed400 * neg;

  Nnow = (Nnow - 2 * times * (int)neg + 8) % 8;

  md.setSpeeds(-400 * neg, 400 * neg);
  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }
  md.setBrakes(400, 400);
}

void rotateLeft3(int quarter) {
  float neg = 1.0;
  if (quarter < 0) neg = -1.0;
  Nnow = (Nnow - quarter + 8) % 8;
  int des = N[Nnow];
  
  float now = getHeading();

  md.setSpeeds(-150 * neg, 150 * neg);
  
  // for (int i = 0; i < 8; i++) {
  //   Serial.print("  i: ");
  //   Serial.print(i);
  //   Serial.print(", N[i]: ");
  //   Serial.print(N[i]);
  // }
  // Serial.print("\nNnow: ");
  // Serial.println(Nnow);
  // Serial.print("des: ");
  // Serial.println(des);
  // Serial.print("now: ");
  // Serial.println(now);
 
  while (des - now > 2 || des - now < -2) {
    now = getHeading();
    Serial.println(now);
  }

  md.setBrakes(400, 400);
}
void goAhead3(float grid) {
  float neg = 1.0;
  if (grid < 0) neg = -1.0;
  int need = grid * oneGrid * neg;

  int spe = 150 * neg;
  
  int a = need / 100;
  int b = need % 100; // need = a * 100 + b

  // target = N[Nnow];

  md.setSpeeds(spe + leftCompensate150 * neg, spe);

  for (int i = 0; i < a; i++) {
    need = 100;
    while (need--) {
      while (digitalRead(enLeft));
      while (!digitalRead(enLeft));
    }

    // input = (int)smoothOutput(getHeading(), inputWindow, inputSum, inputMarker);
    // // input = getHeading();
    // pid.Compute();
    // output *= neg;
    // Serial.println("========");
    // Serial.println(target);
    // Serial.println(input);
    // Serial.println(output);
    // md.setSpeeds(spe + (leftCompensate150 + output) * neg, spe - output * neg);
  }

  while (b--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  md.setBrakes(400, 400);
  
  // if (grid == 1) {
  //   if (Nnow == 0) curPos[0]++;
  //   if (Nnow == 2) curPos[1]++;
  //   if (Nnow == 4) curPos[0]--;
  //   if (Nnow == 6) curPos[1]--;
  // }
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