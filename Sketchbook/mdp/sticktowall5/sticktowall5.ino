// no servo
// complete sticktowall
//

#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <PID_v1.h>

#define goalX 20
#define goalY 15

#define rd 562.0
#define oneGrid 281.0
#define one360 1650.0

#define urPWM 3
#define urTRIG 5
#define leftHeadPin 17
// #define leftTailPin 16
#define leftFrontPin 16
#define enLeft 11

DualVNH5019MotorShield md;
HMC5883L compass;
double input, output, target;
PID pid(&input, &output, &target, 5, 2, 0, DIRECT);

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

// const int urPWM = 3; // PWM Output 0－25000US，Every 50US represent 1cm
// const int urTRIG = 5; // PWM trigger pin
// const int leftHeadPin = 17; // A5
// const int leftTailPin = 16; // A4

int disFL;
int disFR;
int disL;

int N[8];
int Nnow;

void go() {
  while (1) {
    // while (1) {
    //   leftHead = smoothOutput(getDis21(leftHeadPin) - 8, leftHeadWindow, leftHeadSum, leftHeadMarker);
    //   leftTail = smoothOutput(getDis21(leftTailPin) - 8, leftTailWindow, leftTailSum, leftTailMarker);
    //   // Serial.print("front..............................: ");
    //   // Serial.println(front);
    //   // Serial.print("leftHead...........................: ");
    //   // Serial.println(leftHead);
    //   // Serial.print("leftTail...........................: ");
    //   // Serial.println(leftTail);
      
    //   int re = selfAdjust(leftHead, leftTail, 4, 6, 10);
      
    //   if (re == 1) {
    //     // Serial.println("done");
    //     break;
    //   }
    //   delay(100);
    // }

    // if (curPos[0] == X - 1 && curPos[1] == Y - 1)
    //   break;

    // Serial.print("leftEmpty: ");
    // Serial.println(leftEmpty);
    // while (!Serial.available());
    // Serial.read();

    disL = getDis21(leftHeadPin) - 8;

    if (disL > 10) {
      leftEmpty++;
    } else {
      leftEmpty = 0;
    }

    if (leftEmpty == 3) {
      leftEmpty = 0;
      // Serial.println("turn left");
      rotateLeft3(2);
      goAhead3(1);
      continue;
    } 

    disFL = getDis21(leftFrontPin) - 8;
    disFR = PWM_Mode_getDis();
    if (disFL < 10 || disFR < 10) {
      // Serial.println("turn right");
      rotateLeft3(-2);
      if (disFR > 10) {
        leftEmpty = 1;
      } else {
        leftEmpty = 0;
      }
      continue;
    }

    // Serial.println("go ahead");
    goAhead3(1);
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
  delay(1000);

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
  float heading = atan2(scaled.YAxis + 15, scaled.XAxis + 85);
  if (heading < 0)
    heading += 2 * PI;
  return heading * 180.0 / M_PI;
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

  int spe = 200 * neg;
  int leftCompensate = 23 * neg;
  
  int a = need / 50;
  int b = need % 50; // need = a * 100 + b
  
  target = N[Nnow];

  md.setSpeeds(spe, spe);

  for (int i = 0; i < a; i++) {
    need = 50;
    while (need--) {
      while (digitalRead(enLeft));
      while (!digitalRead(enLeft));
    }

    input = smoothOutput(getHeading(), inputWindow, inputSum, inputMarker);
    // Serial.println(input);
    pid.Compute();
    output *= neg;
    md.setSpeeds(spe + leftCompensate + output, spe - output);
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
void shiftLeft(float grid) {
  rotateLeft3(2);
  goAhead3(grid);
  rotateLeft3(-2);
}
int selfAdjust(float leftHead, float leftTail, int S, int M, int L) {
  if (L < leftHead && L < leftTail) {
    return 1; // reutn 1 if done
  } else if (M < leftHead && leftHead < L && M < leftTail && leftTail < L) {
    shiftLeft(0.1);
  } else if (leftHead < S && leftTail < S) {
    shiftLeft(-0.1);
  } else if (leftHead < M && M < leftTail) {
    rotateLeft(-5);
  } else if (M < leftHead && leftTail < M) {
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