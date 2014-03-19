1. pid

2. measure one grid

3. test ultrasonic sensor

4. rotateLeft slow down


#include <Wire.h>
#include <HMC5883L.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>
#include <Servo.h>

#define rd 562.0
#define oneGrid 281.0
#define one360 1650.0 // hall stair
#define one360 1633.0 // hall room floor
#define one360 1450.0 // hall room floor 90`
#define compassX 135
#define compassY 135

DualVNH5019MotorShield md; // motor1 is on the right // motor2 is on the left
HMC5883L compass;
double input, output, target;
PID pid(&input, &output, &target, 10, 0, 0, DIRECT);
Servo myservo;

float smooth[3] = {0, 0, 0};
float smoothSum = 0;
int marker = 0;

float N[8] = {N, NE, E, SE, S, SW, W, NW};
int Nnow;
________________________________________
****************************************
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

float getDis21(int pin) { // small
  
  return 12343.85 * pow(analogRead(pin),-1.15); // directly use voltage level to determine wall?
}

float getDis02(int pin) { // big

    return 30431 * pow(analogRead(pin), -1.169);
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

void setCompass() {
  compass = HMC5883L();
  compass.SetMeasurementMode(Measurement_Continuous);
}
float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  float heading = atan2(scaled.YAxis + compassX, scaled.XAxis + compassY);
  if (heading < 0)
    heading += 2 * PI;
  return heading * 180.0 / M_PI;
}

void setPID() {
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-100, 100);
  pid.SetSampleTime(200); // ?
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
void goAhead2(float grid) {
  float neg = 1.0;
  if (grid < 0) neg = -1.0;
  int need = grid * oneGrid * neg;
  int spe = 150 * neg;

  int a = need / 50;
  int b = need % 50; // need = a * 50 + b

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
    Serial.println(input);
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

void rotateLeft(int degree) { // require md, encoder left, encoder righ, 
  float neg = 1.0;
  if (degree < 0) neg = -1.0;
  int need = degree / 360.0 * one360 * neg;

  md.setSpeeds(-150 * neg, 150 * neg);

  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  // md.setSpeeds(100 * neg, -100 * neg); // brake compensate // try
  // delay(50);

  md.setBrakes(400, 400);
}
void rotateLeft2(int degree) {
  float neg = 1.0;
  if (degree < 0) neg = -1.0;
  float now = getHeading();
  int des = ((int)now - degree) % 360;

  md.setSpeeds(-100 * neg, 100 * neg);
  while (des - now > 1 || des - now < -1)
    now = getHeading();
  md.setBrakes(400, 400);
}
void rotateLeft3(int quarter) {
  float neg = 1.0;
  if (quarter < 0) neg = -1.0;
  Nnow = (Nnow - quarter + 8) % 8;
  int des = N[Nnow];
  
  float now = getHeading();

  md.setSpeeds(-150 * neg, 150 * neg);
  
  while (des - now > 4 || des - now < -4) {
    now = getHeading();
    Serial.println(now);
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
int selfAdjust2(float leftHead, float leftTail, int S, int M, int L) {

  // longer distance
  //
  //
  //   Serial.println("do nothing");
  //   return 1; // reutn 1 if done
  // } else if (M < leftHead && leftHead < L && M < leftTail && leftTail < L) {
  //   Serial.println("shift left");
  //   shiftLeft(0.1);
  // } else if (leftHead < S && leftTail < S) {
  //   Serial.println("shift right");
  //   shiftLeft(-0.1);
  // } else if (leftHead < M && M < leftTail) {
  //   Serial.println("turn right");
  //   rotateLeft(-5);
  // } else if (M < leftHead && leftTail < M) {
  //   Serial.println("turn left");
  //   rotateLeft(5);
  // } else {
  //   return 1;
  // }
    
  return 0; // return 0 if not done
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

  md.setSpeeds(spe - (leftTime - rightTime) * 0.1, spe + (leftTime - rightTime) * 0.1);
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
void storeDirection() {
  delay(100);
  int now = getHeading();
  delay(100);
  now = (now + getHeading()) / 2;
  for (int i = 0; i < 8; i++)
    N[i] = (now + i * 45) % 360;
}
void setCompassOffset() {
  MagnetometerRaw raw;
  md.setSpeeds(200, -200);

  int minX = 600;
  int maxX = -600;
  int minY = 600;
  int maxY = -600;

  int need = one90speed200 * 8;
  while (need--) {
    raw = compass.ReadRawAxis();
    if (abs(raw.XAxis) <= 400 && abs(raw.YAxis) <= 400) {
      minX = min(raw.XAxis, minX);
      maxX = max(raw.XAxis, maxX);
      minY = min(raw.YAxis, minY);
      maxY = max(raw.YAxis, maxY);
    }

    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  md.setBrakes(400, 400);
  
  compassX = (minX + maxX) / 2;
  compassY = (minY + maxY) / 2;

  delay(1000);
}

// compensate for brake

// sense twice for stable data

// raise the direction of ultrasonic if placed vertical

// function to get performance of motor with same speed

void checklistA1() {
  // disable "storeDirection"
  padMode();
}
void checklistA2() {
  // disable "storeDirection"
  while (1) {
    Serial.print("distance: ");
    Serial.println(PWM_Mode_getDis());
    delay(250);
  }
}
void checklistA3() {
  // disable "storeDirection"
  goAhead4(10);
}
void checklistA4() {
  // disable "storeDirection"
  Nnow = 0;
  N[0] = getHeading();
  int degree = 180;
  int times = degree / 90;
  for (int i = 0; i < times; i++) {
    rotateLeft4(1);
    if (Nnow == 0)
      correct();
  }
}
void checklistA5() {
  // disable "storeDirection"
  Nnow = 0;
  N[Nnow] = getHeading();
  for (int x = 0; x < 2; x++) {
    for (int i = 0; i < 15; i++) {
      // delay(200);
      int disFM = -1;
      while (disFM == -1) {
        disFM = PWM_Mode_getDis() - 1;
      }
      if (disFM > 10) {
        goAhead4(1);
        correct();
      } else {
        rotateLeft4(1);
        goAhead4(3);
        rotateLeft4(-1);
        goAhead4(4);
        rotateLeft4(-1);
        goAhead4(3);
        rotateLeft4(1);
        i += 3;

        correct();
      }
    }
    delay(300);
    rotateLeft4(1);
    rotateLeft4(1);
    delay(300);
    N[Nnow] = getHeading();
  }
}
void checklistA6() {
  // disable "storeDirection"
  goAhead4(1);
  driftLeft();
  goAhead4(1);
}