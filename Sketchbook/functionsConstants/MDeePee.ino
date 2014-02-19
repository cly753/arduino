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
#define compassX 125
#define compassY -10

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
************update parameter************
________________________________________

float getDis21(int pin) {
  return 12343.85 * pow(analogRead(pin),-1.15);
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
  
  float target2 = getHeading();
  delay(200);
  target = (getHeading() + target2) / 2.0;
  delay(200);

  // target = N[Nnow];

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

void rotateLeft(int degree) { // require md, encoder left, encoder righ, 
  float neg = 1.0;
  if (degree < 0) neg = -1.0;
  int need = degree / 360.0 * one360 * neg;

  // int now = getHeading();
  // target = now + degree;
  // int pidTime = 10;
  // if (target > 360)
  //   target -= 360;
  // if (target < 0)
  //   target += 360;

  md.setSpeeds(-150 * neg, 150 * neg);

  while (need--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }

  md.setSpeeds(100 * neg, -100 * neg); // brake compensate // try
  delay(50);

  md.setBrakes(400, 400);

  // while (pidTime--) {
  //   input = getHeading();
  //   pid.Compute();
  //   md.setSpeeds(- (int)output, (int)output);

  //   Serial.print("\nheading: ");
  //   Serial.println(input);
  //   Serial.print("output: ");
  //   Serial.println(output);
  //   delay(100);
  // }
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
  int des = getTargetDirection(quarter);

  md.setSpeeds(-100 * neg, 100 * neg);
  while (des - now > 1 || des - now < -1)
    now = getHeading();
  md.setBrakes(400, 400);
}

void shiftLeft(float grid) {
    rotateLeft(90);
    goAhead(grid);
    rotateLeft(-90);
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
    rotateLeft(-5);
  } else if (M < leftHead && leftTail < M) {
    Serial.println("turn left");
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
// void adjustHeading() {
//     int st = getHeading();
//     int error;

//     // go...go...

//     error = getHeading() - st;
//     rotateLeft(error);
// }

// compensate for brake

// sense twice for stable data

// raise the direction of ultrasonic if placed vertical

// function to get performance of motor with same speed