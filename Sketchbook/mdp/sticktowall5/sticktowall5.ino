// no servo
// complete sticktowall
//

#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <PID_v1.h>

#define X 20
#define Y 15

#define rd 562.0
#define oneGrid 281.0
#define one360 1650.0

#define urPWM 3
#define urTRIG 5
#define leftHeadPin 17
#define leftTailPin 16
#define enLeft 11

DualVNH5019MotorShield md;
HMC5883L compass;
double input, output, target;
PID pid(&input, &output, &target, 0, 0, 0, DIRECT);

int currentPos[2] = {2, 2}; // {desX, desY}
int obstacleX[X][Y + 1];  // [0 ~ X-1][0 ~ Y]
int obstacleY[X + 1][Y];  // [0 ~ X][0 ~ Y-1]

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
int front;
int frontLeft;
int frontRight;
float left;
float leftHead;
float leftTail;
// int enLeft = 11;

int N[8];
int Nnow;

int getFront() {
  int dis;
  if (Nnow == 0) {
    if (obstacleX[currentPos[0]-1][currentPos[1]+1] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleX[currentPos[0]-1][currentPos[1]+1] = 0;
      else
        obstacleX[currentPos[0]-1][currentPos[1]+1] = 1;
    }

    if (obstacleX[currentPos[0]][currentPos[1]+1] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleX[currentPos[0]][currentPos[1]+1] = 0;
      else
        obstacleX[currentPos[0]][currentPos[1]+1] = 1;
    }

    if (obstacleX[currentPos[0]+1][currentPos[1]+1] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleX[currentPos[0]+1][currentPos[1]+1] = 0;
      else
        obstacleX[currentPos[0]+1][currentPos[1]+1] = 1;
    }

    if (obstacleX[currentPos[0]-1][currentPos[1]+1] + obstacleX[currentPos[0]][currentPos[1]+1] + obstacleX[currentPos[0]+1][currentPos[1]+1] != 0)
      return 1;
    return 0;
  }

  if (Nnow == 2) {
    if (obstacleY[currentPos[0]+1][currentPos[1]+1] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleY[currentPos[0]+1][currentPos[1]+1] = 0;
      else
        obstacleY[currentPos[0]+1][currentPos[1]+1] = 1;
    }

    if (obstacleY[currentPos[0]+1][currentPos[1]] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleY[currentPos[0]+1][currentPos[1]] = 0;
      else
        obstacleY[currentPos[0]+1][currentPos[1]] = 1;
    }

    if (obstacleY[currentPos[0]+1][currentPos[1]-1] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleY[currentPos[0]+1][currentPos[1]-1] = 0;
      else
        obstacleY[currentPos[0]+1][currentPos[1]-1] = 1;
    }

    if (obstacleY[currentPos[0]+1][currentPos[1]-1] + obstacleY[currentPos[0]+1][currentPos[1]] + obstacleY[currentPos[0]+1][currentPos[1]+1] != 0)
      return 1;
    return 0;
  }

  if (Nnow == 4) {
    if (obstacleX[currentPos[0]+1][currentPos[1]-2] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleX[currentPos[0]+1][currentPos[1]-2] = 0;
      else
        obstacleX[currentPos[0]+1][currentPos[1]-2] = 1;
    }

    if (obstacleX[currentPos[0]][currentPos[1]-2] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleX[currentPos[0]][currentPos[1]-2] = 0;
      else
        obstacleX[currentPos[0]][currentPos[1]-2] = 1;
    }

    if (obstacleX[currentPos[0]-1][currentPos[1]-2] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleX[currentPos[0]-1][currentPos[1]-2] = 0;
      else
        obstacleX[currentPos[0]-1][currentPos[1]-2] = 1;
    }

    if (obstacleX[currentPos[0]-1][currentPos[1]-2] + obstacleX[currentPos[0]][currentPos[1]] + obstacleX[currentPos[0]+1][currentPos[1]-2] != 0)
      return 1;
    return 0;
  }

  if (Nnow == 6) {
    if (obstacleY[currentPos[0]-2][currentPos[1]-1] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleY[currentPos[0]-2][currentPos[1]-1] = 0;
      else
        obstacleY[currentPos[0]-2][currentPos[1]-1] = 1;
    }

    if (obstacleY[currentPos[0]-2][currentPos[1]] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleY[currentPos[0]-2][currentPos[1]] = 0;
      else
        obstacleY[currentPos[0]-2][currentPos[1]] = 1;
    }

    if (obstacleY[currentPos[0]-2][currentPos[1]+1] == 2) {
      dis = PWM_Mode_getDis();
      if (dis > 10)
        obstacleY[currentPos[0]-2][currentPos[1]+1] = 0;
      else
        obstacleY[currentPos[0]-2][currentPos[1]+1] = 1;
    }

    if (obstacleY[currentPos[0]-2][currentPos[1]-1] + obstacleY[currentPos[0]-2][currentPos[1]] + obstacleY[currentPos[0]-2][currentPos[1]+1] != 0)
      return 1;
    return 0;
  }
}

int getLeft() {
  int dis;
  if (Nnow == 0) {
    if (obstacleY[currentPos[0]-2][currentPos[1]+1] == 2) {
      dis = getDis21(leftHeadPin);
      if (dis > 10)
        obstacleY[currentPos[0]-2][currentPos[1]+1] = 0;
      else 
        obstacleY[currentPos[0]-2][currentPos[1]+1] = 1;
    }
      
    if (obstacleY[currentPos[0]-2][currentPos[1]-1] + obstacleY[currentPos[0]-2][currentPos[1]] + obstacleY[currentPos[0]-2][currentPos[1]+1] != 0)
      return 1;
    return 0;
  }

  if (Nnow == 2) {
    if (obstacleX[currentPos[0]+1][currentPos[1]+1] == 2) {
      dis = getDis21(leftHeadPin);
      if (dis > 10)
        obstacleX[currentPos[0]+1][currentPos[1]+1] = 0;
      else 
        obstacleX[currentPos[0]+1][currentPos[1]+1] = 1;
    }

    if (obstacleX[currentPos[0]-1][currentPos[1]+1] + obstacleY[currentPos[0]][currentPos[1]+1] + obstacleY[currentPos[0]+1][currentPos[1]+1] != 0)
      return 1;
    return 0;
  }

  if (Nnow == 4) {
    if (obstacleY[currentPos[0]+1][currentPos[1]-1] == 2) {
      dis = getDis21(leftHeadPin);
      if (dis > 10)
        obstacleY[currentPos[0]+1][currentPos[1]-1] = 0;
      else 
        obstacleY[currentPos[0]+1][currentPos[1]-1] = 1;
    }

    if (obstacleY[currentPos[0]+1][currentPos[1]+1] + obstacleY[currentPos[0]+1][currentPos[1]] + obstacleY[currentPos[0]+1][currentPos[1]-1] != 0)
      return 1;
    return 0;
  }

  if (Nnow == 6) {
    if (obstacleX[currentPos[0]-1][currentPos[1]-2] == 2) {
      dis = getDis21(leftHeadPin);
      if (dis > 10)
        obstacleX[currentPos[0]-1][currentPos[1]-2] = 0;
      else 
        obstacleX[currentPos[0]-1][currentPos[1]-2] = 1;
    }

    if (obstacleX[currentPos[0]+1][currentPos[1]-2] + obstacleY[currentPos[0]][currentPos[1]-2] + obstacleY[currentPos[0]-1][currentPos[1]-2] != 0)
      return 1;
    return 0;
  }
}

void go() {
  while (1) {
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
        // Serial.println("done");
        break;
      }
      delay(100);
    }

    
    
    // front = PWM_Mode_getDis();
    // rotateLeft3(1);
    // frontRight = PWM_Mode_getDis() / 1.1;
    // rotateLeft3(-1);
    // frontLeft = PWM_Mode_getDis();
    // Serial.print("left: ");
    // Serial.println(frontLeft);
    // Serial.print("right: ");
    // Serial.println(frontRight);

    // leftHead = smoothOutput(getDis21(leftHeadPin) - 8, leftHeadWindow, leftHeadSum, leftHeadMarker);
    // leftTail = smoothOutput(getDis21(leftTailPin) - 8, leftTailWindow, leftTailSum, leftTailMarker);
    // Serial.print("front..............................: ");
    // Serial.println(front);
    // Serial.print("leftHead...........................: ");
    // Serial.println(leftHead);
    // Serial.print("leftTail...........................: ");
    // Serial.println(leftTail);

    // left = 15;
    // if (leftHead < 10 || leftTail < 10)
    //   left = 5;
      
    // front = 15;
    // if (frontLeft < 10 || frontRight < 10)
    //   front = 10;

    left = getLeft();
    front = getFront();
    if (!left) {
      rotateLeft3(2);
      goAhead3(1);
    } else if (!front) {
      goAhead3(1);
    } else {
      rotateLeft3(-2);
    }

    // Serial.println("here");
    delay(100);
    if (currentPos[0] == X && currentPos[1] == Y)
      break;
  }
  
  while(1) rotateLeft3(2);
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    PWM_Mode_Setup();
    setCompass();
    md.init();
    setPID();
    storeDirection();
    mazeInit();
    delay(1000);
    
    // go();
    // md.setSpeeds(200, 200);
}

void loop() {
 // goAhead3(10);
 // rotateLeft3(2);
 // rotateLeft3(2);
 // goAhead3(10);
}

void mazeInit() {
  for (int i = 0; i < X; i++)
    for (int j = 1; j < Y; j++)
      obstacleX[i][j] = 2;
  for (int i = 1; i < X; i++)
    for (int j = 0; j < Y; j++)
      obstacleY[i][j] = 2;
  for (int i = 0; i < X; i++) {
    obstacleX[i][0] = 1;
    obstacleX[i][Y] = 1;
  }
  for (int j = 0; j < Y; j++) {
    obstacleY[0][j] = 1;
    obstacleY[X][j] = 1;
  }
}
void storeDirection() {
  delay(100);
  float now = getHeading();
  delay(100);
  now = (now + getHeading()) / 2.0;
  for (int i = 0; i < 8; i++)
    N[i] = ((int)now + i * 45) % 360;
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
float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  float heading = atan2(scaled.YAxis + 135, scaled.XAxis + 135);
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
  while ((float)des - now > 3 || (float)des - now < -3)
    now = getHeading();
  md.setBrakes(400, 400);
}
void goAhead3(float grid) {
  float neg = 1.0;
  if (grid < 0) neg = -1.0;
  int need = grid * oneGrid * neg;
  int spe = 200 * neg;
  int leftCompensate = 10;

  int a = need / 100;
  int b = need % 100; // need = a * 100 + b
  
  target = N[Nnow];

  md.setSpeeds(spe + leftCompensate, spe);

  for (int i = 0; i < a; i++) {
    need = 100;
    while (need--) {
      while (digitalRead(enLeft));
      while (!digitalRead(enLeft));
    }

    input = smoothOutput(getHeading(), inputWindow, inputSum, inputMarker);
    pid.Compute();
    output *= neg;
    md.setSpeeds(spe + leftCompensate + output, spe - output);
  }

  while (b--) {
    while (digitalRead(enLeft));
    while (!digitalRead(enLeft));
  }
  md.setBrakes(400, 400);
  
  if (grid == 1) {
    if (Nnow == 0) currentPos[0]++;
    if (Nnow == 2) currentPos[1]++;
    if (Nnow == 4) currentPos[0]--;
    if (Nnow == 6) currentPos[1]--;
  }
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