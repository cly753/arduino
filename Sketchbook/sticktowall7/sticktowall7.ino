#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define oneGridInterruptspeed200 289 // ?

#define one90Interruptspeed200 393
#define one90ToRightInterruptspeed200 392

#define one90Interruptspeed350 363
#define one90ToRightInterruptspeed350 363

#define urPWM 13
#define urTRIG 5
#define leftHeadPin A3
#define leftFrontPin A2
#define rightFrontPin A1 
#define rearPin A0
#define enLeftPin 11
#define enRightPin 3

#define frontThreshold 350
#define leftThreshold 300
// right threshold: ---530---410---330
#define leftDesiredMax 500
#define leftDesiredMin 400
#define nToCorrect 3

#define driftSpeed 130
#define driftToLeftInterruptEnLeft 500 // radius * (2 * pi) * (0.25) * (289 / 10) + calibration
#define driftToRightInterruptEnRight 500
#define driftToLeftRatio 2.75 // size of the drift circle // 
#define driftToRightRatio 2.75

DualVNH5019MotorShield md;
HMC5883L compass;

int curPos[2]; // [0]: X, [1]: Y
int goalX;
int goalY;
int leftEmpty;

int N[8];
int Nnow;

int disFL;
int disFR;
int disFM;
int disL;
int disR;

int nLeft; // after nLeft move, call 

volatile int enLeft;
volatile int enRight;
volatile int leftCompensate;
volatile int neg;
volatile int driftToLeft;
volatile int fastMode;

void go2() {
  leftEmpty = 0; // number of empty space on the left
  while (1) {

    // correct();
    correctPosition(0);
    delay(50);
    correctPositionByIRSensor(0);
    delay(50);
    // if (correctDirectionByIRSensor())
      // nLeft = nToCorrect;

    if (nLeft-- == 0) {
      rotateToCorrect();
      nLeft = nToCorrect;
    }

    int pause = 25;

    delay(pause);
    disL = smoothByMedian(leftHeadPin, 0);
    delay(pause);
    disFL = smoothByMedian(leftFrontPin, 0);
    delay(pause);
    disFM = -1;
    while (disFM == -1)
      disFM = PWM_Mode_getDis() - 1;
    delay(pause);
    disFR = smoothByMedian(rightFrontPin, 0);
    delay(pause);
    disR = smoothByMedian(rearPin, 0);

    Serial.println("================");
    Serial.println("SL" + String(disL, DEC));
    Serial.println("SFL" + String(disFL, DEC));
    Serial.println("SFM" + String(disFM, DEC));
    Serial.println("SFR" + String(disFR, DEC));
    Serial.println("SR" + String(disR, DEC));

    // while (!Serial.available() || Serial.read() != 'S'); // sense, wait for 'S', then go

    if (disL < leftThreshold)
      leftEmpty++;
    else
      leftEmpty = 0;

    if (leftEmpty == 3) {
      leftEmpty = 0;
      rotateLeft4(1);
      delay(750);
      goAhead4(1);
      
      Serial.println('L');
      Serial.println("G1");
      continue;
    }

    if (disFL > frontThreshold || disFR > frontThreshold || disFM < 10) {
      rotateLeft4(-1);

      Serial.println('R');

      leftEmpty = 0;
      if (disFR < frontThreshold && disFM > 10)
        leftEmpty = 1;
      continue;
    }
    goAhead4(1);
    Serial.println("G1");

    if (curPos[0] >= goalX - 1 && goalX + 1 >= curPos[0] && curPos[1] >= goalY - 1 && goalY + 1 >= curPos[1])
      break;  
  }
}

void correct() {
  // return ;

  int des = N[Nnow];
  int now = getHeading();
  int delta;
  int temp;
  int temp2;
  int spe = 100;

  temp  = des - now;
  delta = abs(temp);
  temp2 = 360 - delta;

  if (min(delta, temp2) < 10)
    return ;

  while (min(delta, temp2) > 1) {
    now = getHeading();
    temp  = des - now;
    delta = abs(temp);
    temp2 = 360 - delta;

    if (getDir(now, des))
      md.setSpeeds(spe, -spe);
    else 
      md.setSpeeds(-spe, spe);
  }

  md.setBrakes(400, 400);
}
bool getDir(int now, int des) {
  des -= now;
  if (des < 0) des += 360;
  return des < 180;
}
boolean correctDirectionByIRSensor() {
  int spe = 60;
  int n = 0;

  int disFL = smoothByMedian(leftFrontPin, 1);
  int disFR = smoothByMedian(rightFrontPin, 1);

  if (disFL < frontThreshold || disFR < frontThreshold)
    return false; // not corrected

  while (1) {
    n++;
    if (n == 1000)
      break;
    if (disFL > disFR) {
      md.setSpeeds(-spe, spe);
    } else {
      md.setSpeeds(spe, -spe);
    }
    disFL = smoothByMedian(leftFrontPin, 1);
    disFR = smoothByMedian(rightFrontPin, 1);
  }
  md.setBrakes(400, 400);

  return true; // corrected
}
boolean correctPosition(int atGoal) {
  int front = -1;
  delay(50);
  while (front == -1)
    front = PWM_Mode_getDis() - 1;

  if (atGoal == 0 && front > 10)
    return false; // not corrected

  while (front != 5) {   
    if (front < 5)
      md.setSpeeds(-100, -100);
    else
      md.setSpeeds(100, 100);
    front = PWM_Mode_getDis() - 1;
  }
  md.setBrakes(400, 400);

  return true; // corrected
}
boolean correctPositionByIRSensor(int atGoal) {
  delay(50);
  int front = max(smoothByMedian(leftFrontPin, 1), smoothByMedian(rightFrontPin, 1));

  if (atGoal == 0 && front < frontThreshold)
    return false; // not corrected

  long t = millis();
  while (front > 460 || front < 440) {
    if (millis() - t > 3000)
      break;
    if (front > 460)
      md.setSpeeds(-100, -100);
    else
      md.setSpeeds(100, 100);
    front = max(smoothByMedian(leftFrontPin, 1), smoothByMedian(rightFrontPin, 1));
  }
  md.setBrakes(400, 400);
  return true; // corrected
}
void rotateToCorrect() {
  //rotate left
  int pause = 100;
  rotateLeft4(1);
  delay(pause);
  
  //detect if there are obstacles in front of left front and right front sensors
  correctDirectionByIRSensor();
  delay(pause);
  correctPositionByIRSensor(0);
  delay(pause);
  //rotate right
  rotateLeft4(-1);
  delay(pause);
}
void correctToGoal() {
  for (int i = 0; i < 3; i++) {

    int pause = 20;
    delay(pause);
    disFL = smoothByMedian(leftFrontPin, 0);
    delay(pause);
    disFM = -1;
    while (disFM == -1)
      disFM = PWM_Mode_getDis() - 1;
    delay(pause);
    disFM = (PWM_Mode_getDis() - 1 + disFM) / 2;
    delay(pause);
    disFR = smoothByMedian(rightFrontPin, 0);

    if (disFL < frontThreshold && disFM > 10 && disFR < frontThreshold) {
      goAhead4(1);
    }

    if (Nnow == 0 || Nnow == 4) {
      rotateLeft4(-1);
    } else {
      rotateLeft4(1);
    } 
  }

  correctPositionByIRSensor(1);
  correctDirectionByIRSensor();

  if (Nnow == 0 || Nnow == 2) {
    while (Nnow != 4) {
      delay(300);
      rotateLeft4(-1);
    }
  } else {
    while (Nnow != 0) {
      delay(300);
      rotateLeft4(-1);
    }
  }

  rotateToCorrect();

  curPos[0] = goalX;
  curPos[1] = goalY;
}

void setup() {
  Serial.begin(9600);
  setPins();
  Wire.begin();
  // PWM_Mode_Setup();
  md.init();
  setCompass();

  while (!Serial.available() || Serial.read() != 'S');

  nLeft = nToCorrect;

  goalX = 19; // 19
  goalY = 14; // 14
  curPos[0] = curPos[1] = 2;
  go2();

  Serial.println("X");
  correctToGoal();
  delay(2000);

  goalX = 2;
  goalY = 2;
  go2();

  correctToGoal();
  delay(2000);
  Serial.println("Y");

  while (!Serial.available() || Serial.read() != 'S');

  pcMode();
}
void loop() {
  Serial.println("==========");
  // disL = smoothByMedian(leftHeadPin, 0);
  // disFL = smoothByMedian(leftFrontPin, 0);
  // disFR = smoothByMedian(rightFrontPin, 0);
  disR = smoothByMedian(rearPin, 0);
  // disFM = -1;
  // while (disFM == -1) {
  //   disFM = PWM_Mode_getDis() - 1;
  // }

  // Serial.println("SL" + String(disL, DEC));
  // Serial.println("SFL" + String(disFL, DEC));
  // Serial.println("SFR" + String(disFR, DEC));
  Serial.println("SR" + String(disR, DEC));
  // Serial.println("SFM" + String(disFM, DEC));

  // Serial.println("==========");

  // disL = analogRead(leftHeadPin);
  // Serial.println("ADC: " + String(disL, DEC));
  // Serial.print("voltage: ");
  // Serial.println(disL / 1024.0 * 5);
  // Serial.print("SFR: ");
  // Serial.print(12343.85 * pow(disL,-1.15));
  // Serial.println("cm");

  // disFL = analogRead(leftFrontPin);
  // Serial.println("ADC: " + String(disFL, DEC));
  // Serial.print("voltage: ");
  // Serial.println(disFL / 1024.0 * 5);
  // Serial.print("SFL: ");
  // Serial.print(12343.85 * pow(disFL,-1.15));
  // Serial.println("cm");
  // Serial.println();

  // disFR = analogRead(rightFrontPin);
  // Serial.println("ADC: " + String(disFR, DEC));
  // Serial.print("voltage: ");
  // Serial.println(disFR / 1024.0 * 5);
  // Serial.print("SFR: ");
  // Serial.print(12343.85 * pow(disFR,-1.15));
  // Serial.println("cm");

  // disFM = PWM_Mode_getDis();
  // Serial.print("SFM: ");
  // Serial.print(disFM);
  // Serial.println("cm");

  // Serial.print(getDis02(rearPin));
  // Serial.println("cm");
  // Serial.println(analogRead(leftHeadPin));
  delay(500);
}

void setPins() {
  pinMode(enLeft, INPUT);
  pinMode(enRight, INPUT);
  pinMode(leftFrontPin, INPUT);
  pinMode(rightFrontPin, INPUT);
  pinMode(rearPin, INPUT);
  
  pinMode(urTRIG, OUTPUT);
  pinMode(urPWM, INPUT);
}

void storeDirectionByRotation() {
  Nnow = 0;
  N[0] = getHeading();
  delay(50);
  N[0] = (N[0] + getHeading()) / 2;
  for (int i = 1; i < 4; i++) {
    rotateLeft4(-1);
    correctDirectionByIRSensor();
    correctPositionByIRSensor(0);
    delay(300);
    N[i*2] = getHeading();
    delay(300);
  }
  rotateLeft4(-1);
}
void setCompass() {
  compass = HMC5883L();
  compass.SetMeasurementMode(Measurement_Continuous);
}
void PWM_Mode_Setup() {
  // pinMode(urTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  // digitalWrite(urTRIG,HIGH);                  // Set to HIGH
  // pinMode(urPWM0, INPUT);                      // Sending Enable PWM mode command
  // uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
  // for(int i=0;i<4;i++)
  //     Serial.write(EnPwmCmd[i]);
}

char getChar() {
  while (!Serial.available());
  return Serial.read();
}
void pcMode() {
  int cmd;
  while (1) {
  //   correctPosition(0);
  //   correctPositionByIRSensor(0);
  //   correctDirectionByIRSensor();
    cmd = getChar();
    // delay(300);
    if (cmd == 'G') {
      int len = 0;
      while (1) {
        char c = getChar();
        if (c == '|') break;
        int digit = c - '0';
        
        len = len * 10 + digit;
      }
      goAhead5(len);
    } else if (cmd == 'L') {
      rotateLeft5(1, one90Interruptspeed350);
    } else if (cmd == 'R') {
      rotateLeft5(-1, one90ToRightInterruptspeed350);
    } else if (cmd == 'C') {
      correctToGoal();
    }
  }
}
void pcModeAllByPulse() {
  int cmd;
  int len;
  while (1) {
  //   correctPosition(0);
  //   correctPositionByIRSensor(0);
  //   correctDirectionByIRSensor();
    // delay(300);
    cmd = getChar();
    len = 0;
    while (1) {
      char c = getChar();
      if (c == '|') break;
      int digit = c - '0';
      
      len = len * 10 + digit;
    }

    if (cmd == 'G') {
      goAhead5(len);
    } else if (cmd == 'L') {
      rotateLeft5(1, len);
    } else if (cmd == 'R') {
      rotateLeft5(-1, len);
    }
  }
}

int PWM_Mode_getDis() {
  digitalWrite(urTRIG, LOW);
  digitalWrite(urTRIG, HIGH);
  return pulseIn(urPWM, LOW) / 50;
}
int getHeading() {
  // MagnetometerScaled scaled = compass.ReadScaledAxis();
  // float heading = atan2(scaled.YAxis + 2, scaled.XAxis + 105);
  MagnetometerRaw raw = compass.ReadRawAxis();
  float heading = atan2(raw.YAxis + 27, raw.XAxis - 92);
  if (heading < 0)
    heading += 2 * PI;
  // Serial.println((int)(raw.XAxis + 109) + " " + (int)(raw.YAxis - 13));
  // Serial.print(raw.XAxis - 92);
  // Serial.print(" ");
  // Serial.println(raw.YAxis + 27);
  // Serial.println(heading * 180.0 / M_PI);
  // return heading * 180.0 / M_PI;
  return heading * 180.0 / PI; // change M_PI to PI <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
}

void rotateLeft4(int times) {
  neg =  - times / abs(times);
  fastMode = 0;
  enLeft = one90Interruptspeed200 * abs(times);
  if (times < 0)
    enLeft = one90ToRightInterruptspeed200 * abs(times);
  enRight = enLeft;
  leftCompensate = 0;

  Nnow = (Nnow - 2 * times + 16) % 8;

  setTimerInterrupt();
  attachInterrupt(1, countRight, RISING);
  md.init(); // =================================================???

  md.setSpeeds(200 * neg, -200 * neg);

  while (enLeft--) {
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }
  detachInterrupt(1);
  detachTimerInterrupt();

  md.setBrakes(400, 400);
  // brake();
}
void rotateLeft5(int toLeft, int numOfPulse) { // ==================================================== does not update the direction!!
  neg = - toLeft;
  fastMode = 1;

  enLeft = enRight = numOfPulse;
  // if (toLeft)
  //   enLeft = enRight = one45Interruptspeed200;
  // else
  //   enLeft = enRight = one45ToRightInterruptspeed200;

  leftCompensate = 0;

  setTimerInterrupt();
  attachInterrupt(1, countRight, RISING);
  md.init(); // =================================================???

  md.setSpeeds(350 * neg, -350 * neg);

  while (enLeft--) {
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }
  detachInterrupt(1);
  detachTimerInterrupt();

  // md.setBrakes(400, 400);
  brake();
}

void goAhead4(int grid) {
  neg = 1;
  fastMode = 0;
  enLeft = enRight = oneGridInterruptspeed200 * grid;
  leftCompensate = 0;

  setTimerInterrupt();
  attachInterrupt(1, countRight, RISING);

  md.init(); // =================================================???
  md.setSpeeds(200, 200);
  while (enLeft--) {
    while (digitalRead(enLeftPin)); // change to catch the posedge
    while (!digitalRead(enLeftPin)); // change to catch the posedge
  }

  detachInterrupt(1);
  detachTimerInterrupt();

  md.setBrakes(400, 400);
  // brake();

  if (Nnow == 0) curPos[0]++;
  if (Nnow == 2) curPos[1]++;
  if (Nnow == 4) curPos[0]--;
  if (Nnow == 6) curPos[1]--;
}
void goAhead5(int numOfPulse) {
  neg = 1;
  fastMode = 1;
  enLeft = enRight = numOfPulse;
  leftCompensate = 0;

  setTimerInterrupt();
  attachInterrupt(1, countRight, RISING);

  md.init(); // =================================================???
  md.setSpeeds(350, 350);
  while (enLeft--) {
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }

  detachInterrupt(1);
  detachTimerInterrupt();

  // md.setBrakes(400, 400);
  brake();
}
void brake() {
  for (int i = 0; i < 50; i++) { // ABS...-  -
    delay(1);
    md.setBrakes(285, 285);
    delay(1);
    md.setBrakes(400, 400);
  }
}

int smoothByMedian(int pin, int fast) {
  int a;
  if (fast == 1) {
    a = analogRead(pin);
    return (a + analogRead(pin)) / 2;
  }

  int data[5];
  int length = 5;
  int temp;
  for (int i = 0; i < length; i++) {
    delay(25);
    data[i] = analogRead(pin);
  }

  for (int i = length - 1; i > length / 2 - 1; i--) {
    for (int j = 0; j < i; j++) {
      if (data[j] < data[j+1]) {
        temp = data[j];
        data[j] = data[j+1];
        data[j+1] = temp;
      }
    }
  }

  return data[length/2];
}

void driftLeft(int driftToLeft) { 
  // leftCompensate = 0;
  neg = 1;

  if (driftToLeft == 1) {
    // enLeft = driftToLeftInterruptEnLeft;
    enRight = driftToLeftRatio * driftToLeftInterruptEnLeft;
  } else {
    enRight = driftToRightInterruptEnRight;
    // enLeft = driftRatio * driftToRightInterruptEnRight;
  }

  // setTimerInterrupt();
  attachInterrupt(1, countRightDrift, RISING);

  md.init();
  
  if (driftToLeft == 1)
    md.setSpeeds(driftSpeed, driftSpeed * driftToLeftRatio);
  else 
    md.setSpeeds(driftSpeed * driftToRightRatio, driftSpeed);

  // while (enLeft--) {
  //   while (digitalRead(enLeftPin));
  //   while (!digitalRead(enLeftPin));
  // }

  while (enRight > 0);

  detachInterrupt(1);
  // detachTimerInterrupt();
  md.setBrakes(400, 400);
}

void countRight() {
  enRight--;
  leftCompensate = enLeft - enRight;
}
void countRightDrift() {

  enRight--;
}

void setTimerInterrupt() {
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // set compare match register to desired timer count:
  OCR1A = 1513; // scale = 1024, so OCR1A = (xxx * 10^8 / 6.25 / 1024) // 3124
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}
void detachTimerInterrupt() {
  cli();
  TIMSK1 = 0;
  sei();
}
ISR(TIMER1_COMPA_vect) {
  if (fastMode == 0)
    md.setM1Speed((200 + leftCompensate) * neg); // need test <---------
  else
    md.setM1Speed((350 + leftCompensate) * neg);
}