// to do
// 0. test parameter for goAhead, rotation, brake
// -. global goAhead(use front & left sensor)/rotation PID ?
// 1. watchdog timer
// 2. disable compass
// 3. go diagonally

#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <avr/io.h>
#include <avr/interrupt.h>
// #include <PID_v1.h>

#define oneGridspeed150 290 // ?
#define oneGridInterruptspeed200 291 // ?
// #define oneGridInterruptspeed200 285 // software project lab

// #define one360speed150 1600
#define one90speed150 380
#define one90speed200 392
// #define one90speed200 380 // software project lab
#define one90speed300 390 // ? 
#define one90speed400 380

#define one90Interruptspeed200 397
#define one90ToRightInterruptspeed200 397

#define leftCompensate350 18 // ?
#define leftCompensate250 5
#define leftCompensate150 5

#define urPWM 13
#define urTRIG 5
#define leftHeadPin A3
#define leftFrontPin A2
#define rightFrontPin A1 
#define rearPin A0
#define enLeftPin 11
#define enRightPin 3

#define goalX 9
#define goalY 14

DualVNH5019MotorShield md;
HMC5883L compass;
// double input, output, target;
// PID pid(&input, &output, &target, 4, 1, 0, DIRECT);

int curPos[2]; // [0]: X, [1]: Y
int leftEmpty;

int N[8];
int Nnow;

int disFL;
int disFR;
int disFM;
int disL;
int disR;

int cmd;

volatile int enLeft;
volatile int enRight;
volatile int leftCompensate;
volatile int neg;
volatile int negRight;

void go(int stopConditon) {
  leftEmpty = 0; // number of empty space on the left
  while (1) {
    // correct();
    correctPosition();

    disL = smooth21(leftHeadPin) - 8;
    disFL = smooth21(leftFrontPin) - 10;
    disFM = -1;
    while (disFM == -1)
      disFM = PWM_Mode_getDis() - 1;
    disFR = smooth21(rightFrontPin) - 10;
    disR = smooth02(rearPin) - 16;

    Serial.println("================");
    Serial.println("SL" + String(disL, DEC));
    Serial.println("SFL" + String(disFL, DEC));
    Serial.println("SFM" + String(disFM, DEC));
    Serial.println("SFR" + String(disFR, DEC));
    // Serial.println("SR" + String(disR, DEC));

    if (disL > 10)
      leftEmpty++;
    else
      leftEmpty = 0;

    if (leftEmpty == 3) {
      leftEmpty = 0;
      rotateLeft4(1);
      correct();
      goAhead4(1);
      correct();
      
      Serial.println('L');
      Serial.println("G1");
      continue;
    }

    if (disFL < 10 || disFR < 10 || disFM < 10) {
      rotateLeft4(-1);
      correct();

      Serial.println('R');

      leftEmpty = 0;
      if (disFR > 10 && disFM > 10)
        leftEmpty = 1;
      continue;
    }
    goAhead4(1);
    correct();
    Serial.println("G1");

    if (stopConditon == 0) {
      if (curPos[0] > goalX - 2 && curPos[1] > goalY - 2)
        break;  
    } else {
      if (Serial.available() && Serial.read() == 'S')
        break;
    }
  }
}

void correct() {
  int des = N[Nnow];
  int now = getHeading();
  int delta;
  int temp;
  int temp2;
  int spe = 100;

  temp  = des - now;
  delta = abs(temp);
  temp2 = 360 - delta;

  if (getDir(now, des))
    md.setSpeeds(spe, -spe);
  else
    md.setSpeeds(-spe, spe);

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
void correct2() {
  int des = N[Nnow];
  int now = getHeading();
  int temp;
  int spe = 100;

  temp = des - now;
  if (temp < 0) temp += 360;

  if (temp < 180)
    des = (now + (int)(0.5 * temp)) % 360;
  else
    des = (now - (int)(0.5 * (360 - temp)) + 360) % 360;

  temp  = des - now;
  if (temp < 0) temp += 360;

  if (temp < 180) {
  } else {
    temp = 360 - temp;
  }

  while (temp > 1) {
    now = getHeading();
    temp  = des - now;
    if (temp < 0) temp += 360;
    if (temp < 180) {
      md.setSpeeds(spe, -spe);
    }
    else {
      temp = 360 - temp;
      md.setSpeeds(-spe, spe);
    }
  }

  md.setBrakes(400, 400);
}
bool getDir(int now, int des) {
  des -= now;
  if (des < 0) des += 360;
  return des < 180;
}
void correctPosition() {
  int front = -1;
  while (front == -1)
    front = PWM_Mode_getDis() - 1;

  if (front > 10)
    return ;

  // while (front != 5 && front != 6) {
  while (front != 5) {   
    if (front < 5)
      md.setSpeeds(-100, -100);
    // else if (front > 5)
    else
      md.setSpeeds(100, 100);
    front = PWM_Mode_getDis() - 1;
  }
  md.setBrakes(400, 400);
}
void correctToGoal() {
  for (int i = 0; i < 3; i++) {
    if (Nnow == 0) // current direction
      rotateLeft4(-1);
    else
      rotateLeft4(1);

    correct();

    disFL = smooth21(leftFrontPin) - 10;
    disFM = -1;
    while (disFM == -1)
      disFM = PWM_Mode_getDis() - 1;
    disFR = smooth21(rightFrontPin) - 10;

    while (disFL >  5 && disFM > 5 && disFR > 5)
      md.setSpeeds(150, 150);
    md.setBrakes(400, 400);
  }
  curPos[0] = goalX;
  curPos[1] = goalY;
}

void setup() {
  Serial.begin(9600);
  setPins();
  Wire.begin();
  PWM_Mode_Setup();
  md.init();
  setCompass();
  // setPID();

  // while (!Serial.available() || Serial.read() != 'S');
  // while (1) {
  //   if (Serial.available())
  //     if (Serial.read() == 'S')
  //       break;
  // }

  // storeDirectionByRotation();
  // correct();

  // curPos[0] = curPos[1] = 2;
  // go(0);

  // delay(1000);

  // curPos[0] = goalX - curPos[0] + 1;
  // curPos[1] = goalY - curPos[1] + 1;
  // go(0);

  // // while (!Serial.available() || Serial.read() != 'S');
  // while (1) {
  //   if (Serial.available())
  //     if (Serial.read() == 'S')
  //       break;
  // }

  // curPos[0] = goalX - curPos[0] + 1;
  // curPos[1] = goalY - curPos[1] + 1;
  // go(0);

  goAhead4(1);
}
void loop() {
  Serial.println("==========");
  disL = smooth21(leftHeadPin) - 8;
  disFL = smooth21(leftFrontPin) - 10;
  disFR = smooth21(rightFrontPin) - 10;
  // disR = smooth02(rearPin) - 16;
  disFM = -1;
  while (disFM == -1)
    disFM = PWM_Mode_getDis() - 1;

  Serial.println("SL" + String(disL, DEC));
  Serial.println("SFL" + String(disFL, DEC));
  Serial.println("SFM" + String(disFM, DEC));
  Serial.println("SFR" + String(disFR, DEC));
  // Serial.println("SR" + String(disR, DEC));

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

  delay(750);
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
void storeDirection() {
  delay(100);
  int now = getHeading();
  delay(100);
  now = (now + getHeading()) / 2;
  for (int i = 0; i < 8; i++)
    N[i] = (now + i * 45) % 360;
  Nnow = 0;
}
void storeDirectionByRotation() {
  Nnow = 0;
  N[0] = getHeading();
  delay(100);
  N[0] = (N[0] + getHeading()) / 2;
  for (int i = 1; i < 4; i++) {
    rotateLeft4(-1);
    delay(500);
    N[i*2] = getHeading();
    Serial.print("N");
    Serial.println(N[i*2]);
    delay(500);
  }
  rotateLeft4(-1);
}
void setCompass() {
  compass = HMC5883L();
  compass.SetMeasurementMode(Measurement_Continuous);
}
// void setPID() {
//   pid.SetMode(AUTOMATIC);
//   pid.SetOutputLimits(-100, 100);
//   pid.SetSampleTime(200); // ???
// }
void PWM_Mode_Setup() {
  pinMode(urTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(urTRIG,HIGH);                  // Set to HIGH
  pinMode(urPWM, INPUT);                      // Sending Enable PWM mode command
  uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
  for(int i=0;i<4;i++)
      Serial.write(EnPwmCmd[i]);
}
void padMode() {
  cmd = 0;
  while (1) {
    Serial.println("waiting...");
    while (!Serial.available());

    cmd = Serial.read();
    if (cmd == 'L')
      rotateLeft90speedX00(1);
    else if (cmd == 'R')
      rotateLeft90speedX00(-1);
    else if (cmd == 'G1')
      goAhead3(1);
    // else if (rec == 'B')
      // rotateLeft90speedX00(2);
    else if (cmd == 'S')
      break;
    else
      Serial.println("");
  }
}
void pcMode() {
  cmd = 0;
  while (1) {
    while (!Serial.available());
    cmd = Serial.read();

    if (cmd == 'G') {
      cmd = Serial.read();
      goAhead4(cmd - '0');
    } else if (cmd == 'L') {
      rotateLeft3(2);
      correct();
    } else if (cmd == 'R') {
      rotateLeft3(-2);
      correct();
    } else if (cmd == 'C') {
      correct();
    } else {

    }
  }
}

int PWM_Mode_getDis() {
  digitalWrite(urTRIG, LOW);
  digitalWrite(urTRIG, HIGH);
  return pulseIn(urPWM, LOW) / 50;
}
int getDis21(int pin) { // small

  return 12343.85 * pow(analogRead(pin),-1.15);
}
int getDis02(int pin) { // big

    return 30431 * pow(analogRead(pin), -1.169);
}
bool isEmpty21(int pin) {

  return analogRead(pin) < 267;
}
bool isEmpty02(int pin) {

  // return analogRead(pin) < ;
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

void rotateLeft90speedX00(int times) {
  int neg = 1;
  if (times < 0) neg = -1;
  int need = times * one90speed200 * neg;

  Nnow = ((Nnow - 2 * times + 8) % 8 + 8) % 8;

  md.setSpeeds(-200 * neg, 200 * neg);
  while (need--) {
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }
  md.setBrakes(400, 400);
}
void rotateLeft3(int quarter) {
  int neg = 1;
  if (quarter < 0) neg = -1;
  Nnow = (Nnow - quarter + 8) % 8;
  int des = N[Nnow];
  
  int now = getHeading();

  md.setSpeeds(-150 * neg, 150 * neg);

  while (des - now > 10 || des - now < -10)
    now = getHeading();
  md.setBrakes(400, 400);
}
void rotateLeft4(int times) {
  neg =  - times / abs(times);
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
void goAhead3(int grid) {
  int neg = 1;
  if (grid < 0) neg = -1;
  int need = grid * oneGridspeed150 * neg;

  int spe = 150 * neg;
  
  int a = need / 100;
  int b = need % 100; // need = a * 100 + b

  // target = N[Nnow];

  md.setSpeeds(spe + leftCompensate150 * neg, spe);

  for (int i = 0; i < a; i++) {
    need = 100;
    while (need--) {
      while (digitalRead(enLeftPin));
      while (!digitalRead(enLeftPin));
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
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }

  md.setBrakes(400, 400);
  
  // if (grid == 1) {
  //   if (Nnow == 0) curPos[0]++;
  //   if (Nnow == 2) curPos[1]++;
  //   if (Nnow == 4) curPos[0]--;
  //   if (Nnow == 6) curPos[1]--;
  // }
}
void goAhead4(int grid) {
  neg = 1;
  negRight = 1;
  enLeft = enRight = oneGridInterruptspeed200 * grid;
  leftCompensate = 0;

  int pre = 0;
  int cur = 0;
 
  setTimerInterrupt();
  attachInterrupt(1, countRight, RISING);

  md.init(); // =================================================???
  md.setSpeeds(200, 200);
  while (enLeft--) {
    while (digitalRead(enLeftPin)); // change to catch the posedge

    // if (enLeft % 100)
    //   md.setM1Speed((200 + leftCompensate) * neg);

    while (!digitalRead(enLeftPin)); // change to catch the posedge

    // while (!(!pre & cur)) {
    //   pre = digitalRead(enLeftPin);
    //   delay(1);
    //   cur = digitalRead(enLeftPin);
    // }
    // pre = 1;
  }

  detachInterrupt(1);
  detachTimerInterrupt();

  md.setBrakes(400, 400);
  // brake();

  Serial.println("=============");
  Serial.println("enLeft : " + String(enLeft));
  Serial.println("enRight: " + String(enRight));
  Serial.println("leftCompensate: " + String(leftCompensate));

  if (Nnow == 0) curPos[0]++;
  if (Nnow == 2) curPos[1]++;
  if (Nnow == 4) curPos[0]--;
  if (Nnow == 6) curPos[1]--;
}
void brake() {
  // for (int i = 0; i <= 400; i += 20){
  //   delay(2);
  //   md.setBrakes(i, i);
  // }
  for (int i = 0; i < 50; i++) { // ABS...-  -
    delay(1);
    md.setBrakes(275, 275);
    delay(1);
    md.setBrakes(400, 400);
  }
}

void countRight() {
  enRight--;
  leftCompensate = enLeft - enRight;
}
void countRightDrift() {
  enRight--;
  leftCompensate = 1.9 * enLeft - enRight;
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
  md.setM1Speed((200 + leftCompensate) * neg); // need test <---------
  // md.setSpeeds(200, 200);
}

int smooth21(int pin) {
  int small = 1000;
  int big = -10;
  int sum = 0;
  int temp = 0;

  for (int i = 0; i < 5; i++) {
    temp = getDis21(pin);
    small = min(small, temp);
    big = max(big, temp);
    sum += temp;
    // delay(50);
  }
  sum = sum - small - big;
  return sum / 3;
}
int smooth02(int pin) {
  int small = 1000;
  int big = -10;
  int sum = 0;
  int temp = 0;

  for (int i = 0; i < 5; i++) {
    temp = getDis02(pin);
    small = min(small, temp);
    big = max(big, temp);
    sum += temp;
    // delay(50);
  }
  sum = sum - small - big;
  return sum / 3;
}

void driftLeft() {
  neg = 1;
  enLeft = (2 * PI) * (2 * oneGridInterruptspeed200) * 0.25 - 35; // degree to rotate
  enRight = 1.9 * enLeft;
  leftCompensate = 0;

  setTimerInterrupt();
  attachInterrupt(1, countRightDrift, RISING);

  md.init();
  md.setSpeeds(200, 200 * 1.9); // n ~ radius of the circle // remember to change the ratio in "countRightDrift()"

  while (enLeft--) {
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }
  detachInterrupt(1);
  detachTimerInterrupt();
  md.setBrakes(400, 400);
}