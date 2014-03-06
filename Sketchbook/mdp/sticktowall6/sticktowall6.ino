// complete sticktowall
// add timer interrupt

#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <avr/io.h>
#include <avr/interrupt.h>
// #include <PID_v1.h>

#define goalX 19
#define goalY 14

#define rd 562.0
#define oneGridspeed150 290 // ?
#define oneGridInterruptspeed200 290 // ?
#define oneGridInterruptspeed200inHall 294 // ?

#define one360speed150 1600
#define one90speed150 395
#define one90speed200 387
#define one90speed300 390 // ? 
#define one90speed400 380
#define leftCompensate350 18 // ?
#define leftCompensate250 5
#define leftCompensate150 5

#define urPWM 13
#define urTRIG 5
#define leftHeadPin 17
#define leftFrontPin 16
#define rightFrontPin 15
#define rearPin 14
#define enLeftPin 11
#define enRightPin 3

DualVNH5019MotorShield md;
HMC5883L compass;
// double input, output, target;
// PID pid(&input, &output, &target, 4, 1, 0, DIRECT);
int distanceNow;

int curPos[2] = {2, 2};
int leftEmpty;

int disFL;
int disFR;
int disL;
int disM;
int disR;

int N[8];
int Nnow;

int cmd;

volatile int enLeft; // for interrupt
volatile int enRight;
volatile int leftCompensate;

void go() {
  leftEmpty = 0; // number of empty space on the left
  while (1) {
    correct();
    correctPosition();
    correct();

    // for (int i = 0; i < 2; i++) {
    //   disL = getDis21(leftHeadPin) - 8;
    //   disFL = getDis21(leftFrontPin) - 10;
    //   disFR = getDis21(rightFrontPin) - 10; // left right sensor
    //   disR = getDis02(rearPin) - 16;
    // }
    disL = smooth21(leftHeadPin) - 8;
    disFL = smooth21(leftFrontPin) - 10;
    disFR = smooth21(rightFrontPin) - 10;
    disR = smooth02(rearPin) - 16;
    disM = -1;
    while (disM == -1)
      disM = PWM_Mode_getDis() - 1;

    Serial.println("SL" + String(disL, DEC));
    Serial.println("SFL" + String(disFL, DEC));
    Serial.println("SFM" + String(disM, DEC));
    Serial.println("SFR" + String(disFR, DEC));
    Serial.println("SR" + String(disR, DEC));

    delay(200);
    if (disL > 10)
      leftEmpty++;
    else
      leftEmpty = 0;

    if (leftEmpty == 3) {
      leftEmpty = 0;
      // rotateLeft90speedX00(1);
      rotateLeft3(2);
      correct();
      goAhead4(1);
      Serial.println('L');
      Serial.println("G1");
      continue;
    }

    if (disFL < 10 || disFR < 10 || disM < 10) {
      // rotateLeft90speedX00(-1);
      rotateLeft3(-2);
      Serial.println('R');

      leftEmpty = 0;
      if (disFR > 10 && disM > 10)
        leftEmpty = 1;
      continue;
    }
    goAhead4(1);
    Serial.println("G1");
    if (curPos[0] == goalX && curPos[1] == goalY)
      break;
  }
}

void correct() {
  int des = N[Nnow];
  int now = getHeading();
  int delta;
  int temp;
  int temp2;

  temp  = des - now;
  delta = abs(temp);
  temp2 = 360 - delta;

  if (0 < temp && delta == min(delta, temp2))
    md.setSpeeds(60, -60);
  else
    md.setSpeeds(-60, 60);

  while (min(delta, temp2) > 2) {
    now = getHeading();
    temp  = des - now;
    delta = abs(temp);
    temp2 = 360 - delta;

    if (0 < temp && delta == min(delta, temp2)) 
      md.setSpeeds(60, -60);
    else 
      md.setSpeeds(-60, 60);
  }
  md.setBrakes(400, 400);
}
void correctPosition() {
  int front = PWM_Mode_getDis() - 1;

  if (front > 10)
    return ;

  if (front < 5) {
    md.setSpeeds(-60, -60);
  } else if (front > 5) {
    md.setSpeeds(60, 60);
  }

  while (front != 5) {
    front = PWM_Mode_getDis() - 1;
    if (front < 5) {
      md.setSpeeds(-60, -60);
    } else if (front > 5) {
      md.setSpeeds(60, 60);
    }
  }
  md.setBrakes(400, 400);
}

void setup() {
  delay(1000);
  Serial.begin(9600);
  Wire.begin();
  PWM_Mode_Setup();
  md.init();
  setCompass();
  // setPID();
  // storeDirectionByRotation();

  pinMode(enLeftPin, INPUT);
  pinMode(enRightPin, INPUT);

  // Serial.println("xxxx");

  // go();
  // goAhead4(5);
  driftLeft();
}

void loop() {
  // goAhead4(1);
  // delay(1000);
  // Serial.println(smooth21(leftFrontPin) - 10);
  // Serial.println(PWM_Mode_getDis() - 1);
  // delay(250);
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
  for (int i = 1; i < 4; i++) {
    rotateLeft90speedX00(-1);
    delay(500);
    N[i*2] = getHeading();
    delay(500);
  }
  rotateLeft90speedX00(-1);
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
// void playMode() {
//   cmd = 0;
//   while (1) {
//     Serial.println("waiting...");
//     while (!Serial.available());

//     cmd = Serial.read();
//     if (cmd == 'L')
//       rotateLeft90speedX00(1);
//     else if (cmd == 'R')
//       rotateLeft90speedX00(-1);
//     else if (cmd == 'G1')
//       goAhead3(1);
//     // else if (rec == 'B')
//       // rotateLeft90speedX00(2);
//     else if (cmd == 'S')
//       break;
//     else
//       Serial.println("");
//   }
// }
// void pcMode() {
//   cmd = 0;
//   while (1) {
//     while (!Serial.available());
//     cmd = Serial.read();

//     if (cmd == 'G') {
//       cmd = Serial.read();
//       goAhead4(cmd - '0');
//     } else if (cmd == 'L') {
//       rotateLeft3(2);
//     } else if (cmd == 'R') {
//       rotateLeft3(-2);
//     } else if (cmd == 'C') {
//       correct();
//     } else {

//     }
//   }
// }

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

void rotateLeft360(int times) {
  int neg = 1;
  if (times < 0) neg = -1;
  int need = times * one360speed150 * neg;

  md.setSpeeds(-150 * neg, 150 * neg);
  while (need--) {
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }

  md.setSpeeds(100 * neg, -100 * neg); // brake compensate // try
  delay(50);
  md.setBrakes(400, 400);
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
  enLeft = oneGridInterruptspeed200inHall * abs(grid);
  enRight = enLeft;
  leftCompensate = 0;

  setTimerInterrupt();
  attachInterrupt(1, countRight, RISING);
  md.init(); // =================================================???

  if (grid > 0)
    md.setSpeeds(200, 200);
  else
    md.setSpeeds(-200, -200);

  while (enLeft--) {
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }
  detachInterrupt(1);
  detachTimerInterrupt();
  md.setBrakes(400, 400);

  if (Nnow == 0) curPos[0]++;
  if (Nnow == 2) curPos[1]++;
  if (Nnow == 4) curPos[0]--;
  if (Nnow == 6) curPos[1]--;
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
  OCR1A = 3124; // scale = 1024, so OCR1A = (xxx * 10^8 / 6.25 / 1024)
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
  // md.setSpeeds(200 + 3 * leftCompensate, 200);
  md.setM1Speed(200 + 50 * leftCompensate);
}

int smooth21(int pin) {
  int small = 1000;
  int big = -1;
  int sum = 0;
  int temp = 0;

  for (int i = 0; i < 5; i++) {
    temp = getDis21(pin);
    small = min(small, temp);
    big = max(big, temp);
    sum += temp;
  }
  sum = sum - small - big;
  return sum / 3;
}

int smooth02(int pin) {
  int small = 1000;
  int big = -1;
  int sum = 0;
  int temp = 0;

  for (int i = 0; i < 5; i++) {
    temp = getDis02(pin);
    small = min(small, temp);
    big = max(big, temp);
    sum += temp;
  }
  sum = sum - small - big;
  return sum / 3;
}

void driftLeft() {
  enLeft = 0.5 * PI * 2 * oneGridInterruptspeed200inHall - 35;
  enRight = 1.9 * enLeft;
  leftCompensate = 0;

  setTimerInterrupt();
  attachInterrupt(1, countRightDrift, RISING);

  md.init();
  md.setSpeeds(200, 200 * 1.9);

  while (enLeft--) {
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }
  detachInterrupt(1);
  detachTimerInterrupt();
  md.setBrakes(400, 400);
}