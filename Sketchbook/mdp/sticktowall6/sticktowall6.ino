// complete sticktowall
// add timer interrupt

#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <avr/io.h>
#include <avr/interrupt.h>
// #include <PID_v1.h>

#define goalX 20
#define goalY 15

#define rd 562.0
#define oneGridspeed150 290 // ?
#define oneGridInterruptspeed200 290 // ?

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
#define middlePin 15
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

int N[8];
int Nnow;

// int compassX;
// int compassY;

int rec;

volatile int enLeft;
volatile int enRight;
volatile int leftCompensate;

void go() {
  leftEmpty = 0; // number of empty space on the left
  while (1) {
    correct();
    // correctPosition();
    // correct();
    // delay(300);

    disL = getDis21(leftHeadPin) - 9;
    if (disL > 10) {
      leftEmpty++;
    } else {
      leftEmpty = 0;
    }

    if (leftEmpty == 3) {
      leftEmpty = 0;
      // rotateLeft90speedX00(1);
      rotateLeft3(2);
      // delay(300);
      goAhead3(1);
      continue;
    } 

    disM = getDis21(middlePin) - 9;
    disFL = getDis21(leftFrontPin) - 9;
    disFR = PWM_Mode_getDis(); // left right sensor
    if (disFL < 10 || disFR < 10 || disM < 10) {
      // rotateLeft90speedX00(-1);
      rotateLeft3(-2);

      leftEmpty = 0;
      if (disFR > 10 && disM > 10)
        leftEmpty = 1;
      continue;
    }
    goAhead3(1);
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

  if (0 < temp && delta == min(delta, temp2)) {
    md.setSpeeds(60, -60);
  } else {
    md.setSpeeds(-60, 60);
  }

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
  int front = PWM_Mode_getDis() - 5;
  int temp = abs(front);
  if (temp > 5)
    return ;

  if (temp < 1)
    return ;

  if (front < 0) {
    md.setSpeeds(-60, -60);
  } else if (front > 5) {
    md.setSpeeds(60, 60);
  }

  while (temp > 1) {
    front = PWM_Mode_getDis() - 5;
    temp = abs(front);
    if (front < 0) {
      md.setSpeeds(-60, -60);
    } else if (front > 0) {
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

  // go();
  // md.setSpeeds(100, 100);
  // goAhead4(1);
}

void loop() {
  // goAhead4(1);
  // delay(1000);
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
void playMode() {
  rec = 0;
  while (1) {
    Serial.println("waiting...");
    while (!Serial.available());

    rec = Serial.read();
    if (rec == 'l')
      rotateLeft90speedX00(1);
    else if (rec == 'r')
      rotateLeft90speedX00(-1);
    else if (rec == 'f')
      goAhead3(1);
    else if (rec == 'b')
      rotateLeft90speedX00(2);
    else if (rec == '1')
      break;
    else
      Serial.println("what??");
  }
}

int PWM_Mode_getDis() {
  digitalWrite(urTRIG, LOW);
  digitalWrite(urTRIG, HIGH);
  return pulseIn(urPWM, LOW) / 50;
}
float getDis21(int pin) { // small

  return 12343.85 * pow(analogRead(pin),-1.15);
}
float getDis02(int pin) { // big

    return 30431 * pow(analogRead(pin), -1.169);
}
float getHeading() {
  // MagnetometerScaled scaled = compass.ReadScaledAxis();
  // float heading = atan2(scaled.YAxis + 2, scaled.XAxis + 105);
  MagnetometerRaw raw = compass.ReadRawAxis();
  float heading = atan2(raw.YAxis -13, raw.XAxis + 109);

  if (heading < 0)
    heading += 2 * PI;

  // Serial.println((int)(raw.XAxis + 105) + " " + (int)(raw.YAxis + 2));

  // Serial.print(raw.XAxis + 109);
  // Serial.print(" ");
  // Serial.println(raw.YAxis -13);
  // Serial.println(heading * 180.0 / M_PI);
  return heading * 180.0 / M_PI;
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
void goAhead3(float grid) {
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
  enLeft = oneGridInterruptspeed200 * abs(grid);
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
}

void countRight() {
  enRight--;
  leftCompensate = enLeft - enRight;
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
  md.setM1Speed(200 + leftCompensate);
}