#include <DualVNH5019MotorShield.h>
#include <TimerOne.h>

#define enLeftPin 11
#define enRightPin 3

DualVNH5019MotorShield md;

volatile int enLeft;
volatile int enRight;
volatile int leftCompensate;

void setup() {
  Serial.begin(9600);
  // setTimerInterrupt();
  
  
  // enLeft = 270;
  // enRight = 270;
  // leftCompensate = 0;

  // attachInterrupt(1, countRight, RISING);

  // md.setSpeeds(200, 200);
  // while (enLeft--) {
    // while (digitalRead(enLeftPin));
    // while (!digitalRead(enLeftPin));
  // }

  // detachInterrupt(1);
  // detachTimerInterrupt();

  // md.setBrakes(400, 400);
  // goAhead4(1);

  // setTimerInterrupt();
  md.init();
  Timer1.initialize(100000);
  Timer1.attachInterrupt( timerIsr );
  md.init();

  pinMode(enLeftPin, INPUT);
  pinMode(enRightPin, INPUT);

  enLeft = 300;
  enRight = 300;
  leftCompensate = 0;

  
  attachInterrupt(1, countRight, RISING);

  md.setSpeeds(200, 200);

  while (enLeft--) {
    while (digitalRead(enLeftPin));
    while (!digitalRead(enLeftPin));
  }
  detachInterrupt(1);
  detachTimerInterrupt();
  md.setBrakes(400, 400);
  
  // Serial.println("enLeft: ");
  // Serial.println(enLeft);
  // Serial.println("enRight: ");
  // Serial.println(enRight);
  // Serial.println("leftCompensate: ");
  // Serial.println(leftCompensate);
}

void loop() {
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
  OCR1A = 3124;
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

void timerIsr() {
  md.setM1Speed(200 + leftCompensate);
}