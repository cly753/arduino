#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

const int rd = 562;
const int one360 = 1650;
volatile int m1;
volatile int m2;
int enRight1 = 3;
int enRight2 = 5;

void test1() {
  int res1 = 3;
  int res2 = 5;
  int count;
  int count2;

  long pulse;

  int xxx = 0;
  int target = 562 * 5;
  int target2 = 562 * 5;

  int spe;

  pinMode(enRight1, INPUT);
  pinMode(enRight2, INPUT);

  Serial.println("start.\n");

  count = 0;
  count2 = 0;

  for (spe = 30; spe < 400; spe += 10) {
    md.setSpeeds(spe, spe);
    delay(200);

    pulse = pulseIn(3, HIGH);

    //Serial.print("speed: ");
    Serial.print(spe);
    Serial.print(".");
    Serial.print(pulse);
    // Serial.print(". pulse2: ");
    // Serial.print(pulse2);
    // Serial.print(". pulse3: ");
    // Serial.print(pulse3);
    Serial.println();
  }

  md.setBrakes(400, 400);

  // Serial.print("count: ");
  // Serial.println(count);
  Serial.println("\nstop.");

  delay(200);
  md.setBrakes(0, 0);
}

void test2(int spe) {
  md.setSpeeds(spe, spe);

  delay(1500);

  md.setBrakes(400, 400);
}
void test3(int spe) {
  md.setSpeeds(spe, spe);

  delay(900);

  md.setBrakes(400, 400);

  delay(300);
  md.setSpeeds(spe, spe);

  delay(500);

  md.setBrakes(400, 400);
}

void test4() {
  m1 = 0;

  attachInterrupt(3, add, FALLING);
  
  md.setSpeeds(100, 100);
  while (m1 != rd);
  md.setBrakes(400, 400);
}
void add() {
  Serial.println("add.");
  m1++;
}

void testTurn() {
    int need = one360 * 0;
    int need2 = one360;

    md.setSpeeds(400, -400);
    while (need) {
        while (digitalRead(enRight1));
        while (!digitalRead(enRight1));
        need--;
    }
    md.setSpeeds(100, -100);
    while (need2) {
        while (digitalRead(enRight1));
        while (!digitalRead(enRight1));
        need2--;
    }
    md.setBrakes(400, 400);
}

void testTurn2() {
  int need;
  for (int i = 1; i < 9; i++) {
    need = one360 / 4;

    md.setSpeeds(50, -50);
    while (need) {
        while (digitalRead(enRight1));
        while (!digitalRead(enRight1));
        need--;
    }
    md.setBrakes(400, 400);

    delay(1000);
  }
}

void testTurn3() {
  
  for (int i = 0; i < 4; i++) {
    int need = one360 / 4;
    int need2 = rd * 2;

    md.setSpeeds(100, 100);
    while (need2) {
        while (digitalRead(enRight1));
        while (!digitalRead(enRight1));
        need2--;
    }
    md.setBrakes(400, 400);
    delay(100);

    md.setSpeeds(100, -100);
    while (need) {
        while (digitalRead(enRight1));
        while (!digitalRead(enRight1));
        need--;
    }
    md.setBrakes(400, 400);
    delay(100);
  }
}

void setup() {
  Serial.begin(9600);
  md.init();

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);

  testTurn2();
}

void loop() {

}
