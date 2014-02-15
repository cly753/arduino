// testStraight.ino
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;
const int rd = 562 * 5;

volatile int left;
volatile int right;

int enLeft = 3;
int enRight = 5;

void setup() {
    md.init();

    attachInterrupt(0, enLeft, FALLING);
}

void loop() {

}

