#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

void test() {
  md.setSpeeds(300, 300);
}

void test2() {
  int res1 = 3;
  int res2 = 5;
  int count;
  int xxx = 0;
  int target = 551 * 4;

  pinMode(res1, INPUT);
  pinMode(res2, INPUT);

  Serial.println("start.\n");
  md.setSpeeds(400, 400);
  count = 0;
  Serial.print("count: ");
  Serial.println(count);

  while(1) {
    while(digitalRead(res1));
    while(!digitalRead(res1));
    count++;
    // Serial.print("count: ");
    // Serial.println(count);
    if(count == target) {
      md.setBrakes(400, 400);
      break;
    }
  }

  // while (1) {
  //   if (digitalRead(res1)) {
  //     //Serial.println(millis());
  //     Serial.println("||||");
  //   } else {
  //     Serial.println();
  //   }
  //   // Serial.print("\nanalogRead1: ");
  //   // Serial.println(digitalRead(res1));
  //   //Serial.print("analogRead2: ");
  //   //Serial.println(digitalRead(res2));
  //   delay(50);
  // }
  
  // for (count = 0; count < 12; count++) {
  //   Serial.println(count);

  //   while (xxx) {
  //     xxx = digitalRead(res1);
  //     // Serial.print("high: ");
  //     // Serial.println(xxx);
  //     Serial.println("====");
  //     delay(100);
  //   }

  //   while (!xxx) {
  //     xxx = digitalRead(res1);
  //     // Serial.print("low: ");
  //     // Serial.println(xxx);
  //     Serial.println();
  //     delay(100);
  //   }
  // }
  delay(200);
  md.setBrakes(0, 0);
  Serial.println("stop.\n\n");
}

void test3(int spe) {
  md.setSpeeds(spe, spe);

  delay(1500);

  md.setBrakes(400, 400);
}

void test4(int spe) {
  md.setSpeeds(spe, spe);

  delay(900);

  md.setBrakes(400, 400);

  delay(300);
  md.setSpeeds(spe, spe);

  delay(500);

  md.setBrakes(400, 400);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  Serial.println("Initialized...");

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);

  test2();
}

void loop() {

}
