#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

void stopIfFault() {
  if (md.getM1Fault()) {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault()) {
    Serial.println("M2 fault");
    while(1);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  Serial.println("Initialized...");

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}

void loop() {
  md.setM1Speed(150);
  md.setM2Speed(150);

  while (1) {
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
  }
}

// void loop() {
//   for (int i = 0; i <= 400; i++) {
//     md.setM1Speed(i);
//     stopIfFault();
//     if (i%200 == 100) {
//       Serial.print("M1 current: ");
//       Serial.println(md.getM1CurrentMilliamps());
//     }
//     delay(2);
//   }
  
//   for (int i = 400; i >= -400; i--) {
//     md.setM1Speed(i);
//     stopIfFault();
//     if (i%200 == 100) {
//       Serial.print("M1 current: ");
//       Serial.println(md.getM1CurrentMilliamps());
//     }
//     delay(2);
//   }
  
//   for (int i = -400; i <= 0; i++) {
//     md.setM1Speed(i);
//     stopIfFault();
//     if (i%200 == 100) {
//       Serial.print("M1 current: ");
//       Serial.println(md.getM1CurrentMilliamps());
//     }
//     delay(2);
//   }

//   for (int i = 0; i <= 400; i++) {
//     md.setM2Speed(i);
//     stopIfFault();
//     if (i%200 == 100) {
//       Serial.print("M2 current: ");
//       Serial.println(md.getM2CurrentMilliamps());
//     }
//     delay(2);
//   }
  
//   for (int i = 400; i >= -400; i--) {
//     md.setM2Speed(i);
//     stopIfFault();
//     if (i%200 == 100) {
//       Serial.print("M2 current: ");
//       Serial.println(md.getM2CurrentMilliamps());
//     }
//     delay(2);
//   }
  
//   for (int i = -400; i <= 0; i++) {
//     md.setM2Speed(i);
//     stopIfFault();
//     if (i%200 == 100)
//     {
//       Serial.print("M2 current: ");
//       Serial.println(md.getM2CurrentMilliamps());
//     }
//     delay(2);
//   }
// }
