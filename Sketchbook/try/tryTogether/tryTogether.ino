#include "DualVNH5019MotorShield.h"
#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;
DualVNH5019MotorShield motor;

int USalert = 3; // get low (near) signal from ultrasomic sensor through pin 3
int alert;

void setCompass() {
  int error = 0;
  compass = HMC5883L();
  error = compass.SetScale(1.3);
  if (error)
    Serial.println("compass error");
  error = compass.SetMeasurementMode(Measurement_Continuous);
  if (error)
    Serial.println("compass error");
}

void setUSsensor() {
  // auto mode, threshold: 5cm
  int i;

  pinMode(USalert, INPUT);
  uint8_t cmmdLow[4]={0x44,0x00,0x08,0x4c}; //lower 8 bit of the threshold stored in address 0x00
  uint8_t cmmdHigh[4]={0x44,0x01,0x00,0x45};  //higher 8 bit of the threshold stored in the address 0x01
  uint8_t cmmdAuto[4]={0x44,0x02,0xaa,0xf0};  // Autonomous mode. write 0xaa into address 0x02
 
  for (i = 0; i < 4; i++)
    Serial.write(cmmdLow[i]);

  delay(200);
  for (i = 0; i < 4; i++)
    Serial.write(cmmdHigh[i]);

  delay(200);
  for (i = 0; i < 4; i++)
    Serial.write(cmmdAuto[i]);
}

float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  heading -= 2 * PI;
  if (heading < 0)
    heading += 4 * PI;

  Serial.print("heading: ");
  Serial.println(heading * 180 / PI);
  return heading * 180 / PI;
  //return heading * 180 / M_PI;
}
void turn90() {
  float initialDir = getHeading();

  motor.setM1Speed(0);
  motor.setM2Speed(0);
  delay(200);
  motor.setM1Speed(100);
  motor.setM2Speed(-100);
  
  while (getHeading() - initialDir < 90 || getHeading() - initialDir > -90)
    ;

  motor.setM1Speed(0);
  motor.setM2Speed(0);
}

void goForward() {
  motor.setM1Speed(150);
  motor.setM2Speed(150);
}

void goStopTurn() {
  motor.setM1Speed(0);
  motor.setM2Speed(0);
  delay(200);

  motor.setM1Speed(-150);
  motor.setM2Speed(150);

  delay(500);

  motor.setM1Speed(0);
  motor.setM2Speed(0);
  delay(200);
}

void setup() {
  Serial.begin(9600);
//  Wire.begin();

  Serial.println("this is a simple go and turn");

  motor.init();
  //setCompass();
  setUSsensor();

  //attachInterrupt(USalert, turn90, FALLING);
}

void loop() {
  alert = digitalRead(USalert);
  if (!alert) {
    Serial.println("a obstacle!");
    //turn90();

    goStopTurn();
  } else {
    goForward();
  }
}
