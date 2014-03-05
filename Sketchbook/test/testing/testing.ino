#include <Wire.h>
#include <HMC5883L.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>
#include <Servo.h>

Servo sv;
DualVNH5019MotorShield md;

const int urPWM = 3;
const int urTRIG = 5;

void servoSensor() {
    int front;
    sv.write(60);
    delay(150);
    front = PWM_Mode_getDis();
    Serial.print("\n60`: ");
    Serial.println(front);

    sv.write(120);
    delay(150);
    front = PWM_Mode_getDis();
    Serial.print("120`: ");
    Serial.println(front);
}

void setup() {
  Serial.begin(9600);
  PWM_Mode_Setup();
  md.init();
  sv.attach(13, 600, 2200);
  
  Serial.println("start...");
  
  md.setSpeeds(200, 200);
}

void loop() {
  servoSensor();
  delay(2000);
}

void PWM_Mode_Setup() {
  pinMode(urTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(urTRIG,HIGH);                  // Set to HIGH
  pinMode(urPWM, INPUT);                      // Sending Enable PWM mode command
  uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
  for(int i=0;i<4;i++)
    Serial.write(EnPwmCmd[i]);
}
int PWM_Mode_getDis() { // a low pull on pin COMP/TRIG  triggering a sensor reading
  digitalWrite(urTRIG, LOW);
  digitalWrite(urTRIG, HIGH);               // reading Pin PWM will output pulses
  return pulseIn(urPWM, LOW) / 50;
}
