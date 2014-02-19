#include <Wire.h>
#include <HMC5883L.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>
#include <Servo.h>

Servo myservo;
int servoPin = 13;

const int urPWM = 3;
const int urTRIG = 5;

void servoSensor() {
    int front;
    myservo.write(60);
    delay(100);
    front = PWM_Mode_getDis();
    Serial.print("60`: ");
    Serial.println(front);

    myservo.write(120);
    delay(100);
    front = PWM_Mode_getDis();
    Serial.print("60`: ");
    Serial.println(front);
}

void setup() {
    PWM_Mode_Setup();
    myservo.attach(13, 600, 2200);

    servoSensor();
}

void loop() {}

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