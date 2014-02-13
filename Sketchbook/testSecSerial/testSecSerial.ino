#include <SoftwareSerial.h>

SoftwareSerial secSerial(18, 19); // RX, TX

int URPWM = 3;
int URTRIG=5;
unsigned int Distance=0;
uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};

void setup() {
  Serial.begin(9600);
  Serial.println("Goodnight moon!");
  secSerial.begin(9600);
  PWM_Mode_Setup();
}

void loop() {
  Serial.print("distance: ");
  Serial.print(PWM_Mode_getDis());
  Serial.println("cm");
}

int PWM_Mode_getDis() {
    digitalWrite(URTRIG, HIGH);
    return pulseIn(URPWM, LOW) / 50;
}

void PWM_Mode_Setup() {
  pinMode(URTRIG,OUTPUT);
  digitalWrite(URTRIG,HIGH);
  
  pinMode(URPWM, INPUT);
  
  for(int i=0;i<4;i++){
      secSerial.write(EnPwmCmd[i]);
  } 
}