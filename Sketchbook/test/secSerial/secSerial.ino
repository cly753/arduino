#include <SoftwareSerial.h>

SoftwareSerial secSerial(18, 19); // RX, TX

int urPWM = 3;
int urTRIG=5;
unsigned int Distance=0;

void setup() {
  Serial.begin(9600);
  Serial.println("Goodnight moon!");
  secSerial.begin(9600);
  secSerial.println("Goodmorning sun!");
  PWM_Mode_Setup();
}

void loop() {
  Serial.print("distance: ");
  Serial.print(PWM_Mode_getDis());
  Serial.println("cm");
}

int PWM_Mode_getDis() {
  digitalWrite(urTRIG, LOW);
  digitalWrite(urTRIG, HIGH);              
  return pulseIn(urPWM, LOW) / 50;
}

void PWM_Mode_Setup() {
  pinMode(urTRIG,OUTPUT);
  digitalWrite(urTRIG,HIGH);
  pinMode(urPWM, INPUT);
  uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
  for(int i=0;i<4;i++){
      secSerial.write(EnPwmCmd[i]);
  } 
}