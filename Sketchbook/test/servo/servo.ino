#include <Servo.h> 
 
Servo myservo;
int pos = 0;
int servoPin = 13;
 
void setup()  { 
  myservo.attach(servoPin, 600, 2200);
} 

void loop()  { 
  for(pos = 0; pos < 180; pos += 1) {
    myservo.write(pos);
    delay(15);
  } 
  for(pos = 180; pos>=1; pos-=1) {                                
    myservo.write(pos);           
    delay(15);              
  } 
}

//// test writeMicroseconds
//void setup() { 
//  myservo.attach(7);
//  myservo.writeMicroseconds(2200);  // set servo to mid-point
//} 
//
//void loop() {} 
