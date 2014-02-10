#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield md;

int URPWM = 3; // PWM Output 0－25000US，Every 50US represent 1cm
int URTRIG=5; // PWM trigger pin
int front;
int right;

float getDis02() {
    Serial.print("ir: ");
    Serial.println(30431 * pow(analogRead(A1), -1.169));
    return 30431 * pow(analogRead(A1), -1.169);
}

void PWM_Mode_Setup() {
  pinMode(URTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG,HIGH);                  // Set to HIGH
  
  pinMode(URPWM, INPUT);                      // Sending Enable PWM mode command
  
  uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
  for(int i=0;i<4;i++)
      Serial.write(EnPwmCmd[i]);
}

int PWM_Mode_getDis() {                              // a low pull on pin COMP/TRIG  triggering a sensor reading
    digitalWrite(URTRIG, LOW);
    digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses
    
    return pulseIn(URPWM, LOW) / 50;
}

void turnLeft() {
  md.setSpeeds(200, -200);

  delay(850);

  md.setBrakes(400, 400);
}

void turnRight() {
  md.setSpeeds(-200, 200);

  delay(850);

  md.setBrakes(400, 400);

  delay(1000);
}

void goOneGrid() {
  md.setSpeeds(200, 200);

  delay(1000);

  md.setBrakes(400, 400);
}

void go() {
  Serial.println("11111");
  while (1) {
    Serial.println("2222");

    front = (int)PWM_Mode_getDis();
    Serial.print("front............................... ");
    Serial.println(front);

    right = getDis02();
    Serial.print("right.............................. ");
    Serial.println(right);


    if (right > 25) {
      turnRight();
    } else if (front > 10) {
      goOneGrid();
    } else {
      turnLeft();
    }
    
  }
}

void setup() {
    Serial.begin(9600);
    PWM_Mode_Setup();

    md.init();

    go();
}

void loop() {

}

