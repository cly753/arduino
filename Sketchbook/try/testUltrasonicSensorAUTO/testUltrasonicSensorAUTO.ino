// # Editor    :Holiday from DFRobot
// # Data      :30.05.2013
 
// # Product name:ultrasonic scanner 
// # Product SKU:SEN0001
// # Version :  3.2
 
// # Description:
// # This sample shows how to use the Autonomous trigger mode by writing its EEPROM 
 
// # Connection:
// #       Pin 1 VCC (URM V3.2) -> VCC (Arduino)
// #       Pin 2 GND (URM V3.2) -> GND (Arduino)
// #       Pin 6 COMP/TRIG (URM V3.2) -> Pin 2 (Arduino)
// #
 
uint8_t cmmd1[4]={0x44,0x00,0x0A,0x4E};//lower 8 bit of the threshold stored in address 0x00
uint8_t cmmd2[4]={0x44,0x01,0x00,0x45};//higher 8 bit of the threshold stored in the address 0x01
uint8_t cmmd3[4]={0x44,0x02,0xaa,0xf0};// Autonomous mode. write 0xaa into address 0x02
//int cmmd3[4]={0x44,0x02,0xbb,0x01};  // PWM mode. write 0xbb into address 0x02

int i;
int LED = 13;
int TRIGGER = 3;
//volatile int state = LOW;

void setup(){ 
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(TRIGGER, INPUT);
  Serial.begin(9600);
  A_Mode_Setup();
  //attachInterrupt(0, user_diy, CHANGE);  //The ON/OFF switch can be used as a singal of interruption
}

void loop() {
  int res = digitalRead(TRIGGER);
  if (!res) {
    Serial.print("result: ");
    Serial.print(TRIGGER);
    Serial.println("near!");

    for (i=0; i<5; i++) {
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);
    }
  } else {
    for (i=0; i<5; i++) {
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
      delay(500);
    }
  }
}                      
 
void A_Mode_Setup() { 
  //write the data into the URM37 EEPROM
  for (i=0;i<4;i++)
    Serial.write(cmmd1[i]);
  delay(200);
 
  for (i=0;i<4;i++)
    Serial.write(cmmd2[i]);
  delay(200);
  
  for (i=0;i<4;i++)
    Serial.write(cmmd3[i]);                             
  delay(200);  
  
  for (i=0; i<4; i++) {
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    delay(250);
  }
}
