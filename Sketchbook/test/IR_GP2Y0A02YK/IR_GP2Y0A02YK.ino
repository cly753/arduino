// got a lot of library
int irPin = 0;
int irVal;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  irVal = analogRead(irPin);
  Serial.print("adc value: ");
  Serial.println(irVal);
  Serial.print(30431 * pow(irVal, -1.169));
  Serial.println("cm\n");
  
  digitalWrite(13, HIGH);
  delay(irVal);
  digitalWrite(13, LOW);
  delay(irVal);
}
