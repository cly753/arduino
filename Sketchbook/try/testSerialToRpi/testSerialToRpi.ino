int i;
int LED = 13;

void setup() {
  Serial.begin(9600);
  Serial.println("send every 500ms");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void loop() {
  
  for (i = 0; ; i++) {
    Serial.print("now is ");
    Serial.println(i);
    
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    delay(250);
  }
}
