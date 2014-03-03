int pin = 13;
volatile int state = LOW;

void setup() {
  Serial.begin(9600);
  Serial.println("begin...");
  pinMode(pin, OUTPUT);
  pinMode(3, INPUT);
  attachInterrupt(0, blink, CHANGE);
}

void loop() {
  digitalWrite(pin, state);

}

void blink() {
  state = !state;
  // Serial.println("interrupt");
}