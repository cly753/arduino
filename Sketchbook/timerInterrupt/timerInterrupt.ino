// timerInterrupt.ino
#include <avr/io.h>
#include <avr/interrupt.h>

#define LED 13

void setup() {

  pinMode(LED, OUTPUT);

  setTimerInterrupt();
}

void loop() {

}

void setTimerInterrupt() {
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // set compare match register to desired timer count:
  OCR1A = 3124; // scale = 1024, so OCR1A = (xxx * 10^8 / 6.25 / 1024)
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

ISR(TIMER1_COMPA_vect) {
    digitalWrite(LED, !digitalRead(LED));
}