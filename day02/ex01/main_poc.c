/* This is a testing program to firstly make the concept of changing compare
 * value OCR1A to modulate dirty cycle works. This is a POC before doing
 * the same (OCR1A variation) with an second timer */

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define LED2 1 // PB1

volatile uint8_t direction;

ISR(TIMER1_OVF_vect) {
  if (direction) {
    OCR1A += 1;
    if (OCR1A == 500) direction = 0;
  } else {
    OCR1A -= 1;
    if (OCR1A == 0) direction = 1;
  }
}

int main() {
  DDRB |= (1 << LED2); /* set port B, pin 1 to ouput */

  /* (ds: 19-4) Clear OC1A on Compare Match, set OC1A at BOTTOM
   * (non-inverting mode)*/
  TCCR1A |= (1 << COM1A1);

  /* (ds: 19-6) set Fast PWM mode 14 so that we can define TOP in ICR1. */
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  /* Set clock on */
  TCCR1B |= (1 << CS10);

  /* Set TOP to 15999. At FCPU=16Mhz, counting up to 16000 takes 1ms */
  ICR1 = 15999;

  /* init OCR1A */
  OCR1A = 0;

  TIMSK1 |= (1 << TOIE1); /* enable overflow interrupt */

  sei();

  direction = 1;

  while (1) {}
  return 0;
}
