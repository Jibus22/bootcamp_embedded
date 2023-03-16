#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define LED2 1 // PB1

int main() {
  DDRB |= (1 << LED2); /* set port B, pin 1 to ouput */

  /* (ds: 19-4) Clear OC1A on Compare Match, set OC1A at BOTTOM
   * (non-inverting mode)*/
  TCCR1A |= (1 << COM1A1);

  /* (ds: 19-6) set Fast PWM mode 14 so that we can define TOP in ICR1. */
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);


  /* Set clock prescale to 1024 so Fclock=16Mhz/1024=15625Hz*/
  TCCR1B |= (1 << CS12) | (1 << CS10);

  /* Input Capture Register is set as a TOP the TCNT1 counter must reach before
   * going back to the BOTTOM. 15624 takes 1sec to reach with a 15kHz clock */
  ICR1 = 15624;

  /* Output Compare Register, in non-inverting mode clears OC1A on compare match
   * and set OC1A at BOTTOM. Compare match happens when TCNT1==OCR1A. Here we
   * set the value to 10% of TOP = 10% of 15624 = 1562. That is, 10% of a second
   * so 10ms. The led is bright 10ms per second */
  OCR1A = 1562;

  while (1) {}
  return 0;
}
