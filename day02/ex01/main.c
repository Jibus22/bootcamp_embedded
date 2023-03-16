/* Here, timer1 is a PWM timer at 1kHz which controls OC1A pin: Clear OC1A on
 * Compare Match, set OC1A at BOTTOM. Its OCR1A register which determines
 * its duty cycle is updated from 0 to 1599 (0 to 100%) by the interrupt
 * TIMER0_COMPA.
 * Timer0 is a CTC timer at 1kHz which triggers a COMP_A interrupt
 * every 1ms. The ISR is in charge of updating OCR1A. The goal is
 * that OCR1A (so the duty cycle) increases during 500ms, then decrease 
 * for the last 500ms, and again. */

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define LED2 1 // PB1

volatile uint8_t direction;

ISR(TIMER0_COMPA_vect ){
  if (direction) {
    OCR1A += 31; /* 15999 / 500 */
    if (OCR1A >= 15500) direction = 0;
  } else {
    OCR1A -= 31;
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


  /* (ds: 18-9) set CTC mode to timer0 */
  TCCR0A |= (1 << WGM01);
  /* (ds: table 18-10) set prescaler to 64. 16Mhz/64=250KHz. So it takes 250
   * cycles to wait 1ms. */
  TCCR0B |= (1 << CS00) | (1 << CS01);

  /* Output Compare Match to 250, takes 1ms to the clock 0 to reach it before
   * reseting TCNT0 to 0 and triggering our ISR */
  OCR0A = 250;

  /* enable Output Compare A Match Interrupt (ds:18.9.3) */
  TIMSK0 |= (1 << OCIE0A);

  sei();

  direction = 1;

  while (1) {}
  return 0;
}
