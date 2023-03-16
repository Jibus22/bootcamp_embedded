#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define LED 1

int main() {
  DDRB |= (1 << LED); /* set port B, pin 1 to ouput */

  /* Set CTC mode (ds: 19-6). And set up timer with an internal clock prescale
   * (ds: 19-7)
   * I choosed a prescale to CS12 (256) because 16Mhz/256=62500, so 0.5 second
   * can be counted into a 16bit register without overflow. */
  TCCR1B |= (1 << WGM12) | (1 << CS12);

  /* Toggle OC1A on Compare Match (ds: 19-3). OC1A is on the pin PB1.
   * So, whenever TCNT1 becomes equal to OCR1A = 31249, toggle OC1A (PB1) */
  TCCR1A |= (1 << COM1A0);

  /* defining the TOP value (ds: 19.9.5).
   * Following the formula 'timercount=required_delay/clocktime_period' we get
   * this value so that the TOP value the timer/counter needs to reach spend
   * 500ms to the clock. The counter is cleared to ZERO when the counter value
   * (TCNT1) matches the OCR1A.*/
  OCR1A = 31249;

  while (1)
    ;
  return 0;
}
