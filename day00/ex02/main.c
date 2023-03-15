#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define LED_PIN 0

int main() {
  unsigned long busywait_cntr;
  /* 1st we set the pinB0 direction to output */
  DDRB |= (1 << LED_PIN);
  while (1) {
    PORTB ^= (1 << LED_PIN); /* toggles pinB0 off/on (low/high) */

    /* nb of cpu cycle to achieve, to wait 500ms */
    busywait_cntr = F_CPU / (6 * (1000 / 500));
    while (busywait_cntr-- > 0)
      ; /* busy wait */
  }
}
