#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define LED_PIN 0 /* led connected to port B pin 0 */
#define BUTTON 2  /* button switch connected to port D pin 2 */

int main() {
  DDRB |= (1 << LED_PIN); /* set the pinB0 direction to output */
  DDRD &= ~(1 << BUTTON); /* Makes second pin of PORTD as Input */
  PORTB = 0x0u;           /* set all pinB low (turns off) */

  while (1) {
    /* the button is pressed when BUTTON bit is clear */
    if (!(PIND & (1 << BUTTON))) {
      PORTB |= (1 << LED_PIN); /* set pin 0 of port B to high (turns on) */
      while (!(PIND & (1 << BUTTON)))
        ; /* wait while the button is pressed */
    }
    PORTB &= ~(1 << LED_PIN); /* reset led of port B, pin 0, to off */
  }
}
