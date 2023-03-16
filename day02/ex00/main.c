#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define LED_D1 0 /* led connected to port B pin 0 */
#define SW1 2  /* button switch connected to port D pin 2 */
#define DEBOUNCE_TIME 1 /* time to wait while "de-bouncing" button (ms) */

void MCUInit(void) {
  DDRB |= (1 << LED_D1); /* set the pinB0 direction to output */
  DDRD &= ~(1 << SW1); /* Makes PD2 as Input */

  /* (ds: 16.2.20) enable external pin interrupt INT0 */
  EIMSK |= (1 << INT0);

  /* (ds: 16.2.1) set interrupt sense control so the falling edge of
   * INT0 generates an interrupt request.*/
  EICRA |= (1 << ISC00);

  sei(); /* Enable global interrupts */
}

/* pin INT0 change interrupt ISR */
ISR(INT0_vect) {
  PORTB ^= (1 << LED_D1); /* toggle pin 0 of port B to high */
  _delay_ms(DEBOUNCE_TIME);

  /* (ds: 16.2.3) clear the INTF0 flag to prevent boucing
   * (the flag is set whenever an interrupt happen so if ever a boucing
   * interrupt happens during ISR execution we don't want it so clear it) */
  EIFR |= (1 << INTF0);
}

int main(void) {
  MCUInit();
  while(1) {}
}
