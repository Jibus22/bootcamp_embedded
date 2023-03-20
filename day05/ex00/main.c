#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ******************** SWITCH ******************** */
#define SW1 2  /* button switch connected to port D pin 2 */

void switch_init() {
  DDRD &= ~(1 << SW1); /* Makes second pin of PORTD as Input */
}

/* ******************** LED ******************** */

#define LED_D1 0 /* PB0 */
#define LED_D2 1 /* PB1 */
#define LED_D3 2 /* PB2 */
#define LED_D4 4 /* PB4 */

void led_init() {
  DDRB |= (1 << LED_D1) | (1 << LED_D2) | (1 << LED_D3) | (1 << LED_D4);
  PORTB &= ~((1 << LED_D1) | (1 << LED_D2) | (1 << LED_D3) | (1 << LED_D4));
}

/* Display nb in binary with leds */
void led_display_nb(int nb) {
  PORTB = 0;
  PORTB |= (((nb & (1 << 0)) > 0) << LED_D1);
  PORTB |= (((nb & (1 << 1)) > 0) << LED_D2);
  PORTB |= (((nb & (1 << 2)) > 0) << LED_D3);
  PORTB |= (((nb & (1 << 3)) > 0) << LED_D4);
}

/* ******************** PGM ******************** */

int main() {
  uint8_t init = eeprom_read_byte((uint8_t*)0);
  uint8_t nb;
  led_init();
  switch_init();

  if (init != 0xDE) {
    eeprom_write_byte((uint8_t*)0, 0xDE);
    nb = 0;
    eeprom_write_byte((uint8_t*)1, nb);
  } else {
    nb = eeprom_read_byte((uint8_t*)1);
  }

  led_display_nb(nb);
  while (1) {
    if (!(PIND & (1 << SW1))) {
      nb++;
      if (nb == 16) nb = 0;
      eeprom_write_byte((uint8_t*)1, nb);
      led_display_nb(nb);
      _delay_ms(15);
      while (!(PIND & (1 << SW1)))
        ; /* wait while the button is pressed */
      _delay_ms(15);
    }
  }
  return 0;
}
