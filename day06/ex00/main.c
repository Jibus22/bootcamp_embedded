#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ******************** LED ******************** */

#define TOGGLE_LED(port, pin) port ^= (1 << pin)
#define SET_LED(port, pin) port |= (1 << pin)
#define CLEAR_LED(port, pin) port &= ~(1 << pin)

#define LED_R PD5
#define LED_G PD6
#define LED_B PD3

void led_init() {
  DDRD |= (1 << LED_R) | (1 << LED_G) | (1 << LED_B);
  PORTD &= ~((1 << LED_R) | (1 << LED_G) | (1 << LED_B));
}

/* ******************** PGM ******************** */

#define iter_plus(i, max) i = ((i != max) * (i + 1))

int main() {
  char pin[3] = {LED_R, LED_G, LED_B};
  int i = 2;
  led_init();

  while (1) {
    CLEAR_LED(PORTD, pin[i]);
    iter_plus(i, 2);
    SET_LED(PORTD, pin[i]);
    _delay_ms(1000);
  };
  return 0;
}
