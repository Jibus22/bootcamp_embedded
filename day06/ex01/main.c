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
#define COLORS_NB 7

void set_color(uint8_t rgb[3]) {
  char pin[3] = {LED_R, LED_G, LED_B};

  for (uint8_t i = 0; i < 3; i++) {
    if (rgb[i])
      SET_LED(PORTD, pin[i]);
    else
      CLEAR_LED(PORTD, pin[i]);
  }
}

int main() {
  int i = COLORS_NB - 1;
  uint8_t colors[COLORS_NB][3] = {{0xff, 0x00, 0x00},
                                  {0x00, 0xff, 0x00},
                                  {0x00, 0x00, 0xff},
                                  {0xff, 0xff, 0x00},
                                  {0x00, 0xff, 0xff},
                                  {0xff, 0x00, 0xff},
                                  {0xff, 0xff, 0xff}};
  led_init();

  while (1) {
    iter_plus(i, COLORS_NB - 1);
    set_color(colors[i]);
    _delay_ms(1000);
  };
  return 0;
}
