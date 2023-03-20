#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ******************** SWITCH ******************** */
#define BUTTON_PUSHED(port, pin) (!(port & (1 << pin)))
#define WAIT_RELEASE_BUTTON(port, pin) \
  do {} while (BUTTON_PUSHED(port, pin))

#define SW1 2  /* button switch connected to port D pin 2 */
#define SW2 4

void switch_init() {
  DDRD &= ~((1 << SW1) | (1 << SW2));
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

#define MAGIC_NB 0xDE

typedef struct s_mem {
  uint8_t val;
  uint8_t *addr;
} t_mem;

/* Restore from eeprom or initialize counter */
void init_mem(uint8_t *nb, uint8_t *addr) {
  uint8_t init = eeprom_read_byte(addr);

  if (init != MAGIC_NB) {
    eeprom_write_byte(addr, MAGIC_NB);
    *nb = 0;
    eeprom_write_byte(addr + 1, *nb);
  } else {
    *nb = eeprom_read_byte(addr + 1);
  }
}

/* Write mem->val value at eeprom address mem->addr+1 (as mem->addr is MN) */
void write_mem(t_mem *mem) { eeprom_write_byte(mem->addr + 1, mem->val); }

int main() {
  t_mem cnt[5], *i = &cnt[4];
  led_init();
  switch_init();

  for (uint8_t j = 0; j < 5; j++) {
    cnt[j].addr = (uint8_t*)(j * 2);
    init_mem(&(cnt[j].val) , cnt[j].addr);
  }

  led_display_nb(cnt[i->val].val);
  while (1) {
    if (BUTTON_PUSHED(PIND, SW1)) {
      cnt[i->val].val++;
      if (cnt[i->val].val == 16) cnt[i->val].val = 0;
      write_mem(&cnt[i->val]);
      led_display_nb(cnt[i->val].val);
      _delay_ms(15);
      WAIT_RELEASE_BUTTON(PIND, SW1);
      _delay_ms(15);
    }

    if (BUTTON_PUSHED(PIND, SW2)) {
      (i->val)++;
      if (i->val == 4) i->val = 0;
      write_mem(i);
      led_display_nb(cnt[i->val].val);

      _delay_ms(15);
      WAIT_RELEASE_BUTTON(PIND, SW2);
      _delay_ms(15);
    }
  }
  return 0;
}
