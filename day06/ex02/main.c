#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define iter_plus(i, max) i = ((i != max) * (i + 1))

/* ******************** LED ******************** */

#define LED_R PD5
#define LED_G PD6
#define LED_B PD3

void led_init() {
  DDRD |= (1 << LED_R) | (1 << LED_G) | (1 << LED_B);
  PORTD &= ~((1 << LED_R) | (1 << LED_G) | (1 << LED_B));
}

/* ******************** TIMERS ******************** */

volatile uint8_t pos;

void set_rgb(uint8_t r, uint8_t g, uint8_t b){
  OCR0A = g;
  OCR0B = b;
  OCR2B = r;
}

void wheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) {
    set_rgb(255 - pos * 3, 0, pos * 3);
  } else if (pos < 170) {
    pos = pos - 85;
    set_rgb(0, pos * 3, 255 - pos * 3);
  } else {
    pos = pos - 170;
    set_rgb(pos * 3, 255 - pos * 3, 0);
  }
}

/* interrupt triggered 85 times per seconds */
ISR(TIMER1_COMPA_vect) {
  wheel(pos);
  iter_plus(pos, 255);
}

void init_rgb() {
  /* TIMER0. Set fastPWM mode 3, and non-inverting mode on OC0B and OC0A pins */
  TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1);
  /* Set prescaler to 64 */
  TCCR0B = (1 << CS01) | (1 << CS00);

  /* TIMER2. Set fastPWM mode 3, and non-inverting mode on OC2B and OC2A pins */
  TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2A1) | (1 << COM2B1);
  /* Set prescaler to 64 */
  TCCR2B = (1 << CS22);

  /* TIMER1. Set prescaler to 1024 because we don't need precision.
   * 16MHz/1024=15625 Hz. CTC mode 4. */
  TCCR1B = (1 << WGM12) | (1 << CS12 | (1 << CS10));
  OCR1A = 183; /* = 15625/85. We reach TOP 85times per second -> 85Hz. */
  /* enable Output Compare A Match Interrupt */
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

/* ******************** PGM ******************** */

int main() {
  led_init();
  pos = 0;
  init_rgb();

  while (1) {};
  return 0;
}
