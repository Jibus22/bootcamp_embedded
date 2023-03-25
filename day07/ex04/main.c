#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

long ft_pow(int base, int power) {
  long result = 1;

  while (power--) result *= base;
  return (result);
}

/* ******************** LED ******************** */

#define LED_R PD5
#define LED_G PD6
#define LED_B PD3

void led_rgb_init() {
  DDRD |= (1 << LED_R) | (1 << LED_G) | (1 << LED_B);
  PORTD &= ~((1 << LED_R) | (1 << LED_G) | (1 << LED_B));
}

#define LED_D1 0 /* PB0 */
#define LED_D2 1 /* PB1 */
#define LED_D3 2 /* PB2 */
#define LED_D4 4 /* PB4 */

void led_init() {
  DDRB |= (1 << LED_D1) | (1 << LED_D2) | (1 << LED_D3) | (1 << LED_D4);
  PORTB &= ~((1 << LED_D1) | (1 << LED_D2) | (1 << LED_D3) | (1 << LED_D4));
}

/* switch led on according to the value */
void led_display_gauge(uint8_t nb) {
  uint8_t gauge = nb * 100 / 255;
  PORTB = 0;
  PORTB |= ((gauge >= 25) << LED_D1);
  PORTB |= ((gauge >= 50) << LED_D2);
  PORTB |= ((gauge >= 75) << LED_D3);
  PORTB |= ((gauge == 100) << LED_D4);
}

/* ******************** UART ******************** */

#define BAUD_RATE 115200

void uart_init() {
  /* Table 24-1. Equations for Calculating Baud Rate Register Setting */
  unsigned int baud = ((F_CPU / (8 * BAUD_RATE)) - 1) / 2;

  UBRR0H = (uint8_t)(baud >> 8);   /* write to higher byte */
  UBRR0L = (uint8_t)(baud & 0xFF); /* write to lower byte */

  /* Enable receiver and transmitter */
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);
  UCSR0C = (3 << UCSZ00); /* be sure 8 bits enabled */
}

void uart_tx(char c) {
  while (!(UCSR0A & (1 << UDRE0)))
    ;       /* wait for transmit buffer to be empty */
  UDR0 = c; /* load data into transmit register */
}

char uart_rx(void) {
  /* (ds: 24.8 Data Reception) Wait for data to be received */
  while (!(UCSR0A & (1 << RXC0)))
    ;
  return UDR0; /* Get and return received data from buffer */
}

void uart_printstr(const char *str) {
  while (*str) uart_tx(*str++);
}

/* ******************** ADC ******************** */

void adc_init() {
  ADMUX = (1 << REFS0) | (1 << ADLAR); /* AREF = AVcc, left adjusted */
  /* ADC Enable and prescaler of 128. 16000000/128 = 125000 */
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint8_t adc_read(uint8_t ch) {
  ch &= 0b00000111;            /* Make sure ch value is between 0 and 7 */
  ADMUX = (ADMUX & 0xF8) | ch; /* clears the bottom 3 bits before ORing */
  ADCSRA |= (1 << ADSC);       /* start single convertion */

  while (ADCSRA & (1 << ADSC))
    ; /* wait for conversion to complete */

  return ADCH;
}

/* ******************** TIMERS ******************** */

volatile uint8_t init;

void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
  if (init == 0) {
    /* Set prescaler to 64, start clock */
    TCCR2B = (1 << CS22);
    /* Set prescaler to 64, start clock */
    TCCR0B = (1 << CS01) | (1 << CS00);
    init = 1;
  }

  OCR0A = g;
  OCR0B = r;
  OCR2B = b;
}

void init_timer() {
  /* TIMER0. Set fastPWM mode 3, and non-inverting mode on OC0B and OC0A pins */
  TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1);

  /* TIMER2. Set fastPWM mode 3, and non-inverting mode on OC2B and OC2A pins */
  TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2A1) | (1 << COM2B1);
}

/* ******************** PGM ******************** */

#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC_POT ADC0
#define ADC_LDR ADC1
#define ADC_NTC ADC2

void print_dec_value(uint16_t n) {
  unsigned char value[5] = {0};
  unsigned int e = n / 10;
  uint16_t res;
  uint16_t i = 1;

  while (e) {
    e /= 10;
    i++;
  }
  while (i--) {
    res = ((n / ft_pow(10, e++)) % 10);
    res += 48;
    value[i] = res;
  }
  uart_printstr((char *)value);
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

int main() {
  uint8_t pot;
  init = 0;

  led_rgb_init();
  led_init();
  init_timer();
  /* uart_init(); */
  adc_init();

  while (1) {
    pot = adc_read(ADC_POT);
    wheel(pot);
    led_display_gauge(pot);
    /* print_dec_value(pot); */
    /* uart_printstr("\r\n"); */
    _delay_ms(20);
  };
  return 0;
}
