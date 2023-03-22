#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

long ft_pow(int base, int power)
{
	long result = 1;

	while (power--)
		result *= base;
	return (result);
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
  ADMUX = (1 << REFS0); /* AREF = AVcc */

  /* ADC Enable and prescaler of 128. 16000000/128 = 125000 */
  ADCSRA = (1 << ADEN) | (1 << ADPS2 ) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t ch) {
  ch &= 0b00000111; /* Make sure ch value is between 0 and 7 */
  ADMUX = (ADMUX & 0xF8) | ch; /* clears the bottom 3 bits before ORing */

  ADCSRA |= (1 << ADSC); /* start single convertion */

  while(ADCSRA & (1 << ADSC)); /* wait for conversion to complete */

  return ADC;
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
	while (i--)
	{
		res = ((n / ft_pow(10, e++)) % 10);
		res += 48;
		value[i] = res;
	}
  uart_printstr((char*)value);
}

int main() {
  uint16_t val;
  uart_init();
  adc_init();

  while (1) {
    val = adc_read(ADC_POT);
    print_dec_value(val);
    uart_printstr(", ");
    val = adc_read(ADC_LDR);
    print_dec_value(val);
    uart_printstr(", ");
    val = adc_read(ADC_NTC);
    print_dec_value(val);
    uart_printstr("\r\n");
    _delay_ms(20);
  };
  return 0;
}
