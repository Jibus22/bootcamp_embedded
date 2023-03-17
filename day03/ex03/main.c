#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD_RATE 115200

void uart_init() {
  /* Table 24-1. Equations for Calculating Baud Rate Register Setting */
  unsigned int baud = ((F_CPU/(8*BAUD_RATE))-1)/2;
  
  UBRR0H = (uint8_t)(baud >> 8); /* write to higher byte */
  UBRR0L = (uint8_t)(baud & 0xFF); /* write to lower byte */

  /* 24.12.3 USART Control and Status Register n B.
   * Bit 3 – TXEN: Transmitter Enable Writing this bit to one enables
   * the USART Transmitter.  */
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
  /* (ds: 24.12.3 ) Bit 7 – RXCIE: RX Complete Interrupt Enable */
  UCSR0B |= (1 << RXCIE0);
}

void uart_tx(char c) {
    while(!(UCSR0A & (1 << UDRE0))); /* wait for transmit buffer to be empty */
    
    UDR0 = c; /* load data into transmit register */
}

char switch_case(char c) {
  if ((c >= 65 && c <= 90) || (c >= 97 && c <= 122))
    c ^= 32;
  return c;
}

ISR(USART_RX_vect) {
  char c = UDR0; /* Get buffered data */
  uart_tx(switch_case(c)); /* Send data back */
}

int main() {
  uart_init();
  sei();

  while (1) {}
  return 0;
}
