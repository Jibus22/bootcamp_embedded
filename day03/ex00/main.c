#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD_RATE 115200

#define setBit(reg, bit) (reg = reg | (1 << bit))
#define clearBit(reg, bit) (reg = reg & ~(1 << bit))
#define toggleBit(reg, bit) (reg = reg ^ (1 << bit))
#define clearFlag(reg, bit) (reg = reg | (1 << bit))

void uart_init() {
  /* Table 24-1. Equations for Calculating Baud Rate Register Setting */
  unsigned int baud = ((F_CPU/(8*BAUD_RATE))-1)/2;
  
  UBRR0H = (uint8_t)(baud >> 8); /* write to higher byte */
  UBRR0L = (uint8_t)(baud & 0xFF); /* write to lower byte */

  /* 24.12.3 USART Control and Status Register n B.
   * Bit 3 – TXEN: Transmitter Enable Writing this bit to one enables
   * the USART Transmitter.  */
  UCSR0B |= (1 << TXEN0);
}

void uart_tx(char c) {
    while(!(UCSR0A & (1 << UDRE0))); /* wait for transmit buffer to be empty */
    
    UDR0 = c; /* load data into transmit register */
}

int main() {
  uart_init();

  while (1) {
    uart_tx('Z');
    _delay_ms(1000);
  }
  return 0;
}
