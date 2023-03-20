#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ******************** UTILS ******************** */

void ft_bzero(void *s, unsigned int n) {
  unsigned int i = 0;
  while (i < n) {
    ((char *)s)[i] = 0;
    i++;
  }
}

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

/* ******************** UART ******************** */

#define BAUD_RATE 115200

void uart_init() {
  /* Table 24-1. Equations for Calculating Baud Rate Register Setting */
  unsigned int baud = ((F_CPU / (8 * BAUD_RATE)) - 1) / 2;

  UBRR0H = (uint8_t)(baud >> 8);   /* write to higher byte */
  UBRR0L = (uint8_t)(baud & 0xFF); /* write to lower byte */

  /* 24.12.3 USART Control and Status Register n B.
   * Bit 3 – TXEN: Transmitter Enable Writing this bit to one enables
   * the USART Transmitter.  */
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
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

/* ******************** PGM ******************** */

#define MAGIC_NB 0xEA

#define bool _Bool
#define true 1
#define false 0

#define OFFSET_1 10
#define OFFSET_2 30

/* void eeprom_read_block (void *__dst, const void *__src, size_t __n); */
/* void eeprom_write_block (const void *__src, void *__dst, size_t __n); */

int main() {
  uint8_t buf[32] = {0};
  uart_init();

  eeprom_read_block(buf, 0, 32);
  uart_printstr("-->\r\n");
  uart_printstr((char*)buf);
  uart_printstr(".\r\n");

  eeprom_write_block("Coucouahaha\r\n", (void*)0, 13);

  eeprom_read_block(buf, 0, 32);
  uart_printstr("-->\r\n");
  uart_printstr((char*)buf);
  uart_printstr(".\r\n");
  while (1) {};
  return 0;
}

/* int main() { */
/*   uint8_t buf[1000] = {0}; */
/*   uart_init(); */
/*   eeprom_write_block(buf, (void*)0, 1000); */
/*   while (1) {}; */
/*   return 0; */
/* } */

/* bool safe_eeprom_read(void *buffer, size_t offset, size_t length) { */
/*   eeprom_read_block (buffer, (void*)offset, length); */
/*   return true; */
/* } */

/* bool safe_eeprom_write(void *buffer, size_t offset, size_t length) { */
/*   uint8_t MN = MAGIC_NB, mn; */
/*   if (offset == 0) return false; */
/*   mn = eeprom_read_byte((uint8_t*)(offset - 1)); */
/*   if (mn == MAGIC_NB) { */
/*     uart_tx(mn); */
/*     uart_printstr(" ------------------  eeprom already wrote\n\r"); */
/*     return false; /1* mem area already written *1/ */
/*   } */
/*   eeprom_write_block (&MN, (void*)(offset - 1), (1 * sizeof(uint8_t))); */
/*   eeprom_write_block (buffer, (void*)offset, length); */
/*   uart_printstr("------------------   wrote eeprom\n\r"); */
/*   return true; */
/* } */

/* void read_print_clean(const char *msg, uint8_t *buf, size_t offset, size_t len) { */
/*   uart_printstr(msg); */
/*   _delay_ms(30); */
/*   safe_eeprom_read((void*)buf, offset, len); */
/*   _delay_ms(30); */
/*   uart_printstr((char*)buf); */
/*   _delay_ms(30); */
/*   ft_bzero(buf, 32); */
/* } */

/* int main() { */
/*   bool ret; */
/*   uint8_t buf[32] = {0}; */
/*   uart_init(); */


/*   read_print_clean("read garbage:\r\n   ", buf, OFFSET_1, 10); */

/*   ret = safe_eeprom_write("hhhhhh!!\r\n", OFFSET_1, 10); */
/*   _delay_ms(20); */
/*   uart_printstr(ret > 0 ? "write ok\n\r" : "write false\r\n"); */
/*   read_print_clean("read 1:\r\n   ", buf, OFFSET_1, 10); */

/*   ret = safe_eeprom_write("YYYYYY!!\r\n", OFFSET_1, 10); */
/*   _delay_ms(20); */
/*   uart_printstr(ret > 0 ? "write ok\n\r" : "write false\r\n"); */
/*   read_print_clean("read 2:\r\n   ", buf, OFFSET_1, 10); */


/*   read_print_clean("read garbage2:\r\n   ", buf, OFFSET_2, 16); */
  
/*   ret = safe_eeprom_write("aaaaaaaaaaaaa!\r\n", OFFSET_2, 16); */
/*   _delay_ms(20); */
/*   uart_printstr(ret > 0 ? "write ok\n\r" : "write false\r\n"); */
/*   read_print_clean("read 3:\r\n   ", buf, OFFSET_2, 16); */

/*   ret = safe_eeprom_write("Ooooooooooooi!\r\n", OFFSET_2, 16); */
/*   _delay_ms(20); */
/*   uart_printstr(ret > 0 ? "write ok\n\r" : "write false\r\n"); */
/*   read_print_clean("read 4:\r\n   ", buf, OFFSET_2, 16); */

/*   while (1) {} */
/*   return 0; */
/* } */
