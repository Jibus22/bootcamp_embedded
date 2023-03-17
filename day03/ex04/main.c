#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD_RATE 115200

#define ENTER 13
#define BUFLEN 128

int ft_isprint(unsigned char c) {
  return ((c >= 32 && c <= 126));
}

void bzero(void *s, unsigned int n) {
  unsigned int i = 0;
  while (i < n) {
    ((char*)s)[i] = 0;
    i++;
  }
}

int strcmp(const char *s1, const char *s2) {
  while (*s1 && *s2) {
    if (*s1 != *s2)
      return *s1 - *s2;
    s1++;
    s2++;
  }
  return *s1 - *s2;
}

void uart_init() {
  /* Table 24-1. Equations for Calculating Baud Rate Register Setting */
  unsigned int baud = ((F_CPU/(8*BAUD_RATE))-1)/2;
  
  UBRR0H = (uint8_t)(baud >> 8); /* write to higher byte */
  UBRR0L = (uint8_t)(baud & 0xFF); /* write to lower byte */

  /* 24.12.3 USART Control and Status Register n B.
   * Bit 3 – TXEN: Transmitter Enable Writing this bit to one enables
   * the USART Transmitter.  */
  UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
}

void uart_tx(char c) {
    while(!(UCSR0A & (1 << UDRE0))); /* wait for transmit buffer to be empty */
    UDR0 = c; /* load data into transmit register */
}

char uart_rx(void)
{
  /* (ds: 24.8 Data Reception) Wait for data to be received */
  while (!(UCSR0A & (1<<RXC0)));
  return UDR0; /* Get and return received data from buffer */
}

void uart_printstr(const char *str) {
  while (*str) uart_tx(*str++);
}

#define PWD 1

void prompt(char *buf, char type) {
  int i = 0;
  char c;

  while (i < BUFLEN) {
    c = uart_rx();
    if (ft_isprint(c)) {
      buf[i] = c;
      if (type == PWD)
        uart_tx('*');
      else
        uart_tx(c);
    } else if (c == ENTER) {
      uart_printstr("\n\r");
      break;
    }
    i++;
  }
}

int main() {
  char *username = "jle-corr";
  char *pwd = "abcdefgh";
  char username_buf[BUFLEN];
  char pwd_buf[BUFLEN];

  uart_init();

  while (1) {
    bzero(username_buf, BUFLEN);
    bzero(pwd_buf, BUFLEN);

    uart_printstr("Enter your login:\n\r    username: ");
    prompt(username_buf, 0);
    uart_printstr("    password: ");
    prompt(pwd_buf, PWD);

    if (!strcmp(username_buf, username) && !strcmp(pwd_buf, pwd)) {
      uart_printstr("Hello ");
      uart_printstr(username);
      uart_printstr("!\n\rDo you want to faire la revolution ?\n\r");
    } else {
      uart_printstr("Bad combinaison username/password\n\r");
    }

  }
  return 0;
}
