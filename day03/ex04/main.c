#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD_RATE 115200

#define LED_D1 0 /* PB0 */
#define LED_D2 1 /* PB1 */
#define LED_D3 2 /* PB2 */
#define LED_D4 4 /* PB4 */

int ft_isprint(unsigned char c) { return ((c >= 32 && c <= 126)); }

void ft_bzero(void *s, unsigned int n) {
  unsigned int i = 0;
  while (i < n) {
    ((char *)s)[i] = 0;
    i++;
  }
}

int ft_strcmp(const char *s1, const char *s2) {
  while (*s1 && *s2) {
    if (*s1 != *s2) return *s1 - *s2;
    s1++;
    s2++;
  }
  return *s1 - *s2;
}

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

void print_stars(const char *str) {
  while (*str) {
    uart_tx('*');
    str++;
  }
}

#define BUFLEN 128
#define PWD 1
#define DEL 127
#define BELL 7
#define ENTER 13

void write_refresh(const char *prompt, const char *buf, char type) {
  uart_printstr("\r"); /* Cursor to left edge */
  uart_printstr("\x1b[0K"); /* Erase to right */
  uart_printstr(prompt);
  if (type != PWD)
    uart_printstr(buf);
  else
    print_stars(buf);
}

void prompt(const char *prompt, char *buf, char type) {
  int i = 0;
  char c;

  write_refresh(prompt, buf, type);
  while (i < BUFLEN) {

    c = uart_rx();

    if (i == BUFLEN - 1) {
      if (c != ENTER && c != DEL) {
        uart_tx(BELL);
        continue;
      }
    }

    if (ft_isprint(c)) {
      buf[i] = c;
      write_refresh(prompt, buf, type);
      i++;
    } else if (c == ENTER) {
      uart_printstr("\n\r");
      break;
    } else if (c == DEL) {
      if (i > 0) {
        i--;
        buf[i] = 0;
        write_refresh(prompt, buf, type);
      } else
        uart_tx(BELL);
    } else
      uart_tx(BELL);
  }
}

void led_blink() {
  int i = 15;
  while (i--) {
    PORTB ^= (1 << LED_D1) | (1 << LED_D2) | (1 << LED_D3) | (1 << LED_D4);
    _delay_ms(80);
  }
  PORTB &= ~((1 << LED_D1) | (1 << LED_D2) | (1 << LED_D3) | (1 << LED_D4));
}

void del_init() {
  DDRB |= (1 << LED_D1) | (1 << LED_D2) | (1 << LED_D3) | (1 << LED_D4);
  PORTB &= ~((1 << LED_D1) | (1 << LED_D2) | (1 << LED_D3) | (1 << LED_D4));
}

int main() {
  char *username = "jle-corr";
  char *pwd = "abcdefgh";
  char username_buf[BUFLEN];
  char pwd_buf[BUFLEN];

  del_init();
  uart_init();

  while (1) {
    ft_bzero(username_buf, BUFLEN);
    ft_bzero(pwd_buf, BUFLEN);

    uart_printstr("Enter your login:\n\r");
    prompt("    username: ", username_buf, 0);
    prompt("    password: ", pwd_buf, PWD);

    if (!ft_strcmp(username_buf, username) && !ft_strcmp(pwd_buf, pwd)) {
      uart_printstr("Hello ");
      uart_printstr(username);
      uart_printstr("!\n\rDo you want to faire la revolution ?\n\n\r");
      led_blink();
    } else {
      uart_printstr("Bad combinaison username/password\n\n\r");
    }
  }
  return 0;
}
