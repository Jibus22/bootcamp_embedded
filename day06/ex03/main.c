#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define iter_plus(i, max) i = ((i != max) * (i + 1))

int ft_isprint(unsigned char c) { return ((c >= 32 && c <= 126)); }

void ft_bzero(void *s, unsigned int n) {
  unsigned int i = 0;
  while (i < n) {
    ((char *)s)[i] = 0;
    i++;
  }
}

/* ******************** LED ******************** */

#define LED_R PD5
#define LED_G PD6
#define LED_B PD3

void led_init() {
  DDRD |= (1 << LED_R) | (1 << LED_G) | (1 << LED_B);
  PORTD &= ~((1 << LED_R) | (1 << LED_G) | (1 << LED_B));
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

/* ******************** TIMERS ******************** */

volatile uint8_t init;

void set_rgb(uint8_t r, uint8_t g, uint8_t b){
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

#define BUFLEN 8
#define DEL 127
#define BELL 7
#define ENTER 13

void prompt(char *buf) {
  int i = 0;
  char c;

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
      uart_tx(c);
      i++;
    } else if (c == ENTER) {
      uart_printstr("\n\r");
      break;
    } else if (c == DEL) {
      if (i > 0) {
        i--;
        buf[i] = 0;
        uart_printstr("\b \b");
      } else
        uart_tx(BELL);
    } else
      uart_tx(BELL);
  }
}

uint8_t	ft_atox(const char *str) {
	uint8_t nbr;
	int i = 0;

	nbr = 0;
	while (i < 2) {
		nbr = nbr * 16 + (str[i] - ((str[i] >= 48 && str[i] <= 57) ? 48 : 55));
    i++;
  }
	return nbr;
}

int convert_input(uint8_t *rgb, const char *buf) {
  char hexa[16] = "0123456789ABCDEF";
  int found;

  if (buf[0] != '#') return 1;
  for (int i = 1; i < 7; i++) {
    found = 0;
    for (int j = 0; j < 16; j++)
      if (buf[i] == hexa[j])
        found = 1;
    if (!found) return 1;
  }
  rgb[0] = ft_atox(&buf[1]);
  rgb[1] = ft_atox(&buf[3]);
  rgb[2] = ft_atox(&buf[5]);
  return 0;
}

int main() {
  uint8_t rgb[3];
  char buf[BUFLEN] = {0};

  init = 0;
  led_init();
  init_timer();
  uart_init();

  while (1) {
    ft_bzero(buf, BUFLEN);
    prompt(buf);
    
    if (!convert_input(rgb, buf))
      set_rgb(rgb[0], rgb[1], rgb[2]);
    else
      uart_printstr("error: wrong input. Usage: #RRGGBB\r\n");
  };
  return 0;
}
