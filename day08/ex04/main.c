#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif


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

/* ******************** SPI ******************** */

#define SPI_DDR DDRB
#define SS PINB2
#define MOSI PINB3
#define SCK PINB5

void SPI_MasterInit(void) {
  /* Set MOSI, Slave Select and SCK output, all others input */
  SPI_DDR = (1 << MOSI) | (1 << SCK) | (1 << SS);
  /* Enable SPI, Master, set clock rate fck/16 */
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

uint8_t SPI_MasterTransmit(uint8_t cData) {
  /* Start transmission */
  SPDR = cData;
  /* Wait for transmission complete */
  while(!(SPSR & (1 << SPIF)));
  return SPDR;
}

/* ******************** APA102 ******************** */

void apa102_start_frame() {
  for (int i = 0; i < 4; i++) { SPI_MasterTransmit(0x00); }
}

void apa102_end_frame() {
  for (int i = 0; i < 4; i++) { SPI_MasterTransmit(0xFF); }
}

void apa102_led_frame(uint8_t brightness, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t led_frame[4] = {((brightness & 0x1F) | 0xE0), b, g, r};
  for (int i = 0; i < 4; i++) { SPI_MasterTransmit(led_frame[i]); }
}

/* ******************** PGM ******************** */

#define iter_plus(i, max) i = ((i != max) * (i + 1))
#define COLORS_NB 7
#define BRIGHT 1 /* brightness strength: 0 to 31 */

void set_apa_led(uint8_t led, uint8_t rgb[3]) {
  uint8_t led_type[3] = {1,2,4};

  led &= 0x07;
  apa102_start_frame();
  for (uint8_t i = 0; i < 3; i++) {
    apa102_led_frame(((led & led_type[i]) > 0) * BRIGHT, rgb[0], rgb[1], rgb[2]);
  }
  apa102_end_frame();
}

uint8_t apa_led_gauge(uint8_t nb) {
  uint8_t gauge = nb * 100 / 255;
  return (((gauge >= 33) * 1) + ((gauge >= 66) * 2) + ((gauge == 100) * 4));
}

#define BUFLEN 13
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
  int found, led;

  led = ((buf[8] == '6') * 1) + ((buf[8] == '7') * 2) + ((buf[8] == '8') * 4);
  if (buf[0] != '#' || buf[7] != 'D' || !led || buf[9])
    return 0;

  for (int i = 1; i < 7; i++) {
    found = 0;
    for (int j = 0; j < 16; j++)
      if (buf[i] == hexa[j])
        found = 1;
    if (!found) return 0;
  }
  rgb[0] = ft_atox(&buf[1]);
  rgb[1] = ft_atox(&buf[3]);
  rgb[2] = ft_atox(&buf[5]);
  return led;
}

volatile uint8_t pos;

void set_rgb(uint8_t r, uint8_t g, uint8_t b){
  set_apa_led(0x7, (uint8_t[3]){r,g,b});
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
  /* TIMER1. Set prescaler to 1024. 16MHz/1024=15625 Hz. CTC mode 4. */
  TCCR1B = (1 << WGM12);
  OCR1A = 183; /* = 15625/85. We reach TOP 85times per second -> 85Hz. */
  /* enable Output Compare A Match Interrupt */
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

int main() {
  uint8_t rgb[3], led;
  char buf[BUFLEN] = {0};
  pos = 0;

  init_rgb();
  uart_init();
  SPI_MasterInit();

  while (1) {
    ft_bzero(buf, BUFLEN);
    prompt(buf);
    
    led = convert_input(rgb, buf);
    if (led) {
      TCCR1B &= ~((1 << CS12) | (1 << CS10)); /* stop timer */
      set_apa_led(led, rgb);
    } else {
      if (!ft_strcmp(buf, "#FULLRAINBOW")) {
        TCCR1B |= ((1 << CS12) | (1 << CS10)); /* start timer */
      }
      else
        uart_printstr("error: wrong input. Usage: #RRGGBBDX\r\n");
    }
  }
  return 0;
}
