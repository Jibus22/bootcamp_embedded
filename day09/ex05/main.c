#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

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
  while (!(UCSR0A & (1 << UDRE0))); /* wait for transmit buffer to be empty */
  UDR0 = c; /* load data into transmit register */
}

char uart_rx(void) {
  /* (ds: 24.8 Data Reception) Wait for data to be received */
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0; /* Get and return received data from buffer */
}

void uart_printstr(const char *str) {
  while (*str) uart_tx(*str++);
}

/* ******************** TWI utils ******************** */

long ft_pow(int base, int power)
{
	long result = 1;

	while (power--)
		result *= base;
	return (result);
}

void print_status(const char *id) {
  uart_printstr(id);
  uart_printstr("\n\r    ");

  switch(TWSR & 0xF8) {
    case TW_START:
      return uart_printstr("A START condition has been transmitted\r\n");
    case TW_REP_START:
      return uart_printstr("A repeated START condition has been transmitted\r\n");
    case TW_MT_SLA_ACK:
      return uart_printstr("SLA+W has been transmitted; ACK has been received\r\n");
    case TW_MT_SLA_NACK:
      return uart_printstr("SLA+W has been transmitted;"
          "NOT ACK has been received\r\n");
    case TW_MT_DATA_ACK:
      return uart_printstr("Data byte has been transmitted;"
          "ACK has been received\r\n");
    case TW_MT_DATA_NACK:
      return uart_printstr("Data byte has been transmitted;"
          "NOT ACK has been received\r\n");
    case TW_MT_ARB_LOST:
      return uart_printstr("Arbitration lost in SLA+W or data bytes\r\n");
    default:
      return uart_printstr("Unknown error\r\n");
  }
}

void print_hex_value(unsigned char n) {
  unsigned char value[3] = {0};
	unsigned int e = n / 16;
	short int res;
	int i = 1;

	while (e) {
		e /= 16;
		i++;
	}
	while (i--)
	{
		res = ((n / ft_pow(16, e++)) % 16);
		if (res < 10)
			res += 48;
		else
			res += 87;
		value[i] = res;
	}
  uart_printstr((char*)value);
  uart_printstr(" ");
}

/* ******************** TWI ******************** */

#define SCL 100000UL /* I2C SCL at 100kHz */

#define ERROR(msg) \
  do {                    \
    print_status(msg); \
    return;               \
  } while (0)

#define WAIT_I2C(reg) \
  do {} while (!(reg & (1 << TWINT)))

#define ACK_TYPE(type) ((TWSR & 0xF8) == (type))

void i2c_init() {
  /* (ds: 26.5.2) Given the formula of datasheet, setting bit rate register
   * value so that I2C SCL frequency is 100kHz. */
  TWBR = (char)((F_CPU / SCL - 16) / 2);
}

void i2c_stop() {
  /* Transmit STOP condition. */
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void i2c_write(uint8_t data) {
  TWDR = data; /* Load data into Data Register */
  TWCR = (1 << TWINT) | (1 << TWEN); /* Start transmision */

  WAIT_I2C(TWCR); /* Wait confirmation TWI interface sent DATA */

  /* Check ACK of DATA */ 
  if (!ACK_TYPE(TW_MT_DATA_ACK)) ERROR("W data failed\n\r");
}

#define RD_ACK 0
#define RD_NACK 1

char i2c_read(char type) {
  if (type == RD_ACK)
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); /* Start Rx, with ACK */
  else
    TWCR = (1 << TWINT) | (1 << TWEN); /* Start Rx */

  /* Received data can be read from TWDR when TWINT is set high by hardware,
   * so wait for it. */
  WAIT_I2C(TWCR);
  return TWDR; /* Received data is now into TWDR */
}

#define ADDR_W 0 /* Tx mode (0th address bit) */
#define ADDR_R 1 /* Rx mode (0th address bit) */

/* Send slave address */
void i2c_transmit_addr(char addr, char type) {
  TWDR = (addr << 1) + type; /* Load 7bits addr + R/W bit */
  TWCR = (1 << TWINT) | (1 << TWEN); /* Start transmision */

  WAIT_I2C(TWCR); /* Wait confirmation TWI interface sent SLA+W/R */

  /* Check ACK of SLAVE ADDRESS */ 
  if (!ACK_TYPE(type == ADDR_W ? TW_MT_SLA_ACK : TW_MR_SLA_ACK))
    ERROR("W addr failed\n\r");
}

/* Enter in Master mode, send START condition */
void i2c_start() {
  /* (ds: 26-7-2) generate a START condition as soon as the bus becomes free */
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

  WAIT_I2C(TWCR); /* Wait confirmation TWI interface sent START */
  if (!ACK_TYPE(TW_START) && !ACK_TYPE(TW_REP_START))
    ERROR("start failed\n\r"); /* Check ACK of START or REP_START */ 
}

/* ******************** PCA9555 EXPANDER  ******************** */

/* PCA9555 address, as configured on the dev dipswitchs (0,0,0) */
#define PCA9555_ADDR 0x20

/* PCA9555 Command byte commands */
#define CMD_IN_P0 0x00
#define CMD_IN_P1 0x01
#define CMD_OUT_P0 0x02
#define CMD_OUT_P1 0x03
#define CMD_POL_INV_P0 0x04
#define CMD_POL_INV_P1 0x05
#define CMD_CONF_P0 0x06
#define CMD_CONF_P1 0x07

/* Facilities. In PCA9555, input is 1, output is 0 */
#define PCA9555_SET_OUT(reg, val) (reg & ~(val))
#define PCA9555_SET_IN(reg, val) (reg | (val))

void pca9555_write(uint8_t cmd, uint8_t data) {
  i2c_start();
  i2c_transmit_addr(PCA9555_ADDR, ADDR_W);
  i2c_write(cmd);
  i2c_write(data);
  i2c_stop();
}

uint8_t pca9555_read(uint8_t cmd) {
  uint8_t received;

  i2c_start();
  i2c_transmit_addr(PCA9555_ADDR, ADDR_W);
  i2c_write(cmd);
  i2c_start();
  i2c_transmit_addr(PCA9555_ADDR, ADDR_R);
  received = i2c_read(RD_NACK);
  i2c_stop();
  return received;
}

/* ******************** LED_SCREEN ******************** */
/* PCA9555 IO routing */
/* PORT0 */
#define DIG_1 (1 << 4)
#define DIG_2 (1 << 5)
#define DIG_3 (1 << 6)
#define DIG_4 (1 << 7)

/* PORT1 */
#define SEG_A (1 << 0)
#define SEG_B (1 << 1)
#define SEG_C (1 << 2)
#define SEG_D (1 << 3)
#define SEG_E (1 << 4)
#define SEG_F (1 << 5)
#define SEG_G (1 << 6)
#define SEG_DOT (1 << 7)

/* Initialize PORT0 & PORT1 led screen pins to output and the rest to
 * input direction */
void led_init() {
  /* Set led pins direction to ouput */
  pca9555_write(CMD_CONF_P0, PCA9555_SET_OUT(0xFF, DIG_1 | DIG_2 | DIG_3 | DIG_4));
  pca9555_write(CMD_CONF_P1, PCA9555_SET_OUT(0xFF, 0xFF));
  /* Set leds to off */
  pca9555_write(CMD_OUT_P0, PCA9555_SET_IN(0xFF, DIG_1 | DIG_2 | DIG_3 | DIG_4));
  pca9555_write(CMD_OUT_P1, PCA9555_SET_IN(0xFF, 0xFF));
}

/* ******************** PGM ******************** */
#define iter_plus(i, max) i = ((i != max) * (i + 1))

#define SET_DIGIT(dig) pca9555_write(CMD_OUT_P0, PCA9555_SET_OUT(0xFF, dig));
#define SET_NB(nb) pca9555_write(CMD_OUT_P1, PCA9555_SET_OUT(0xFF, nb));

volatile uint8_t nbrs[4], dig;
static const uint8_t digit[4] = {DIG_1, DIG_2, DIG_3, DIG_4},
                     numbers[10] = {
                                    SEG_G | SEG_DOT,
                                    SEG_A | SEG_B | SEG_C | SEG_G | SEG_D | SEG_DOT,
                                    SEG_F | SEG_C | SEG_DOT,
                                    SEG_F | SEG_E | SEG_DOT,
                                    SEG_A | SEG_E | SEG_D | SEG_DOT,
                                    SEG_B | SEG_E | SEG_DOT,
                                    SEG_B | SEG_DOT,
                                    SEG_F | SEG_G | SEG_E | SEG_D | SEG_DOT,
                                    SEG_DOT,
                                    SEG_E | SEG_DOT,
                                   };

/* Se fait trigger ttes les 10 ms */
void led_display_nb() {
    SET_NB(numbers[nbrs[dig]]);
    SET_DIGIT(digit[dig]);
    iter_plus(dig, 3);
}

/* un interrupt qui trigger toutes les 10 ms pour afficher un int volatile */
ISR(TIMER0_COMPA_vect) {
  led_display_nb();
}

void init_timer() {
  /* TIMER0. mode ctc 2 */
  TCCR0A = (1 << WGM01);
  /* Set prescaler to 1024 */
  TCCR0B = (1 << CS02) | (1 << CS00);
  /* Timer/Counter0 Compare Match A interrupt enabled */
  TIMSK0 = (1 << OCIE0A);
  /* set TOP so that it is reached every 10 ms */
  OCR0A = 82;
  sei();
}

/* long long	ft_pow(int base, int power) { */
/* 	long long result; */

/* 	result = 1; */
/* 	while (power--) */
/* 		result *= base; */
/* 	return result; */
/* } */

/* Break down nb in four digits. Store each digit in nbrs variable */
void break_down(uint16_t n) {
  uint8_t result[4] = {0,0,0,0};
	unsigned int e = n / 10;
	int i = 1, j = 3;

	while (e) {
		e /= 10;
		i++;
	}
	while (i--)
		result[j--] = ((n / ft_pow(10, e++)) % 10);
  for (uint8_t k = 0; k < 4; k++) nbrs[k] = result[k];
}

int main() {
  uint16_t nb = 0;
  dig = 0;

  uart_init();
  i2c_init();
  led_init();
  init_timer();

  while (1) {
    break_down(nb);
    _delay_ms(100);
    iter_plus(nb, 9999);
  }
  return 0;
}
