#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

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

/* ******************** TWI utils ******************** */

long ft_pow(int base, int power) {
  long result = 1;

  while (power--) result *= base;
  return (result);
}

void print_status(const char *id) {
  uart_printstr(id);
  uart_printstr("\n\r    ");

  switch (TWSR & 0xF8) {
    case TW_START:
      return uart_printstr("A START condition has been transmitted\r\n");
    case TW_REP_START:
      return uart_printstr(
          "A repeated START condition has been transmitted\r\n");
    case TW_MT_SLA_ACK:
      return uart_printstr(
          "SLA+W has been transmitted; ACK has been received\r\n");
    case TW_MT_SLA_NACK:
      return uart_printstr(
          "SLA+W has been transmitted;"
          "NOT ACK has been received\r\n");
    case TW_MT_DATA_ACK:
      return uart_printstr(
          "Data byte has been transmitted;"
          "ACK has been received\r\n");
    case TW_MT_DATA_NACK:
      return uart_printstr(
          "Data byte has been transmitted;"
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
  while (i--) {
    res = ((n / ft_pow(16, e++)) % 16);
    if (res < 10)
      res += 48;
    else
      res += 87;
    value[i] = res;
  }
  uart_printstr((char *)value);
  uart_printstr(" ");
}

/* ******************** TWI ******************** */

#define SCL 100000UL /* I2C SCL at 100kHz */

#define ERROR(msg)     \
  do {                 \
    print_status(msg); \
    return;            \
  } while (0)

#define WAIT_I2C(reg) \
  do {                \
  } while (!(reg & (1 << TWINT)))

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
  TWDR = data;                       /* Load data into Data Register */
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
  TWDR = (addr << 1) + type;         /* Load 7bits addr + R/W bit */
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

/* ******************** LED ******************** */
/* PCA9555 IO routing */
#define LED_D9 (1 << 3)
#define LED_D10 (1 << 2)
#define LED_D11 (1 << 1)

/* Display nb in binary with leds */
void led_binary_display(uint8_t nb) {
  pca9555_write(CMD_OUT_P0,
                PCA9555_SET_OUT(0xFF, (LED_D9 * ((nb & (1 << 0)) > 0)) |
                                          (LED_D10 * ((nb & (1 << 1)) > 0)) |
                                          (LED_D11 * ((nb & (1 << 2)) > 0))));
}

/* Initialize PORT0 leds to output and the rest to input direction */
void led_init() {
  /* Set led pins direction to ouput */
  pca9555_write(CMD_CONF_P0, PCA9555_SET_OUT(0xFF, LED_D9 | LED_D10 | LED_D11));
  /* Set leds to low */
  pca9555_write(CMD_OUT_P0, PCA9555_SET_IN(0xFF, LED_D9 | LED_D10 | LED_D11));
}

/* ******************** SWITCH ******************** */
/* PCA9555 IO routing */
#define SW3 (1 << 0)

#define BUTTON_PUSHED(port, pin) (!(port & (pin)))
#define WAIT_RELEASE_BUTTON(port, pin) \
  do {                                 \
    port = pca9555_read(CMD_IN_P0);    \
  } while (BUTTON_PUSHED(port, pin))

/* ******************** PGM ******************** */

int main() {
  uint8_t counter = 0, port0;

  uart_init();
  i2c_init();
  led_init();

  while (1) {
    port0 = pca9555_read(CMD_IN_P0);
    if (BUTTON_PUSHED(port0, SW3)) {
      _delay_ms(15);
      counter++;
      if (counter == 8) counter = 0;
      led_binary_display(counter);
      WAIT_RELEASE_BUTTON(port0, SW3);
      _delay_ms(15);
    }
    _delay_ms(20);
  }
  return 0;
}
