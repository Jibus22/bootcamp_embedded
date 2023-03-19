#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdlib.h>

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

void i2c_write(unsigned char data) {
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

/* ******************** PGM ******************** */

#define AHT20_ADDR 0x38 /* address of sensor */

#define AHT20_STATUS_CMD 0x71

#define AHT20_INIT_CMD 0xBE
#define AHT20_INIT_CMD_P1 0x08
#define AHT20_INIT_CMD_P2 0x00

#define AHT20_MEASURE_CMD 0xAC
#define AHT20_MEASURE_CMD_P1 0x33
#define AHT20_MEASURE_CMD_P2 0x00
#define AHT20_DATA_ANSWER_LEN 7

/* Ask for AHT20 status then read and return answer */
unsigned char get_aht20_status() {
  /* Ask for sensor status */
  i2c_start();
  i2c_transmit_addr(AHT20_ADDR, ADDR_W);
  i2c_write(AHT20_STATUS_CMD);

  /* Read sensor status */
  i2c_start();
  i2c_transmit_addr(AHT20_ADDR, ADDR_R);
  return i2c_read(RD_NACK);
}

/* Send initialization command to aht20 */
void aht20_sensor_init() {
  i2c_start();
  i2c_transmit_addr(AHT20_ADDR, ADDR_W);
  i2c_write(AHT20_INIT_CMD);
  i2c_write(AHT20_INIT_CMD_P1);
  i2c_write(AHT20_INIT_CMD_P2);
  _delay_ms(10);
}

void aht20_sensor_power_on() {
  unsigned char status;

  _delay_ms(40);
  status = get_aht20_status();
  /* if status say sensor isnt calibrated, send initialization command */
  if (!(status & (1 << 3))) {
    uart_printstr("Not calibrated, init sensor\r\n");
    aht20_sensor_init();
  }
  i2c_stop();
}

void hex_print_aht20_measure(unsigned char *data) {
  for (int i = 0; i < AHT20_DATA_ANSWER_LEN; i++)
    print_hex_value(data[i]);
  uart_printstr("\n\r");
}

void aht20_measure() {
  unsigned char data[AHT20_DATA_ANSWER_LEN] = {0};
  long value;
  char str_temp[8] = {0}, str_hum[8] = {0};

  /* trigger measurement */
  i2c_start();
  i2c_transmit_addr(AHT20_ADDR, ADDR_W);
  i2c_write(AHT20_MEASURE_CMD);
  i2c_write(AHT20_MEASURE_CMD_P1);
  i2c_write(AHT20_MEASURE_CMD_P2);
  i2c_stop();
  _delay_ms(80);

  /* read sensor answer */
  i2c_start();
  i2c_transmit_addr(AHT20_ADDR, ADDR_R);
  data[0] = i2c_read(RD_ACK);
  /* wait until sensor says measurement is done */
  while ((data[0] & (1 << 7))) {
    uart_printstr("measurement not done, wait...\r\n");
    _delay_ms(10);
    data[0] = i2c_read(RD_ACK);
  }

  /* read all following values returned by aht20 sensor */
  for (int i = 1; i < AHT20_DATA_ANSWER_LEN; i++)
    data[i] = i2c_read(i == (AHT20_DATA_ANSWER_LEN - 1));
  i2c_stop();

  hex_print_aht20_measure(data);

  value = (data[3] & 0x0F);
  value = (value << 8) + (data[4]);
  value = (value << 8) + (data[5]);
  dtostrf((((double)value / ((long)1 << 20)) * 200.0 - 50.0), 3, 2, str_temp);
  value = data[1];
  value = (value << 8) + (data[2]);
  value = (value << 4) + (data[3] >> 4);
  dtostrf((((double)value / ((long)1 << 20)) * 100.0), 3, 2, str_hum);
  uart_printstr("Temperature: ");
  uart_printstr(str_temp);
  uart_printstr(".C, Humidity: ");
  uart_printstr(str_hum);
  uart_printstr("%");
  uart_printstr("\n\r");
}

int main() {
  uart_init();
  i2c_init();
  aht20_sensor_power_on();

  while (1) {
    aht20_measure();
    _delay_ms(2000); /* Max frequency recommended */
  }
  return 0;
}
