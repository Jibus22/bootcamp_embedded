#include <avr/io.h>
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

/* ******************** TWI ******************** */

#define SCL 100000UL /* I2C SCL at 100kHz */

#define WAIT_I2C(reg) \
  do {} while (!(reg & (1 << TWINT)))

void i2c_init() {
  /* (ds: 26.5.2) Given the formula of datasheet, setting bit rate register
   * value so that I2C SCL frequency is 100kHz. */
  TWBR = (char)((F_CPU / SCL - 16) / 2);
}

void i2c_stop() {
  /* Transmit STOP condition. */
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

#define AHT20_ADR 0x38

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

void twi_write_addr() {
  /* Load SLA_W into TWDR Register. Clear TWINT bit in TWCR to start
   * transmission of address */
  /* left shift by 1 bc the address is 7bits long and transfered from the MSB
   * to the LSB. We leave a 0 to the 0th position because it is the last bit
   * transfered and interpreted as the R/W bit. Here, 0 as Write mode */
  TWDR = AHT20_ADR << 1;
  TWCR = (1 << TWINT) | (1 << TWEN);

  /* Wait for TWINT Flag set. This indicates that the SLA+W has been
   * transmitted, and ACK/NACK has been received. */
  WAIT_I2C(TWCR);

  /* Here we check if we have indeed an acknowledge of slave address sent */
  /* if ((TWSR & 0xF8) != TW_MT_SLA_ACK) ERROR("WR addr failed\n\r"); */
  print_status("twi_write_addr()");
}

void i2c_start() {
  /* (ds: 26-2) send START condition */
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

  /* Wait for TWINT Flag set. This indicates that the START condition has been
   * transmitted to the hardware.*/
  WAIT_I2C(TWCR);

  /* Check value of TWI Status Register. Mask prescaler bits.
   * If status different from START go to ERROR.*/
  /* if ((TWSR & 0xF8) != TW_START) ERROR("start failed\n\r"); */
  print_status("i2c_start()");
}

int main() {
  int i = 15;

  uart_init();
  i2c_init();

  while (i--) {
    i2c_start();
    twi_write_addr();
    i2c_stop();
  }
  return 0;
}
