#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define circular_inc(i, max) i = ((i != (max)) * (i + 1))
#define circular_dec(i, max) i = (((i > 0) * (i - 1)) + ((i == 0) * (max)))

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
#define SS (1 << PINB2)
#define MOSI (1 << PINB3)
#define MISO (1 << PINB4)
#define SCK (1 << PINB5)

void SPI_MasterInit(void) {
  /* Set MOSI, Slave Select and SCK output, all others input */
  SPI_DDR |= MOSI | SCK | SS;
  /* MISO must be an input pin */
  SPI_DDR &= ~MISO;
  /* Enable SPI, Master, set clock rate fck/16 */
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

void SPI_MasterClear(void) { SPCR &= ~(1 << SPE); }

uint8_t SPI_MasterTransmit(uint8_t cData) {
  /* Start transmission */
  SPDR = cData;
  /* Wait for transmission complete */
  while (!(SPSR & (1 << SPIF)))
    ;
  return SPDR;
}

/* ******************** APA102 ******************** */

void apa102_start_frame() {
  for (int i = 0; i < 4; i++) {
    SPI_MasterTransmit(0x00);
  }
}

void apa102_end_frame() {
  for (int i = 0; i < 4; i++) {
    SPI_MasterTransmit(0xFF);
  }
}

void apa102_led_frame(uint8_t brightness, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t led_frame[4] = {((brightness & 0x1F) | 0xE0), b, g, r};
  for (int i = 0; i < 4; i++) {
    SPI_MasterTransmit(led_frame[i]);
  }
}

#define COLORS_NB 3
#define BRIGHT 1 /* brightness strength: 0 to 31 */

void set_apa102_led(uint8_t led, const uint8_t rgb[3]) {
  uint8_t led_type[3] = {1, 2, 4};

  led &= 0x07;
  apa102_start_frame();
  for (uint8_t i = 0; i < 3; i++) {
    apa102_led_frame(((led & led_type[i]) > 0) * BRIGHT, rgb[0], rgb[1],
                     rgb[2]);
  }
  apa102_end_frame();
}

#define CLEAR_APA102_LED set_apa102_led(0x00, (uint8_t[3]){0x00, 0x00, 0x00});

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

/* ******************** ADC ******************** */
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC_RV1 ADC0
#define ADC_LDR ADC1
#define ADC_NTC ADC2

#define ADC_VOLTAGE_AVCC (1 << REFS0)
#define ADC_VOLTAGE_INTERNAL ((1 << REFS0) | (1 << REFS1))

void adc_init_10bits_autotrigger(uint8_t voltage_reference) {
  ADMUX = voltage_reference;

  /* ADC Enable and prescaler of 128. 16000000/128 = 125000,
   * Auto Trigger Enable, ADC Interrupt Enable */
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | (1 << ADPS2) |
           (1 << ADPS1) | (1 << ADPS0);
  /* Timer/Counter0 Overflow */
  ADCSRB = (1 << ADTS2);
}

void adc_set_channel(uint8_t ch) {
  ch &= 0b00000111;            /* Make sure ch value is between 0 and 7 */
  ADMUX = (ADMUX & 0xF8) | ch; /* clears the bottom 3 bits before ORing */
}

void adc_set_read_temp(uint8_t nul) {
  (void)nul;
  ADMUX = (ADMUX & 0xF8) | 8;
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

/* ******************** AHT20 I2C SENSOR ******************** */

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
  for (int i = 0; i < AHT20_DATA_ANSWER_LEN; i++) print_hex_value(data[i]);
  uart_printstr("\n\r");
}

void aht20_measure(uint8_t data[AHT20_DATA_ANSWER_LEN]) {
  /* trigger measurement */
  i2c_start();
  i2c_transmit_addr(AHT20_ADDR, ADDR_W);
  i2c_write(AHT20_MEASURE_CMD);
  i2c_write(AHT20_MEASURE_CMD_P1);
  i2c_write(AHT20_MEASURE_CMD_P2);
  i2c_stop();
  sei();
  _delay_ms(80);
  cli();

  /* read sensor answer */
  i2c_start();
  i2c_transmit_addr(AHT20_ADDR, ADDR_R);
  data[0] = i2c_read(RD_ACK);
  /* wait until sensor says measurement is done */
  while ((data[0] & (1 << 7))) {
    uart_printstr("measurement not done, wait...\r\n");
    sei();
    _delay_ms(10);
    cli();
    data[0] = i2c_read(RD_ACK);
  }

  /* read all following values returned by aht20 sensor */
  for (int i = 1; i < AHT20_DATA_ANSWER_LEN; i++)
    data[i] = i2c_read(i == (AHT20_DATA_ANSWER_LEN - 1));
  i2c_stop();
}

uint8_t aht20_data_to_temperature(const uint8_t *data) {
  long value;
  double result;

  value = (data[3] & 0x0F);
  value = (value << 8) + (data[4]);
  value = (value << 8) + (data[5]);
  result = (((double)value / ((long)1 << 20)) * 200.0 - 50.0);
  return (uint8_t)result;
}

uint8_t aht20_data_to_humidity(const uint8_t *data) {
  long value;
  double result;

  value = data[1];
  value = (value << 8) + (data[2]);
  value = (value << 4) + (data[3] >> 4);
  result = (((double)value / ((long)1 << 20)) * 100.0);
  return (uint8_t)result;
}

/* execute cb safely from interrupts */
void int_safe3(void (*cb)(uint8_t *), uint8_t *data) {
  cli();
  cb(data);
  sei();
}

/* ******************** PCA9555 I2C EXPANDER  ******************** */

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
#define PCA9555_SET 0x00
#define PCA9555_CLEAR 0x01
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

/* Set or clear 'data' bits according to 'action' in port0 of pca9555 */
void pca9555_safe_write_port0(uint8_t data, uint8_t action) {
  uint8_t pca9555_port0 = pca9555_read(CMD_IN_P0);

  if (action == PCA9555_SET)
    pca9555_write(CMD_OUT_P0, PCA9555_SET_OUT(pca9555_port0, data));
  else if (action == PCA9555_CLEAR)
    pca9555_write(CMD_OUT_P0, PCA9555_SET_IN(pca9555_port0, data));
}

/* execute cb safely from interrupts */
uint8_t int_safe(uint8_t (*cb)(uint8_t), uint8_t cmd) {
  uint8_t ret;

  cli();
  ret = cb(cmd);
  sei();
  return ret;
}

/* execute cb safely from interrupts */
void int_safe2(void (*cb)(uint8_t, uint8_t), uint8_t data, uint8_t action) {
  cli();
  cb(data, action);
  sei();
}

/* ******************** PCF8563 I2C RTC ******************** */

/* PCF8563 address */
#define PCF8563_ADDR 0x51

/* PCF8563 registers addresses */
#define PCF8563_REG_CTRL1 0x00
#define PCF8563_REG_CTRL2 0x01
#define PCF8563_REG_SEC 0x02
#define PCF8563_REG_MIN 0x03
#define PCF8563_REG_HOURS 0x04
#define PCF8563_REG_DAYS 0x05
#define PCF8563_REG_WEEKDAYS 0x06
#define PCF8563_REG_CENTURY_MONTHS 0x07
#define PCF8563_REG_YEARS 0x08

#define PCF8563_FULL_DATE_LEN 0x07

/* fill buf with full date. buf must be of size PCF8563_FULL_DATE_LEN */
void pcf8563_read_date(uint8_t *buf, uint8_t reg) {
  i2c_start();
  i2c_transmit_addr(PCF8563_ADDR, ADDR_W);
  i2c_write(reg);
  i2c_start();
  i2c_transmit_addr(PCF8563_ADDR, ADDR_R);

  for (uint8_t i = 0; i < PCF8563_FULL_DATE_LEN; i++)
    buf[i] = i2c_read(i == (PCF8563_FULL_DATE_LEN - 1));

  i2c_stop();
}

uint8_t pcf8563_data_to_min(const uint8_t *data) {
  return (((data[1] & 0x70) >> 4) * 10) + (data[1] & 0xF);
}

uint8_t pcf8563_data_to_hour(const uint8_t *data) {
  return (((data[2] & 0x30) >> 4) * 10) + (data[2] & 0xF);
}

uint8_t pcf8563_data_to_day(const uint8_t *data) {
  return (((data[3] & 0x30) >> 4) * 10) + (data[3] & 0xF);
}

uint8_t pcf8563_data_to_weekday(const uint8_t *data) {
  return data[4] & 0x07;
}

uint8_t pcf8563_data_to_month(const uint8_t *data) {
  return (((data[5] & 0x10) >> 4) * 10) + (data[5] & 0xF);
}

uint8_t pcf8563_data_to_year(const uint8_t *data) {
  return (((data[6] & 0xF0) >> 4) * 10) + (data[6] & 0xF);
}

/* ******************** SWITCH ******************** */
#define DEBOUNCE_DLY 5 /* delay anti bounce */
#define BUTTON_PUSHED(port, pin) (!(port & (pin)))

#define WAIT_RELEASE_BUTTON(port, pin) \
  do {                                 \
  } while (BUTTON_PUSHED(port, pin))

#define SW1 (1 << 2) /* button switch connected to port D pin 2 */
#define SW2 (1 << 4)

void switch_init() { DDRD &= ~(SW1 | SW2); }

/* **** SWITCH I2C EXPANDER **** */
/* PCA9555 IO routing */
#define SW3 (1 << 0)

#define WAIT_RELEASE_BUTTON_I2C(port, pin)    \
  do {                                        \
    port = int_safe(pca9555_read, CMD_IN_P0); \
  } while (BUTTON_PUSHED(port, pin))

/* ******************** LED ******************** */

#define SET_LED(port, led) port |= (led)
#define CLEAR_LED(port, led) port &= ~(led)

#define LED_D5_R (1 << PD5)
#define LED_D5_G (1 << PD6)
#define LED_D5_B (1 << PD3)

static void led_rgb_init() {
  DDRD |= LED_D5_R | LED_D5_G | LED_D5_B;
  CLEAR_LED(PORTD, (LED_D5_R | LED_D5_G | LED_D5_B));
}

#define LED_D1 (1 << PINB0)
#define LED_D2 (1 << PINB1)
#define LED_D3 (1 << PINB2)
#define LED_D4 (1 << PINB4)

static void led_init() {
  DDRB = LED_D1 | LED_D2 | LED_D3 | LED_D4;
  CLEAR_LED(PORTB, (LED_D1 | LED_D2 | LED_D3 | LED_D4));
}

/* Display nb in binary with leds */
static void led_binary_display(uint8_t nb) {
  CLEAR_LED(PORTB, (LED_D1 | LED_D2 | LED_D3 | LED_D4));
  PORTB |= (LED_D1 * ((nb & (1 << 0)) > 0)) | (LED_D2 * ((nb & (1 << 1)) > 0)) |
           (LED_D3 * ((nb & (1 << 2)) > 0)) | (LED_D4 * ((nb & (1 << 3)) > 0));
}

/* ******************** I2C_LED ******************** */
/* PCA9555 IO routing */
#define LED_D9 (1 << 3)
#define LED_D10 (1 << 2)
#define LED_D11 (1 << 1)

/* set LED_NB */
#define I2C_LED_ON(LED_NB) \
  int_safe2(pca9555_safe_write_port0, LED_NB, PCA9555_SET);

/* clear LED_NB */
#define I2C_LED_OFF(LED_NB) \
  int_safe2(pca9555_safe_write_port0, LED_NB, PCA9555_CLEAR);

/* ******************** I2C_LED_SCREEN ******************** */
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

void i2c_expander_init_port0() {
  /* Set led pins direction of port0 to ouput, the rest to input */
  pca9555_write(CMD_CONF_P0,
                PCA9555_SET_OUT(0xFF, DIG_1 | DIG_2 | DIG_3 | DIG_4 | LED_D9 |
                                          LED_D10 | LED_D11));
  /* Set leds to off */
  pca9555_write(CMD_OUT_P0,
                PCA9555_SET_IN(0xFF, DIG_1 | DIG_2 | DIG_3 | DIG_4 | LED_D9 |
                                         LED_D10 | LED_D11));
}

void i2c_expander_init_port1() {
  /* Set all led pins direction of port1 to ouput */
  pca9555_write(CMD_CONF_P1, PCA9555_SET_OUT(0xFF, 0xFF));
  /* Set leds to off */
  pca9555_write(CMD_OUT_P1, PCA9555_SET_IN(0xFF, 0xFF));
}

/* ******************** TIMER ******************** */
/* TC facilities */
#define TC_CLEAR 0x00, 0x00, 0x00, 0x00, 0x00

/* TIMER/COUNTER 0 SETTINGS */
#define TC0_PRESCALER_1024 ((1 << CS02) | (1 << CS00))

/* TIMER/COUNTER 1 SETTINGS */
#define TC1_MODE4_A 0x00
#define TC1_MODE4_B ((1 << WGM12))
#define TC1_PRESCALER_1024 ((1 << CS12) | (1 << CS10))
#define TC1_COMPA ((1 << OCIE1A))
#define TC1_COMPB ((1 << OCIE1B))

/* TIMER/COUNTER 2 SETTINGS */
#define TC2_MODE2_A ((1 << WGM21))
#define TC2_MODE2_B 0x00
#define TC2_PRESCALER_1024 ((1 << CS22) | (1 << CS21) | (1 << CS20))
#define TC2_COMPA ((1 << OCIE2A))

static void set_timer0(uint8_t mode_a, uint8_t mode_b, uint8_t prescaler,
                       uint8_t mask, uint8_t top) {
  TCCR0A = mode_a;
  TCCR0B = mode_b;
  TCCR0B |= prescaler;
  TIMSK0 = mask;
  if (top) OCR0A = top;
}

static void set_timer1(uint8_t mode_a, uint8_t mode_b, uint8_t prescaler,
                       uint8_t mask, uint16_t top) {
  TCCR1A = mode_a;
  TCCR1B = mode_b;
  TCCR1B |= prescaler;
  TIMSK1 = mask;
  if (top) OCR1A = top;
}

static void set_timer2(uint8_t mode_a, uint8_t mode_b, uint8_t prescaler,
                       uint8_t mask, uint8_t top) {
  TCCR2A = mode_a;
  TCCR2B = mode_b;
  TCCR2B |= prescaler;
  TIMSK2 = mask;
  if (top) OCR2A = top;
}

/* ******************** DIGIT DISPLAY LOGIC ******************** */
#define SET_DIGIT(dig) pca9555_safe_write_port0(dig, PCA9555_SET)
#define CLEAR_DIGIT(dig) pca9555_safe_write_port0(dig, PCA9555_CLEAR)
#define SET_NB(nb) pca9555_write(CMD_OUT_P1, PCA9555_SET_OUT(0xFF, nb))

#define SLR03_0 (SEG_G | SEG_DOT)
#define SLR03_1 (SEG_A | SEG_G | SEG_D | SEG_E | SEG_F | SEG_DOT)
#define SLR03_2 (SEG_F | SEG_C | SEG_DOT)
#define SLR03_3 (SEG_F | SEG_E | SEG_DOT)
#define SLR03_4 (SEG_A | SEG_E | SEG_D | SEG_DOT)
#define SLR03_5 (SEG_B | SEG_E | SEG_DOT)
#define SLR03_6 (SEG_B | SEG_DOT)
#define SLR03_7 (SEG_F | SEG_G | SEG_E | SEG_D | SEG_DOT)
#define SLR03_8 (SEG_DOT)
#define SLR03_9 (SEG_E | SEG_DOT)
#define SLR03_HYPHEN (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_DOT)
#define SLR03_C (SEG_B | SEG_C | SEG_G)
#define SLR03_F (SEG_B | SEG_C | SEG_D)
#define SLR03_H (SEG_A | SEG_D)
#define SLR03_VOID \
  (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G | SEG_DOT)

volatile uint16_t led_nb;
volatile uint8_t nbrs[4], dig;
static const uint8_t digit[4] = {DIG_1, DIG_2, DIG_3, DIG_4},
                     numbers[10] = {SLR03_0, SLR03_1, SLR03_2, SLR03_3,
                                    SLR03_4, SLR03_5, SLR03_6, SLR03_7,
                                    SLR03_8, SLR03_9};

static void led_screen_display_nb() {
  CLEAR_DIGIT(DIG_1 | DIG_2 | DIG_3 | DIG_4);
  SET_NB(numbers[nbrs[dig]]);
  SET_DIGIT(digit[dig]);
  circular_inc(dig, 3);
}

static void led_screen_display_42() {
  static const uint8_t displayft[4] = {SLR03_HYPHEN, SLR03_4, SLR03_2,
                                       SLR03_HYPHEN};

  CLEAR_DIGIT(DIG_1 | DIG_2 | DIG_3 | DIG_4);
  SET_NB(displayft[dig]);
  SET_DIGIT(digit[dig]);
  circular_inc(dig, 3);
}

static void led_screen_display_cel() {
  static const uint8_t celsius[2] = {SLR03_VOID, SLR03_C};

  CLEAR_DIGIT(DIG_1 | DIG_2 | DIG_3 | DIG_4);
  if (dig == 0 || dig == 1)
    SET_NB(numbers[nbrs[dig + 2]]);
  else
    SET_NB(celsius[dig - 2]);
  SET_DIGIT(digit[dig]);
  circular_inc(dig, 3);
}

static void led_screen_display_fahr() {
  static const uint8_t fahr[2] = {SLR03_VOID, SLR03_F};

  CLEAR_DIGIT(DIG_1 | DIG_2 | DIG_3 | DIG_4);
  if (dig == 0 || dig == 1)
    SET_NB(numbers[nbrs[dig + 2]]);
  else
    SET_NB(fahr[dig - 2]);
  SET_DIGIT(digit[dig]);
  circular_inc(dig, 3);
}

static void led_screen_display_hum() {
  static const uint8_t hum[2] = {SLR03_VOID, SLR03_H};

  CLEAR_DIGIT(DIG_1 | DIG_2 | DIG_3 | DIG_4);
  if (dig == 0 || dig == 1)
    SET_NB(numbers[nbrs[dig + 2]]);
  else
    SET_NB(hum[dig - 2]);
  SET_DIGIT(digit[dig]);
  circular_inc(dig, 3);
}

static void led_screen_display_hourmin() {
  uint8_t current_dig = numbers[nbrs[dig]];

  CLEAR_DIGIT(DIG_1 | DIG_2 | DIG_3 | DIG_4);
  if (dig == 1 || dig == 3)
    current_dig &= ~SEG_DOT;
  SET_NB(current_dig);
  SET_DIGIT(digit[dig]);
  circular_inc(dig, 3);
}

static void led_screen_display_daymonth() {
  uint8_t current_dig = numbers[nbrs[dig]];

  CLEAR_DIGIT(DIG_1 | DIG_2 | DIG_3 | DIG_4);
  if (dig == 3)
    current_dig &= ~SEG_DOT;
  SET_NB(current_dig);
  SET_DIGIT(digit[dig]);
  circular_inc(dig, 3);
}

/* Break down nb in four digits. Store each digit in 'nbrs' global variable */
static void break_down(uint16_t n) {
  uint8_t result[4] = {0, 0, 0, 0};
  unsigned int e = n / 10;
  int i = 1, j = 3;

  while (e) {
    e /= 10;
    i++;
  }
  while (i--) result[j--] = ((n / ft_pow(10, e++)) % 10);
  for (uint8_t k = 0; k < 4; k++) nbrs[k] = result[k];
}

/* ******************** MODE_SELECTION ******************** */

#define MAX_MODE_NB 10

#define DECL_MODE_INIT(name) static void name()

volatile uint8_t mode_select;

void adc_display_mode_init(void (*adc_channel_select)(uint8_t), uint8_t adc,
                           uint8_t voltage_reference) {
  adc_init_10bits_autotrigger(voltage_reference);
  adc_channel_select(adc);

  /* tc0 used to trigger adc at each OV */
  set_timer0(0x00, 0x00, TC0_PRESCALER_1024, 0x00, 0x00);
  /* tc2 used to display led screen at compA match. mode 2 ctc */
  set_timer2(TC2_MODE2_A, TC2_MODE2_B, TC2_PRESCALER_1024, TC2_COMPA, 70);
}

DECL_MODE_INIT(rtc_init) {
  /* tc2 used to display led screen at compA match. mode 2 ctc */
  set_timer2(TC2_MODE2_A, TC2_MODE2_B, TC2_PRESCALER_1024, TC2_COMPA, 70);
  /* tc1 used to count every seconds */
  set_timer1(TC1_MODE4_A, TC1_MODE4_B, TC1_PRESCALER_1024, TC1_COMPB, 15624);
}

DECL_MODE_INIT(sensor_measurement_init) {
  /* tc2 used to display led screen at compA match. mode 2 ctc */
  set_timer2(TC2_MODE2_A, TC2_MODE2_B, TC2_PRESCALER_1024, TC2_COMPA, 70);
  /* tc1 used to count every seconds */
  set_timer1(TC1_MODE4_A, TC1_MODE4_B, TC1_PRESCALER_1024, TC1_COMPB, 15624);
}

DECL_MODE_INIT(mode4_init) {
  SPI_MasterInit();
  led_rgb_init();
  /* tc2 used to display led screen at compA match. mode 2 ctc */
  set_timer2(TC2_MODE2_A, TC2_MODE2_B, TC2_PRESCALER_1024, TC2_COMPA, 70);
  /* tc1 used to count every seconds */
  set_timer1(TC1_MODE4_A, TC1_MODE4_B, TC1_PRESCALER_1024, TC1_COMPB, 15624);
}

DECL_MODE_INIT(temp_init) {
  adc_display_mode_init(adc_set_read_temp, 0, ADC_VOLTAGE_INTERNAL);
}

DECL_MODE_INIT(ntc_init) {
  adc_display_mode_init(adc_set_channel, ADC_NTC, ADC_VOLTAGE_AVCC);
}

DECL_MODE_INIT(ldr_init) {
  adc_display_mode_init(adc_set_channel, ADC_LDR, ADC_VOLTAGE_AVCC);
}

DECL_MODE_INIT(rv1_init) {
  adc_display_mode_init(adc_set_channel, ADC_RV1, ADC_VOLTAGE_AVCC);
}

/* ******************** ADC_MODES ******************** */

#define DECL_ADC_MODE(name) static void name(uint16_t data)

DECL_ADC_MODE(adc_nul) { (void)data; }

DECL_ADC_MODE(display_adc_temp) {
  data -= 342;
  if (data != led_nb) {
    led_nb = data;
    break_down(led_nb);
  }
}

DECL_ADC_MODE(display_adc_value) {
  if (data != led_nb) {
    led_nb = data;
    break_down(led_nb);
  }
}

/* ******************** TC1_COMPB MODES ******************** */

#define DECL_TC1_COMPB_MODE(name) static void name()

DECL_TC1_COMPB_MODE(tc1cmpB_nul) {}

DECL_TC1_COMPB_MODE(mode_4) {
  static uint8_t col_idx = 0;
  static const uint8_t led_rgb[3] = {LED_D5_R, LED_D5_G, LED_D5_B},
                       colors[COLORS_NB][3] = {{0xff, 0x00, 0x00},
                                               {0x00, 0xff, 0x00},
                                               {0x00, 0x00, 0xff}};

  CLEAR_LED(PORTD, (LED_D5_R | LED_D5_G | LED_D5_B));
  SET_LED(PORTD, led_rgb[col_idx]);
  set_apa102_led(0x7, colors[col_idx]);
  circular_inc(col_idx, COLORS_NB - 1);
}

volatile uint8_t loop_action;

#define ACTION_SENSOR_MEASURE 0x06

DECL_TC1_COMPB_MODE(sensor_measurement) {
  static uint8_t i = 2;
  /* i runs between 0 and 2. aht20 measure must be done only every 2sec */
  circular_inc(i, 2);
  /* gives signal to action callback in main loop to take a measure or not */
  loop_action = ((i == 0) * ACTION_SENSOR_MEASURE);
}

DECL_TC1_COMPB_MODE(get_rtc_hm) {
  uint8_t date[PCF8563_FULL_DATE_LEN];

  pcf8563_read_date(date, PCF8563_REG_SEC);
  break_down((pcf8563_data_to_hour(date) * 100) + (pcf8563_data_to_min(date)));
}

DECL_TC1_COMPB_MODE(get_rtc_daymonth) {
  uint8_t date[PCF8563_FULL_DATE_LEN];

  pcf8563_read_date(date, PCF8563_REG_SEC);
  break_down((pcf8563_data_to_day(date) * 100) + (pcf8563_data_to_month(date)));
}

/* ******************** ISR ******************** */

volatile uint8_t cnt;

static void (*led_screen_display_mode[MAX_MODE_NB])() = {
  led_screen_display_nb,   led_screen_display_nb, led_screen_display_nb,
  led_screen_display_nb,   led_screen_display_42, led_screen_display_cel,
  led_screen_display_fahr, led_screen_display_hum, led_screen_display_hourmin,
  led_screen_display_daymonth};

/* ISR used to call various flavors of LED screen displaying callbacks */
ISR(TIMER2_COMPA_vect) { led_screen_display_mode[mode_select](); }

static void (*tc1_cmpB_mode[MAX_MODE_NB])() = {
  tc1cmpB_nul, tc1cmpB_nul,        tc1cmpB_nul,        tc1cmpB_nul,
  mode_4,      sensor_measurement, sensor_measurement, sensor_measurement,
  get_rtc_hm, get_rtc_daymonth};

/* ISR used to call various flavors of callbacks */
ISR(TIMER1_COMPB_vect) { tc1_cmpB_mode[mode_select](); }

/* start routine */
ISR(TIMER1_COMPA_vect) {
  cnt++;
  if (cnt == 3) {
    /* clear TC2 (= stop displaying led_nb) */
    set_timer2(TC_CLEAR);
    CLEAR_DIGIT(DIG_1 | DIG_2 | DIG_3 | DIG_4);
  } else if (cnt == 4) {
    /* clear TC1 */
    set_timer1(TC_CLEAR);
  }
}

static void (*adc_mode[MAX_MODE_NB])(uint16_t) = {
  display_adc_value, display_adc_value, display_adc_value,
  display_adc_temp,  adc_nul,           adc_nul};

/* autotriggered by TC0 overflow. read and display adc value. */
ISR(ADC_vect) {
  uint16_t val = ADC;

  adc_mode[mode_select](val);

  TIFR0 = (1 << TOV0);
}

/* ******************** ACTION_MODE ******************** */

#define DECL_ACTION_MODE(name) static void name()

DECL_ACTION_MODE(action_nul) {}

/* returns temperature in celsius, or humidity */
uint8_t sensor_take_measure(uint8_t (*aht20_data_convert_cb)(const uint8_t *)) {
  static uint8_t data[AHT20_DATA_ANSWER_LEN];

  int_safe3(aht20_measure, data);
  return aht20_data_convert_cb(data);
}

DECL_ACTION_MODE(sensor_get_celsius) {
  if (loop_action != ACTION_SENSOR_MEASURE) return;
  break_down(sensor_take_measure(aht20_data_to_temperature));
}

DECL_ACTION_MODE(sensor_get_fahrenheit) {
  if (loop_action != ACTION_SENSOR_MEASURE) return;
  break_down((sensor_take_measure(aht20_data_to_temperature) * 9 / 5) + 32);
}

DECL_ACTION_MODE(sensor_get_humidity) {
  if (loop_action != ACTION_SENSOR_MEASURE) return;
  break_down(sensor_take_measure(aht20_data_to_humidity));
}

/* ******************** MAIN ******************** */

void mode_init_decorator(void (*set_mode)(), uint8_t mode) {
  static uint8_t last_mode;
  cli();

  if (last_mode == 4) {
    CLEAR_APA102_LED;
    CLEAR_LED(PORTD, (LED_D5_R | LED_D5_G | LED_D5_B));
    SPI_MasterClear();
    DDRB |= LED_D4;
  }

  set_timer0(TC_CLEAR);
  set_timer1(TC_CLEAR);
  set_timer2(TC_CLEAR);

  led_binary_display(mode);

  break_down(0);

  sei();

  set_mode();
  last_mode = mode;
}

void loop() {
  static void (*action[MAX_MODE_NB])() = {action_nul,
                                          action_nul,
                                          action_nul,
                                          action_nul,
                                          action_nul,
                                          sensor_get_celsius,
                                          sensor_get_fahrenheit,
                                          sensor_get_humidity,
                                          action_nul,
                                          action_nul};
  static void (*mode_set[MAX_MODE_NB])() = {rv1_init,
                                            ldr_init,
                                            ntc_init,
                                            temp_init,
                                            mode4_init,
                                            sensor_measurement_init,
                                            sensor_measurement_init,
                                            sensor_measurement_init,
                                            rtc_init,
                                            rtc_init};

  mode_init_decorator(mode_set[mode_select], mode_select);
  while (1) {
    action[mode_select]();
    if (BUTTON_PUSHED(PIND, SW1)) {
      circular_inc(mode_select, MAX_MODE_NB - 1);
      mode_init_decorator(mode_set[mode_select], mode_select);
      _delay_ms(DEBOUNCE_DLY);
      WAIT_RELEASE_BUTTON(PIND, SW1);
      _delay_ms(DEBOUNCE_DLY);
    }
    if (BUTTON_PUSHED(PIND, SW2)) {
      circular_dec(mode_select, MAX_MODE_NB - 1);
      mode_init_decorator(mode_set[mode_select], mode_select);
      _delay_ms(DEBOUNCE_DLY);
      WAIT_RELEASE_BUTTON(PIND, SW2);
      _delay_ms(DEBOUNCE_DLY);
    }
  }
}

void start_routine() {
  uint8_t port0;

  while (cnt < 4) {
    if (BUTTON_PUSHED(PIND, SW1)) {
      I2C_LED_ON(LED_D9);
      _delay_ms(DEBOUNCE_DLY);
      while (BUTTON_PUSHED(PIND, SW1) && cnt < 4)
        ;
      I2C_LED_OFF(LED_D9);
      _delay_ms(DEBOUNCE_DLY);
    }
    if (BUTTON_PUSHED(PIND, SW2)) {
      I2C_LED_ON(LED_D10);
      _delay_ms(DEBOUNCE_DLY);
      while (BUTTON_PUSHED(PIND, SW2) && cnt < 4)
        ;
      I2C_LED_OFF(LED_D10);
      _delay_ms(DEBOUNCE_DLY);
    }
    port0 = int_safe(pca9555_read, CMD_IN_P0);
    if (BUTTON_PUSHED(port0, SW3)) {
      I2C_LED_ON(LED_D11);
      _delay_ms(DEBOUNCE_DLY);
      do {
        port0 = int_safe(pca9555_read, CMD_IN_P0);
      } while (BUTTON_PUSHED(port0, SW3) && cnt < 4);
      I2C_LED_OFF(LED_D11);
      _delay_ms(DEBOUNCE_DLY);
    }
    _delay_ms(20);
  }
}

int main() {
  cnt = dig = mode_select = loop_action = 0;
  led_nb = 8888;

  break_down(led_nb);

  switch_init();
  led_init();

  uart_init();

  i2c_init();
  aht20_sensor_power_on();
  i2c_expander_init_port0();
  i2c_expander_init_port1();

  /* tc1 used to count every seconds */
  set_timer1(TC1_MODE4_A, TC1_MODE4_B, TC1_PRESCALER_1024, TC1_COMPA, 15624);

  /* tc2 used to display led screen at compA match. mode 2 ctc */
  set_timer2(TC2_MODE2_A, TC2_MODE2_B, TC2_PRESCALER_1024, TC2_COMPA, 70);
  sei();

  start_routine();

  loop();
  return 0;
}
