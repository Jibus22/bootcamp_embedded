#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define iter_inc(i, max) i = ((i != (max)) * (i + 1))
#define iter_dec(i, max) i = (((i > 0) * (i - 1)) + ((i == 0) * (max)))

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
uint8_t int_safe(uint8_t(*cb)(uint8_t), uint8_t cmd) {
  uint8_t ret;

  cli();
  ret = cb(cmd);
  sei();
  return ret;
}

/* execute cb safely from interrupts */
void int_safe2(void(*cb)(uint8_t, uint8_t), uint8_t data, uint8_t action) {
  cli();
  cb(data, action);
  sei();
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
                PCA9555_SET_OUT(0xFF, DIG_1 | DIG_2 | DIG_3 | DIG_4 |
                                LED_D9 | LED_D10 | LED_D11));
  /* Set leds to off */
  pca9555_write(CMD_OUT_P0,
                PCA9555_SET_IN(0xFF, DIG_1 | DIG_2 | DIG_3 | DIG_4 |
                                LED_D9 | LED_D10 | LED_D11));
}

void i2c_expander_init_port1() {
  /* Set all led pins direction of port1 to ouput */
  pca9555_write(CMD_CONF_P1, PCA9555_SET_OUT(0xFF, 0xFF));
  /* Set leds to off */
  pca9555_write(CMD_OUT_P1, PCA9555_SET_IN(0xFF, 0xFF));
}

/* ******************** TIMER ******************** */
/* TC facilities */
#define TC_CLEAR 0x00,0x00,0x00,0x00,0x00

/* TIMER/COUNTER 0 SETTINGS */
#define TC0_PRESCALER_1024 ((1 << CS02) | (1 << CS00))

/* TIMER/COUNTER 1 SETTINGS */
#define TC1_MODE4_A 0x00
#define TC1_MODE4_B ((1 << WGM12))
#define TC1_PRESCALER_1024 ((1 << CS12) | (1 << CS10))
#define TC1_COMPA ((1 << OCIE1A))

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
#define SET_DIGIT(dig) \
  pca9555_safe_write_port0(dig, PCA9555_SET);
#define CLEAR_DIGIT(dig) \
  pca9555_safe_write_port0(dig, PCA9555_CLEAR);
#define SET_NB(nb) \
  pca9555_write(CMD_OUT_P1, PCA9555_SET_OUT(0xFF, nb));

volatile uint16_t led_nb;
volatile uint8_t nbrs[4], dig;
static const uint8_t digit[4] = {DIG_1, DIG_2, DIG_3, DIG_4},
                     numbers[10] = {
                         SEG_G | SEG_DOT,
                         SEG_A | SEG_G | SEG_D | SEG_E | SEG_F | SEG_DOT,
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
static void led_display_nb() {
  CLEAR_DIGIT(DIG_1 | DIG_2 | DIG_3 | DIG_4);
  SET_NB(numbers[nbrs[dig]]);
  SET_DIGIT(digit[dig]);
  iter_inc(dig, 3);
}

/* Break down nb in four digits. Store each digit in nbrs variable */
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

/* ISR triggered every 4ms to display one of four led digits and shift the
 * index of the next digit to display */
ISR(TIMER2_COMPA_vect) { led_display_nb(); }

/* ******************** LED ******************** */

#define LED_D1 (1 << 0)
#define LED_D2 (1 << 1)
#define LED_D3 (1 << 2)
#define LED_D4 (1 << 4)

static void led_init() {
  DDRB = LED_D1 | LED_D2 | LED_D3 | LED_D4;
  PORTB &= ~(LED_D1 | LED_D2 | LED_D3 | LED_D4);
}

/* Display nb in binary with leds */
static void led_binary_display(uint8_t nb) {
  PORTB &= ~(LED_D1 | LED_D2 | LED_D3 | LED_D4);
  PORTB |= (LED_D1 * ((nb & (1 << 0)) > 0)) |
           (LED_D2 * ((nb & (1 << 1)) > 0)) |
           (LED_D3 * ((nb & (1 << 2)) > 0)) |
           (LED_D4 * ((nb & (1 << 3)) > 0));
}

/* ******************** MODE_SELECTION ******************** */

#define MAX_MODE_NB 4

#define DECL_MODE_INIT(name) \
  static void name()

volatile uint8_t mode_select;


void adc_display_mode_init(void (*adc_select)(uint8_t),
                           uint8_t adc,
                           uint8_t voltage_reference) {
  adc_init_10bits_autotrigger(voltage_reference);
  adc_select(adc);

  set_timer0(TC_CLEAR);
  set_timer2(TC_CLEAR);

  /* tc0 used to trigger adc at each OV */
  set_timer0(0x00, 0x00, TC0_PRESCALER_1024, 0x00, 0x00);
  /* tc2 used to display led screen at compA match. mode 2 ctc */
  set_timer2(TC2_MODE2_A, TC2_MODE2_B, TC2_PRESCALER_1024, TC2_COMPA, 70);
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

#define DECL_ADC_MODE(name) \
  static void name(uint16_t data)

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

static void (*adc_mode[MAX_MODE_NB])(uint16_t) = {
  display_adc_value, display_adc_value, display_adc_value, display_adc_temp
};

/* ******************** PGM ******************** */

volatile uint8_t cnt;

/* triggered every seconds */
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

/* autotriggered by TC0 overflow */
ISR(ADC_vect) {
  uint16_t val = ADC;

  adc_mode[mode_select](val);

  TIFR0 = (1 << TOV0);
}

void start_routine() {
  uint8_t port0;

  while (cnt < 4) {
    if (BUTTON_PUSHED(PIND, SW1)) {
      I2C_LED_ON(LED_D9);
      _delay_ms(DEBOUNCE_DLY);
      while (BUTTON_PUSHED(PIND, SW1) && cnt < 4);
      I2C_LED_OFF(LED_D9);
      _delay_ms(DEBOUNCE_DLY);
    }
    if (BUTTON_PUSHED(PIND, SW2)) {
      I2C_LED_ON(LED_D10);
      _delay_ms(DEBOUNCE_DLY);
      while (BUTTON_PUSHED(PIND, SW2) && cnt < 4);
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

void loop() {
  void (*mode_init[MAX_MODE_NB])() = {rv1_init, ldr_init, ntc_init, temp_init};

  led_binary_display(mode_select);
  mode_init[mode_select]();
  while (1) {
    if (BUTTON_PUSHED(PIND, SW1)) {
      iter_inc(mode_select, MAX_MODE_NB - 1);
      led_binary_display(mode_select);
      mode_init[mode_select]();
      _delay_ms(DEBOUNCE_DLY);
      WAIT_RELEASE_BUTTON(PIND, SW1);
      _delay_ms(DEBOUNCE_DLY);
    }
    if (BUTTON_PUSHED(PIND, SW2)) {
      iter_dec(mode_select, MAX_MODE_NB - 1);
      led_binary_display(mode_select);
      mode_init[mode_select]();
      _delay_ms(DEBOUNCE_DLY);
      WAIT_RELEASE_BUTTON(PIND, SW2);
      _delay_ms(DEBOUNCE_DLY);
    }
  }
}

int main() {
  cnt = dig = mode_select = 0;
  led_nb = 8888;

  break_down(led_nb);
  switch_init();
  led_init();
  uart_init();
  i2c_init();

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
