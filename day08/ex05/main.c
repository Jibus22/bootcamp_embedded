#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ******************** SWITCH ******************** */
#define BUTTON_PUSHED(port, pin) (!(port & (1 << pin)))
#define WAIT_RELEASE_BUTTON(port, pin) \
  do {} while (BUTTON_PUSHED(port, pin))

#define SW1 2  /* button switch connected to port D pin 2 */
#define SW2 4

void switch_init() {
  DDRD &= ~((1 << SW1) | (1 << SW2));
}

/* ******************** ADC ******************** */
#define ADC0 0
#define ADC_POT ADC0

void adc_init() {
  ADMUX = (1 << REFS0) | (1 << ADLAR); /* AREF = AVcc, left adjusted */
  /* ADC Enable and prescaler of 128. 16000000/128 = 125000,
   * Auto Trigger Enable, ADC Interrupt Enable */
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | (1 << ADPS2 )
    | (1 << ADPS1) | (1 << ADPS0);
  /* Timer/Counter0 Overflow */
  ADCSRB = (1 << ADTS2);
}

void adc_set_channel(uint8_t ch) {
  ch &= 0b00000111; /* Make sure ch value is between 0 and 7 */
  ADMUX = (ADMUX & 0xF8) | ch; /* clears the bottom 3 bits before ORing */
}

/* ******************** TIMER ******************** */

void init_timer() {
  /* TIMER0. mode normal */
  TCCR0A = 0x00;
  /* Set prescaler to 1024 */
  TCCR0B = (1 << CS02) | (1 << CS00);
  sei();
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
#define BRIGHT 1 /* brightness strength: 0 to 31 */

void set_apa_led(uint8_t lled, uint8_t col[3][3]) {
  uint8_t led_type[3] = {1,2,4};

  lled &= 0x07;
  apa102_start_frame();
  for (uint8_t i = 0; i < 3; i++) {
    apa102_led_frame((((lled & led_type[i]) > 0) * BRIGHT),
                    col[i][0], col[i][1], col[i][2]);
  }
  apa102_end_frame();
}

volatile uint8_t pot, led, color, rgb[3][3];

ISR(ADC_vect) {
  pot = ADCH;
  uint8_t col[3][3] = {{rgb[0][0],rgb[0][1],rgb[0][2]},
                       {rgb[1][0],rgb[1][1],rgb[1][2]},
                       {rgb[2][0],rgb[2][1],rgb[2][2]}};

  col[led][color] = pot;
  set_apa_led(0x7, col);
  TIFR0 = (1 << TOV0);
}

int main() {
  pot = led = color = 0;

  switch_init();
  init_timer();
  adc_init();
  adc_set_channel(ADC_POT);
  SPI_MasterInit();
  while (1) {
    if (BUTTON_PUSHED(PIND, SW1)) {
      _delay_ms(15);

      rgb[led][color] = pot;
      iter_plus(color, 2);
      WAIT_RELEASE_BUTTON(PIND, SW1);
      _delay_ms(15);
    }

    if (BUTTON_PUSHED(PIND, SW2)) {
      _delay_ms(15);

      iter_plus(led, 2);
      WAIT_RELEASE_BUTTON(PIND, SW2);
      _delay_ms(15);
    }
  };
  return 0;
}
