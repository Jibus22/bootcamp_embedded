#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* ******************** ADC ******************** */
#define ADC0 0
#define ADC_POT ADC0

void adc_init() {
  ADMUX = (1 << REFS0) | (1 << ADLAR); /* AREF = AVcc, left adjusted */
  /* ADC Enable and prescaler of 128. 16000000/128 = 125000 */
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint8_t adc_read(uint8_t ch) {
  ch &= 0b00000111;            /* Make sure ch value is between 0 and 7 */
  ADMUX = (ADMUX & 0xF8) | ch; /* clears the bottom 3 bits before ORing */
  ADCSRA |= (1 << ADSC);       /* start single convertion */

  while (ADCSRA & (1 << ADSC))
    ; /* wait for conversion to complete */

  return ADCH;
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

/* ******************** PGM ******************** */

#define iter_plus(i, max) i = ((i != max) * (i + 1))
#define COLORS_NB 7

void set_apa_led(uint8_t led, uint8_t rgb[3]) {
  uint8_t led_type[3] = {1, 2, 4};

  led &= 0x07;
  apa102_start_frame();
  for (uint8_t i = 0; i < 3; i++) {
    apa102_led_frame(((led & led_type[i]) > 0), rgb[0], rgb[1], rgb[2]);
  }
  apa102_end_frame();
}

uint8_t apa_led_gauge(uint8_t nb) {
  uint8_t gauge = nb * 100 / 255;
  return (((gauge >= 33) * 1) + ((gauge >= 66) * 2) + ((gauge == 100) * 4));
}

int main() {
  int i = COLORS_NB - 1;
  uint8_t pot, gauge = 0, cmp = 0,
               colors[COLORS_NB][3] = {{0xff, 0x00, 0x00}, {0x00, 0xff, 0x00},
                                       {0x00, 0x00, 0xff}, {0xff, 0xff, 0x00},
                                       {0x00, 0xff, 0xff}, {0xff, 0x00, 0xff},
                                       {0xff, 0xff, 0xff}};
  adc_init();
  SPI_MasterInit();
  while (1) {
    pot = adc_read(ADC_POT);
    gauge = apa_led_gauge(pot);
    if (cmp != gauge) iter_plus(i, COLORS_NB - 1);
    cmp = gauge;
    set_apa_led(gauge, colors[i]);
    _delay_ms(20);
  };
  return 0;
}
