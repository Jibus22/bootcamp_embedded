#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

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

void set_D6(uint8_t rgb[3]) {
  apa102_start_frame();
  apa102_led_frame(0x01, rgb[0], rgb[1], rgb[2]);
  apa102_led_frame(0x00, 0x00, 0x00, 0x00);
  apa102_led_frame(0x00, 0x00, 0x00, 0x00);
  apa102_end_frame();
}

int main() {
  int i = COLORS_NB - 1;
  uint8_t colors[COLORS_NB][3] = {{0xff, 0x00, 0x00},
                                  {0x00, 0xff, 0x00},
                                  {0x00, 0x00, 0xff},
                                  {0xff, 0xff, 0x00},
                                  {0x00, 0xff, 0xff},
                                  {0xff, 0x00, 0xff},
                                  {0xff, 0xff, 0xff}};
  SPI_MasterInit();
  while (1) {
    iter_plus(i, COLORS_NB - 1);
    set_D6(colors[i]);
    _delay_ms(1000);
  };
  return 0;
}
