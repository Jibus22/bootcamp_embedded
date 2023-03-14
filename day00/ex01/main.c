#include <avr/io.h>

#define LED_PIN 0

int main() {
	/* We set the pinB0 direction to output with 1st instruction then its
	 * output level to high with the 2nd one. Datasheet chapter 17 */
	DDRB |= (1 << LED_PIN);
	PORTB |= (1 << LED_PIN);
}
