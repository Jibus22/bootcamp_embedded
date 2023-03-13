#include <avr/io.h>

int main() {
	/* Set both first pin of PINB and PORTB IO registers to enable output high
	 * according to the table 17.1 in chapter 17 of the datasheet */
	PINB |= (1 << PINB0);
	PORTB |= (1 << PORTB0);
}
