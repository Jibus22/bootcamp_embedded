#include <unistd.h>

long ft_pow(int base, int power)
{
	long result = 1;

	while (power--)
		result *= base;
	return (result);
}

void printstr(const char *str) {
  while (*str) write(1, str++, 1);
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
  printstr((char*)value);
  write(1, " ", 1);
}

int main() {
  unsigned char buf[16] = {0x01,0xA1,0xFB,0x41,0x11,0x9A,0x00,0xB3,0xC9,0x09,
                           0x11,0x9A,0x00,0xB3,0xC9,0x09};

  for (int i = 0; i < 16; i++)
    print_hex_value(buf[i]);
  write(1, "\n", 1);
  return 0;
}
