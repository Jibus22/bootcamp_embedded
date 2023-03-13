FILENAME   = main
PORT       = /dev/ttyUSB0
DEVICE     = atmega328p
PROGRAMMER = arduino
BAUD       = 115200
F_CPU      = 1200000
CC         = avr-gcc
OBJCOPY    = avr-objcopy 
CFLAGS     = -Wall -Os -mmcu=$(DEVICE) -DF_CPU=${F_CPU}
TARGET     = main
SRCS       = main.c


all: hex flash

hex:
	$(CC) $(CFLAGS) -o $(TARGET).bin $(SRCS)
	$(OBJCOPY) -j .text -j .data -O ihex $(TARGET).bin $(TARGET).hex 

flash:
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(TARGET).hex:i

clean:
	rm $(TARGET).bin
	rm $(TARGET).hex
