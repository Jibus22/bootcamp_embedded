DEVICE     = atmega328p
PROGRAMMER = arduino
BAUD       = 115200
F_CPU      = 16000000UL

CC         = avr-gcc
OBJCOPY    = avr-objcopy 
CFLAGS     = -Wall -Os -mmcu=$(DEVICE) -DF_CPU=${F_CPU}
TARGET     = main
SRCS       = main.c

##### OSTYPE #####
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
	PORT       = /dev/tty.usbserial-1420
else
	PORT       = /dev/ttyUSB0
endif


all: hex flash

hex:
	$(CC) $(CFLAGS) -o $(TARGET).bin $(SRCS)
	$(OBJCOPY) -j .text -j .data -O ihex $(TARGET).bin $(TARGET).hex 

flash:
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(TARGET).hex:i

clean:
	rm $(TARGET).bin
	rm $(TARGET).hex
