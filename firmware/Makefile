MCU=atmega328p
MCUFREQ=-DF_CPU=16000000UL
SERPORT=/dev/ttyUSB0
PROGTYPE=butterfly
BAUDRATE=38400
SOURCES=i2c.c motor.c
LIBDIR=lib
LIBS=serial.c twi.c ssi.c serencode.c
INCDIR=include
# HEADERS=bitset.h motor.h serial.h twi.h ssi.h serencode.h adc.h
TARGET=main

.PHONY: build upload

build: $(TARGET).hex

upload: build
	avrdude -P $(SERPORT) -p $(MCU) -c $(PROGTYPE) -b $(BAUDRATE) -U flash:w:$(TARGET).hex

$(TARGET).elf: $(SOURCES) $(HEADERS:%=$(INCDIR)/%) Makefile
	avr-gcc -I$(INCDIR) $(MCUFREQ) -Os -o $@ -mmcu=$(MCU) $(SOURCES) $(LIBS:%=$(LIBDIR)/%)
	
$(TARGET).hex: $(TARGET).elf Makefile
	avr-objcopy -O ihex -R .eeprom $< $@
