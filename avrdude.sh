#!/bin/bash
VERSION="Pepcio v1.0 (2019-01-21)"

#### AVR
#sudo apt-get install gcc-avr binutils-avr gdb-avr avr-libc avrdude
#
sudo avrdude -c usbasp -p m8 -t -B 10
sudo avrdude -c usbasp -p m8 -u -U flash:w:avr-miernik.hex -B 10
sudo avrdude -c usbasp -p m8 -u -U eeprom:w:avr-miernik.eep -B 10
sudo avrdude -c usbasp -p m8 -u -U flash:w:avr-miernik.hex -U eeprom:w:avr-miernik.eep -B 10
