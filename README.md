# asus-battery-pack-Arduino-xface
A software library to access an Asus Li-Polymer battery pack though SMBus and "unlock" it and make it work again
Very useful to be able to rebuild the battery by employing generic Li-Po cells, and convince the battery chip to
reenable the pack

You must connect GND, SDA and SCL signals from the Arduino (check I2C.hpp for pins to use) to the battery pack, and also add 
1kohm pullup resistors from SDA to VCC and from SCL to VCC. VCC is 5v for Atmega based Arduinos, and is 3.3v for all 32bit 
arduinos. Both will work.



