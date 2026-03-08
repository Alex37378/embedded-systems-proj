#ifndef RFID_SEQ_H
#define RFID_SEQ_H

#include <stdint.h>

void spiWriteByte(uint8_t data);
uint8_t SPIReadByte(void);
uint8_t readRegister(uint8_t address);
void writeRegister(uint8_t address, uint8_t data);
void RFID_Reset(void);
void SPI_setup(void);
uint32_t read_tag();
void transmit_to_home();

#endif