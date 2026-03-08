#include <xc.h>
#include <stdint.h>
#include "pcu_api.h"
#include "util.h"
#include "main.h"
#include <stdbool.h>


#pragma config FEXTOSC = ECH
#pragma config RSTOSC = HFINT32
#pragma config CLKOUTEN = OFF
#pragma config CSWEN = ON
#pragma config FCMEN = OFF

#pragma config MCLRE = ON
#pragma config PWRTE = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = ON
#pragma config BORV = LO
#pragma config ZCD = OFF
#pragma config PPS1WAY = ON
#pragma config STVREN = ON

#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = OFF
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC

#pragma config WRT = OFF
#pragma config SCANE = available
#pragma config LVP = ON

#pragma config CP = OFF
#pragma config CPD = OFF

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 32000000UL

void spiWriteByte(uint8_t data)
{
    SSP1BUF = data;
    while (!SSP1STATbits.BF) {;}
}

uint8_t SPIReadByte(void)
{
    spiWriteByte(0x00);
    return SSP1BUF;
}

uint8_t readRegister(uint8_t address)
{
    LATBbits.LATB4 = 0;

    //CHANGED FOR RFID, reg address can't be on LSB
    uint8_t control_byte = ((address << 1) & 0x7E) | 0x80;

    spiWriteByte(control_byte);
    uint8_t readB = SPIReadByte();

    LATBbits.LATB4 = 1;
    return readB;
}

void writeRegister(uint8_t address, uint8_t data)
{
    LATBbits.LATB4 = 0;

    //CHANGED FOR RFID, reg address can't be on LSB
    spiWriteByte((address << 1) & 0x7E);
    spiWriteByte(data);

    LATBbits.LATB4 = 1;
}

/*
//probably not necessary, RFID automatically does resets
void RFID_Reset(void)
{
    LATBbits.LATB5 = 0;
    __delay_ms(10);
    LATBbits.LATB5 = 1;
    __delay_ms(10);
}
*/


void SPI_setup(void)
{
    SSP1CON1bits.SSPEN = 0;

    SSP1DATPPS = 0x09;
    TRISBbits.TRISB1 = 1;
    ANSELBbits.ANSB1 = 0;

    RB2PPS = 0x15;
    TRISBbits.TRISB2 = 0;
    ANSELBbits.ANSB2 = 0;

    RB3PPS = 0x14;
    TRISBbits.TRISB3 = 0;
    ANSELBbits.ANSB3 = 0;

    SSP1STATbits.CKE = 1;
    SSP1CON1bits.CKP = 0;
    SSP1STATbits.SMP = 1;
    SSP1CON1bits.SSPM = 0b1010;
    /*
    //RFID reset output 
    TRISBbits.TRISB5 = 0;
    ANSELBbits.ANSB5 = 0;
    */

    SSP1ADD = 7;
    SSP1CON1bits.SSPEN = 1;

    ANSELBbits.ANSB4 = 0;
    TRISBbits.TRISB4 = 0;

    //NSS chip select idle high
    LATBbits.LATB4 = 1;

    //reseting RFID
    //RFID_Reset();

    //TxControlReg register address
    //Writing 1 to Tx2RFEn and Tx1RFEn
    //Which enables RF signal on TX2 and TX1
    //Which turns on antenna field and allows detecting tags
    writeRegister(0x14, 0x03);

}


//read RFID tag, first 8 digits specifically
uint32_t read_tag() {
    
    // FIFO max size
    uint8_t uid[64];

    uint8_t len = readRegister(0xA);

    for (int i = 0; i < len; i++)
    {
        //read from FIFO buffer n times to get UID
        uid[i] = readRegister(0x09);
    }

    //take first 8 hex digits / 4 bytes
    uint32_t tagValue = ((uint32_t)uid[0] << 24) |
               ((uint32_t)uid[1] << 16) |
               ((uint32_t)uid[2] << 8)  |
               ((uint32_t)uid[3]);

    return tagValue;
}

//transmit the 2 least sig bytes to home base
void transmit_to_home (uint16_t val, uint16_t task) {

    uint8_t highByteVal = (val >> 8) & 0xFF;
    uint8_t lowByteVal = val & 0xFF;

    uint8_t highByteTask = (task >> 8) & 0xFF;
    uint8_t lowByteTask = task & 0xFF;

    uint8_t cmd[] = { PCU_SYNC1, PCU_SYNC2, 0x01, 0x0A, 0x00, 0x00, 
                    lowByteTask, highByteTask, lowByteVal, highByteVal};

    UART_Write(cmd, sizeof(cmd));
}