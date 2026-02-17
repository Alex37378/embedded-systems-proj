/*
 * File:   main.c
 * Author: edungdivinefavour
 *
 * Created on February 7, 2026, 3:38 AM
 */


//
// ================= CONFIG BITS =================
// CONFIG1
#include <builtins.h>
#pragma config FEXTOSC = ECH
#pragma config RSTOSC = HFINT32
#pragma config CLKOUTEN = OFF
#pragma config CSWEN = ON
#pragma config FCMEN = ON
// CONFIG2
#pragma config MCLRE = ON
#pragma config PWRTE = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = ON
#pragma config BORV = LO
#pragma config ZCD = OFF
#pragma config PPS1WAY = ON
#pragma config STVREN = ON
// CONFIG3
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = OFF
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC
// CONFIG4
#pragma config WRT = OFF
#pragma config SCANE = available
#pragma config LVP = ON
// CONFIG5
#pragma config CP = OFF
#pragma config CPD = OFF



// ================= INCLUDES =================
#include <xc.h>
#include <stdint.h>
#include "pcu_api.h"
#include "util.h"

#define _XTAL_FREQ 32000000UL



// ================= UART =================
void UART_Init(void)
{   
    TRISCbits.TRISC6 = 0;
    ANSELCbits.ANSC6 = 0;
    RC6PPS = 0x10;

    TRISCbits.TRISC5 = 1;
    ANSELCbits.ANSC5 = 0;
    RXPPS  = 0x15;

    // 115200 baud @ 32MHz
    SP1BRGL = 68;
    SP1BRGH = 0;

    BAUD1CONbits.BRG16 = 1;
    TX1STAbits.BRGH = 1;
    TX1STAbits.SYNC = 0;
    RC1STAbits.SPEN = 1;
    TX1STAbits.TXEN = 1;
    RC1STAbits.CREN = 1;

    ANSELAbits.ANSA5 = 0;
    TRISAbits.TRISA5 = 1;
    IOCAFbits.IOCAF5 = 0;
    IOCANbits.IOCAN5 = 1;
    IOCIE = 1;
    ANSELAbits.ANSA0 = 0;
    TRISAbits.TRISA0 = 0;

    PEIE = 1;
    GIE = 1;
    PIE3bits.RCIE = 0; 
}

void UART_Write(const uint8_t *buf, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        while (!TX1STAbits.TRMT);

        TX1REG = buf[i];
    }
}

uint8_t UART_Read(void)
{
    if (RC1STAbits.OERR) {
        RC1STAbits.CREN = 0;
        RC1STAbits.CREN = 1;
    }
    while (!PIR3bits.RCIF) { };

    return RC1REG;
}




// ================= PCU =================
const PCU_Info_t *PCU_GetInfo(void)
{
    static PCU_Info_t info;
    uint8_t cmd[] = { PCU_SYNC1, PCU_SYNC2, 0x01, 0x04, 0x00, 0x00 };
    uint8_t b[PCU_GET_INFO_RESPONSE_LEN];

    UART_Write(cmd, sizeof(cmd));
    for (uint8_t i = 0; i < 6; i++) (void)UART_Read();  /* skip header */
    for (uint8_t i = 0; i < PCU_GET_INFO_RESPONSE_LEN; i++) b[i] = UART_Read();

    info.team   = b[0];
    info.player = b[1];
    info.health = scale_u8_to_u16(&b[2]);
    info.shield = b[4];
    info.repair = b[5];

    return &info;
}

const PCU_UserData_t *PCU_GetUserData(void)
{
    static PCU_UserData_t u;
    uint8_t cmd[] = { PCU_SYNC1, PCU_SYNC2, 0x01, 0x05, 0x00, 0x00 };
    uint8_t b[PCU_GET_USER_DATA_RESPONSE_LEN];

    UART_Write(cmd, sizeof(cmd));
    for (uint8_t i = 0; i < 6; i++) (void)UART_Read();  /* skip header */
    for (uint8_t i = 0; i < PCU_GET_USER_DATA_RESPONSE_LEN; i++) b[i] = UART_Read();

    u.right_x   = scale_u8_to_u16(&b[0]);
    u.right_y   = scale_u8_to_u16(&b[2]);
    u.left_y    = scale_u8_to_u16(&b[4]);
    u.left_x    = scale_u8_to_u16(&b[6]);
    u.switch_a  = scale_u8_to_u16(&b[8]);
    u.switch_b  = scale_u8_to_u16(&b[10]);
    u.switch_c  = scale_u8_to_u16(&b[12]);
    u.switch_d  = scale_u8_to_u16(&b[14]);
    u.pot_vra   = scale_u8_to_u16(&b[16]);
    u.pot_vrb   = scale_u8_to_u16(&b[18]);

    return &u;
}

// ================= SET MOTOR =================
void PCU_SetMotor(const MotorSettings_t *motor)
{
    uint8_t buf[] = {
        PCU_SYNC1, PCU_SYNC2, 0x01, 0x06, 0x04, 0x00,
        motor->dirA, motor->pwmA, motor->dirB, motor->pwmB
    };

    UART_Write(buf, sizeof(buf));
}




// ================= MAIN =================
#define JOY_FWD_THRESHOLD   1450u
#define JOY_FWD_MAX         2000u
#define JOY_BACK_THRESHOLD  500u
#define POLL_INTERVAL_MS    (1000u / 10u)

int main(void)
{
    UART_Init();
    __delay_ms(1000);

    while (1) {
        const PCU_UserData_t *fly = PCU_GetUserData();
        uint16_t y = fly->right_y;

        uint8_t dir = MotorDir_Brake;
        uint8_t pwm = 0;

        if (y >= JOY_FWD_THRESHOLD) {
            dir = MotorDir_Forward;
            pwm = scale_u16_to_pwm(y, JOY_FWD_THRESHOLD, JOY_FWD_MAX);
        } 
        
        else if (y <= JOY_BACK_THRESHOLD) {
            dir = MotorDir_Backward;
            pwm = scale_u16_to_pwm(JOY_BACK_THRESHOLD - y, 0u, JOY_BACK_THRESHOLD);
        }

        MotorSettings_t motor = { dir, pwm, dir, pwm };
        PCU_SetMotor(&motor);

        __delay_ms(POLL_INTERVAL_MS);
    }
}
