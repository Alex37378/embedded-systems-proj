#include <xc.h>
#include <stdint.h>
#include "pcu_api.h"
#include "util.h"
#include "main.h"
#include <stdbool.h>

PCU_Info_t * info;

//check if shield code has been received (every 500ms)
bool poll_for_shield_code() {

    //getting info repsonse cmd 0402h
    info = PCU_GetInfo();
    uint8_t shield_flag = info->shield;

    if (shield_flag == 1) {
        return true;
    }

    return false;
}

//transmit shield code
void transmit_shield_code() {

    //transmitting shield code cmd 0902h
    uint8_t cmd[] = { PCU_SYNC1, PCU_SYNC2, 0x02, 0x09, 0x00, 0x00 };

    UART_Write(cmd, sizeof(cmd));
    
    info->shield = 0;
}