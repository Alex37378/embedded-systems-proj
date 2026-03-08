#include <xc.h>
#include <stdint.h>
#include "pcu_api.h"
#include "util.h"
#include "main.h"
#include "rfid_seq.h"
#include <stdbool.h>

int currentZone = -1;

//idk how many zones
int numZones = 6;

//samples IR receiver output
int sample_IR_outp () {

    currentZone = -1;
    int i;

    //If receiver output is low,
    if(PORTAbits.RA2 == 0)
    {
        PCU_UserData_t * flysk = PCU_GetUserData();
        uint32_t dial_a = flysk -> pot_vra;

        int range = 1000 / numZones;
        int start = 1000;
        int end = start + range;

        for (i =1; i< numZones+1; i++) {
            if (dial_a >= start && dial_a < end) {
                currentZone = i;
                break;
            }
            start = end + 1;
            end = start + range;
        }
    }
    return currentZone;
}