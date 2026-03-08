#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>

const PCU_Info_t *PCU_GetInfo(void);
void UART_Write(const uint8_t *buf, uint8_t len);
uint8_t UART_Read(void);
const PCU_UserData_t *PCU_GetUserData(void);
void PCU_SetMotor(const MotorSettings_t *motor);
int main(void);

#endif