#ifndef PCU_API_H
#define PCU_API_H

#include <stdint.h>

#define PCU_SYNC1  0xFE
#define PCU_SYNC2  0x19

//MotorDir_t type can only be 0,1,2
typedef enum {
    MotorDir_Brake    = 0,
    MotorDir_Forward  = 1,
    MotorDir_Backward = 2
} MotorDir_t;


// =========== Important structs ============

//struct which holds information abt general team info
typedef struct {
    uint8_t  team;
    uint8_t  player;
    uint16_t health;
    uint8_t  shield;
    uint8_t  repair;
} PCU_Info_t;

//struct which holds information abt all 10 channels, including the joysticks
typedef struct {
    uint16_t right_x;
    uint16_t right_y;
    uint16_t left_y;
    uint16_t left_x;
    uint16_t switch_a;
    uint16_t switch_b;
    uint16_t switch_c;
    uint16_t switch_d;
    uint16_t pot_vra;
    uint16_t pot_vrb;
} PCU_UserData_t;

//struct which holds info abt motor direction and speed for the two motors
typedef struct {
    uint8_t dirA;
    uint8_t pwmA;
    uint8_t dirB;
    uint8_t pwmB;
} MotorSettings_t;


// ============= PCU API ================
const PCU_Info_t     *PCU_GetInfo(void);
#define PCU_GET_INFO_RESPONSE_LEN    6u

const PCU_UserData_t *PCU_GetUserData(void);
#define PCU_GET_USER_DATA_RESPONSE_LEN     20u

void PCU_SetMotor(const MotorSettings_t *motor);
#define MOTOR_PWM_MIN  5 // We are deliberately starting at 5 as the lower value instead of 0 since anything smaller is unnoticable
#define MOTOR_PWM_MAX  100

#endif
