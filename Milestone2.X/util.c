#include "util.h"
#include "pcu_api.h"

uint16_t scale_u8_to_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

uint8_t scale_u16_to_u8(uint16_t val, uint16_t lo, uint16_t hi)
{
    return (uint8_t)((uint32_t)(val - lo) * 255u / (hi - lo));
}

uint8_t scale_u16_to_pwm(uint16_t val, uint16_t lo, uint16_t hi)
{
    if (val <= lo) return MOTOR_PWM_MIN;
    if (val >= hi) return MOTOR_PWM_MAX;
    
    uint8_t raw = (uint8_t)((uint32_t)scale_u16_to_u8(val, lo, hi) * MOTOR_PWM_MAX / 255u);
    
    return (uint8_t)(MOTOR_PWM_MIN + (uint32_t)raw * (MOTOR_PWM_MAX - MOTOR_PWM_MIN) / MOTOR_PWM_MAX);
}
