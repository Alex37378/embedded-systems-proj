#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

uint16_t scale_u8_to_u16(const uint8_t *p);
uint8_t scale_u16_to_u8(uint16_t val, uint16_t lo, uint16_t hi);
uint8_t scale_u16_to_pwm(uint16_t val, uint16_t lo, uint16_t hi);

#endif
