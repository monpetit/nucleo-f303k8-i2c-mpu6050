#ifndef __ERGO_CONFIG_H
#define __ERGO_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t         u08;
typedef uint16_t        u16;
typedef uint32_t        u32;
typedef int8_t          s08;
typedef int16_t         s16;
typedef int32_t         s32;

#define _BV(N)      (1 << N)

#ifndef MPU6050_I2C_ADDRESS
#if defined(BOARD_TINYBLE)
#define MPU6050_I2C_ADDRESS         (0x69U<<1)           // AD0: 0(LOW) ---> 0x68, AD0: 1(HIGH) ---> 0x69
#else
#define MPU6050_I2C_ADDRESS         (0x68U<<1)           // AD0: 0(LOW) ---> 0x68, AD0: 1(HIGH) ---> 0x69
#endif
#endif /* MPU6050_I2C_ADDRESS */

#endif /* __ERGO_CONFIG_H */

