#ifndef ____MPU6050_SENSOR_H
#define ____MPU6050_SENSOR_H

#include "ergo_config.h"

typedef enum {
    SENSOR_CLOCK_ACTIVE     = 0,
    SENSOR_CLOCK_CYCLE,
    SENSOR_CLOCK_SLEEP
} sensor_clock_mode_t;


typedef enum {
    FS_0                    = 0,
    FS_1                    = 1,
    FS_2                    = 2,
    FS_3                    = 3
} full_scale_select_t;


void setup_mpu6050(full_scale_select_t fs, full_scale_select_t afs);
void sleep_mpu6050(void);
void wakeup_mpu6050_cycle_mode(void);
void wakeup_mpu6050_full_mode(void);
void mpu6050_read_sensor(u08 * buffer, u08 length);
void mpu6050_set_gyro_fullscale(full_scale_select_t fs);
void mpu6050_set_accel_fullscale(full_scale_select_t afs);

#endif /* ____MPU6050_SENSOR_H */

