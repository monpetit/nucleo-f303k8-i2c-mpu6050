#ifndef ____MOTION_SENSOR_H
#define ____MOTION_SENSOR_H

#include "mpu6050_sensor.h"

typedef struct {
    sensor_clock_mode_t _clock_mode;
    u32 gyro_count;
    // fs_sel:  gyro  full scale range select
    // afs_sel: accel full scale range select
    full_scale_select_t fs, afs;
} motion_sensor_t;

#ifdef __cplusplus
extern "C" {
#endif

void motion_sensor_create(void);
void motion_sensor_init(sensor_clock_mode_t clock_mode);
void motion_sensor_mode_activate_ever(void);
void motion_sensor_mode_activate(u16 count);
void motion_sensor_mode_cycle(void);
void motion_sensor_mode_sleep(void);
sensor_clock_mode_t motion_sensor_get_mode(void);
u32 motion_sensor_countdown(void);
void motion_sensor_read(u08 * buffer, u08 length);
void motion_sensor_read_s16(s16 * buffer);
void motion_sensor_read_each(s16* ax, s16* ay, s16* az, s16* gx, s16* gy, s16* gz);
void motion_sensor_update_gyro_fullscale(full_scale_select_t fs);
void motion_sensor_update_accel_fullscale(full_scale_select_t afs);
full_scale_select_t motion_sensor_gyro_fullscale(void);
full_scale_select_t motion_sensor_accel_fullscale(void);

#ifdef __cplusplus
}
#endif

#endif /* ____MOTION_SENSOR_H */
