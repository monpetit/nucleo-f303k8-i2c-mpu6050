
#include "motion_sensor.h"
#include <limits.h>

motion_sensor_t sensor;

/*
motion_sensor_t::motion_sensor_t(void) : gyro_count(0), fs(FS_0), afs(FS_0)
{
}
*/

void motion_sensor_create(void)
{
    sensor.gyro_count = 0;
    sensor.fs = FS_0;
    sensor.afs = FS_0;
    motion_sensor_init(SENSOR_CLOCK_SLEEP);
}

void motion_sensor_init(sensor_clock_mode_t clock_mode)
{
    sensor._clock_mode = clock_mode;

    setup_mpu6050(sensor.fs, sensor.afs);

    switch ((u08)sensor._clock_mode) {
    case SENSOR_CLOCK_ACTIVE:
        wakeup_mpu6050_full_mode();
        break;
    case SENSOR_CLOCK_CYCLE:
        wakeup_mpu6050_cycle_mode();
        break;
    case SENSOR_CLOCK_SLEEP:
        sleep_mpu6050();
        break;
    }
}


void motion_sensor_mode_activate_ever(void)
{
    sensor.gyro_count = UINT_MAX;                // 4294967295
    sensor._clock_mode = SENSOR_CLOCK_ACTIVE;
    wakeup_mpu6050_full_mode();
}


void motion_sensor_mode_activate(u16 count)
{
    sensor.gyro_count = (u32)count;
    sensor._clock_mode = SENSOR_CLOCK_ACTIVE;
    wakeup_mpu6050_full_mode();
}


void motion_sensor_mode_cycle(void)
{
    sensor.gyro_count = 0;
    sensor._clock_mode = SENSOR_CLOCK_CYCLE;
    wakeup_mpu6050_cycle_mode();
}


void motion_sensor_mode_sleep(void)
{
    sensor.gyro_count = 0;
    sensor._clock_mode = SENSOR_CLOCK_SLEEP;
    sleep_mpu6050();
}


sensor_clock_mode_t motion_sensor_get_mode(void)
{
    return sensor._clock_mode;
}


u32 motion_sensor_countdown(void)
{
    u32 gcnt = sensor.gyro_count;

    if (sensor.gyro_count > 0)
        sensor.gyro_count--;

    return gcnt;
}

/**
** u08 buffer[14], length: 14
**/
void motion_sensor_read(u08 * buffer, u08 length)
{
    mpu6050_read_sensor(buffer, length);
}

/**
** s16 buffer[6]
**/
void motion_sensor_read_s16(s16 * buffer)
{
    static u08 buff[14];
    mpu6050_read_sensor(buff, sizeof(buff));

    buffer[0] = (buff[0] << 8) | buff[1];       // ax
    buffer[1] = (buff[2] << 8) | buff[3];       // ay
    buffer[2] = (buff[4] << 8) | buff[5];       // az

    buffer[3] = (buff[8] << 8) | buff[9];       // gx
    buffer[4] = (buff[10] << 8) | buff[11];     // gy
    buffer[5] = (buff[12] << 8) | buff[13];     // gz
}


/**
** s16 buffer[6]
**/
void motion_sensor_read_each(s16* ax, s16* ay, s16* az, s16* gx, s16* gy, s16* gz)
{
    static u08 buff[14];
    mpu6050_read_sensor(buff, sizeof(buff));

    *ax = (buff[0] << 8) | buff[1];       // ax
    *ay = (buff[2] << 8) | buff[3];       // ay
    *az = (buff[4] << 8) | buff[5];       // az

    *gx = (buff[8] << 8) | buff[9];       // gx
    *gy = (buff[10] << 8) | buff[11];     // gy
    *gz = (buff[12] << 8) | buff[13];     // gz
}


void motion_sensor_update_gyro_fullscale(full_scale_select_t fs)
{
    sensor.fs = fs;
    mpu6050_set_gyro_fullscale(sensor.fs);
}

void motion_sensor_update_accel_fullscale(full_scale_select_t afs)
{
    sensor.afs = afs;
    mpu6050_set_accel_fullscale(sensor.afs);
}


full_scale_select_t motion_sensor_gyro_fullscale(void)
{
    return sensor.fs;
}


full_scale_select_t motion_sensor_accel_fullscale(void)
{
    return sensor.afs;
}


