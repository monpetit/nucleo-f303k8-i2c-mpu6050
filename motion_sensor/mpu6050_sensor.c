#include "mpu6050.h"
#include "mpu6050_registers.h"
#include "mpu6050_sensor.h"

void setup_mpu6050(full_scale_select_t fs, full_scale_select_t afs)
{
    if (twi_master_init()) {
        if (mpu6050_init(MPU6050_I2C_ADDRESS)) {
            mpu6050_set_gyro_fullscale(fs);
            mpu6050_set_accel_fullscale(afs);
        }
    }
}

void sleep_mpu6050(void)
{
    mpu6050_register_write(MPU6050_PWR_MGMT_1, _BV(MPU6050_SLEEP));      // sleep mode
}


void wakeup_mpu6050_cycle_mode(void)
{
    // clock select: 0, disable temperature sensor, cycle mode(sample and sleep)
    mpu6050_register_write(MPU6050_PWR_MGMT_1, _BV(MPU6050_CLKSEL0) | _BV(MPU6050_TEMP_DIS) | _BV(MPU6050_CYCLE));

    // LP_WAKE_CTRL: wakeup frequency 20 Hz
    mpu6050_register_write(MPU6050_PWR_MGMT_2, _BV(MPU6050_LP_WAKE_CTRL1));
}


void wakeup_mpu6050_full_mode(void)
{
#define CLKSEL_INTOSC_8MHZ      0
    mpu6050_register_write(MPU6050_PWR_MGMT_1, CLKSEL_INTOSC_8MHZ);
}


void mpu6050_read_sensor(u08 * buffer, u08 length)
{
    mpu6050_register_read(MPU6050_ACCEL_XOUT_H, buffer, length);
}

void mpu6050_set_gyro_fullscale(full_scale_select_t fs)
{
    // Gyroscope Configuration (GYRO_CONFIG)
    mpu6050_register_write(MPU6050_GYRO_CONFIG, ((u08)fs << 3));
}

void mpu6050_set_accel_fullscale(full_scale_select_t afs)
{
    // Accelerometer Configuration (ACCEL_CONFIG)
    mpu6050_register_write(MPU6050_ACCEL_CONFIG, ((u08)afs << 3));
}
