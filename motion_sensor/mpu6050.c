/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx.h"
#include "i2c.h"
#include "mpu6050.h"
#include "ergo_config.h"

/*lint ++flb "Enter library region" */

#define USE_I2C_DMA             1

#define ADDRESS_WHO_AM_I          (0x75U) // !< WHO_AM_I register identifies the device. Expected value is 0x68.
#define ADDRESS_SIGNAL_PATH_RESET (0x68U) // !<

typedef HAL_StatusTypeDef ret_code_t;

#if defined(BOARD_TINYBLE)
static const uint8_t expected_who_am_i = 0x69U; // !< Expected value to get from WHO_AM_I register.
#else
static const uint8_t expected_who_am_i = 0x68U; // !< Expected value to get from WHO_AM_I register.
#endif
static uint8_t       m_device_address;          // !< Device address in bits [7:1]



bool twi_master_init(void)
{
    MX_I2C1_Init();
    return true;
}


bool mpu6050_init(uint8_t device_address)
{
    bool transfer_succeeded = true;

    m_device_address = device_address; // (uint8_t)(device_address << 1);

    // Do a reset on signal paths
    uint8_t reset_value = 0x04U | 0x02U | 0x01U; // Resets gyro, accelerometer and temperature sensor signal paths.
    transfer_succeeded &= mpu6050_register_write(ADDRESS_SIGNAL_PATH_RESET, reset_value);

    // Read and verify product ID
    transfer_succeeded &= mpu6050_verify_product_id();

    return transfer_succeeded;
}

bool mpu6050_verify_product_id(void)
{
    uint8_t who_am_i;

    if (mpu6050_register_read(ADDRESS_WHO_AM_I, &who_am_i, 1)) {
        if (who_am_i != expected_who_am_i) {
            return false;
        }
        else {
            return true;
        }
    }
    else {
        return false;
    }
}

bool mpu6050_register_write(uint8_t register_address, uint8_t value)
{
    ret_code_t ret;

#if USE_I2C_DMA
    i2c1_transmit_complete = false;
    ret = HAL_I2C_Mem_Write_DMA(&hi2c1, m_device_address, register_address, I2C_MEMADD_SIZE_8BIT, &value, 1);
    while (!i2c1_transmit_complete) {
        __WFE();
    }
#else
    ret = HAL_I2C_Mem_Write(&hi2c1, m_device_address, register_address, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
#endif

    return (ret == HAL_OK);

// HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
}

bool mpu6050_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    ret_code_t ret;

#if USE_I2C_DMA
    i2c1_receive_complete = false;
    ret = HAL_I2C_Mem_Read_DMA(&hi2c1, m_device_address, register_address, I2C_MEMADD_SIZE_8BIT, destination, number_of_bytes);
    while (!i2c1_receive_complete) {
        __WFE();
    }
#else
    ret = HAL_I2C_Mem_Read(&hi2c1, m_device_address, register_address, I2C_MEMADD_SIZE_8BIT, destination, number_of_bytes, 100);
#endif

    return (ret == HAL_OK);

// HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
}

/*lint --flb "Leave library region" */
