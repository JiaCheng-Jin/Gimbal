#ifndef BMI088_H
#define BMI088_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

// chip selection
    #define BMI088_ACC_CS_HIGH() HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);

    #define BMI088_ACC_CS_LOW() HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);

    #define BMI088_GYRO_CS_HIGH() HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);

    #define BMI088_GYRO_CS_LOW() HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);

// bmi088 init
void bmi088_init(void);

// bmi088 read/write
void bmi088_write_byte(uint8_t tx_data);
void bmi088_write_reg(uint8_t reg, uint8_t data);
void bmi088_read_byte(uint8_t *rx_data, uint8_t length);

void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif // BMI088_H
