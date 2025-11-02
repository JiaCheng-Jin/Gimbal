//
// Created by a2005 on 2025/10/12.
//

#ifndef INC_1012_SPI_BMI088_HPP
#define INC_1012_SPI_BMI088_HPP

#include <SPI.h>
#include <gpio.h>

#define BMI088_ACC_CS_HIGH() HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);

#define BMI088_ACC_CS_LOW() HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);

#define BMI088_GYRO_CS_HIGH() HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);

#define BMI088_GYRO_CS_LOW() HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);

#define CONSTRAIN(x, MIN, MAX) (x = (x < MIN) ? MIN : ((x > MAX) ? MAX : x))

void BMI088_WriteByte(uint8_t tx_data);

void BMI088_ReadByte(uint8_t* rx_data, uint8_t length);

void BMI088_WriteReg(uint8_t reg_addr, uint8_t data);

void bmi088_accel_write_single_reg(uint8_t reg_addr, uint8_t data);
// 加速度计读取
void bmi088_accel_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length);

void bmi088_gyro_write_single_reg(uint8_t reg_addr, uint8_t data);

void bmi088_gyro_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length);

void bmi088_init();

class IMU {
private:
    uint8_t rx_acc_data[6] = { 0 };
    uint8_t rx_gyro_data[6] = { 0 };
    uint8_t acc_range = 0, gyro_range = 0;
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float yaw, pitch, roll;

public:
    explicit IMU() {
        acc_x = acc_y = acc_z = 0;
        gyro_x = gyro_y = gyro_z = 0;
        yaw = pitch = roll = 0;
    }
    void acc_calculate();
    void gyro_calculate();
    void update_orientation(uint16_t ms);
};

#endif //INC_1012_SPI_BMI088_HPP