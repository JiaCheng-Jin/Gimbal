//
// Created by a2005 on 2025/10/12.
//

#include "BMI088.hpp"
#include <cmath>

#define PI 3.14159265358979

void BMI088_WriteByte(uint8_t tx_data) {
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX)
        ;
}

void BMI088_ReadByte(uint8_t* rx_data, uint8_t length) {
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX)
        ;
}

void BMI088_WriteReg(uint8_t reg_addr, uint8_t data) {
    BMI088_WriteByte(0x7F & reg_addr);
    BMI088_WriteByte(data);
}

void bmi088_accel_write_single_reg(uint8_t reg_addr, uint8_t data) {
    BMI088_GYRO_CS_HIGH();
    BMI088_ACC_CS_LOW();

    BMI088_WriteReg(reg_addr, data);

    BMI088_ACC_CS_HIGH();
}
// 加速度计读取
void bmi088_accel_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length) {
    BMI088_GYRO_CS_HIGH();
    BMI088_ACC_CS_LOW();

    BMI088_WriteByte(0x80 | reg);
    // Discard Dummy Byte
    BMI088_ReadByte(rx_data, 1);
    BMI088_ReadByte(rx_data, length);

    BMI088_ACC_CS_HIGH();
}

void bmi088_gyro_write_single_reg(uint8_t reg_addr, uint8_t data) {
    BMI088_ACC_CS_HIGH();
    BMI088_GYRO_CS_LOW();

    BMI088_WriteReg(reg_addr, data);

    BMI088_GYRO_CS_HIGH();
}

void bmi088_gyro_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length) {
    BMI088_ACC_CS_HIGH();
    BMI088_GYRO_CS_LOW();

    BMI088_WriteByte(0x80 | reg);
    BMI088_ReadByte(rx_data, length);

    BMI088_GYRO_CS_HIGH();
}

void bmi088_init() {
    // Soft Reset ACCEL
    BMI088_ACC_CS_LOW();
    BMI088_WriteReg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
    HAL_Delay(10);
    BMI088_ACC_CS_HIGH();

    // Soft Reset GYRO
    BMI088_GYRO_CS_LOW();
    BMI088_WriteReg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
    HAL_Delay(100);
    BMI088_WriteReg(0x10, 0x82);
    HAL_Delay(10);
    BMI088_GYRO_CS_HIGH();

    // Switch ACCEL to Normal Mode
    BMI088_ACC_CS_LOW();
    HAL_Delay(10);
    BMI088_WriteReg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    HAL_Delay(10);
    BMI088_ACC_CS_HIGH();
}

void IMU::acc_calculate() {
    static float g = 9.8;
    // 1. 设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
    bmi088_accel_read_reg(0x41, &acc_range, 1);
    // code here

    // 2. 读取acc0x12寄存器中的6位acc数据
    bmi088_accel_read_reg(0x12, rx_acc_data, 6);

    // 3. 用量程系数将原始数据转换为常用单位
    // code here
    int16_t acc_x_raw = rx_acc_data[1] << 8 | rx_acc_data[0];
    int16_t acc_y_raw = rx_acc_data[3] << 8 | rx_acc_data[2];
    int16_t acc_z_raw = rx_acc_data[5] << 8 | rx_acc_data[4];

    acc_x = acc_x_raw / 32768.f * (1 << (acc_range + 1)) * 1.5 * g;
    acc_y = acc_y_raw / 32768.f * (1 << (acc_range + 1)) * 1.5 * g;
    acc_z = acc_z_raw / 32768.f * (1 << (acc_range + 1)) * 1.5 * g;
}

void IMU::gyro_calculate() {
    // 1. 设置/读取gyro0x0F寄存器中的量程range参数，并换算为量程系数
    // 2. 读取gyro0x02寄存器中的6位gyro数据
    // 3. 用量程系数将原始数据转换为常用单位
    bmi088_gyro_read_reg(0x0F, &gyro_range, 1);
    float resolution = 61.f / (1 << gyro_range);

    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);

    int16_t gyro_x_raw = rx_gyro_data[1] << 8 | rx_gyro_data[0];
    int16_t gyro_y_raw = rx_gyro_data[3] << 8 | rx_gyro_data[2];
    int16_t gyro_z_raw = rx_gyro_data[5] << 8 | rx_gyro_data[4];

    gyro_x = gyro_x_raw * resolution / 1000;
    gyro_y = gyro_y_raw * resolution / 1000;
    gyro_z = gyro_z_raw * resolution / 1000;
}

void IMU::update_orientation(uint16_t ms) {
    static float K = 0.8;
    float roll_a = atan2f(acc_y, acc_z);
    float pitch_a = -atan2f(acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z));
    float roll_g = gyro_x + (sinf(pitch * PI / 180) * sinf(roll * PI / 180) / cosf(pitch * PI / 180)) * gyro_y
        + (cosf(roll * PI / 180) * sinf(pitch * PI / 180) * cosf(pitch * PI / 180)) * gyro_z;
    float pitch_g = cosf(roll * PI / 180) * gyro_y - sinf(roll * PI / 180) * gyro_z;
    float yaw_g = sinf(roll * PI / 180) / cosf(pitch * PI / 180) * gyro_y
        - cosf(roll * PI / 180) / cosf(pitch * PI / 180) * gyro_z;

    roll += (roll_a - roll_g) * ms / 1000 * K;
    roll = fmodf(roll, 360.f);
    pitch += (pitch_a - pitch_g) * ms / 1000 * K;
    yaw += yaw_g * ms / 1000 * K;

    roll = fmodf(roll, 360.f);
    if (roll < 0) {
        roll += 360.f;
    }
    pitch = fmodf(pitch, 360.f);
    if (pitch < 0) {
        pitch += 360.f;
    }
    yaw = fmodf(yaw, 360.f);
    if (yaw < 0) {
        yaw += 360.f;
    }
}
