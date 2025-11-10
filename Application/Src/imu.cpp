#include "imu.h"

#include "arm_math.h"
#include "bmi088.h"
#include "cstring"

IMU::IMU(const float& dt, const float& kg, const float& g_thres, const float R_imu[3][3],const float gyro_bias[3]):
    raw_data_(ImuRawData_t {}),
    gyro_sensor_ { 0 },
    gyro_world_ { 0 },
    gyro_sensor_dps_ {0},
    gyro_world_dps_ {0},
    accel_sensor_ {0},
    accel_world_ {0},
    mahony_(dt, kg, g_thres)
{
    memcpy(R_imu_, R_imu, 9 * sizeof(float));
    memcpy(gyro_bias_, gyro_bias, 3 * sizeof(float));
}

void IMU::init(EulerAngle_t euler_deg_init) {
    

}

void IMU::readSensor() {
    uint8_t acc_range, gyro_range;
    uint8_t rx_data[6];
    // 1. 设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
    bmi088_accel_read_reg(0x41, &acc_range, 1);

    // 2. 读取acc0x12寄存器中的6位acc数据
    bmi088_accel_read_reg(0x12, rx_data, 6);
    int16_t acc_x_raw = rx_data[1] << 8 | rx_data[0];
    int16_t acc_y_raw = rx_data[3] << 8 | rx_data[2];
    int16_t acc_z_raw = rx_data[5] << 8 | rx_data[4];

    accel_sensor_[0] = acc_x_raw / 32768.f * (1 << (acc_range + 1)) * 1.5 * gravity_accel;
    accel_sensor_[1] = acc_y_raw / 32768.f * (1 << (acc_range + 1)) * 1.5 * gravity_accel;
    accel_sensor_[2] = acc_z_raw / 32768.f * (1 << (acc_range + 1)) * 1.5 * gravity_accel;

    // 读取陀螺仪数据
    bmi088_gyro_read_reg(0x0F, &gyro_range, 1);
    float resolution = 61.f / (1 << gyro_range);
    bmi088_gyro_read_reg(0x02, rx_data, 6);

    int16_t gyro_x_raw = rx_data[1] << 8 | rx_data[0];
    int16_t gyro_y_raw = rx_data[3] << 8 | rx_data[2];
    int16_t gyro_z_raw = rx_data[5] << 8 | rx_data[4];

    gyro_sensor_dps_[0] = gyro_x_raw * resolution / 1000;
    gyro_sensor_dps_[1] = gyro_y_raw * resolution / 1000;
    gyro_sensor_dps_[2] = gyro_z_raw * resolution / 1000;
    gyro_sensor_[0] = gyro_sensor_dps_[0] * PI / 180.f;
    gyro_sensor_[1] = gyro_sensor_dps_[1] * PI / 180.f;
    gyro_sensor_[2] = gyro_sensor_dps_[2] * PI / 180.f;
}

// 线性滤波
void IMU::update() {
    float K = 0.8;
    float acc_x = accel_sensor_[0], acc_y = accel_sensor_[1], acc_z = accel_sensor_[2];
    float gyro_x = gyro_sensor_[0], gyro_y = gyro_sensor_[1], gyro_z = gyro_sensor_[2];
    float yaw = euler_rad_.yaw, pitch = euler_rad_.pitch, roll = euler_rad_.roll;
    
    float roll_a = atan2f(acc_y, acc_z);
    float pitch_a = -atan2f(acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z));
    float roll_g = gyro_x + (sinf(pitch) * sinf(roll) / cosf(pitch)) * gyro_y
        + (cosf(roll) * sinf(pitch) * cosf(pitch)) * gyro_z;
    float pitch_g = cosf(roll) * gyro_y - sinf(roll) * gyro_z;
    float yaw_g = sinf(roll) / cosf(pitch) * gyro_y
        - cosf(roll) / cosf(pitch) * gyro_z;

    roll += (roll_a - roll_g) * 1 / 1000 * K;
    pitch += (pitch_a - pitch_g) * 1 / 1000 * K;
    yaw += yaw_g * 1 / 1000 * K;

    roll = fmodf(roll, 2*PI);
    if (roll < 0) {
        roll += 2*PI;
    }
    pitch = fmodf(pitch, 2*PI);
    if (pitch < 0) {
        pitch += 2*PI;
    }
    yaw = fmodf(yaw, 2*PI);
    if (yaw < 0) {
        yaw += 2*PI;
    }

    euler_rad_.yaw = yaw, euler_rad_.pitch = pitch, euler_rad_.roll = roll;
    euler_deg_.yaw = yaw / PI * 180.f, euler_deg_.pitch = pitch / PI * 180.f, euler_deg_.roll = roll / PI * 180.f;
}

