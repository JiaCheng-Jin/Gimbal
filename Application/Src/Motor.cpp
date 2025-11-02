
#include "Motor.hpp"

#include "Motor.hpp"
#include "PID.hpp"
#include <can.h>
#include <cmath>

extern uint32_t can_tx_mailbox;
extern uint8_t tx_data[8];
extern CAN_TxHeaderTypeDef tx_header;
bool stop_flag = true;

inline float linear_mapping(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template<class T>
T clamp(const T& value, const T& min, const T& max) {
    return value < min ? min : (value > max ? max : value);
}

Motor::Motor(uint8_t can_id, MotorType type, float ratio):
can_id_(can_id), type_(type),ratio_(ratio), control_method_(ControlMethod::POSITION_SPEED),
ppid_(198, 0.4, 57, 30, 3000, 0.05),
spid_(0.022f, 0, 0.011f, 0, 2, 0.03) {}


void Motor::parse_can_msg_callback(const uint8_t rx_data[8]) {
    // Get Current EncoderAngle, Map to [0, 360)
    ecd_angle_ = linear_mapping(static_cast<float>(rx_data[0] << 8 | rx_data[1]), 0, 8191, 0, 360);
    // 第一次回调 -> delta为0
    if (init_) {
        init_ = false;
        last_ecd_angle_ = ecd_angle_;
    }
    // Get DeltaEncoderAngle, range [-180, 180)
    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle_ < -180) {
        delta_ecd_angle_ += 360;
    }
    if (delta_ecd_angle_ > 180) {
        delta_ecd_angle_ -= 360;
    }
    // Accumulate Actual Angle
    last_ecd_angle_ = ecd_angle_;
    delta_angle_ = delta_ecd_angle_ / ratio_;
    fdb_angle_ += delta_angle_;
    if (fdb_angle_ > 360) {
        fdb_angle_ -= 360;
    }
    if (fdb_angle_ < 0) {
        fdb_angle_ += 360;
    }

    // Get other information
    fdb_speed_ = static_cast<int16_t>(rx_data[2] << 8 | rx_data[3]);
    current_ = linear_mapping(static_cast<int16_t>(rx_data[4] << 8 | rx_data[5]), -16384, 16384, -20, 20);
    temp_ = static_cast<float>(rx_data[6]);
}

void Motor::set_position(float target_position) {
    target_angle_ = target_position;
    control_method_ = ControlMethod::POSITION_SPEED;
    ppid_.reset();
    spid_.reset();
}

void Motor::set_speed(float target_speed) {
    target_speed_ = target_speed;
    control_method_ = ControlMethod::SPEED;
    ppid_.reset();
    spid_.reset();
}

void Motor::set_intensity(float intensity) {
    output_intensity_ = intensity;
    control_method_ = ControlMethod::TORQUE;
    ppid_.reset();
    spid_.reset();
}

float Motor::feedforward_intensity_calc(float current_angle) {
    float torque = 0.5f * 9.8 * 0.05524 * cosf((current_angle - 90) * 3.1415 / 180);
    float current = torque / 3 * 8;
    return current * 16384 / 20;
}

void Motor::handle() {
    feedforward_intensity_ = feedforward_intensity_calc(fdb_angle_);
    switch (control_method_) {
        case ControlMethod::TORQUE: {
            break;
        }
        case ControlMethod::SPEED: {
            // 速度环
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
            break;
        }
        case ControlMethod::POSITION_SPEED: {
            // 位置环
            target_speed_ = ppid_.calc(target_angle_, fdb_angle_) + feedforward_speed_;
            // 速度环
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
        }
    }
    output_intensity_ = clamp<int16_t>(static_cast<int16_t>(output_intensity_), -16384, 16384);
    // Protection
    if (stop_flag || fabsf(fdb_speed_) > 6000) {
        output_intensity_ = 0;
    }
    // CAN Message
    uint8_t high_byte = static_cast<int16_t>(output_intensity_) >> 8;
    uint8_t low_byte = static_cast<int16_t>(output_intensity_) & 0x00FF;
    tx_data[0] = tx_data[2] = tx_data[4] = tx_data[6] = high_byte;
    tx_data[1] = tx_data[3] = tx_data[5] = tx_data[7] = low_byte;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mailbox);
}
