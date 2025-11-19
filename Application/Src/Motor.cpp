
#include "Motor.hpp"
#include "algorithm.hpp"
#include "PID.hpp"
#include <can.h>
#include <cmath>

bool stop_flag = true;

Motor::Motor(uint8_t __can_id, MotorType __type, bool inverse, float __ratio, uint8_t *const __tx_data, PID&& __ppid, PID&& __spid):
can_id_(__can_id), type_(__type),ratio_(__ratio), inverse(inverse), tx_addr_(__tx_data), 
control_method_(ControlMethod::POSITION_SPEED),
ppid_(__ppid),
spid_(__spid) {}

void Motor::init(float init_angle) {
    fdb_angle_ = init_angle;
    target_angle_ = init_angle;
}

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
    if (inverse) {
        delta_angle_ = -delta_angle_;
    }
    fdb_angle_ += delta_angle_;
    if (fdb_angle_ > 180) {
        fdb_angle_ -= 360;
    }
    if (fdb_angle_ < -180) {
        fdb_angle_ += 360;
    }

    // Get other information
    fdb_speed_ = static_cast<int16_t>(rx_data[2] << 8 | rx_data[3]);
    if (inverse) {
        fdb_speed_ = -fdb_speed_;
    }
    current_ = linear_mapping(static_cast<int16_t>(rx_data[4] << 8 | rx_data[5]), -16384, 16384, -20, 20);
    if (inverse) {
        current_ = -current_;
    }
    temp_ = static_cast<float>(rx_data[6]);
}

void Motor::set_position(float target_position) {
    target_angle_ = target_position;
    control_method_ = ControlMethod::POSITION_SPEED;
    //ppid_.reset();
    //spid_.reset();
}

void Motor::set_speed(float target_speed) {
    target_speed_ = target_speed;
    control_method_ = ControlMethod::SPEED;
    //ppid_.reset();
    //spid_.reset();
}

void Motor::set_intensity(float intensity) {
    output_intensity_ = intensity;
    control_method_ = ControlMethod::TORQUE;
    //ppid_.reset();
    //spid_.reset();
}

void Motor::set_forward_intensity(float f_intensity) {
    feedforward_intensity_ = f_intensity;
}

int16_t Motor::intensity_to_command() const {
    switch (type_) {
        case MotorType::M3508: {
            return clamp<int16_t>(output_intensity_ / 20 * 16384, -16384, 16384);
        }
        case MotorType::GM6020: {
            return clamp<int16_t>(output_intensity_ / 3 * 16384, -16384, 16384);
        }
        default: {
            return 0;
        }
    }
}

void Motor::handle() {
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
            if (target_angle_ - fdb_angle_ > 180) {
                target_angle_ -= 360;
            }
            else if (target_angle_ - fdb_angle_ < -180) {
                target_angle_ += 360;
            }
            target_speed_ = ppid_.calc(target_angle_, fdb_angle_) + feedforward_speed_;
            // 速度环
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
        }
    }
    int16_t output_command = intensity_to_command();
    // Protection
    if (stop_flag || fabsf(fdb_speed_) > 3000) {
        output_command = 0;
    }
    if (inverse) {
        output_command = -output_command;
    }
    // CAN Message
    uint8_t high_byte = output_command >> 8;
    uint8_t low_byte = output_command & 0x00FF;
    if (can_id_ < 5) {
        tx_addr_[2 * can_id_ - 2] = high_byte;
        tx_addr_[2 * can_id_ - 1] = low_byte;
    }
    else {
        tx_addr_[2 * can_id_ - 10] = high_byte;
        tx_addr_[2 * can_id_ - 9] = low_byte;
    }
}
