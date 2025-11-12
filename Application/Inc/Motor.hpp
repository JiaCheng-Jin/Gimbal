#ifndef C_BOARD_MOTOR_HPP
#define C_BOARD_MOTOR_HPP

#include <cstdint>
#include "PID.hpp"

class Motor {
public:
    enum class MotorType{ M3508 = 0, GM6020 = 1 };
    enum class ControlMethod { TORQUE = 0, SPEED = 1, POSITION_SPEED = 2 };
    const uint8_t can_id_;
    
private:
    const MotorType type_;
    const float ratio_;
    uint8_t *const tx_addr_;
    ControlMethod control_method_;
    PID ppid_, spid_;

    float delta_angle_ = 0;
    float ecd_angle_ = 0;
    float last_ecd_angle_ = 0;
    float delta_ecd_angle_ = 0;
    float current_ = 0;
    float temp_ = 0;
    bool init_ = true; // 第一次读取时，delta应为0

    float target_angle_ = 0, fdb_angle_ = 0;
    float target_speed_ = 0, fdb_speed_ = 0, feedforward_speed_ = 0;
    float feedforward_intensity_ = 0, output_intensity_ = 0;

public:
    Motor() = delete;
    explicit Motor(uint8_t __can_id, MotorType __type, float __ratio, uint8_t* __tx_data, PID&& __ppid, PID&& __spid);
    void init(float init_angle);
    void set_position(float target_position);
    void set_speed(float target_speed);
    void set_intensity(float intensity);
    void set_forward_intensity(float f_intensity);
    int16_t intensity_to_command() const;
    void parse_can_msg_callback(const uint8_t rx_data[8]);
    void handle();
};



#endif //C_BOARD_MOTOR_HPP