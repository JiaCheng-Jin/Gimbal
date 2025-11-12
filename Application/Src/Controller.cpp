//
// Created by a2005 on 2025/10/25.
//

#include "Controller.hpp"
#include <cstring>

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

inline float linear_mapping(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Controller::Controller():
    rx_data { 0 },
    data { 0, 0, 0, 0, Position::DOWN, Position::DOWN }
    {}

bool Controller::test_connection() const {
    uint32_t now = HAL_GetTick();
    return (now - last_tick_ < 10000);
}

void Controller::handle() {
    uint16_t ch0 = ((int16_t)rx_data[0] | ((int16_t)rx_data[1] << 8)) & 0x07FF;
    uint16_t ch1 = (((int16_t)rx_data[1] >> 3) | ((int16_t)rx_data[2] << 5)) & 0x07FF;
    uint16_t ch2 = (((int16_t)rx_data[2] >> 6) | ((int16_t)rx_data[3] << 2) | ((int16_t)rx_data[4] << 10)) & 0x07FF;
    uint16_t ch3 = (((int16_t)rx_data[4] >> 1) | ((int16_t)rx_data[5] << 7)) & 0x07FF;
    data.left_joystick_x_ = linear_mapping(static_cast<float>(ch2), RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, -1, 1);
    data.left_joystick_y_ = linear_mapping(static_cast<float>(ch3), RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, -1, 1);
    data.right_joystick_x_ = linear_mapping(static_cast<float>(ch0), RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, -1, 1);
    data.right_joystick_y_ = linear_mapping(static_cast<float>(ch1), RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, -1, 1);
    data.S1_ = static_cast<Position>(((rx_data[5] >> 4) & 0x000C) >> 2);
    data.S2_ = static_cast<Position>(((rx_data[5] >> 4) & 0x0003));

    last_tick_ = HAL_GetTick();
}
