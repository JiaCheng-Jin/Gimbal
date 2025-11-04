//
// Created by a2005 on 2025/11/2.
//

#ifndef C_BOARD_ALGORITHM_HPP
#define C_BOARD_ALGORITHM_HPP

inline float linear_mapping(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template<class T>
T clamp(const T& value, const T& min, const T& max) {
    return value < min ? min : (value > max ? max : value);
}
#endif //C_BOARD_ALGORITHM_HPP
