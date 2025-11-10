//
// Created by a2005 on 2025/10/18.
//

#include "PID.hpp"
#include "../../Algorithm/Inc/algorithm.hpp"

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k) {
    kp_ = kp, ki_ = ki, kd_ = kd;
    i_max_ = i_max, out_max_ = out_max;
    d_filter_k_ = d_filter_k;

    this->reset();
}
// Mutable Parameters
void PID::reset() {
    output_ = 0;
    ref_ = fdb_ = 0;
    err_ = err_sum_ = last_err_ = 0;
    pout_ = iout_ = dout_ = last_dout_ = 0;
}

float PID::calc(float ref, float fdb) {
    // Update Status
    ref_ = ref, fdb_ = fdb;
    last_err_ = err_;
    err_ = ref_ - fdb_;
    err_sum_ += err_;
    last_dout_ = dout_;

    // Calculate Output
    pout_ = err_ * kp_;
    iout_ = clamp<float>(err_sum_ * ki_, -i_max_, i_max_);
    dout_ = (err_ - last_err_) * kd_;
    // Apply D-Filter
    dout_ = last_dout_ + d_filter_k_ * (dout_ - last_dout_);

    output_ = clamp<float>(pout_ + iout_ + dout_, -out_max_, out_max_);
    return output_;
}
