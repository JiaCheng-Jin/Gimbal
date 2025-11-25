
#include "task.hpp"
#include "Motor.hpp"
#include "Controller.hpp"
#include "can.h"
#include "imu.h"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "algorithm.hpp"

#include <cmath>

extern Controller rc;
extern bool stop_flag;

uint32_t can_tx_mailbox;
uint8_t can_rx_data[8];
uint8_t m6020_1_4_tx_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t m6020_5_8_tx_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
CAN_TxHeaderTypeDef m6020_1_4_tx_header = {   .StdId = 0x1FE,
                                              .ExtId = 0x000,
                                              .IDE = CAN_ID_STD,
                                              .RTR = CAN_RTR_DATA,
                                              .DLC = 8,
                                              .TransmitGlobalTime = DISABLE };
CAN_TxHeaderTypeDef m6020_5_8_tx_header = {   .StdId = 0x2FE,
                                              .ExtId = 0x000,
                                              .IDE = CAN_ID_STD,
                                              .RTR = CAN_RTR_DATA,
                                              .DLC = 8,
                                              .TransmitGlobalTime = DISABLE };

Motor gimbal_yaw_motor(1, Motor::MotorType::GM6020, false, 1.f, m6020_1_4_tx_data,
    PID(8, 0.1, 350, 40, 320, 0.05),
    PID(0.005f, 0, 0, 0, 2.5f, 0.03));
Motor gimbal_pitch_motor(4, Motor::MotorType::GM6020, true, 1.f, m6020_1_4_tx_data,
    PID(12, 0.2, 300, 50, 320, 0.05),
    PID(0.005f, 0, 0, 0, 2.5f, 0.03));

/* Pitch轴前馈计算函数 */
float gimbal_pitch_feedforward(float angle) {
		// For Old Gimbal
    // return 2.3769f * sinf(0.0262 * angle - 0.6386) + 2.2238 * sinf(0.0331 * angle + 2.7408); 
		// For New Gimbal
		return 0.5569 * sinf(0.0290 * angle - 2.081);
}

osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
    .name = "mainTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };

/* IMU Task 相关变量 */
float r_imu[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float gyro_bias[3] = {0, 0, 0};
IMU imu(0.001, 0.5, 1, r_imu, gyro_bias);
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
    .name = "IMUTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };


/* Pitch 限位[-30, 30] */
[[noreturn]] void main_task(void* params) {
    gimbal_pitch_motor.init(34);
    gimbal_pitch_motor.bind_feedforward_func(&gimbal_pitch_feedforward);
    while (true) {
        HAL_IWDG_Refresh(&hiwdg);
        // 根据遥控器控制电机
        const auto ctrl_frame = rc.data;
        // 急停 + 速度控制
        stop_flag = ctrl_frame.S2_ == Controller::Position::DOWN;
        float yaw_pos = ctrl_frame.right_joystick_x_ * 180;
        float pitch_pos = ctrl_frame.right_joystick_y_ * 30;
        // 死区设置
        if (yaw_pos > -2 && yaw_pos < 2) {
            yaw_pos = 0;
        }
        if ( pitch_pos > -2 && pitch_pos < 2) {
            pitch_pos = 0;
        }
        // Pitch轴软件限位
        pitch_pos = clamp<float>(pitch_pos, -30, 30);
        gimbal_yaw_motor.set_position(yaw_pos);
        gimbal_pitch_motor.set_position(pitch_pos);

        // 发送Can包
        gimbal_yaw_motor.handle();
        gimbal_pitch_motor.handle();
        HAL_CAN_AddTxMessage(&hcan1, &m6020_1_4_tx_header, m6020_1_4_tx_data, &can_tx_mailbox);
        osDelay(1);
    }
}

[[noreturn]] void test_task(void* params) {
    gimbal_pitch_motor.init(34);
    gimbal_pitch_motor.bind_feedforward_func(&gimbal_pitch_feedforward);
    gimbal_pitch_motor.set_position(0);
    stop_flag = false;
    while (true) {
        HAL_IWDG_Refresh(&hiwdg);
        // 发送Can包
        gimbal_yaw_motor.handle();
        gimbal_pitch_motor.handle();
        HAL_CAN_AddTxMessage(&hcan1, &m6020_1_4_tx_header, m6020_1_4_tx_data, &can_tx_mailbox);
        osDelay(1);
    }
}


[[noreturn]] void imu_task(void* params) {
    while (true) {
        imu.readSensor();
        imu.update_mahony();
        osDelay(1);
    }
}

void register_tasks() {
    mainTaskHandle = osThreadNew(test_task, nullptr, &mainTask_attributes);
    imuTaskHandle = osThreadNew(imu_task, nullptr, &imuTask_attributes);
}



