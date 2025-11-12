
#include "task.hpp"
#include "Motor.hpp"
#include "Controller.hpp"
#include "can.h"
#include "imu.h"
#include "cmsis_os2.h"
#include "iwdg.h"

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

Motor gimbal_yaw_motor(3, Motor::MotorType::GM6020, 1.f, m6020_1_4_tx_data,
    PID(35, 0.4, 100, 50, 3000, 0.05),
    PID(0.0012f, 0, 0.0006f, 0, 1.5f, 0.03));
Motor gimbal_pitch_motor(1, Motor::MotorType::GM6020, 1.f, m6020_1_4_tx_data,
    PID(35, 0.4, 100, 50, 3000, 0.05),
    PID(0.0012f, 0, 0.0006f, 0, 1.5f, 0.03));


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


[[noreturn]] void main_task(void* params) {
    while (true) {
        HAL_IWDG_Refresh(&hiwdg);
        // 根据遥控器控制电机
        const auto ctrl_frame = rc.data;
        // 急停 + 速度控制
        stop_flag = ctrl_frame.S2_ == Controller::Position::DOWN;
        float yaw_spd = ctrl_frame.left_joystick_x_ * 1000;
        float pitch_spd = ctrl_frame.left_joystick_y_ * 1000;
        // 死区设置
        if (yaw_spd > -100 && yaw_spd < 100) {
            yaw_spd = 0;
        }
        if ( pitch_spd > -100 && pitch_spd < 100 ) {
            pitch_spd = 0;
        }
        // Pitch轴软件限位
        if ((imu.euler_deg_.pitch < -30 && pitch_spd < 0) || (imu.euler_deg_.pitch > 30 && pitch_spd > 0) ) {
            pitch_spd = 0;
        }
        gimbal_yaw_motor.set_speed(yaw_spd);
        gimbal_pitch_motor.set_speed(pitch_spd);

        // 发送Can包
        gimbal_yaw_motor.handle();
        gimbal_pitch_motor.handle();
        HAL_CAN_AddTxMessage(&hcan1, &m6020_1_4_tx_header, m6020_1_4_tx_data, &can_tx_mailbox);
        osDelay(1);
    }
}

[[noreturn]] void feedward_task(void* params) {
    gimbal_pitch_motor.init(-30);
    gimbal_pitch_motor.set_position(-30);
    int16_t i = -30;
    int32_t cnt = 0;
    bool up = true;
    while (true) {
        HAL_IWDG_Refresh(&hiwdg);
        
        // 根据遥控器控制电机
        const auto ctrl_frame = rc.data;
        // 急停 + 速度控制
        stop_flag = false;
        
        gimbal_yaw_motor.set_intensity(0);
        if (cnt % 3000 == 0) {
            if (up) {
                if (++i == 30) {
                    up = false;
                }
            }
            else {
                if (--i == -30) {
                    up = true;
                }
            }
            gimbal_pitch_motor.set_position(i);
        }
        gimbal_pitch_motor.handle();
        ++cnt;
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
    mainTaskHandle = osThreadNew(feedward_task, nullptr, &mainTask_attributes);
    //imuTaskHandle = osThreadNew(imu_task, nullptr, &imuTask_attributes);
}



