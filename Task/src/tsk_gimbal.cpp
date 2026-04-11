/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : 测试任务
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "crt_gimbal.hpp"
#include "tsk_isr.hpp"
#include "dvc_vofa.hpp"

/* Define --------------------------------------------------------------------*/
/******************************************************************************
 *                            电机相关
 ******************************************************************************/
/* PID -----------------------------------------*/
// Yaw
CascadePID::PIDParam yawOuterParam = {
    YAW_OUTER_KP,        // Kp
    YAW_OUTER_KI,        // Ki
    YAW_OUTER_KD,        // Kd
    YAW_OUTER_OUT_LIMIT, // outputLimit
    YAW_OUTER_IOUT_LIMIT // intergralLimit
};
CascadePID::PIDParam yawInnerParam = {
    YAW_INNER_KP,        // Kp
    YAW_INNER_KI,        // Ki
    YAW_INNER_KD,        // Kd
    YAW_INNER_OUT_LIMIT, // outputLimit
    YAW_INNER_IOUT_LIMIT // intergralLimit
};
LowPassFilter<fp32> yawInnerLPF(YAW_INNER_LOWPASS_FILTER_PARA);
CascadePID yawPID(yawOuterParam, yawInnerParam, nullptr, &yawInnerLPF);
// Pitch
CascadePID::PIDParam pitchOuterParam = {
    PITCH_OUTER_KP,        // Kp
    PITCH_OUTER_KI,        // Ki
    PITCH_OUTER_KD,        // Kd
    PITCH_OUTER_OUT_LIMIT, // outputLimit
    PITCH_OUTER_IOUT_LIMIT // intergralLimit
};
CascadePID::PIDParam pitchInnerParam = {
    PITCH_INNER_KP,        // Kp
    PITCH_INNER_KI,        // Ki
    PITCH_INNER_KD,        // Kd
    PITCH_INNER_OUT_LIMIT, // outputLimit
    PITCH_INNER_IOUT_LIMIT // intergralLimit
};
LowPassFilter<fp32> pitchInnerLPF(PITCH_INNER_LOWPASS_FILTER_PARA);
CascadePID pitchPID(pitchOuterParam, pitchInnerParam, nullptr, &pitchInnerLPF);
// Friction
SimplePID::PIDParam leftfrictionPIDParam = {
    FRICTION_KP,        // Kp
    FRICTION_KI,        // Ki
    FRICTION_KD,        // Kd
    FRICTION_OUT_LIMIT, // outputLimit
    FRICTION_IOUT_LIMIT // intergralLimit
};
SimplePID::PIDParam rightfrictionPIDParam = {
    FRICTION_KP,        // Kp
    FRICTION_KI,        // Ki
    FRICTION_KD,        // Kd
    FRICTION_OUT_LIMIT, // outputLimit
    FRICTION_IOUT_LIMIT // intergralLimit
};

SimplePID leftFrictionPID(SimplePID::PID_POSITION, leftfrictionPIDParam);
SimplePID rightFrictionPID(SimplePID::PID_POSITION, rightfrictionPIDParam);
// Rammer
// 使用双环PID：外环(位置) -> 内环(速度) -> 电流
CascadePID::PIDParam rammerOuterParam = {
    RAMMER_OUTER_KP,
    RAMMER_OUTER_KI,
    RAMMER_OUTER_KD,
    RAMMER_OUTER_OUT_LIMIT,
    RAMMER_OUTER_IOUT_LIMIT};
CascadePID::PIDParam rammerInnerParam = {
    RAMMER_INNER_KP,
    RAMMER_INNER_KI,
    RAMMER_INNER_KD,
    RAMMER_INNER_OUT_LIMIT,
    RAMMER_INNER_IOUT_LIMIT};
LowPassFilter<fp32> rammerInnerLPF(RAMMER_INNER_LOWPASS_FILTER_PARA);
CascadePID rammerPID(rammerOuterParam, rammerInnerParam, nullptr, &rammerInnerLPF );

/* Motor ---------------------------------------------*/

MotorGM6020 yawMotor(3, &yawPID, 0);
MotorDM4310 pitchMotor(1, 3, 3.141593f, 30, 10, &pitchPID);
MotorM2006 rammerMotor(6, &rammerPID, 0, 36);
MotorM3508 leftFrictionMotor(4, &leftFrictionPID);
MotorM3508 rightFrictionMotor(1, &rightFrictionPID);

Vofa<12> vofa;

/******************************************************************************
 *                            IMU相关
 ******************************************************************************/
// AHRS算法
Mahony ahrs(AHRS_AUTO_FREQ, AHRS_DEFAULT_FILTER, MAHONY_KP, MAHONY_KI);
// IMU校准数据
BMI088::CalibrationInfo cali = {
    {GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z},    // gyroOffset
    {ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z}, // accelOffset
    {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z},       // magnetOffset
    {INSTALL_SPIN_MATRIX}                             // installSpinMatrix
};
// IMU类定义
BMI088 imu(&ahrs, {&hspi1, GPIOA, GPIO_PIN_4}, {&hspi1, GPIOB, GPIO_PIN_0}, cali);

// gimbal
Gimbal gimbal(&yawMotor, &pitchMotor, &rammerMotor, &leftFrictionMotor, &rightFrictionMotor, &imu);

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

extern "C" void gimbal_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);       // 初始化CAN1
    UART_Init(&huart3, dr16RxCallback, 36); // 初始化DR16串口
    vofa.Init();
    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 获取任务开始时间
    gimbal.init();
    // pitchPID.setInnerLoopOutputPolarity(false);
    // pitchMotor.setControllerOutputPolarity(false);
    while (1) {
        gimbal.controlLoop();
        vofa.writeData(rammerPID.getOuterLoop().pidGetData().setPoint);
        vofa.writeData(rammerPID.getOuterLoop().pidGetData().feedBackData);
        vofa.writeData(rammerPID.getOuterLoop().pidGetData().output);
        vofa.writeData(rammerPID.getInnerLoop().pidGetData().output);
        vofa.writeData((fp32)rammerMotor.getCurrentTorqueCurrent());
        vofa.writeData(rammerMotor.getCurrentAngle());
        vofa.sendFrame();
        vTaskDelayUntil(&taskLastWakeTime, 1); // 确保任务以定周期1ms运行
    }
}
