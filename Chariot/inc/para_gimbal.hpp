/**
 ******************************************************************************
 * @file           : para_gimbal.hpp
 * @brief          : 云台参数宏定义，方便修改调参
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "math_const.h"

/* Defines -------------------------------------------------------------------*/
/******************************************************************************
 *                            CAN ID
 ******************************************************************************/

/******************************************************************************
 *                            PID参数
 ******************************************************************************/
// 云台Yaw电机 (GM6020)
#define YAW_OUTER_KP                  25.0f    // 35.0f
#define YAW_OUTER_KI                  0.0f     // 0.0f
#define YAW_OUTER_KD                  0.0f     // 1.0f
#define YAW_OUTER_OUT_LIMIT           50.0f    // 50.0f
#define YAW_OUTER_IOUT_LIMIT          0.0f     // 0.0f
#define YAW_INNER_KP                  4500.0f  // 7000.0f
#define YAW_INNER_KI                  0.0f     // 0.0f
#define YAW_INNER_KD                  0.0f     // 26.0f
#define YAW_INNER_OUT_LIMIT           16384.0f // 16384.0f
#define YAW_INNER_IOUT_LIMIT          0.0f     // 0.0f
#define YAW_INNER_LOWPASS_FILTER_PARA 0.8f     // 1.0f
// 云台Pitch电机(DM4310)
#define PITCH_OUTER_KP                  40.0f // 外环 15.0
#define PITCH_OUTER_KI                  0.1f  // 0.0
#define PITCH_OUTER_KD                  80.0f  // 0.0
#define PITCH_OUTER_OUT_LIMIT           10.0f // 18.0
#define PITCH_OUTER_IOUT_LIMIT          0.5f  // 0.1
#define PITCH_INNER_KP                  0.65f // 内环 0.35
#define PITCH_INNER_KI                  0.0f  // 0.0
#define PITCH_INNER_KD                  0.05f  // 0.05
#define PITCH_INNER_OUT_LIMIT           10.0f  // 5.0
#define PITCH_INNER_IOUT_LIMIT          0.0f  // 0.0
#define PITCH_INNER_LOWPASS_FILTER_PARA 0.9f  // 1.0
// 重力补偿前馈（Nm）
#define PITCH_GRAVITY_COMPENSATE 0.0f
// 摩擦轮(M3508)
#define FRICTION_KP         380.0f // 360
#define FRICTION_KI         0.0f   // 10.0f
#define FRICTION_KD         0.5f   // 1.0
#define FRICTION_OUT_LIMIT  15000.0f
#define FRICTION_IOUT_LIMIT 2000.0f
// 拨弹轮(M2006)
#define RAMMER_INNER_KP                  6500.0f
#define RAMMER_INNER_KI                  0.2f
#define RAMMER_INNER_KD                  0.0f
#define RAMMER_INNER_OUT_LIMIT           10000.0f
#define RAMMER_INNER_IOUT_LIMIT          3000.0f
#define RAMMER_OUTER_KP                  11.0f
#define RAMMER_OUTER_KI                  0.0f
#define RAMMER_OUTER_KD                  8.0f
#define RAMMER_OUTER_OUT_LIMIT           60.0f
#define RAMMER_OUTER_IOUT_LIMIT          0.0f
#define RAMMER_INNER_LOWPASS_FILTER_PARA 0.4f

/******************************************************************************
 *                            IMU参数
 ******************************************************************************/
// Mahony算法参数
#define AHRS_AUTO_FREQ      0
#define AHRS_DEFAULT_FILTER 0
#define MAHONY_KP           1.0f
#define MAHONY_KI           0.0f
// IMU零飘补偿
#define GYRO_OFFSET_X  0.0f
#define GYRO_OFFSET_Y  0.0f
#define GYRO_OFFSET_Z  0.0f
#define ACCEL_OFFSET_X 0.0f
#define ACCEL_OFFSET_Y 0.0f
#define ACCEL_OFFSET_Z 0.0f
#define MAG_OFFSET_X   0.0f
#define MAG_OFFSET_Y   0.0f
#define MAG_OFFSET_Z   0.0f
// 安装朝向修正旋转矩阵
#define INSTALL_SPIN_MATRIX GSRLMath::Matrix33f((fp32[3][3]){{1, 0, 0}, {0, -1, 0}, {0, 0, -1}})

/******************************************************************************
 *                            遥控器灵敏度与死区
 ******************************************************************************/
#define DT7_STICK_DEAD_ZONE         0.05f
#define DT7_STICK_PITCH_SENSITIVITY 0.01f
#define DT7_STICK_YAW_SENSITIVITY   0.01f
#define DT7_NORMALIZED_INPUT_LIMIT  1.0f
#define DT7_KEYBOARD_MOVE_SCALE     1.0f
#define DT7_MOUSE_YAW_STICK_GAIN    40.0f
#define DT7_MOUSE_PITCH_STICK_GAIN  30.0f

/******************************************************************************
 *                            云台角度限制
 ******************************************************************************/
// 目标角度限制（软限位 - 用于指令限制）
#define PITCH_UPPER_LIMIT 0.16f
#define PITCH_LOWER_LIMIT -0.58f

// Pitch motor encoder hard limits. Unit: rad, signed motor angle [-PI, PI).
// These limits are independent from the IMU pitch target soft limits above.
#define PITCH_MOTOR_ENCODER_UPPER_LIMIT_RAD 6.2f
#define PITCH_MOTOR_ENCODER_LOWER_LIMIT_RAD 6.0f

// Yaw motor encoder hard limits. Unit: rad, signed motor angle [-PI, PI).
// Do not limit yaw by fixed IMU yaw because the quadrotor body yaw can move.
#define YAW_MOTOR_ENCODER_UPPER_LIMIT_RAD 2.6f
#define YAW_MOTOR_ENCODER_LOWER_LIMIT_RAD 1.3f
/******************************************************************************
 *                            发射机构参数
 ******************************************************************************/
#define FRICTION_TARGET_ANGULAR_VELOCITY 640.0f // 700
// 滚轮单发触发阈值
#define FIRE_HOLD_MS 80

// 拨弹电机减速比
#define FEEDER_REDUCTION_RATIO 36.0f

// 单发步进（8弹位：1/8圈 * 减速比）
#define FEED_STEP_REV (0.125f * FEEDER_REDUCTION_RATIO)
// 到位误差允许值也要乘以减速比放大
#define FEED_REV_EPS   (0.020f * FEEDER_REDUCTION_RATIO) // 0.01
#define FEED_SPEED_EPS 1.0f

// 连发节拍：连发模式下每隔多少ms触发一次“单发步进”
#define CONT_FIRE_PERIOD_MS 50.0f // 90
