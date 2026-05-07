/**
 ******************************************************************************
 * @file           : crt_gimbal.cpp
 * @brief          : 云台控制
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
#include "dvc_remotecontrol.hpp"
#include "para_gimbal.hpp"
#include "tsk_isr.hpp"
#include "drv_misc.h"
#include <math.h>
#include "tim.h" /* For htim1 */
#include "usbd_cdc_if.h"

/* Typedef -------------------------------------------------------------------*/
enum LedColor {
    LED_RED,
    LED_BLUE,
};
static LedColor currentLedColor = LED_RED;
static bool isLedChanged        = true;

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            Gimbal类实现
 ******************************************************************************/

Gimbal::Gimbal(MotorGM6020 *yawMotor, MotorDM4310 *pitchMotor, MotorM2006 *rammerMotor, MotorM3508 *frictionLeftMotor, MotorM3508 *frictionRightMotor, IMU *imu)
    : m_yawMotor(yawMotor), m_pitchMotor(pitchMotor),
      m_rammerMotor(rammerMotor), m_frictionLeftMotor(frictionLeftMotor), m_frictionRightMotor(frictionRightMotor),
      m_imu(imu),
      m_gimbalMode(GIMBAL_NO_FORCE),
      m_yawTargetAngle(0.0f), m_pitchTargetAngle(0.0f),
      m_rammerState(false),
      m_frictionState(false),
      m_remoteControl(),
      m_vt13RemoteControl(),
      m_ws2812(&htim1, TIM_CHANNEL_1),
      m_isInitComplete(false),
      m_lastShootCmd(0) {}

void Gimbal::init()
{
    DWT_Init();
    CAN_Init(&hcan1, can1RxCallback);
    CAN_Init(&hcan2, can2RxCallback);
    UART_Init(&huart3, dr16RxCallback, 36);
    UART_Init(&huart1, vt13RxCallback, UART_BUFFER_SIZE);
    m_imu->init();
    m_ws2812.Init();

    for (int i = 0; i < WS2812_LED_NUM; i++) {
        m_ws2812.SetColor(i, 120, 0, 0);
    }
    m_ws2812.Update();

    m_isInitComplete = true;
}

void Gimbal::controlLoop()
{
    if (!m_isInitComplete) return;
    modeSelect();
    targetOrientationPlan();
    shootPlan();
    pitchControl();
    yawControl();
    shootControl();
    ledControl();
    transmitGimbalMotorData();
    transmitGimbalDataViaUsb();
}

uint8_t Gimbal::sendUsbData()
{
    uint32_t txbufIndex      = 0;
    m_usbTxBuf[txbufIndex++] = m_usbTxSOF; // SOF 0x3A

    // (1-4)
    const float rollAngle = m_imu->getEulerAngle().x;
    memcpy(m_usbTxBuf + txbufIndex, &rollAngle, sizeof(float));
    txbufIndex += sizeof(float);

    // (5-8)
    const float pitchAngle = -m_imu->getEulerAngle().y;
    memcpy(m_usbTxBuf + txbufIndex, &pitchAngle, sizeof(float)); // rad unit
    txbufIndex += sizeof(float);

    // (9-12)
    const float yawAngle = m_imu->getEulerAngle().z;
    memcpy(m_usbTxBuf + txbufIndex, &yawAngle, sizeof(float)); // rad unit
    txbufIndex += sizeof(float);

    // (13-28)
    const float *quaternion = m_imu->getQuaternion();
    memcpy(m_usbTxBuf + txbufIndex, quaternion, sizeof(float) * 4);
    txbufIndex += sizeof(float) * 4;

    // (29-32)
    const float bulletSpeed = 0.0f;
    memcpy(m_usbTxBuf + txbufIndex, &bulletSpeed, sizeof(float));
    txbufIndex += sizeof(float);

    // (33)
    // add 1 byte keyboard toggle target press status
    // cause' the remote control is running in 1000hz update frequency, and gimbal is 200 hz update freequency,
    // if we use keyboard event to send KEY_B press event,
    // then will be an risk that most of times the press event will be reset as not pressed
    // in the remote control update loop, and seldomly the press event can be send correctly;
    // one solution is set an press key b flag and cosume it in the sendUsbData function.
    // but it will result in shit code.
    // so plan b: the nuc will deal with the press event. we only send key status there.
    // 26.3.30 Xinrong.Li
    const bool dr16Connected      = m_remoteControl.isConnected();
    const bool vt13Connected      = m_vt13RemoteControl.isConnected();
    const bool dr16ManualEnabled  =
        dr16Connected && m_remoteControl.getRightSwitchStatus() == DR16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE;
    const bool vt13ControlEnabled = vt13Connected && (!dr16Connected || dr16ManualEnabled);
    uint8_t keyBPressed           = 0u;
    if (vt13ControlEnabled &&
        m_vt13RemoteControl.getKeyboardKeyStatus(VT13RemoteControl::KeyboardKeyIndex::KEY_B) == RemoteControl::KeyStatus::KEY_PRESS) {
        keyBPressed = 1u;
    }
    m_usbTxBuf[txbufIndex] = keyBPressed;
    txbufIndex += 1;

    // (34)
    m_usbTxBuf[txbufIndex] = m_usbTxEOF; // EOF
    txbufIndex += 1;

    return CDC_Transmit_FS(m_usbTxBuf, static_cast<uint16_t>(txbufIndex));
}

void Gimbal::imuLoop()
{
    if (!m_isInitComplete) return;
    m_eulerAngle = m_imu->solveAttitude();
}

void Gimbal::receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (m_yawMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_pitchMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_rammerMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_frictionLeftMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_frictionRightMotor->decodeCanRxMessageFromISR(rxMessage)) return;
}

void Gimbal::receiveRemoteControlDataFromISR(const uint8_t *rxData)
{
    m_remoteControl.receiveRxDataFromISR(rxData);
}

void Gimbal::receiveVt13RemoteControlDataFromISR(const uint8_t *rxData)
{
    m_vt13RemoteControl.receiveRxDataFromISR(rxData);
}

// void Gimbal::modeSelect()
// {
//     m_remoteControl.updateEvent();
//     if (!m_remoteControl.isConnected()) {
//         m_gimbalMode = GIMBAL_NO_FORCE;
//         return;
//     }

//     switch (m_remoteControl.getRightSwitchStatus()) {
//         case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN:
//             m_gimbalMode = GIMBAL_NO_FORCE;
//             if (m_remoteControl.getLeftSwitchEvent() == Dr16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
//                 m_gimbalMode = CALIBRATION;
//             }
//             break;

//         case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE:
//             m_gimbalMode = MANUAL_CONTROL;
//             break;

//         case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_UP:
//             m_gimbalMode = AUTO_CONTROL;
//             break;

//         default:
//             break;
//     }
// }

void Gimbal::modeSelect()
{
    m_remoteControl.updateEvent();
    m_vt13RemoteControl.updateEvent();

    const bool dr16Connected      = m_remoteControl.isConnected();
    const bool vt13Connected      = m_vt13RemoteControl.isConnected();
    const bool dr16ManualEnabled  =
        dr16Connected && m_remoteControl.getRightSwitchStatus() == DR16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE;
    const bool vt13ControlEnabled = vt13Connected && (!dr16Connected || dr16ManualEnabled);
    const bool vt13ForceOffRequested =
        vt13Connected && !dr16Connected &&
        m_vt13RemoteControl.getModeSwitchStatus() == VT13RemoteControl::SwitchStatus3Pos::SWITCH_UP;

    if (!dr16Connected && !vt13Connected) {
        m_gimbalMode      = GIMBAL_NO_FORCE;
        return;
    }

    if (vt13ForceOffRequested) {
        m_gimbalMode      = GIMBAL_NO_FORCE;
        return;
    }

    if (!dr16Connected && vt13Connected && m_gimbalMode == GIMBAL_NO_FORCE) {
        m_gimbalMode = MANUAL_CONTROL;
    }

    if (dr16Connected) {
        switch (m_remoteControl.getRightSwitchStatus()) {
            case DR16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN:
                m_gimbalMode  = GIMBAL_NO_FORCE;
                if (m_remoteControl.getLeftSwitchEvent() == DR16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
                    m_gimbalMode = CALIBRATION; // 进入校准模式
                }
                break;

            case DR16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE:
                m_gimbalMode = MANUAL_CONTROL;
                break;

            case DR16RemoteControl::SwitchStatus3Pos::SWITCH_UP:
                m_gimbalMode  = AUTO_CONTROL;
                break;

            default:
                break;
        }
    } 

    if (vt13ControlEnabled &&
        m_vt13RemoteControl.getMouseRightKeyEvent() == RemoteControl::KeyEvent::KEY_TOGGLE_RELEASE_PRESS) {
        if (m_gimbalMode == MANUAL_CONTROL) {
            m_gimbalMode = AUTO_CONTROL;
        } else if (m_gimbalMode == AUTO_CONTROL) {
            m_gimbalMode = MANUAL_CONTROL;
        }
    }
}


void Gimbal::targetOrientationPlan()
{
    auto getMouseStickEquivalent = [](fp32 mouseAxis, fp32 gain) {
        fp32 stickEquivalent = mouseAxis * gain;
        GSRLMath::constrain(stickEquivalent, DT7_NORMALIZED_INPUT_LIMIT );
        return stickEquivalent;
    };

    const bool dr16Connected      = m_remoteControl.isConnected();
    const bool vt13Connected      = m_vt13RemoteControl.isConnected();
    const bool dr16ManualEnabled  =
        dr16Connected && m_remoteControl.getRightSwitchStatus() == DR16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE;
    const bool vt13ControlEnabled = vt13Connected && (!dr16Connected || dr16ManualEnabled);

    // switch (m_gimbalMode) {
    //     case MANUAL_CONTROL:
    //         setYawAngle(m_yawTargetAngle - rcStickDeadZoneFilter(m_remoteControl.getRightStickX()) * DT7_STICK_YAW_SENSITIVITY*0.6);
    //         setPitchAngle(m_pitchTargetAngle - rcStickDeadZoneFilter(-m_remoteControl.getRightStickY()) * DT7_STICK_PITCH_SENSITIVITY*0.6);
    //         break;
    switch (m_gimbalMode) {
        case MANUAL_CONTROL: {
            fp32 yawInput   = 0.0f;
            fp32 pitchInput = 0.0f;

            if (dr16Connected) {
                yawInput += m_remoteControl.getRightStickX();
                pitchInput += m_remoteControl.getRightStickY();
            }

            if (vt13ControlEnabled) {
                yawInput += getMouseStickEquivalent(m_vt13RemoteControl.getMouseX(), DT7_MOUSE_YAW_STICK_GAIN);
                pitchInput += getMouseStickEquivalent(m_vt13RemoteControl.getMouseY(), DT7_MOUSE_PITCH_STICK_GAIN);
            }
            GSRLMath::constrain(yawInput, DT7_NORMALIZED_INPUT_LIMIT);
            GSRLMath::constrain(pitchInput, DT7_NORMALIZED_INPUT_LIMIT);

            setYawAngle(m_yawTargetAngle - yawInput * DT7_STICK_YAW_SENSITIVITY * 0.6);
            setPitchAngle(m_pitchTargetAngle - pitchInput * DT7_STICK_PITCH_SENSITIVITY * 0.6);
            break;
        }

        case AUTO_CONTROL:
            if (rxMsgViaUsb.found) {
                setYawAngle(rxMsgViaUsb.yaw);
                setPitchAngle(rxMsgViaUsb.pitch);
            }
            break;

        default:
            break;
    }
}

// void Gimbal::shootPlan()
// {
//     switch (m_gimbalMode) {
//         case AUTO_CONTROL: {
//     // 摩擦轮开关保持不变
//     if (m_remoteControl.getLeftSwitchEvent() == DR16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
//         m_frictionState = !m_frictionState;
//     }

//     // 允许拨弹条件
//     m_feederArmed = m_frictionState ;//&& (m_leftShooterHeat < 350);
//     if (!m_feederArmed) {
//         m_contFireEnable  = false;
//         m_downHoldMs      = 0;
//         m_downLatched     = false;
//         m_contFireTimerMs = 0;
//         return;
//     }

//     m_singleShotReq = false;
    
//     static float lastScrollWheel = 0.0f;
//     static bool scrollWheelLatched = false;
//     float currentScrollWheel = m_remoteControl.getScrollWheel();

//     // 先检测“离开中位”的一次变化，触发后锁定；只有回中后才允许下一次触发
//     if (!scrollWheelLatched) {
//         if (fabsf(currentScrollWheel - lastScrollWheel) > 0.15f) {
//             if (m_feederArmed) {
//                 m_singleShotReq = true;
//             }
//             scrollWheelLatched = true;
//         }
//     } else if (fabsf(currentScrollWheel) < 0.05f) {
//         scrollWheelLatched = false;
//     }
//     lastScrollWheel = currentScrollWheel;
//             return;
//         }
//         case MANUAL_CONTROL: {
//             // 摩擦轮开关保持不变
//             if (m_remoteControl.getLeftSwitchEvent() == DR16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
//                 m_frictionState = !m_frictionState;
//             }

//             // 允许拨弹条件
//             m_feederArmed = m_frictionState; //&& (m_leftShooterHeat < 350);
//             if (!m_feederArmed) {
//                 m_contFireEnable  = false;
//                 m_downHoldMs      = 0;
//                 m_downLatched     = false;
//                 m_contFireTimerMs = 0;
//                 return;
//             }

//             m_singleShotReq = false;

//             static float lastScrollWheel = 0.0f;
//             static bool scrollWheelLatched = false;
//             float currentScrollWheel       = m_remoteControl.getScrollWheel();

//             // 先检测“离开中位”的一次变化，触发后锁定；只有回中后才允许下一次触发
//             if (!scrollWheelLatched) {
//                 if (fabsf(currentScrollWheel - lastScrollWheel) > 0.15f) {
//                     if (m_feederArmed) {
//                         m_singleShotReq = true;
//                     }
//                     scrollWheelLatched = true;
//                 }
//             } else if (fabsf(currentScrollWheel) < 0.05f) {
//                 scrollWheelLatched = false;
//             }
//             lastScrollWheel = currentScrollWheel;

//             // 左拨杆打到下档 -> 开启连发
//             if (m_remoteControl.getLeftSwitchStatus() == DR16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN) {
//                 m_contFireEnable = m_feederArmed;
//             } else {
//                 m_contFireEnable = false;
//             }

//             // 清除旧逻辑相关的状态变量，防止干扰
//             m_downHoldMs  = 0;
//             m_downLatched = false;
//             break;
//         }

//         default:
//             m_frictionState = false;
//             break;
//     }
// }

void Gimbal::shootPlan()
{
    const bool dr16Connected = m_remoteControl.isConnected();
    const bool vt13ControlEnabled =
        m_vt13RemoteControl.isConnected() &&
        (!dr16Connected || m_remoteControl.getRightSwitchStatus() == DR16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE);
        
    // const bool mouseRightAimMode =
    //     vt13ControlEnabled &&
    //     m_vt13RemoteControl.getMouseRightKeyStatus() == RemoteControl::KeyStatus::KEY_PRESS;
    // const bool mouseLeftToggleFire =
    //     vt13ControlEnabled &&
    //     m_vt13RemoteControl.getMouseLeftKeyStatus() == RemoteControl::KeyStatus::KEY_PRESS;
    // const bool mouseLeftSingleShot =
    //     vt13ControlEnabled &&
    //     m_vt13RemoteControl.getMouseLeftKeyEvent() == RemoteControl::KeyEvent::KEY_TOGGLE_RELEASE_PRESS;
    // const bool vt13TriggerShootCommand =
    //     vt13ControlEnabled &&
    //     m_vt13RemoteControl.getTriggerKeyEvent() == RemoteControl::KeyEvent::KEY_TOGGLE_RELEASE_PRESS;

    if (dr16Connected) {
        switch (m_gimbalMode) {
            case AUTO_CONTROL: {   
               // 摩擦轮开关保持不变
                if (m_remoteControl.getLeftSwitchEvent() == DR16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
                m_frictionState = !m_frictionState;
            }
            // 允许拨弹条件
                m_feederArmed = m_frictionState ;//&& (m_leftShooterHeat < 350);
                if (!m_feederArmed) {
                    m_contFireEnable  = false;
                    m_downHoldMs      = 0;
                    m_downLatched     = false;
                    m_contFireTimerMs = 0;
                    return;
                }

                m_singleShotReq = false;
                
                static float lastScrollWheel = 0.0f;
                static bool scrollWheelLatched = false;
                float currentScrollWheel = m_remoteControl.getScrollWheel();

                // 先检测“离开中位”的一次变化，触发后锁定；只有回中后才允许下一次触发
                if (!scrollWheelLatched) {
                    if (fabsf(currentScrollWheel - lastScrollWheel) > 0.15f) {
                        if (m_feederArmed) {
                            m_singleShotReq = true;
                        }
                        scrollWheelLatched = true;
                    }
                } else if (fabsf(currentScrollWheel) < 0.05f) {
                    scrollWheelLatched = false;
                }
                lastScrollWheel = currentScrollWheel;
                return;
            }

            case MANUAL_CONTROL: {
                // 摩擦轮开关保持不变
                if (m_remoteControl.getLeftSwitchEvent() == DR16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
                    m_frictionState = !m_frictionState;
                }

                // 允许拨弹条件
                m_feederArmed = m_frictionState; //&& (m_leftShooterHeat < 350);
                if (!m_feederArmed) {
                    m_contFireEnable  = false;
                    m_downHoldMs      = 0;
                    m_downLatched     = false;
                    m_contFireTimerMs = 0;
                    return;
                }

                m_singleShotReq = false;

                static float lastScrollWheel = 0.0f;
                static bool scrollWheelLatched = false;
                float currentScrollWheel       = m_remoteControl.getScrollWheel();

                // 先检测“离开中位”的一次变化，触发后锁定；只有回中后才允许下一次触发
                if (!scrollWheelLatched) {
                    if (fabsf(currentScrollWheel - lastScrollWheel) > 0.15f) {
                        if (m_feederArmed) {
                            m_singleShotReq = true;
                        }
                        scrollWheelLatched = true;
                    }
                } else if (fabsf(currentScrollWheel) < 0.05f) {
                    scrollWheelLatched = false;
                }
                lastScrollWheel = currentScrollWheel;

                // 左拨杆打到下档 -> 开启连发
                if (m_remoteControl.getLeftSwitchStatus() == DR16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN) {
                    m_contFireEnable = m_feederArmed;
                } else {
                    m_contFireEnable = false;
                }

                // 清除旧逻辑相关的状态变量，防止干扰
                m_downHoldMs  = 0;
                m_downLatched = false;
                break;
            }

            default:
            m_frictionState = false;
            break;
        }
    }

    if (vt13ControlEnabled) {
        if (m_vt13RemoteControl.getKeyboardKeyEvent(VT13RemoteControl::KeyboardKeyIndex::KEY_CTRL) == RemoteControl::KeyEvent::KEY_TOGGLE_RELEASE_PRESS) {
            m_frictionState = !m_frictionState;
            m_feederArmed   = m_frictionState;
            if (!m_frictionState) {
                m_contFireEnable  = false;
                m_contFireTimerMs = 0;
                m_contFirePending = 0;
            }
        }

        if (m_frictionState) {
            if (m_vt13RemoteControl.getMouseLeftKeyEvent() == RemoteControl::KeyEvent::KEY_TOGGLE_RELEASE_PRESS) {
                m_singleShotReq = true;
            }

            m_contFireEnable = (m_vt13RemoteControl.getMouseLeftKeyStatus() == RemoteControl::KeyStatus::KEY_PRESS);

            if (m_vt13RemoteControl.getTriggerKeyEvent() == RemoteControl::KeyEvent::KEY_TOGGLE_RELEASE_PRESS) {
                m_singleShotReq = true;
            }
        } else {
            m_contFireEnable  = false;
            m_contFireTimerMs = 0;
            m_contFirePending = 0;
        }
    }

    m_feederArmed = m_frictionState; //&& (m_leftShooterHeat < 350);
    if (!m_feederArmed) {
        m_contFireEnable  = false;
        m_downHoldMs      = 0;
        m_downLatched     = false;
        m_contFireTimerMs = 0;
        return;
    }
}

void Gimbal::pitchControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_pitchMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制
            // fp32 fdbData[2] = {GSRLMath::normalizeDeltaAngle(m_pitchTargetAngle - m_eulerAngle.y), -m_imu->getGyro().y};
            fp32 fdbData[2] = {m_pitchTargetAngle - m_eulerAngle.y, m_imu->getGyro().y};
            fp32 pidOutput  = m_pitchMotor->externalClosedloopControl(0.0f, fdbData, 2);
#ifdef PITCH_GRAVITY_COMPENSATE
            fp32 totalTorque = gravityCompensate(pidOutput, m_pitchMotor->getCurrentAngle(), PITCH_GRAVITY_COMPENSATE);
            m_pitchMotor->openloopControl(totalTorque);
#else
            (void)pidOutput; // 防止未使用变量警告
#endif
            break;
        }

        default:
            break;
    }
}

void Gimbal::yawControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_yawTargetAngle = m_eulerAngle.z;
            m_yawMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制
            fp32 fdbData[2] = {m_yawTargetAngle - m_eulerAngle.z, m_imu->getGyro().z};
            m_yawMotor->externalClosedloopControl(0.0f, fdbData, 2);
            break;
        }

        default:
            break;
    }
}

void Gimbal::shootControl()
{
    if (m_gimbalMode == GIMBAL_NO_FORCE) {
        m_frictionState  = false;
        m_singleShotReq  = false;
        m_contFireEnable = false;
        m_feederArmed    = false;

        m_shootState      = stateIdle;
        m_feederTargetRev = 0.0f;
        m_contFireTimerMs = 0;
        m_contFirePending = 0;

        m_jamCounter   = 0;
        m_unjamCounter = 0;
        m_frictionLeftMotor->angularVelocityClosedloopControl(0.0f);
        m_frictionRightMotor->angularVelocityClosedloopControl(0.0f);
        // m_frictionRightMotor->openloopControl(0.0f);
        // m_frictionLeftMotor->openloopControl(0.0f);

        m_rammerMotor->openloopControl(0.0f);
        return;
    } else {
        if (m_frictionState) {
            m_frictionLeftMotor->angularVelocityClosedloopControl(FRICTION_TARGET_ANGULAR_VELOCITY);
            m_frictionRightMotor->angularVelocityClosedloopControl(-FRICTION_TARGET_ANGULAR_VELOCITY);
        } else {
            m_frictionLeftMotor->angularVelocityClosedloopControl(0.0f);
            m_frictionRightMotor->angularVelocityClosedloopControl(0.0f);
            // m_frictionRightMotor->openloopControl(0.0f);
            // m_frictionLeftMotor->openloopControl(0.0f);
        }

        bool singleShotTrigger = false;

        if (m_singleShotReq) {
            singleShotTrigger = true;
            m_singleShotReq   = false;
        }

        if (m_contFireEnable && m_feederArmed) {
            m_contFireTimerMs += 1;
            if (m_contFireTimerMs >= CONT_FIRE_PERIOD_MS) {
                m_contFireTimerMs = 0;
                if (m_shootState == stateIdle) {
                    singleShotTrigger = true;
                } else {
                    m_contFirePending = 1;
                }
            }
        } else {
            m_contFireTimerMs = 0;
            m_contFirePending = 0;
        }

        switch (m_shootState) {

            case stateIdle: {
                m_jamCounter = 0;

                // 未允许拨弹 or 摩擦轮未开：拨弹停
                if (!m_feederArmed || !m_frictionState) {
                    m_rammerMotor->openloopControl(0.0f);
                    m_feederTargetRev = m_rammerMotor->getCurrentRevolutions();
                    break;
                }

                // 连发待发仅在空闲态消费，保证与单发脉冲解耦
                if (!singleShotTrigger && m_contFirePending > 0) {
                    singleShotTrigger = true;
                    m_contFirePending--;
                }

                // 单发触发：目标圈数 + 1/8圈（方向反了就改成 -=）
                if (singleShotTrigger) {
                    m_feederTargetRev += (+FEED_STEP_REV);
                    m_shootState = stateFeeding;
                } else {
                    m_rammerMotor->revolutionsClosedloopControl(m_feederTargetRev);
                }
            } break;

            case stateFeeding: {
                m_rammerMotor->revolutionsClosedloopControl(m_feederTargetRev);

                const fp32 curRev   = m_rammerMotor->getCurrentRevolutions();     // 当前转数
                const fp32 curSpd   = m_rammerMotor->getCurrentAngularVelocity(); // 当前速度
                const fp32 revError = m_feederTargetRev - curRev;                 // 转数误差
                // 到位判定
                if (fabsf(revError) < FEED_REV_EPS && fabsf(curSpd) < FEED_SPEED_EPS) {
                    m_shootState = stateIdle;
                    m_jamCounter = 0;
                    break;
                }

                // 卡弹检测：独立 void 函数（内部可切换状态到 stateUnjamming）
                rammerStuckControl();

            } break;

            case stateUnjamming:
            default: {
                // 解卡动作：建议用 openloop 给反向电流/电压（不走位置环）
                m_rammerMotor->openloopControl(UNJAM_TORQUE);

                // 解卡计时与状态切回：独立 void 函数
                rammerStuckControl();
            } break;
        }
    }
}

void Gimbal::rammerStuckControl()
{
    if (m_shootState == stateFeeding) {

        const fp32 curSpd  = m_rammerMotor->getCurrentAngularVelocity();
        const bool jamCond = (fabsf(curSpd) < JAM_SPEED_TH);

        if (jamCond)
            m_jamCounter++;
        else
            m_jamCounter = 0;

        if (m_jamCounter >= JAM_HOLD_TICKS) {
            m_jamCounter   = 0;
            m_unjamCounter = 0;
            m_shootState   = stateUnjamming;
        }

    } else if (m_shootState == stateUnjamming) {

        m_unjamCounter++;
        if (m_unjamCounter >= UNJAM_TICKS) {
            m_unjamCounter = 0;
            m_shootState   = stateFeeding; // 解卡完成，回去继续这发
        }
    }
}

void Gimbal::ledControl()
{
    float leftStickX            = m_remoteControl.getLeftStickX();
    static bool isStickReturned = true;

    //if (m_remoteControl.getRightSwitchStatus() == Dr16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE) {
    if (leftStickX < -0.5f) {
        if (isStickReturned) {
            currentLedColor = LED_RED;
            isLedChanged    = true;
            isStickReturned = false;
        }
        } else if (leftStickX > 0.5f) {
            if (isStickReturned) {
                currentLedColor = LED_BLUE;
                isLedChanged    = true;
                isStickReturned = false;
            }
        } else if (fabsf(leftStickX) < 0.1f) {
            isStickReturned = true;
        }
    else {
        isStickReturned = true;
    }

    if (isLedChanged) {
        uint8_t r = 0, g = 0, b = 0;
        switch (currentLedColor) {
            case LED_RED:
                    r = 255;
                    g = 0;
                    b = 0;
                    break;
            case LED_BLUE:
                    r = 0;
                    g = 0;
                    b = 120;
                    break;
            default:
                    break;
            }

        for (int i = 0; i < WS2812_LED_NUM; i++) {
            m_ws2812.SetColor(i, r, g, b);
        }
            m_ws2812.Update();

            isLedChanged = false;
        }
    }


void Gimbal::transmitGimbalMotorData()
{
    HAL_CAN_AddTxMessage(&hcan1, m_yawMotor->getMotorControlHeader(), m_yawMotor->getMotorControlData(), NULL);
    HAL_CAN_AddTxMessage(&hcan2, m_pitchMotor->getMotorControlHeader(), m_pitchMotor->getMotorControlData(), NULL);
    HAL_CAN_AddTxMessage(&hcan1, m_frictionLeftMotor->getMotorControlHeader(), m_frictionLeftMotor->getMotorControlData(), NULL);
}

void Gimbal::transmitGimbalDataViaUsb()
{
    m_txMsgViaUsb.header                 = USB_TX_SOF;
    m_txMsgViaUsb.roll                   = m_eulerAngle.x;
    m_txMsgViaUsb.pitch                  = m_eulerAngle.y;
    m_txMsgViaUsb.yaw                    = m_eulerAngle.z;
    m_txMsgViaUsb.q[0]                   = m_imu->getQuaternion()[0]; // w
    m_txMsgViaUsb.q[1]                   = m_imu->getQuaternion()[1]; // x
    m_txMsgViaUsb.q[2]                   = m_imu->getQuaternion()[2]; // y
    m_txMsgViaUsb.q[3]                   = m_imu->getQuaternion()[3]; // z
    m_txMsgViaUsb.toogleTargetKeyPressed = (uint8_t)(m_remoteControl.getKeyboardKeyStatus(DR16RemoteControl::KeyboardKeyIndex::KEY_B) == RemoteControl::KeyStatus::KEY_PRESS);
    m_txMsgViaUsb.EOF_                   = USB_TX_EOF;
    m_txMsgViaUsb.bulletSpeed            = 0.f; // TODO: require referee system's data
    memcpy(m_usbTxBuffer, &m_txMsgViaUsb, sizeof(txMsgViaUsb_t));

    if (CDC_Transmit_FS(m_usbTxBuffer, sizeof(txMsgViaUsb_t)) != USBD_OK) {
        m_usbSendErrCnt++;
    }
}

inline void Gimbal::setPitchAngle(const fp32 &targetAngle)
{
    fp32 constrainedAngle = targetAngle;
    
    // 第一层：目标角度软限位（指令级限制）
    if (constrainedAngle > PITCH_UPPER_LIMIT)
        constrainedAngle = PITCH_UPPER_LIMIT;
    else if (constrainedAngle < PITCH_LOWER_LIMIT)
        constrainedAngle = PITCH_LOWER_LIMIT;
    
    // 第二层：根据编码器位置的硬限位（电机保护）
    fp32 currentEncoderAngle = m_pitchMotor->getCurrentAngle();
    
    // 如果当前已接近硬限位，防止继续往该方向转
    if (currentEncoderAngle >= PITCH_ENCODER_UPPER_LIMIT && constrainedAngle > currentEncoderAngle) {
        constrainedAngle = currentEncoderAngle;
    } else if (currentEncoderAngle <= PITCH_ENCODER_LOWER_LIMIT && constrainedAngle < currentEncoderAngle) {
        constrainedAngle = currentEncoderAngle;
    }
    
    m_pitchTargetAngle = constrainedAngle;
}

inline void Gimbal::setYawAngle(const fp32 &targetAngle)
{
    if (targetAngle > YAW_UPPER_LIMIT)
        m_yawTargetAngle = YAW_UPPER_LIMIT;
    else if (targetAngle < YAW_LOWER_LIMIT)
        m_yawTargetAngle = YAW_LOWER_LIMIT;
    else
        m_yawTargetAngle = targetAngle;
}

inline fp32 Gimbal::rcStickDeadZoneFilter(const fp32 &rcStickValue)
{
    if (rcStickValue > DT7_STICK_DEAD_ZONE)
        return (rcStickValue - DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else if (rcStickValue < -DT7_STICK_DEAD_ZONE)
        return (rcStickValue + DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else
        return 0.0f;
}

/**
 * @brief 重力前馈补偿计算
 * @param baseTorque 基础力矩(PID输出)
 * @param currentAngle 当前角度(rad)
 * @param compensateCoeff 补偿系数(Nm)
 * @return 叠加前馈后的总力矩
 */
inline fp32 Gimbal::gravityCompensate(fp32 baseTorque, fp32 currentAngle, fp32 compensateCoeff)
{
    return baseTorque + compensateCoeff * cos(currentAngle);
}
