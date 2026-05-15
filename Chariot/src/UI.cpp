/**
 ******************************************************************************
 * @file           : UI.cpp
 * @brief          : UI Interface implementation for remote screen display
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "UI.hpp"
#include "drv_uart.h"

/* CRC Tables ----------------------------------------------------------------*/
const unsigned char CRC8_TAB[256] = {
    0x00,
    0x5e,
    0xbc,
    0xe2,
    0x61,
    0x3f,
    0xdd,
    0x83,
    0xc2,
    0x9c,
    0x7e,
    0x20,
    0xa3,
    0xfd,
    0x1f,
    0x41,
    0x9d,
    0xc3,
    0x21,
    0x7f,
    0xfc,
    0xa2,
    0x40,
    0x1e,
    0x5f,
    0x01,
    0xe3,
    0xbd,
    0x3e,
    0x60,
    0x82,
    0xdc,
    0x23,
    0x7d,
    0x9f,
    0xc1,
    0x42,
    0x1c,
    0xfe,
    0xa0,
    0xe1,
    0xbf,
    0x5d,
    0x03,
    0x80,
    0xde,
    0x3c,
    0x62,
    0xbe,
    0xe0,
    0x02,
    0x5c,
    0xdf,
    0x81,
    0x63,
    0x3d,
    0x7c,
    0x22,
    0xc0,
    0x9e,
    0x1d,
    0x43,
    0xa1,
    0xff,
    0x46,
    0x18,
    0xfa,
    0xa4,
    0x27,
    0x79,
    0x9b,
    0xc5,
    0x84,
    0xda,
    0x38,
    0x66,
    0xe5,
    0xbb,
    0x59,
    0x07,
    0xdb,
    0x85,
    0x67,
    0x39,
    0xba,
    0xe4,
    0x06,
    0x58,
    0x19,
    0x47,
    0xa5,
    0xfb,
    0x78,
    0x26,
    0xc4,
    0x9a,
    0x65,
    0x3b,
    0xd9,
    0x87,
    0x04,
    0x5a,
    0xb8,
    0xe6,
    0xa7,
    0xf9,
    0x1b,
    0x45,
    0xc6,
    0x98,
    0x7a,
    0x24,
    0xf8,
    0xa6,
    0x44,
    0x1a,
    0x99,
    0xc7,
    0x25,
    0x7b,
    0x3a,
    0x64,
    0x86,
    0xd8,
    0x5b,
    0x05,
    0xe7,
    0xb9,
    0x8c,
    0xd2,
    0x30,
    0x6e,
    0xed,
    0xb3,
    0x51,
    0x0f,
    0x4e,
    0x10,
    0xf2,
    0xac,
    0x2f,
    0x71,
    0x93,
    0xcd,
    0x11,
    0x4f,
    0xad,
    0xf3,
    0x70,
    0x2e,
    0xcc,
    0x92,
    0xd3,
    0x8d,
    0x6f,
    0x31,
    0xb2,
    0xec,
    0x0e,
    0x50,
    0xaf,
    0xf1,
    0x13,
    0x4d,
    0xce,
    0x90,
    0x72,
    0x2c,
    0x6d,
    0x33,
    0xd1,
    0x8f,
    0x0c,
    0x52,
    0xb0,
    0xee,
    0x32,
    0x6c,
    0x8e,
    0xd0,
    0x53,
    0x0d,
    0xef,
    0xb1,
    0xf0,
    0xae,
    0x4c,
    0x12,
    0x91,
    0xcf,
    0x2d,
    0x73,
    0xca,
    0x94,
    0x76,
    0x28,
    0xab,
    0xf5,
    0x17,
    0x49,
    0x08,
    0x56,
    0xb4,
    0xea,
    0x69,
    0x37,
    0xd5,
    0x8b,
    0x57,
    0x09,
    0xeb,
    0xb5,
    0x36,
    0x68,
    0x8a,
    0xd4,
    0x95,
    0xcb,
    0x29,
    0x77,
    0xf4,
    0xaa,
    0x48,
    0x16,
    0xe9,
    0xb7,
    0x55,
    0x0b,
    0x88,
    0xd6,
    0x34,
    0x6a,
    0x2b,
    0x75,
    0x97,
    0xc9,
    0x4a,
    0x14,
    0xf6,
    0xa8,
    0x74,
    0x2a,
    0xc8,
    0x96,
    0x15,
    0x4b,
    0xa9,
    0xf7,
    0xb6,
    0xe8,
    0x0a,
    0x54,
    0xd7,
    0x89,
    0x6b,
    0x35,
};

const uint16_t wCRC_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/* Global UI Instance --------------------------------------------------------*/
UI g_ui;

/* Private Helper Functions --------------------------------------------------*/
unsigned char UI::calcCrc8(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC8 = 0xff;
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8  = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

uint16_t UI::calcCrc16(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0xffff;
    uint8_t chData;
    if (pchMessage == NULL) {
        return 0xFFFF;
    }
    while (dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

void UI::procStringFrame(ui_string_frame_t *msg)
{
    msg->header.SOF        = UI_SOF;
    msg->header.length     = 51;
    msg->header.seq        = m_seq++;
    msg->header.crc8       = calcCrc8((uint8_t *)msg, 4);
    msg->header.cmd_id     = UI_CMD_ID;
    msg->header.sub_id     = 0x0110;
    msg->header.send_id    = m_selfId;
    msg->header.recv_id    = m_recvId;
    msg->option.str_length = strlen(msg->option.string);
    msg->crc16             = calcCrc16((uint8_t *)msg, 58);
}

HAL_StatusTypeDef UI::sendFrameData(uint8_t *data, uint16_t length)
{
    if (m_huart == nullptr) {
        return HAL_ERROR;
    }
    return UART_Send_Data(m_huart, data, length);
}

/* Public Interface ----------------------------------------------------------*/
void UI::init(UART_HandleTypeDef *huart, uint16_t self_id)
{
    m_huart                = huart;
    m_selfId               = self_id;
    m_recvId               = getClientIDFromRobotID(static_cast<uint8_t>(self_id));
    m_seq                  = 0;
    m_lastTxTick           = 0;
    m_lastInitSuccessTick  = 0;
    m_initStep             = UIInitStep::DONE;
    m_stateUpdateStep      = UIStateUpdateStep::DONE;
    m_staticInitialized    = false;
    m_lastFrictionState    = false;
    m_pendingFrictionState = false;
    m_frictionStateCached  = false;
    m_idsValid             = (m_selfId != 0U) && (m_recvId != 0U);
    m_lastJamState         = false;

    // Initialize shoot frame
    memset(&m_shootFrame, 0, sizeof(m_shootFrame));
    m_frictionStateIndicator = &(m_shootFrame.option);

    // Initialize stuck frame
    memset(&m_stuckFrame, 0, sizeof(m_stuckFrame));
    m_stuckJam = &(m_stuckFrame.option);

    // Initialize state frames
    memset(&m_stateFrameOff, 0, sizeof(m_stateFrameOff));
    memset(&m_stateFrameOn, 0, sizeof(m_stateFrameOn));
    m_stateOff = &(m_stateFrameOff.option);
    m_stateOn  = &(m_stateFrameOn.option);

    // Initialize route frame and pointers
    memset(&m_routeFrame, 0, sizeof(m_routeFrame));
    m_routeCenter = (ui_interface_line_t *)&(m_routeFrame.data[0]);
    m_routeLeft   = (ui_interface_line_t *)&(m_routeFrame.data[1]);
    m_routeRight  = (ui_interface_line_t *)&(m_routeFrame.data[2]);
}

void UI::restartInitialization()
{
    m_staticInitialized   = false;
    m_frictionStateCached = false;
    m_stateUpdateStep     = UIStateUpdateStep::DONE;
    m_initStep            = UIInitStep::REMOVE_SHOOT;
}

bool UI::setRobotID(uint8_t robotID)
{
    const uint16_t clientID = getClientIDFromRobotID(robotID);
    const bool validIDs     = (robotID != 0U) && (clientID != 0U);

    if (!validIDs) {
        m_idsValid = false;
        return false;
    }

    if (m_selfId != robotID || m_recvId != clientID || !m_idsValid) {
        m_selfId   = robotID;
        m_recvId   = clientID;
        m_idsValid = true;
        restartInitialization();
    }

    return true;
}

HAL_StatusTypeDef UI::sendStringFrame(ui_string_frame_t *msg)
{
    if (msg == nullptr) {
        return HAL_ERROR;
    }

    procStringFrame(msg);
    return sendFrameData((uint8_t *)msg, sizeof(ui_string_frame_t));
}

void UI::process(uint32_t now, bool isFrictionOn, bool isJamDetected)
{
    if (m_huart == nullptr || !m_idsValid) {
        return;
    }

    m_pendingFrictionState = isFrictionOn;

    if (!m_staticInitialized) {
        if (m_initStep == UIInitStep::DONE) {
            restartInitialization();
        }
    } else if (isTimeElapsed(now, m_lastInitSuccessTick, UI_PERIODIC_REINIT_INTERVAL_MS)) {
        restartInitialization();
    }

    if (m_initStep != UIInitStep::DONE) {
        if (!isReadyToTransmit(now)) {
            return;
        }

        if (sendInitializationStep(m_initStep) != HAL_OK) {
            return;
        }

        markTransmitSuccess(now);
        m_initStep = getNextInitStep(m_initStep);
        if (m_initStep == UIInitStep::DONE) {
            m_staticInitialized   = true;
            m_lastInitSuccessTick = now;
            m_lastFrictionState   = m_pendingFrictionState;
            m_frictionStateCached = true;
            m_lastJamState        = false;
        }
        return;
    }

    if (!m_frictionStateCached || isFrictionOn != m_lastFrictionState) {
        requestFrictionStateUpdate(isFrictionOn);
    }

    if (m_stateUpdateStep == UIStateUpdateStep::DONE || !isReadyToTransmit(now)) {
        // Continue to jam handling below so stuck display can react independently.
    }

    if (m_stateUpdateStep != UIStateUpdateStep::DONE && isReadyToTransmit(now)) {
        if (sendStateUpdateStep(m_stateUpdateStep) != HAL_OK) {
            return;
        }

        markTransmitSuccess(now);
        m_stateUpdateStep = getNextStateUpdateStep(m_stateUpdateStep);
        if (m_stateUpdateStep == UIStateUpdateStep::DONE) {
            m_lastFrictionState   = m_pendingFrictionState;
            m_frictionStateCached = true;
        }
        return;
    }

    if (isJamDetected != m_lastJamState && isReadyToTransmit(now)) {
        if (isJamDetected ? initStuckUI() : removeStuckUI()) {
            markTransmitSuccess(now);
            m_lastJamState = isJamDetected;
        }
    }
}

HAL_StatusTypeDef UI::initShootUI()
{
    return sendShootUI(1);
}

HAL_StatusTypeDef UI::updateShootUI()
{
    return sendShootUI(2);
}

HAL_StatusTypeDef UI::removeShootUI()
{
    return sendShootUI(3);
}

HAL_StatusTypeDef UI::initStateUI()
{
    HAL_StatusTypeDef status = sendStateOffUI(1);
    if (status != HAL_OK) {
        return status;
    }

    return sendStateOnUI(3);
}

HAL_StatusTypeDef UI::updateFrictionStateUI(bool isFrictionOn)
{
    HAL_StatusTypeDef status = isFrictionOn ? sendStateOnUI(1) : sendStateOffUI(1);
    if (status != HAL_OK) {
        return status;
    }

    return isFrictionOn ? sendStateOffUI(3) : sendStateOnUI(3);
}

HAL_StatusTypeDef UI::updateStateUI()
{
    HAL_StatusTypeDef status = sendStateOffUI(2);
    if (status != HAL_OK) {
        return status;
    }

    return sendStateOnUI(2);
}

HAL_StatusTypeDef UI::removeStateUI()
{
    HAL_StatusTypeDef status = sendStateOffUI(3);
    if (status != HAL_OK) {
        return status;
    }

    return sendStateOnUI(3);
}

HAL_StatusTypeDef UI::initStuckUI()
{
    return sendStuckUI(1);
}

HAL_StatusTypeDef UI::updateStuckUI()
{
    return sendStuckUI(2);
}

HAL_StatusTypeDef UI::removeStuckUI()
{
    return sendStuckUI(3);
}

HAL_StatusTypeDef UI::initRouteUI()
{
    return sendRouteUI(1);
}

HAL_StatusTypeDef UI::updateRouteUI()
{
    return sendRouteUI(2);
}

HAL_StatusTypeDef UI::removeRouteUI()
{
    return sendRouteUI(3);
}

uint16_t UI::getClientIDFromRobotID(uint8_t robotID)
{
    if (robotID >= 1U && robotID <= 9U) {
        return static_cast<uint16_t>(0x0100U + robotID);
    }
    if (robotID >= 101U && robotID <= 109U) {
        return static_cast<uint16_t>(0x0164U + (robotID - 100U));
    }
    return 0U;
}

bool UI::isTimeElapsed(uint32_t now, uint32_t last, uint32_t interval)
{
    return static_cast<uint32_t>(now - last) >= interval;
}

bool UI::isReadyToTransmit(uint32_t now) const
{
    return isTimeElapsed(now, m_lastTxTick, UI_TX_INTERVAL_MS);
}

void UI::markTransmitSuccess(uint32_t now)
{
    m_lastTxTick = now;
}

void UI::requestFrictionStateUpdate(bool isFrictionOn)
{
    m_pendingFrictionState = isFrictionOn;
    if (m_stateUpdateStep == UIStateUpdateStep::DONE) {
        m_stateUpdateStep = UIStateUpdateStep::SHOW_ACTIVE;
    }
}

UI::UIInitStep UI::getNextInitStep(UIInitStep currentStep) const
{
    switch (currentStep) {
        case UIInitStep::REMOVE_SHOOT:
            return UIInitStep::REMOVE_STATE_OFF;
        case UIInitStep::REMOVE_STATE_OFF:
            return UIInitStep::REMOVE_STATE_ON;
        case UIInitStep::REMOVE_STATE_ON:
            return UIInitStep::REMOVE_ROUTE;
        case UIInitStep::REMOVE_ROUTE:
            return UIInitStep::INIT_SHOOT;
        case UIInitStep::INIT_SHOOT:
            return UIInitStep::INIT_STATE_OFF;
        case UIInitStep::INIT_STATE_OFF:
            return UIInitStep::INIT_STATE_ON;
        case UIInitStep::INIT_STATE_ON:
            return UIInitStep::INIT_ROUTE;
        case UIInitStep::INIT_ROUTE:
        case UIInitStep::DONE:
        default:
            return UIInitStep::DONE;
    }
}

UI::UIStateUpdateStep UI::getNextStateUpdateStep(UIStateUpdateStep currentStep) const
{
    switch (currentStep) {
        case UIStateUpdateStep::SHOW_ACTIVE:
            return UIStateUpdateStep::HIDE_INACTIVE;
        case UIStateUpdateStep::HIDE_INACTIVE:
        case UIStateUpdateStep::DONE:
        default:
            return UIStateUpdateStep::DONE;
    }
}

HAL_StatusTypeDef UI::sendInitializationStep(UIInitStep step)
{
    switch (step) {
        case UIInitStep::REMOVE_SHOOT:
            return sendShootUI(3);
        case UIInitStep::REMOVE_STATE_OFF:
            return sendStateOffUI(3);
        case UIInitStep::REMOVE_STATE_ON:
            return sendStateOnUI(3);
        case UIInitStep::REMOVE_ROUTE:
            return sendRouteUI(3);
        case UIInitStep::INIT_SHOOT:
            return sendShootUI(1);
        case UIInitStep::INIT_STATE_OFF:
            return sendStateOffUI(m_pendingFrictionState ? 3 : 1);
        case UIInitStep::INIT_STATE_ON:
            return sendStateOnUI(m_pendingFrictionState ? 1 : 3);
        case UIInitStep::INIT_ROUTE:
            return sendRouteUI(1);
        case UIInitStep::DONE:
        default:
            return HAL_OK;
    }
}

HAL_StatusTypeDef UI::sendStateUpdateStep(UIStateUpdateStep step)
{
    switch (step) {
        case UIStateUpdateStep::SHOW_ACTIVE:
            return m_pendingFrictionState ? sendStateOnUI(1) : sendStateOffUI(1);
        case UIStateUpdateStep::HIDE_INACTIVE:
            return m_pendingFrictionState ? sendStateOffUI(3) : sendStateOnUI(3);
        case UIStateUpdateStep::DONE:
        default:
            return HAL_OK;
    }
}

HAL_StatusTypeDef UI::sendShootUI(uint8_t operateType)
{
    m_shootFrame.option.figure_name[0] = 'f';
    m_shootFrame.option.figure_name[1] = 'r';
    m_shootFrame.option.figure_name[2] = 'i';
    m_shootFrame.option.operate_type   = operateType;

    m_frictionStateIndicator->figure_type  = 7; // String type
    m_frictionStateIndicator->operate_type = operateType;
    m_frictionStateIndicator->layer        = 0;
    m_frictionStateIndicator->color        = 1;
    m_frictionStateIndicator->start_x      = 47;
    m_frictionStateIndicator->start_y      = 670;
    m_frictionStateIndicator->width        = 3;
    m_frictionStateIndicator->font_size    = 30;
    m_frictionStateIndicator->str_length   = 9;
    strcpy(m_frictionStateIndicator->string, "friction:");

    procStringFrame(&m_shootFrame);
    return sendFrameData((uint8_t *)&m_shootFrame, sizeof(m_shootFrame));
}

HAL_StatusTypeDef UI::sendStateOffUI(uint8_t operateType)
{
    m_stateFrameOff.option.figure_name[0] = 'o';
    m_stateFrameOff.option.figure_name[1] = 'f';
    m_stateFrameOff.option.figure_name[2] = 'f';
    m_stateFrameOff.option.operate_type   = operateType;

    m_stateOff->figure_type  = 7; // String type
    m_stateOff->operate_type = operateType;
    m_stateOff->layer        = 0;
    m_stateOff->color        = 1;
    m_stateOff->start_x      = 330;
    m_stateOff->start_y      = 670;
    m_stateOff->width        = 3;
    m_stateOff->font_size    = 30;
    m_stateOff->str_length   = 3;
    strcpy(m_stateOff->string, "off");

    procStringFrame(&m_stateFrameOff);
    return sendFrameData((uint8_t *)&m_stateFrameOff, sizeof(m_stateFrameOff));
}

HAL_StatusTypeDef UI::sendStateOnUI(uint8_t operateType)
{
    m_stateFrameOn.option.figure_name[0] = 'o';
    m_stateFrameOn.option.figure_name[1] = 'n';
    m_stateFrameOn.option.figure_name[2] = ' ';
    m_stateFrameOn.option.operate_type   = operateType;

    m_stateOn->figure_type  = 7; // String type
    m_stateOn->operate_type = operateType;
    m_stateOn->layer        = 0;
    m_stateOn->color        = 1;
    m_stateOn->start_x      = 330;
    m_stateOn->start_y      = 670;
    m_stateOn->width        = 3;
    m_stateOn->font_size    = 30;
    m_stateOn->str_length   = 2;
    strcpy(m_stateOn->string, "on");

    procStringFrame(&m_stateFrameOn);
    return sendFrameData((uint8_t *)&m_stateFrameOn, sizeof(m_stateFrameOn));
}

HAL_StatusTypeDef UI::sendStuckUI(uint8_t operateType)
{
    m_stuckFrame.option.figure_name[0] = 'j';
    m_stuckFrame.option.figure_name[1] = 'a';
    m_stuckFrame.option.figure_name[2] = 'm';
    m_stuckFrame.option.operate_type   = operateType;

    m_stuckJam->figure_type  = 7; // String type
    m_stuckJam->operate_type = operateType;
    m_stuckJam->layer        = 0;
    m_stuckJam->color        = 4;
    m_stuckJam->start_x      = 816;
    m_stuckJam->start_y      = 748;
    m_stuckJam->width        = 4;
    m_stuckJam->font_size    = 40;
    m_stuckJam->str_length   = 7;
    strcpy(m_stuckJam->string, "jamming");

    procStringFrame(&m_stuckFrame);
    return sendFrameData((uint8_t *)&m_stuckFrame, sizeof(m_stuckFrame));
}

HAL_StatusTypeDef UI::sendRouteUI(uint8_t operateType)
{
    const uint8_t routeNames[3][3] = {
        {'r', 'c', '0'},
        {'r', 'l', '1'},
        {'r', 'r', '2'},
    };

    for (int i = 0; i < 3; i++) {
        m_routeFrame.data[i].figure_name[0] = routeNames[i][0];
        m_routeFrame.data[i].figure_name[1] = routeNames[i][1];
        m_routeFrame.data[i].figure_name[2] = routeNames[i][2];
        m_routeFrame.data[i].operate_type   = operateType;
    }
    for (int i = 3; i < 5; i++) {
        m_routeFrame.data[i].operate_type = 0;
    }

    // center line
    m_routeCenter->figure_type  = 0;
    m_routeCenter->operate_type = operateType;
    m_routeCenter->layer        = 0;
    m_routeCenter->color        = 0;
    m_routeCenter->start_x      = 956;
    m_routeCenter->start_y      = 310;
    m_routeCenter->width        = 1;
    m_routeCenter->end_x        = 958;
    m_routeCenter->end_y        = 508;

    // left line
    m_routeLeft->figure_type  = 0;
    m_routeLeft->operate_type = operateType;
    m_routeLeft->layer        = 0;
    m_routeLeft->color        = 8;
    m_routeLeft->start_x      = 573;
    m_routeLeft->start_y      = 0;
    m_routeLeft->width        = 3;
    m_routeLeft->end_x        = 896;
    m_routeLeft->end_y        = 464;

    // right line
    m_routeRight->figure_type  = 0;
    m_routeRight->operate_type = operateType;
    m_routeRight->layer        = 0;
    m_routeRight->color        = 8;
    m_routeRight->start_x      = 1339;
    m_routeRight->start_y      = 0;
    m_routeRight->width        = 3;
    m_routeRight->end_x        = 1001;
    m_routeRight->end_y        = 462;

    procFrame(&m_routeFrame, 0x0103, 15 * 5);
    return sendFrameData((uint8_t *)&m_routeFrame, sizeof(m_routeFrame));
}
