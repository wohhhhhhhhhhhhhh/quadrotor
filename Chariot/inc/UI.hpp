/**
 ******************************************************************************
 * @file           : UI.hpp
 * @brief          : UI Interface for remote screen display
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef UI_HPP
#define UI_HPP

#include <stdint.h>
#include <string.h>

#include "usart.h"

/* Compiler Attributes -------------------------------------------------------*/
#if defined(__GNUC__) || defined(__CC_ARM)
#define MESSAGE_PACKED __attribute__((packed))
#else
#error "MESSAGE_PACKED not defined for this compiler"
#endif

/* Macro Definitions ---------------------------------------------------------*/
#define PRIMITIVE_CAT(x, y) x ## y
#define CAT(x, y) PRIMITIVE_CAT(x, y)

#define UI_FRAME_HEADER_SIZE    6
#define UI_CRC16_SIZE           2
#define UI_SOF                  0xA5
#define UI_CMD_ID               0x0301

/* UI Frame Definitions -------------------------------------------------------*/

/**
 * @brief Define UI message structure for different figure types
 * @param name: Figure name (e.g., figure, line, rect, round, ellipse, arc)
 * @param p_a, p_b, p_c, p_d, p_e: Parameter names for specific figures
 */
#define DEFINE_MESSAGE(name, p_a, p_b, p_c, p_d, p_e)   \
typedef struct {                                        \
uint8_t figure_name[3];                                 \
uint32_t operate_type:3;                                \
uint32_t figure_type:3;                                 \
uint32_t layer:4;                                       \
uint32_t color:4;                                       \
uint32_t PRIMITIVE_CAT(,p_a) :9;                        \
uint32_t PRIMITIVE_CAT(,p_b):9;                         \
uint32_t width:10;                                      \
uint32_t start_x:11;                                    \
uint32_t start_y:11;                                    \
uint32_t PRIMITIVE_CAT(,p_c):10;                        \
uint32_t PRIMITIVE_CAT(,p_d):11;                        \
uint32_t PRIMITIVE_CAT(,p_e):11;                        \
} MESSAGE_PACKED ui_interface_ ## name ##_t

DEFINE_MESSAGE(figure, _a, _b, _c, _d, _e);
DEFINE_MESSAGE(line, _a, _b, _c, end_x, end_y);
DEFINE_MESSAGE(rect, _a, _b, _c, end_x, end_y);
DEFINE_MESSAGE(round, _a, _b, r, _d, _e);
DEFINE_MESSAGE(ellipse, _a, _b, _c, rx, ry);
DEFINE_MESSAGE(arc, start_angle, end_angle, _c, rx, ry);

/* String and Number UI Types ------------------------------------------------*/
typedef struct {
    uint8_t figure_name[3];
    uint32_t operate_type: 3;
    uint32_t figure_type: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t font_size: 9;
    uint32_t _b: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    int32_t number;
} MESSAGE_PACKED ui_interface_number_t;

typedef struct {
    uint8_t figure_name[3];
    uint32_t operate_type: 3;
    uint32_t figure_type: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t font_size: 9;
    uint32_t str_length: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t _c: 10;
    uint32_t _d: 11;
    uint32_t _e: 11;
    char string[30];
} MESSAGE_PACKED ui_interface_string_t;

/* Frame Header and Structures -----------------------------------------------*/
typedef struct {
    uint8_t SOF;
    uint16_t length;
    uint8_t seq, crc8;
    uint16_t cmd_id, sub_id;
    uint16_t send_id, recv_id;
} MESSAGE_PACKED ui_frame_header_t;

#define DEFINE_FIGURE_MESSAGE(num)      \
typedef struct {                        \
ui_frame_header_t header;               \
ui_interface_figure_t data[num];        \
uint16_t crc16;                         \
} MESSAGE_PACKED ui_ ## num##_frame_t

DEFINE_FIGURE_MESSAGE(1);
DEFINE_FIGURE_MESSAGE(2);
DEFINE_FIGURE_MESSAGE(5);
DEFINE_FIGURE_MESSAGE(7);

typedef struct {
    ui_frame_header_t header;
    ui_interface_string_t option;
    uint16_t crc16;
} MESSAGE_PACKED ui_string_frame_t;

/* UI Manager Class ----------------------------------------------------------*/
class UI {
public:
    /**
     * @brief Initialize UI module with UART handle
     * @param huart: UART handle for transmission
     * @param self_id: Self ID for UI communication
     */
    void init(UART_HandleTypeDef *huart, int self_id = 1);

    /**
     * @brief Send string frame via UART
     * @param msg: UI string frame message
     */
    void sendStringFrame(ui_string_frame_t *msg);

    /**
     * @brief Send figure frame via UART
     * @param msg: UI figure frame message
     * @param count: Number of figures in the frame
     */
    template<typename T>
    void sendFrame(T *msg) {
        // Will be implemented to determine frame type and size
        sendFrameData((uint8_t *)msg, sizeof(T));
    }

    /**
     * @brief Initialize shoot state UI
     */
    void initShootUI();

    /**
     * @brief Update shoot state UI
     */
    void updateShootUI();

    /**
     * @brief Remove shoot state UI
     */
    void removeShootUI();

    /**
     * @brief Initialize gimbal state UI
     */
    void initStateUI();

    /**
     * @brief Update gimbal state UI based on friction state
     * @param isFrictionOn: true if friction is ON, false if friction is OFF
     */
    void updateFrictionStateUI(bool isFrictionOn);

    /**
     * @brief Update gimbal state UI (legacy update method)
     */
    void updateStateUI();

    /**
     * @brief Remove gimbal state UI
     */
    void removeStateUI();

    /**
     * @brief Initialize route UI (group route)
     */
    void initRouteUI();

    /**
     * @brief Update route UI
     */
    void updateRouteUI();

    /**
     * @brief Remove route UI
     */
    void removeRouteUI();

private:
    UART_HandleTypeDef *m_huart;
    int m_selfId;
    uint8_t m_seq;

    // UI frame buffers
    ui_string_frame_t m_shootFrame;
    ui_string_frame_t m_stateFrameOff;
    ui_string_frame_t m_stateFrameOn;
    ui_5_frame_t m_routeFrame;

    // Route line pointers
    ui_interface_line_t *m_routeCenter;
    ui_interface_line_t *m_routeLeft;
    ui_interface_line_t *m_routeRight;

    // String pointers for easy access
    ui_interface_string_t *m_frictionStateIndicator;
    ui_interface_string_t *m_stateOff;
    ui_interface_string_t *m_stateOn;

    /**
     * @brief Send raw frame data via UART
     */
    void sendFrameData(uint8_t *data, uint16_t length);

    /**
     * @brief Calculate CRC8 checksum
     */
    static unsigned char calcCrc8(unsigned char *pchMessage, unsigned int dwLength);

    /**
     * @brief Calculate CRC16 checksum
     */
    static uint16_t calcCrc16(uint8_t *pchMessage, uint32_t dwLength);

    /**
     * @brief Process string frame (add headers and checksums)
     */
    void procStringFrame(ui_string_frame_t *msg);

    /**
     * @brief Template function to process figure frames
     */
    template<typename T>
    void procFrame(T *msg, uint16_t sub_id, uint16_t data_size) {
        msg->header.SOF = UI_SOF;
        msg->header.length = 6 + data_size;
        msg->header.seq = m_seq++;
        msg->header.crc8 = calcCrc8((uint8_t *)msg, 4);
        msg->header.cmd_id = UI_CMD_ID;
        msg->header.sub_id = sub_id;
        msg->header.send_id = m_selfId;
        msg->header.recv_id = m_selfId + 256;
        msg->crc16 = calcCrc16((uint8_t *)msg, sizeof(T) - 2);
    }
    
};

#ifdef __cplusplus
extern "C" {
#endif

/* Global UI Instance --------------------------------------------------------*/
extern UI g_ui;

#ifdef __cplusplus
}
#endif

#endif // UI_HPP
