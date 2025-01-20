// Copyright (c) CREATE All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#pragma once

#ifndef CAN_LIB
#define CAN_LIB

#include "driver/twai.h"

#define MAX_TRANSMIT 0 // twai_transmitを直接呼び出したときのタイムアウト時間
#define MAX_READ 0     // twai_receiveを直接呼び出したときのタイムアウト時間

// old config
#define PAR_ERROR 0
#define ACK_ERROR 1
#define CAN_OK 2

// 送信成功、バスのエラー、前回の送信失敗の3つを設定する
#define TWAI_ALERTS_CONFIG TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED

#define old_mode_block                                           \
    do                                                           \
    {                                                            \
        if (!return_new)                                         \
        {                                                        \
            pr_debug("old mode does not support this function"); \
            return INT_MAX;                                      \
        }                                                        \
    } while (0)

#define DEBUG

#ifdef DEBUG
#ifndef pr_debug
#ifdef ARDUINO

inline void pr_debug_checker(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
inline void pr_debug_checker(const char *fmt, ...) {}

// Arduino対応
#define pr_debug(fmt, ...)                              \
    do                                                  \
    {                                                   \
        pr_debug_checker(fmt, ##__VA_ARGS__);           \
        if (Serial) /*Serial.beginされてたらtrue*/ \
        {                                               \
            Serial.printf(fmt, ##__VA_ARGS__);          \
            Serial.println();                           \
        }                                               \
    } while (0)
#else
// ESP IDF対応
#define pr_debug(fmt, ...)            \
    do                                \
    {                                 \
        ESP_LOGI(fmt, ##__VA_ARGS__); \
    } while (0)
#endif
#else
#define pr_debug(fmt, ...)
#endif
#endif

typedef struct
{

} can_setting_t;

enum can_err
{
    CAN_SUCCESS,
    CAN_NO_ALERTS,
    CAN_BUS_ERROR,
    CAN_TX_FAILED,
    CAN_UNKNOWN_ERROR,
}

#endif