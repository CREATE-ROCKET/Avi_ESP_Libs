// Copyright (c) CREATE-ROCKET All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/**
 * @file CANCREATE_lib.h
 * @brief CAN config file
 * @copyright CREATE-ROCKET
 */

#pragma once

#ifndef CAN_LIB
#define CAN_LIB

#include "driver/twai.h"

#define MAX_TRANSMIT 0 /**< @def twai_transmitを直接呼び出したときのタイムアウト時間*/
#define MAX_READ 0     /**< @def twai_receiveを直接呼び出したときのタイムアウト時間*/

// old config
#define PAR_ERROR 0
#define ACK_ERROR 1
#define CAN_OK 2

/** @def TWAI_ALERTS_CONFIG 送信成功、バスのエラー、前回の送信失敗の3つを設定する */
#define TWAI_ALERTS_CONFIG TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED

#define old_mode_block                                           \
    do                                                           \
    {                                                            \
        if (!_return_new)                                        \
        {                                                        \
            pr_debug("old mode does not support this function"); \
            return INT_MAX;                                      \
        }                                                        \
    } while (0)

#define multi_send_block                                   \
    do                                                     \
    {                                                      \
        if (!_multi_send)                                  \
        {                                                  \
            pr_debug("multi send function is restricted"); \
            return -1;                                     \
        }                                                  \
    } while (0)

#ifndef DEBUG_CAN
/**
 * @def DEBUG_CAN
 * pr_debugを利用するかを決める もしoffにしたい場合は、platformio.iniにおいて
 * ```ini
 * build_flags = -DDEBUG_CAN=0
 * ```
 */
#define DEBUG_CAN 1
#endif

#ifndef pr_debug
#if DEBUG_CAN
#ifdef ARDUINO

inline void pr_debug_checker(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
inline void pr_debug_checker(const char *fmt, ...) {}

// Arduino対応
/**
 * @def pr_debug
 * @param #define DEBUG 1 とされているときに、実行されるDEBUG出力
 *         Serial.beginされていたら自動的に出力される
 */
#define pr_debug(fmt, ...)                         \
    do                                             \
    {                                              \
        pr_debug_checker(fmt, ##__VA_ARGS__);      \
        if (Serial) /*Serial.beginされてたらtrue*/ \
        {                                          \
            Serial.printf("CAN_CREATE: ");         \
            Serial.printf(fmt, ##__VA_ARGS__);     \
            Serial.println();                      \
        }                                          \
    } while (0)
#else
// ESP IDF対応
#define pr_debug(fmt, ...)                       \
    do                                           \
    {                                            \
        ESP_LOGI("CAN lib", fmt, ##__VA_ARGS__); \
    } while (0)
#endif
#else
#define pr_debug(fmt, ...)
#endif
#endif

/**
 * @brief CAN.begin の引数に入れてCANの動作を指定する
 * @attention この設定は通信相手と共有することが必要 共有していないと意図しないエラーが発生する恐れあり
 *
 * baudRateとmultiData_sendだけ扱い、filter_configは触らないほうがよい
 *
 * @details
 * twai_filter_config_tがどうなっているかわからない人向け @n
 * あるidを受け取るかどうかは、受け取ったmessage bit と acceptance code bit をxorして
 * それに更にacceptance mask bit and したものがすべて1となるかで決まる
 * よってtest関数でデフォルトで利用されるidの 1 << 11 - 1 を受け取らないようにするには、
 * acceptance code bitに 0
 * acceptance mask bitに 1 << 11 - 2 (最後1桁以外は1)
 * を入れれば良い
 *
 * | message bit | code bit | mask bit | result |
 * | ----------- | -------- | -------- | ------ |
 * | 0           | 0        | 0        | 1      |
 * | 0           | 0        | 1        | 1      |
 * | 0           | 1        | 0        | 0      |
 * | 0           | 1        | 1        | 1      |
 * | 1           | 0        | 0        | 0      |
 * | 1           | 0        | 1        | 1      |
 * | 1           | 1        | 0        | 1      |
 * | 1           | 1        | 1        | 1      |
 */
typedef struct
{
    long baudRate;                      /**< 通信周波数 25kbits 50kbits 100kbits 125kbits 250kbits 500kbits 1Mbits を選択できる */
    bool multiData_send;                /**< 複数文字のデータを送信できるかどうか falseならsendLine sendData系の関数は使えなくなる */
    twai_filter_config_t filter_config; /**< 受け取るidの制限 id 1 << 10 ~ ((1 << 11) - 1) だけ制限する CAN_FILTER_DEFAULT マクロのままが無難 */
} can_setting_t;

// 最上位bitが1のidを制限するfiltering設定
#define CAN_FILTER_DEFAULT \
    ((twai_filter_config_t){.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF - (1 << 31), .single_filter = true})

typedef struct
{
    int size;
    char data[8];
    uint32_t id;
} can_return_t;

/**
 * @enum can_err
 * @brief @sa CAN_CREATE::getStatus 関数の戻り値として渡される値
 */
enum can_err
{
    CAN_SUCCESS,       /**< 正常終了 */
    CAN_UNKNOWN_ERROR, /**< sendCharで送った引数がおかしい等のソフトウェア側のエラー */
    CAN_NO_ALERTS,     /**< 送信中 */
    CAN_BUS_ERROR,     /**< ACK Error等のBus Errorが発生した */
    CAN_TX_FAILED,     /**< Bus Error以外の送信エラー */
};

/**
 * @enum can_test
 * @brief @sa CAN_CREATE::test 関数の戻り値として渡される値
 */
enum can_test
{
    // CAN_SUCCESS = 0,
    // CAN_UNKNOWN_ERROR = 1, これらはcan_errで定義されている
    CAN_NO_RESPONSE_ERROR = 2, /**< 相手側のコントローラーかBUSが動いていないときのエラー */
    CAN_CONTROLLER_ERROR,      /**< 自身のコントローラーが動いていないときのエラー */
};

#endif