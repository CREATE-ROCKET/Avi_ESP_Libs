// Copyright (c) CREATE-ROCKET All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/**
 * @file CANCREATE.cpp
 * @brief TWAI driver wrapper
 * @copyright CREATE-ROCKET
 */

#include "CANCREATE.h"
#include "CANCREATE_lib.h"

#include <stdint.h>
#include <driver/twai.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <string.h>

#if DEBUG_CAN
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <esp_log.h>
#endif
#endif

bool CAN_CREATE::_already_begin = false;

/**
 * @brief CANウォッチドッグタスクのタスクハンドル
 */
TaskHandle_t CanWatchDogTaskHandle;

/**
 * @brief BUS_OFFになった場合に回復させるタスク
 * @details ESP32 twai_driverはエラーが256回以上引き起こされるとBUS_OFFになる。
 *          このため、BUS_OFFになったら自動的に回復するWatchDogを仕掛けることにする。
 *          回復できなければ送信をやめる
 * @param pvParameter タスクのパラメーターへのポインター
 */
void CanWatchDog(void *pvParameter)
{
    twai_status_info_t twai_status;
    for (;;)
    {
        if (twai_get_status_info(&twai_status) == ESP_OK)
        {
            if (twai_status.state == TWAI_STATE_BUS_OFF)
            {
                if (twai_initiate_recovery() == ESP_ERR_INVALID_STATE)
                {
                    pr_debug("[FATAL ERROR] twai driver is bus_off state and cannot recovery");
                    CAN_CREATE::_already_begin = false; // re_configureが必要
                }
            }
        }
        /* ESP_OK以外は
            ESP_ERR_INVALID_ARG: Arguments are invalid
            ESP_ERR_INVALID_STATE: TWAI driver is not installed
            なので一旦無視
        */
        vTaskDelay(500 / portTICK_PERIOD_MS); // 0.5秒おきに
    }
}

// private関数
void CAN_CREATE::bus_on()
{
    if (_bus_off != GPIO_NUM_MAX)
    {
        gpio_set_level(_bus_off, 0 /*LOW*/);
    }
}

// private関数
void CAN_CREATE::bus_off()
{
    if (_bus_off != GPIO_NUM_MAX)
    {
        gpio_set_level(_bus_off, 1 /*HIGH*/);
    }
}

// private関数
/*
 * 昔のライブラリとの互換性を保つためのもの
 * 昔のライブラリは0で失敗、1で成功としていたっぽい
 */
int CAN_CREATE::return_with_compatiblity(int return_int)
{
    if (_return_new)
    {
        return return_int;
    }
    if (return_int == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// private関数
int CAN_CREATE::_begin(can_setting_t settings, twai_mode_t mode)
{
    if (_already_begin)
    {
        pr_debug("[ERROR] Begin function can be called once only.");
        return 5;
    }
    if (!_return_new)
    {
        pr_debug("Warning: This library runs in legacy compatible mode.\r\n"
                 "In this mode, only setPin, begin, read, and sendPacket functions can be used.\r\n"
                 "If you want to use the newer mode, please use CAN_CREATE(true);");
    }
    if (_bus_off != GPIO_NUM_MAX)
    {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(_bus_off))
        {
            pr_debug("[ERROR] invalid bus_off pin selected");
            return 6;
        }
        gpio_pad_select_gpio(_bus_off);
        gpio_set_direction(_bus_off, GPIO_MODE_OUTPUT);
        bus_on();
    }
    if (_rx == GPIO_NUM_MAX || _tx == GPIO_NUM_MAX)
    {
        pr_debug("[ERROR] please set rx and tx pin properly");
        return 1;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(_rx))
    {
        pr_debug("[ERROR] invalid rx pin please check the pin can used for output");
        return 7;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(_tx))
    {
        pr_debug("[ERROR] invalid tx pin please check the pin can used for output");
        return 8;
    }
    gpio_num_t ref_rx = _rx; // privateのままdefineを使いたい
    gpio_num_t ref_tx = _tx;
    _general_config = TWAI_GENERAL_CONFIG_DEFAULT(ref_tx, ref_rx, mode);
    _filter_config = settings.filter_config;
    _settings = settings;
    _multi_send = true;
    switch (settings.baudRate)
    {
    case (long)1000E3:
        _timing_config = TWAI_TIMING_CONFIG_1MBITS();
        break;

    case (long)500E3:
        _timing_config = TWAI_TIMING_CONFIG_500KBITS();
        break;

    case (long)250E3:
        _timing_config = TWAI_TIMING_CONFIG_250KBITS();
        break;

        // twai library doesn't support 200kbps

    case (long)125E3:
        _timing_config = TWAI_TIMING_CONFIG_125KBITS();
        break;

    case (long)100E3:
        _timing_config = TWAI_TIMING_CONFIG_100KBITS();
        break;

        // twai library doesn't support 80kbps

    case (long)50E3:
        _timing_config = TWAI_TIMING_CONFIG_50KBITS();
        break;
    case (long)25E3:
        _timing_config = TWAI_TIMING_CONFIG_25KBITS();
        break;
    default:
        pr_debug("[ERROR] incorrect baudrate!!!");
        return 2;
        break;
    }
    esp_err_t result = twai_driver_install(&_general_config, &_timing_config, &_filter_config);
    if (result != ESP_OK)
    {
        pr_debug("[ERROR] failed to init twai driver %d", result);
        return 3;
    }
    result = twai_start();
    if (result != ESP_OK)
    {
        pr_debug("[ERROR] failed to start twai");
        return 4;
    }
    _already_begin = true;

    if (CanWatchDogTaskHandle)
    {
        vTaskResume(CanWatchDogTaskHandle);
    }

    // エラーとしてtwai_driver_not_installしかないから無視する
    twai_reconfigure_alerts(TWAI_ALERT_TX_SUCCESS, NULL);
    return 0;
}

void CAN_CREATE::_end()
{
    vTaskSuspend(CanWatchDogTaskHandle);
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK)
        return;
    if (status.state == TWAI_STATE_RUNNING || status.state == TWAI_STATE_RECOVERING)
    {
        twai_stop(); // TWAI_STATE_RECOVERINGの場合はだめかもしれない
    }
    // uninstallの前にtwai自体を止める必要がある
    esp_err_t result = twai_driver_uninstall();
    if (result != ESP_OK)
    {
        pr_debug("[ERROR] failed to uninstall twai driver %d", result);
    }
    _already_begin = false;
}

inline twai_message_t getDataMessage(uint32_t id, char *data, int num)
{
    twai_message_t message;
    message.extd = 0;         // standard format message
    message.rtr = 0;          // remote transmission request disabled. もし、dataフレームがなければtrueにしてよい
    message.ss = 0;           // single shot transmission disabled
    message.self = 0;         // self reception disabled. If true, the twai driver will receive its own transmitted data.
    message.dlc_non_comp = 0; // As per ISO 11898-1, data fields are limited to a maximum of 8 bytes.
    message.identifier = id;
    message.data_length_code = num;
    memset(message.data, 0, 8 * sizeof(char)); // data field の初期化
    memcpy(message.data, data, num * sizeof(char));
    return message;
}

/**
 * @brief CANバスでメッセージを送信
 * @param[in] message 送信するメッセージ
 * @param[in] waitTime メッセージが送信されるのを待機する時間
 * @return 成功した場合は0、それ以外の場合はエラーコード
 */

int CAN_CREATE::_send(twai_message_t message, uint32_t waitTime)
{
    esp_err_t result = twai_transmit(&message, waitTime);
    if (result != ESP_OK)
    {
        switch (result)
        {
        case ESP_ERR_INVALID_ARG:
            pr_debug("[ERROR] Failed to transmit data due to invalid arguments");
            return 4;
            break;
        case ESP_ERR_TIMEOUT:
            pr_debug("[ERROR] Failed to transmit data due to timeout\r\n You should increase the TX queue size");
            return 5;
            break;
        case ESP_ERR_INVALID_STATE:
            pr_debug("[ERROR] failed to transmit data due to twai driver not running");
            return 6;
        default:
            pr_debug("[FATAL ERROR] failed to transmit data with unknown error");
            return 7;
            break;
        }
    }
    return 0;
}

int CAN_CREATE::_read(twai_message_t *message, uint32_t waitTime)
{
    esp_err_t result = twai_receive(message, waitTime);
    if (result != ESP_OK)
    {
        switch (result)
        {
        case ESP_ERR_TIMEOUT:
            pr_debug("[ERROR] failed to read from twai due to rx queue has no data\r\nyou must call available function before it");
            return 1;
            break;
        case ESP_ERR_INVALID_ARG:
            pr_debug("[ERROR] failed to read from twai due to the data is invalid");
            return 2;
            break;
        case ESP_ERR_INVALID_STATE:
            pr_debug("[ERROR] failed to read from twai due to the twai driver is not running");
            return 3;
        default:
            pr_debug("[FATAL ERROR] failed to read from twai due to unkown error");
            return 4;
            break;
        }
    }
    return 0;
}

/**
 * @brief データをCANに実際に送信する関数 最大 waitTimeだけ時間がかかる
 * data[num]までのアクセスしかしないことを保証する
 *
 * @note numにはdataの個数が正しく入っており、num <= 8を期待して検証は行わない
 * @note また、char *dataは文字列だが最後にnull characterはないものとする
 *
 * @retval 0 success
 * @retval 3 不正なid idは11bitまで
 * @retval 4 データに誤りがある
 * @retval 5 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
 * @retval 6 twaiドライバが動作していない
 * @retval 7 unknown error
 */
int CAN_CREATE::_sendLine(uint32_t id, char *data, int num, uint32_t waitTime)
{
    if (id >= (1 << 11))
    {
        pr_debug("[ERROR] ID must not exceed (1 << 11 - 1)");
        return 3;
    }
    twai_message_t message = getDataMessage(id, data, num);
    return _send(message, waitTime);
}

CAN_CREATE::CAN_CREATE(bool is_new, bool enableCanWatchDog)
{
    _rx = GPIO_NUM_MAX;
    _tx = GPIO_NUM_MAX;
    _bus_off = GPIO_NUM_MAX;
    _id = UINT32_MAX;
    _already_begin = false;
    _return_new = is_new;

    if (enableCanWatchDog)
    {
        xTaskCreatePinnedToCore(CanWatchDog, "CanWatchDog", 1024, NULL, tskIDLE_PRIORITY, &CanWatchDogTaskHandle, tskNO_AFFINITY);
        vTaskSuspend(CanWatchDogTaskHandle);
    }
}

CAN_CREATE::~CAN_CREATE() noexcept
{
    CAN_CREATE::end();
}

void CAN_CREATE::setPins(int rx, int tx, uint32_t id, int bus_off)
{
    _rx = (gpio_num_t)rx;
    _tx = (gpio_num_t)tx;
    _id = id;
    _bus_off = (gpio_num_t)bus_off;
}

int CAN_CREATE::begin(long baudRate)
{
    can_setting_t settings;
    settings.baudRate = baudRate;
    settings.multiData_send = true;
    settings.filter_config.acceptance_code = 0;
    settings.filter_config.acceptance_mask = (1 << 29) - 2;
    settings.filter_config.single_filter = true;
    return return_with_compatiblity(_begin(settings));
}

int CAN_CREATE::begin(can_setting_t settings, int rx, int tx, uint32_t id, int bus_off)
{
    old_mode_block;
    setPins(rx, tx, id, bus_off);
    return _begin(settings);
}

int CAN_CREATE::begin(long baudRate, int rx, int tx, uint32_t id, int bus_off)
{
    old_mode_block;
    setPins(rx, tx, id, bus_off);
    can_setting_t settings;
    settings.baudRate = baudRate;
    settings.multiData_send = true;
    settings.filter_config = CAN_FILTER_DEFAULT;
    return _begin(settings);
}

int CAN_CREATE::re_configure(can_setting_t settings, twai_mode_t mode)
{
    _end();
    return _begin(settings, mode);
}

void CAN_CREATE::end()
{
    _end();
    vTaskDelete(CanWatchDogTaskHandle);
}

void CAN_CREATE::suspend()
{
    vTaskSuspend(CanWatchDogTaskHandle);
    twai_stop();
    bus_off();
}

void CAN_CREATE::resume()
{
    bus_on();
    twai_start();
    vTaskResume(CanWatchDogTaskHandle);
}

int CAN_CREATE::getStatus()
{
    uint32_t status;
    esp_err_t result = twai_read_alerts(&status, 0);
    if (result == ESP_ERR_TIMEOUT)
    {
        return CAN_NO_ALERTS; // 1: twai driver didn't receive any alerts
    }
    if (result == ESP_OK)
    {
        if (status & TWAI_ALERT_TX_SUCCESS)
        {
            return CAN_SUCCESS; // 0: data was successfully sent
        }
        if (status & TWAI_ALERT_TX_FAILED)
        {
            if (status & TWAI_ALERT_BUS_ERROR)
            {
                return CAN_BUS_ERROR; // 2: failed to send data due to A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus
            }
            return CAN_TX_FAILED; // 3: failed to send data due to other types of error
        }
    }
    pr_debug("[ERROR] failed to get status info");
    return CAN_UNKNOWN_ERROR;
}

inline int CAN_CREATE::_test(uint32_t id)
{
    can_setting_t settings;
    settings.baudRate = 25E3;
    settings.multiData_send = true;
    settings.filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    int result = re_configure(settings, TWAI_MODE_NO_ACK);
    if (result)
    {
        pr_debug("[ERROR] failed to re_configure");
        return CAN_UNKNOWN_ERROR;
    }
    // 送信したデータが自分で受け取れるかを確かめる
    twai_message_t message_self_reception;
    message_self_reception.extd = 0;
    message_self_reception.rtr = 0;
    message_self_reception.ss = 0;
    message_self_reception.self = 1;
    message_self_reception.dlc_non_comp = 0;
    message_self_reception.identifier = id;
    message_self_reception.data_length_code = 0;

    result = _send(message_self_reception, 0);
    if (result)
    {
        return CAN_UNKNOWN_ERROR;
    }
    int i = 0;
    do
    {
        if (++i > 10)
            break; // 1秒以上経っても送信中ならやめる
        vTaskDelay(100 / portTICK_PERIOD_MS);
        result = getStatus();
    } while (result == can_err::CAN_NO_ALERTS);

    if (available())
    {
        can_return_t data;
        int result = readWithDetail(&data);
        if (result)
        {
            return CAN_UNKNOWN_ERROR;
        }
        if (data.id == id)
        {
            return CAN_NO_RESPONSE_ERROR; // 自身のCANコントローラーは生きていてBUSか相手のCANコントローラーが死んでる
        }
        else
        {
            pr_debug("[ERROR] expected id: %u, but found: %u", id, data.id);
        }
    }
    return CAN_CONTROLLER_ERROR;
}

int CAN_CREATE::test(uint32_t id)
{
    old_mode_block;
    twai_message_t message;
    message.extd = 0;             // standard format message
    message.rtr = 0;              // remote transmission request disabled
    message.ss = 0;               // single shot transmission disabled
    message.self = 0;             // self reception request disabled
    message.dlc_non_comp = 0;     // compatible with ISO 11898-1
    message.identifier = id;      // id
    message.data_length_code = 0; // data length
    int result = _send(message, 0);
    if (result)
    {
        return CAN_UNKNOWN_ERROR;
    }
    int i = 0;
    do
    {
        if (++i > 10)
            break; // 1秒以上経っても送信中ならやめる
        vTaskDelay(100 / portTICK_PERIOD_MS);
        result = getStatus();
    } while (result == can_err::CAN_NO_ALERTS);

    if (result == can_err::CAN_SUCCESS)
    {
        return CAN_SUCCESS;
    }
    if (result == can_err::CAN_UNKNOWN_ERROR)
    {
        return CAN_UNKNOWN_ERROR;
    }
    // 通常の送信に失敗したので、自分で送ったデータを自分で受け取れるかを確かめる
    can_setting_t backup_can_setting = _settings; // 今の状態を保存する
    pr_debug("[INFO] CAN test failed, trying self-reception...");
    // throw catch文を使っていたが、abortする等不安定だったので別関数にする。
    result = _test(id);
    if (re_configure(backup_can_setting))
    {
        pr_debug("[FATAL ERROR] can't set setting property in test function\r\ncan turned off...");
        _already_begin = false;
        return CAN_UNKNOWN_ERROR;
    }
    return result;
}

void CAN_CREATE::flush()
{
    if (twai_clear_receive_queue() != ESP_OK)
    {
        pr_debug("[ERROR] failed to clear receive queue");
    }
    if (twai_clear_transmit_queue() != ESP_OK)
    {
        pr_debug("[ERROR] failed to clear transmit queue");
    }
}

int CAN_CREATE::available()
{
    twai_status_info_t twai_status;
    esp_err_t result = twai_get_status_info(&twai_status);
    if (result != ESP_OK)
    {
        pr_debug("[ERROR] failed to get twai status info");
        return 0;
    }
    return twai_status.msgs_to_rx;
}

int CAN_CREATE::readWithDetail(can_return_t *readData, uint32_t waitTime)
{
    old_mode_block;
    twai_message_t message;
    int result = _read(&message, waitTime);
    if (result)
    {
        return result;
    }
    if (message.dlc_non_comp)
    {
        pr_debug("[ERROR] This library needs to follow ISO 11898-1");
        return 6;
    }
    readData->size = message.data_length_code;
    readData->id = message.identifier;
    memcpy(readData->data, message.data, message.data_length_code * sizeof(char));
    return 0;
}

int CAN_CREATE::readLine(char *readData, uint32_t waitTime)
{
    old_mode_block;
    twai_message_t twai_message;
    int result = _read(&twai_message, waitTime);
    if (result)
    {
        return result;
    }
    if (twai_message.dlc_non_comp)
    {
        pr_debug("[ERROR] This library needs to follow ISO 11898-1");
        return 5;
    }
    if (!twai_message.data_length_code)
    {
        pr_debug("[ERROR] No data");
        return 6;
    }
    memcpy(readData, twai_message.data, twai_message.data_length_code * sizeof(char));
    readData[twai_message.data_length_code] = 0;
    return 0;
}

int CAN_CREATE::read(char *readData, uint32_t waitTime)
{                      // old_mode_blockはreadLineで実行される
    char Data[9] = {}; // すべてnull characterで初期化
    int result = readLine(Data, waitTime);
    if (result)
    {
        return result;
    }
    for (int i = 1; i < 9; i++)
    {
        if (Data[i] != 0)
        {
            pr_debug("[INFO] read function does not support 2 or more character\r\nreturn only first character");
            break;
        }
    }
    *readData = Data[0];
    return 0;
}

char CAN_CREATE::read()
{
    twai_message_t message;
    if (_read(&message, 0))
    {
        return 0;
    }
    char readData = message.data[0];
    return readData;
}

int CAN_CREATE::sendChar(uint32_t id, char data, uint32_t waitTime)
{
    old_mode_block;
    // charをそのまま送ってるため、_sendLineでchar[0]以上にアクセスしたら違反
    return _sendLine(id, &data, 1, waitTime);
}

int CAN_CREATE::sendChar(char data, uint32_t waitTime)
{
    old_mode_block;
    if (_id == UINT32_MAX)
    {
        pr_debug("[ERROR] you have to set id in begin or use sendChar(id, data)");
        return 1;
    }
    return sendChar(_id, data, waitTime);
}

uint8_t CAN_CREATE::sendPacket(int id, char data)
{
    int result = _sendLine(static_cast<uint32_t>(id), &data, 1, 0);
    if (!result)
        return 2; // success
    return 0;     // PAR ERROR
}

int CAN_CREATE::sendLine(uint32_t id, char *data, uint32_t waitTime)
{
    old_mode_block;
    multi_send_block;
    char sendData[8];
    int i = 0;
    while (data[i] != 0)
    {               // char型配列文字列の終端にあるnull characterがくるまで なかったら配列外アクセスになるから注意!!!!
        if (i >= 8) // dataが8個以上あった場合エラー
        {
            pr_debug("[ERROR] CAN support to transfer maximum 8 character");
            return 2;
        }
        sendData[i] = data[i];
        i++;
    }

    return _sendLine(id, sendData, i, waitTime);
}

int CAN_CREATE::sendLine(char *data, uint32_t waitTime)
{
    if (_id == UINT32_MAX)
    {
        pr_debug("[ERROR] you have to set id in begin or use sendChar(id, data)");
        return 1;
    }
    return sendLine(_id, data, waitTime);
}

int CAN_CREATE::sendData(uint32_t id, uint8_t *data, int num, uint32_t waitTime)
{
    old_mode_block;
    multi_send_block;
    if (num > 8)
    {
        pr_debug("[ERROR] CAN support to transfer maximum 8 character");
        return 2;
    }
    return _sendLine(id, reinterpret_cast<char *>(data), num, waitTime);
}

int CAN_CREATE::sendData(uint8_t *data, int num, uint32_t waitTime)
{
    if (_id == UINT32_MAX)
    {
        pr_debug("[ERROR] you have to set id in begin or use sendData(id, data)");
        return 1;
    }
    return sendData(_id, data, num, waitTime);
}
