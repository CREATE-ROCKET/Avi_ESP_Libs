// Copyright (c) CREATE-ROCKET All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/**
 * @file CAN.cpp
 * @brief twai driver wrapper
 * @copyright CREATE-ROCKET
 */

#include "CAN.h"

// #include "CAN_lib.h"
#include <stdint.h>
#include <driver/twai.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <string.h>

#ifdef DEBUG
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <esp_log.h>
#endif
#endif

TaskHandle_t CanWatchDogTaskHandle;

/**
 * ESP32 twai_driverは //TODO
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
                    // TODO
                    pr_debug("[FATAL ERROR] twai driver is bus_off state and cannot recovery");
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
        gpio_set_level(_bus_off, LOW);
    }
}

// private関数
void CAN_CREATE::bus_off()
{
    if (_bus_off != GPIO_NUM_MAX)
    {
        gpio_set_level(_bus_off, HIGH);
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
int CAN_CREATE::_begin(can_setting_t settings)
{
    if (_already_begin)
    {
        pr_debug("[ERROR] Begin function can be called once only.");
        return 5;
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
    _general_config = TWAI_GENERAL_CONFIG_DEFAULT(ref_tx, ref_rx, TWAI_MODE_NORMAL);
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
        vTaskResume(&CanWatchDogTaskHandle);
    }

    // エラーとしてtwai_driver_not_installしかないから無視する
    twai_reconfigure_alerts(TWAI_ALERT_TX_SUCCESS, NULL);
    return 0;
}

void CAN_CREATE::_end()
{
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
}

twai_message_t getDataMessage(int id, char *data, int num)
{
    twai_message_t message;
    message.extd = 0;         // standard format message
    message.rtr = 0;          // remote transmission request disabled. もし、dataフレームがなければtrueにしてよい
    message.ss = 0;           // single shot transmission disabled
    message.self = 0;         // self reception disabled. If true, the twai driver will receive its own transmitted data.
    message.dlc_non_comp = 0; // Following ISO 11898-1, data fields are limited to a maximum of 8 bytes.
    message.identifier = id;
    message.data_length_code = num;
    memset(message.data, 0, 8 * sizeof(char)); // data field の初期化
    memcpy(message.data, data, num * sizeof(char));
    return message;
}

int CAN_CREATE::_send(twai_message_t message, uint32_t waitTime)
{
    esp_err_t result = twai_transmit(&message, waitTime);
    if (result != ESP_OK)
    {
        switch (result)
        {
        case ESP_ERR_INVALID_ARG:
            pr_debug("[ERROR] failed to transmit data due to arguments invalid");
            return 2;
            break;
        case ESP_ERR_TIMEOUT:
            pr_debug("[ERROR] failed to transmit data due to time out\r\n you should make tx queue more bigger");
            return 3;
            break;
        case ESP_ERR_INVALID_STATE:
            pr_debug("[ERROR] failed to transmit data due to twai driver not running");
            return 4;
        default:
            pr_debug("[FATAL ERROR] failed to transmit data with unknown error");
            return 5;
            break;
        }
    }
    return 0;
}

/**
 * データをCANに実際に送信する関数
 * 最大 waitTimeだけ時間がかかる
 * numにはdataの個数が正しく入っており、num <= 8を期待して検証は行わない
 * また、char *dataは文字列だが最後にnull characterはないものとする
 *
 * 返り値:
 *  0 success
 *  2 不正なid idは11bitまで
 *  3 データに誤りがある
 *  4 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
 *  5 twaiドライバが動作していない
 *  6 unknown error 出たら教えて
 */
int CAN_CREATE::_sendLine(int id, char *data, int num, uint32_t waitTime)
{
    if (id >= 1 << 11)
    {
        pr_debug("[ERROR] id must not exceed (1 << 11 - 1)");
        return 1;
    }
    twai_message_t message = getDataMessage(id, data, num);
    return _send(message, waitTime);
}

/*
 * CAN_CREATEのコンストラクタ
 *
 * 引数:
 *  return_new 古いreturn手法を用いるか否か true推奨
 *             falseにした場合、setPin begin read sendPacket関数のみを用いる必要がある
 */
CAN_CREATE::CAN_CREATE(bool is_new, bool enableCanWatchDog)
{
    _rx = GPIO_NUM_MAX;
    _tx = GPIO_NUM_MAX;
    _bus_off = GPIO_NUM_MAX;
    _id = -1;
    _already_begin = false;
    _return_new = is_new;
    if (!_return_new)
    {
        pr_debug("Warning: This library runs in legacy compatible mode.\r\n"
                 "In this mode, only setPin, begin, read, and sendPacket functions can be used.\r\n"
                 "If you want to use the newer mode, please use CAN_CREATE(true);");
    }
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

/*
 * @deprecated
 * 互換性のために残してある
 * 利用は非推奨
 */
void CAN_CREATE::setPins(int rx, int tx, int id, int bus_off)
{
    _rx = (gpio_num_t)rx;
    _tx = (gpio_num_t)tx;
    _id = id;
    _bus_off = (gpio_num_t)bus_off;
}

/*
 * @deprecated
 * 互換性のために残してある
 * 利用は非推奨
 */
int CAN_CREATE::begin(long baudRate)
{
    can_setting_t settings;
    settings.baudRate = baudRate;
    settings.multiData_send = true;
    settings.filter_config.acceptance_code = 0;
    settings.filter_config.acceptance_mask = (1 << 32) - 2;
    settings.filter_config.single_filter = true;
    return return_with_compatiblity(_begin(settings));
}

/**
 * @brief setup内で最初に実行するべき関数 一度だけ実行できる
 * @param[in] settings can_setting_t 型の引数 通信周波数等を設定可能 通信相手と共通化しておくこと
 * @param[in] rx rxピンを指定する。CANトランシーバーのrxピンに接続されているピンを指定すること
 * @param[in] tx txピンを指定する。CANトランシーバーのtxピンに接続されているピンを指定すること
 * @param[in] id 送信するときにデフォルトで利用するidを選択する 省略できるが、その場合送るときに逐一idを指定する必要がある
 * @param[in] bus_off CANトランシーバーのSTBYピンに接続されているピンを指定する なければ省略可
 *
 * @retval 0: success
 * @retval 2: 対応していない通信周波数を指定した
 * @retval 3: twai driverがインストールできなかった (txピン、rxピンに使用できないピンが指定されてる)
 * @retval 4: twai driverがstartできなかった (基本ないはず)
 * @retval 5: begin関数が再度呼び出された
 * @retval 6: bus_offピンにOUTPUTに指定できないピンが指定された
 * @retval 7: rxピンにOUTPUTに指定できないピンが指定された
 * @retval 8: txピンにOUTPUTに指定できないピンが指定された
 */
int CAN_CREATE::begin(can_setting_t settings, int rx, int tx, int id, int bus_off)
{
    old_mode_block;
    setPins(rx, tx, id, bus_off);
    return _begin(settings);
}

/*
 * setup内等で最初に実行するべき関数 一度だけのみ実行できる
 * 引数:
 *   通信周波数 通信する相手と揃える必要がある
 *   rxピン
 *   txピン
 *   CANの通信id (send時に逐一切り替えるなら空欄で良い)
 *   canの有効化と無効化を切り替えるピン(利用しなければ空欄で良い)
 * 戻り値:
 *   0 success
 *   2 対応していない通信周波数を指定した
 *   3 twai driverがインストールできなかった (txピン、rxピンに使用できないピンが指定されてる)
 *   4 twai driverがstartできなかった (基本ないはず)
 *   5 begin関数が再度呼び出された
 *   6 bus_offピンにOUTPUTに指定できないピンが指定された
 *   7 rxピンにOUTPUTに指定できないピンが指定された
 *   8 txピンにOUTPUTに指定できないピンが指定された
 */
int CAN_CREATE::begin(long baudRate, int rx, int tx, int id, int bus_off)
{
    old_mode_block;
    setPins(rx, tx, id, bus_off);
    can_setting_t settings;
    settings.baudRate = baudRate;
    settings.multiData_send = true;
    settings.filter_config.acceptance_code = 0;
    settings.filter_config.acceptance_mask = (1 << 32) - 2;
    settings.filter_config.single_filter = true;
    return _begin(settings);
}

int CAN_CREATE::re_configure(can_setting_t settings)
{
    _end();
    return _begin(settings);
}

/*
 * CANの利用終了時に呼び出されるべき関数
 * 基本使わなくて良い プログラムでこれ以上CANを使わないときに呼び出せる
 */
void CAN_CREATE::end()
{
    _end();
    vTaskDelete(CanWatchDogTaskHandle);
}

/**
 *  CANの送信ステータスを確認できる関数
 *
 *  返り値: can_errで定義されている
 *      can_err::CAN_SUCCESS: 前回の送信が正常に成功した
 *      can_err::CAN_NO_ALERTS: まだstatusが届いていない
 *              (send関数のすぐ後の配置などが要因でまだ送信中)
 *      can_err::CAN_BUS_ERROR: ACK ErrorやBit Errorなどのバスのエラー
 *      can_err::CAN_TX_FAILED: CAN_BUS_ERROR以外の理由でデータが送信できなかった
 *      can_err::CAN_UNKNOWN_ERROR: begin関数を実行していないまたは不明なエラー
 */
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

/**
 * @brief CANが動作するかを確かめる関数
 *
 * テストデータを送信して、Bus Errorとなった場合に自分が送ったデータを自分で読み取れるかを確認する
 *
 * @attention この関数は実行に0.1秒以上かかるため、setup関数でのみ利用可能
 * @attention 実行中に送信されてきたメッセージは受け取れない可能性がある。
 * @note デフォルトでは通信id (1 << 11 - 1)で送信するため、このidはフィルタリングされる必要がある
 *          can_setting_tのfilter_configがデフォルト設定なら自動的に設定される
 *
 * @param[in] int id テストで送信するidを指定する。空欄ならid 1 << 11 - 1 で送信される
 *            CANのフィルタリング方法を知らないなら空欄のままが無難
 * @return
 *      CANの動作が正常に終了したかを確かめる
 *      - can_err::CAN_SUCCESS: 正常終了
 *      - can_err::CAN_UNKNOWN_ERROR: 失敗
 *      - can_err::CAN_
 *
 * @warning もしかしたらidを除外しているときはACK出さないかも
 */
int CAN_CREATE::test(int id)
{
    old_mode_block;
    twai_message_t message{
        0,  // standard format message
        0,  // remote transmission request disabled
        0,  // single shot transmission disabled
        0,  // self reception disabled
        id, // id
        0,  // data num
    };
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
        delay(100);
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
    try
    {
        can_setting_t settings;
        settings.baudRate = 25E3;
        settings.multiData_send = true;
        settings.filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        vTaskSuspend(CanWatchDogTaskHandle);
        re_configure(settings);
        vTaskResume(CanWatchDogTaskHandle);
        // 送信したデータが自分で受け取れるかを確かめる
        twai_message_t message_self_reception = {
            0,  // standard format message
            0,  // remote transmission request disabled
            1,  // single shot transmission enabled
            1,  // self reception enabled
            id, // id
            0,  // data num
        };
        int result = _send(message_self_reception, 0);
        if (result)
        {
            throw CAN_UNKNOWN_ERROR;
        }

        do
        {
            if (++i > 10)
                break; // 1秒以上経っても送信中ならやめる
            delay(100);
            result = getStatus();
        } while (result == can_err::CAN_NO_ALERTS);

        if (available())
        {
            can_return_t data;
            readWithDetail(&data);
            if (data.id == (1 << 11) - 1)
            {
                throw CAN_NO_RESPONSE_ERROR; // 自身のCANコントローラーは生きていてBUSか相手のCANコントローラーが死んでる
            }
        }
        throw CAN_CONTROLLER_ERROR;
    }
    catch (int err_type)
    {
        if (re_configure(backup_can_setting))
        {
            pr_debug("[FATAL ERROR] can't set setting property in test function\ncan turned off...");
            _already_begin = false;
        }
        return err_type;
    }
}

/*
 * CANトランシーバーのキューを消去する
 * 受信待ち、送信待ちのデータが消去されるため注意!!!
 */
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

/*
 * CANにデータが送られて来ているかを判別する関数
 * 戻り値が0でなければデータが送られてきており、read関数で読める
 *
 * 戻り値:
 *   0    データが受信されていない or 受信エラー
 *   1以上 データが受信キューにある
 *
 * @warning この関数は性質上エラーも0で返してしまうため注意
 */
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

int CAN_CREATE::readWithDetail(can_return_t *readData)
{
    old_mode_block;
    twai_message_t twai_message;
    esp_err_t result = twai_receive(&twai_message, MAX_READ);
    if (result != ESP_OK)
    {
        switch (result)
        {
        case ESP_ERR_TIMEOUT:
            pr_debug("[ERROR] failed to read from twai due to rx queue has no data\r\nyou must call available function before it");
            return 2;
            break;
        case ESP_ERR_INVALID_ARG:
            pr_debug("[ERROR] failed to read from twai due to the data is invalid");
            return 3;
            break;
        case ESP_ERR_INVALID_STATE:
            pr_debug("[ERROR] failed to read from twai due to the twai driver is not running");
            return 4;
        default:
            pr_debug("[FATAL ERROR] failed to read from twai due to unkown error");
            return 5;
            break;
        }
    }
    if (twai_message.dlc_non_comp)
    {
        pr_debug("[ERROR] This library needs to follow ISO 11898-1");
        return 6;
    }
    readData->size = twai_message.data_length_code;
    readData->id = twai_message.identifier;
    memcpy(readData->data, twai_message.data, twai_message.data_length_code * sizeof(char));
    return 0;
}

/*
 * CANに送られてきたデータを読む関数
 * available関数で1以上が帰ってきたときのみ利用可能
 * 8文字までのデータを受信できるが引数のcharのポインタを作る必要あり
 *
 *
 * 引数:
 *  charの配列のポインタ 9文字は入れられるサイズが必要 これにデータが入る
 * 戻り値:
 *  0 success
 *  1~4 CANに送られてきたデータを読むのに失敗した available関数を実行していない等
 *  5 得られたデータがISO 11898-1互換ではなかった
 *  6 何も入っていないデータが得られた
 */
int CAN_CREATE::readLine(char *readData)
{
    can_return_t readDataInternal;
    int result = readWithDetail(&readDataInternal);
    if (result)
    {
        return result;
    }
    if (!readDataInternal.size)
    {
        pr_debug("[ERROR] No data");
        return 7;
    }
    memcpy(readData, readDataInternal.data, readDataInternal.size * sizeof(char));
    readData[readDataInternal.size] = 0;
    return 0;
}

/*
 * CANに送られてきているデータを読む関数
 * available関数で1以上が帰ってきたときのみ利用可能
 * また、1文字までしか対応しないため相手がsendPacket以外を利用した場合には最初の文字だけを返す
 *
 * 引数:
 *  charのポインタ 成功時これにデータが入る
 * 戻り値:
 *  0 success
 *  1~4 CANに送られてきたデータを読むのに失敗した available関数を実行していない等
 *  5 得られたデータがISO 11898-1互換ではなかった
 *  6 何も入っていないデータが得られた
 */
int CAN_CREATE::read(char *readData)
{                      // old_mode_blockはreadLineで実行される
    char Data[9] = {}; // すべてnull characterで初期化
    int result = readLine(Data);
    if (result)
    {
        return result;
    }
    for (int i = 1; i < 9; i++)
    {
        if (Data[i] != 0)
        {
            pr_debug("[INFO] read function does not support 2 or more character\r\nreturn only first character");
        }
    }
    *readData = Data[0];
    return 0;
}

/*
 * CANに送られてきているデータを読む関数 互換性のために残してある
 * 読むのに失敗したときの挙動が安全でないため利用は非推奨
 */
char CAN_CREATE::read()
{ // TODO old_mode_blockが実行されちゃう
    char readData;
    if (read(&readData))
    {
        return 0; // null character
    }
    return readData;
}

/*
 * charをCANで送る関数 idを逐一指定する
 * 引数:
 *  int型の送信するid
 *  char型の送信するデータ
 *
 * 返り値:
 *  0 success
 *  2 不正なid idは11bitまで
 *  3 データに誤りがある
 *  4 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
 *  5 twaiドライバが動作していない
 *  6 unknown error 出たら教えて
 */
int CAN_CREATE::sendChar(int id, char data)
{
    old_mode_block;
    // charをそのまま送ってるため、_sendLineでchar[0]以上にアクセスしたら違反
    return _sendLine(id, &data, 1, MAX_TRANSMIT);
}

/*
 * charをCANで送る関数 beginでidを指定していなければfailする
 * 引数:
 *  char型の送信するデータ
 *
 * 返り値:
 *  0 success
 *  2 不正なid idは11bitまで
 *  3 データに誤りがある
 *  4 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
 *  5 twaiドライバが動作していない
 *  6 unknown error 出たら教えて
 */
int CAN_CREATE::sendChar(char data)
{
    old_mode_block;
    if (_id == -1)
    {
        pr_debug("[ERROR] you have to set id in begin or use sendChar(id, data)");
        return 1;
    }
    return sendChar(_id, data);
}

/*
 * 互換性のために残してある
 * 利用は非推奨
 */
uint8_t CAN_CREATE::sendPacket(int id, char data)
{
    int result = sendChar(id, data);
    if (!result)
        return 2;
    return 0; // TODO ACK ERROR
}

int CAN_CREATE::sendLine(int id, const char *data)
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

    return _sendLine(id, sendData, i, MAX_TRANSMIT);
}

int CAN_CREATE::sendLine(const char *data)
{
    old_mode_block;
    multi_send_block;
    if (_id == -1)
    {
        pr_debug("[ERROR] you have to set id in begin or use sendChar(id, data)");
        return 1;
    }
    return sendLine(_id, data);
}

int CAN_CREATE::sendData(int id, uint8_t *data, int num)
{
    old_mode_block;
    multi_send_block;
    if (num > 8)
    {
        pr_debug("[ERROR] CAN support to transfer maximum 8 character");
        return 1;
    }
    char sendData[8];
    memcpy(sendData, data, num * sizeof(int8_t));
    return _sendLine(id, sendData, num, MAX_TRANSMIT);
}

int CAN_CREATE::sendData(uint8_t *data, int num)
{
    old_mode_block;
    multi_send_block;
    if (_id == -1)
    {
        pr_debug("[ERROR] you have to set id in begin or use sendData(id, data)");
        return 1;
    }
    return sendData(_id, data, num);
}