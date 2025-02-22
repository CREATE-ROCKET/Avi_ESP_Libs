//   ____    _    _   _    ____ ____  _____    _  _____ _____
//  / ___|  / \  | \ | |  / ___|  _ \| ____|  / \|_   _| ____|
// | |     / _ \ |  \| | | |   | |_) |  _|   / _ \ | | |  _|
// | |___ / ___ \| |\  | | |___|  _ <| |___ / ___ \| | | |___
//  \____/_/   \_\_| \_|  \____|_| \_\_____/_/   \_\_| |_____|
//
// ver 1.0.0
//
// Copyright (c) CREATE-ROCKET All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/**
 * @file CANCREATE.h
 * @brief CANCREATE.cpp header
 * @copyright CREATE-ROCKET
 */

#pragma once

#ifndef CAN_H
#define CAN_H

#include "CANCREATE_lib.h"
#include "stdint.h"

/**
 *  @class CAN_CREATE
 */
class CAN_CREATE
{
private:
    twai_general_config_t _general_config;
    twai_filter_config_t _filter_config;
    twai_timing_config_t _timing_config;
    gpio_num_t _rx;
    gpio_num_t _tx;
    gpio_num_t _bus_off;
    uint32_t _id;
    bool _return_new;
    static bool _already_begin;
    bool _multi_send;
    can_setting_t _settings;

    int _begin(can_setting_t settings, twai_mode_t mode = TWAI_MODE_NORMAL);
    void _end();
    int _test(uint32_t id);
    int _sendLine(uint32_t id, char *data, int num, uint32_t waitTime);
    int _send(twai_message_t message, uint32_t waitTime);
    int _read(twai_message_t *message, uint32_t waitTime);
    void bus_on();
    void bus_off();
    int return_with_compatiblity(int return_int);

    friend void CanWatchDog(void *pvParameter);

public:
    CAN_CREATE(can_setting_t settings);
    CAN_CREATE(bool return_new = true, bool enableCanWatchDog = true);
    ~CAN_CREATE();

    void setPins(int rx, int tx, uint32_t id = UINT32_MAX, int bus_off = GPIO_NUM_MAX);
    int begin(long baudRate);
    int begin(can_setting_t settings, int rx, int tx, uint32_t id = UINT32_MAX, int bus_off = GPIO_NUM_MAX);
    int begin(long baudRate, int rx, int tx, uint32_t id = UINT32_MAX, int bus_off = GPIO_NUM_MAX);
    int begin();
    int re_configure(can_setting_t settings, twai_mode_t mode = TWAI_MODE_NORMAL);
    void end();
    void suspend();
    void resume();
    int getStatus();
    int test(uint32_t id = (1 << 11) - 1);
    void flush();
    int available();
    char read();
    int read(char *readData, uint32_t waitTime = MAX_READ);
    int readLine(char *readData, uint32_t waitTime = MAX_READ);
    int readWithDetail(can_return_t *data, uint32_t waitTime = MAX_READ);
    uint8_t sendPacket(int id, char data);
    int sendChar(uint32_t id, char data, uint32_t waitTime = MAX_TRANSMIT);
    int sendChar(char data, uint32_t waitTime = MAX_TRANSMIT);
    int sendLine(uint32_t id, char *data, uint32_t waitTime = MAX_TRANSMIT);
    int sendLine(char *data, uint32_t waitTime = MAX_TRANSMIT);
    int sendData(uint32_t id, uint8_t *data, int num, uint32_t waitTime = MAX_TRANSMIT);
    int sendData(uint8_t *data, int num, uint32_t waitTime = MAX_TRANSMIT);
};

#endif