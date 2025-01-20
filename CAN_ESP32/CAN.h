//   ____ ____  _____    _  _____ _____    ____    _    _   _
//  / ___|  _ \| ____|  / \|_   _| ____|  / ___|  / \  | \ | |
// | |   | |_) |  _|   / _ \ | | |  _|   | |     / _ \ |  \| |
// | |___|  _ <| |___ / ___ \| | | |___  | |___ / ___ \| |\  |
//  \____|_| \_\_____/_/   \_\_| |_____|  \____/_/   \_\_| \_|
//
// ver 0.2.0
//
// Copyright (c) CREATE All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
//
// This library uses Nomal mode in Operating Mode

#pragma once

#ifndef CAN_H
#define CAN_H

#include "CAN_lib.h"
#include "stdint.h"

class CAN_CREATE
{
private:
    twai_general_config_t _general_config;
    twai_filter_config_t _filter_config;
    twai_timing_config_t _timing_config;
    gpio_num_t _rx;
    gpio_num_t _tx;
    gpio_num_t _bus_off;
    int _id;
    int _begin(long baudRate);
    int _sendLine(int id, char *data, int num, uint32_t waitTime);
    void bus_on();
    void bus_off();
    int return_with_compatiblity(int return_int);
    bool return_new;
    bool already_begin;

public:
    // CAN_CREATE(can_setting_t settings);
    CAN_CREATE(bool return_new = false, bool enableCanWatchDog = true);
    ~CAN_CREATE();

    void setPins(int rx, int tx, int id = -1, int bus_off = GPIO_NUM_MAX);
    int begin(long baudRate);
    int begin(long baudRate, int rx, int tx, int id = -1, int bus_off = GPIO_NUM_MAX);
    int begin();
    void end();
    int test();
    void flush();
    int available();
    char read();
    int read(char *readData);
    int readLine(char *readData);
    uint8_t sendPacket(int id, char data);
    int sendChar(int id, char data);
    int sendChar(char data);
    int sendLine(int id, const char *data);
    int sendLine(const char *data);
    int sendData(int id, uint8_t *data, int num);
    int sendData(uint8_t *data, int num);
};

#endif