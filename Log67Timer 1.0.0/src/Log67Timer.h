// Copyright [2023] <CREATE-ROCKET>
// version: 1.0.0
#pragma once

#ifndef LogTIMER_H
#define LogTIMER_H
#include <Arduino.h>

class Log67Timer {
   public:
    unsigned int64 Gettime_record();
    unsigned int64 start_time;
    unsigned int64 time;
    bool start_flag = true;
};

unsigned int64 Log67Timer::Gettime_record() {
    time = micros();
    time -= start_time;
    return time;
}

#endif
