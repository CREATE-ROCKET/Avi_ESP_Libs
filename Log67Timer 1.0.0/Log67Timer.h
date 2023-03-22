// version: 1.0.0
#pragma once

#ifndef LogTIMER_H
#define LogTIMER_H
#include <Arduino.h>

class Log67Timer
{
public:
    unsigned long Gettime_record();
    unsigned long start_time;
    unsigned long time;
    bool start_flag = true;
};

unsigned long Log67Timer::Gettime_record()
{
    time = micros();
    time -= start_time;
    return time;
}

#endif