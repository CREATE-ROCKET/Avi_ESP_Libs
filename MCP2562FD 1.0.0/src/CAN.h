// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#ifndef CAN_H
#define CAN_H

#include "ESP32SJA1000.h"

class CAN_CREATE : public ESP32SJA1000Class {
   public:
    int sendPacket(int id, char data);  // send data
    int available();                    // receive data
    int read(long* pID);
    int sendBytes(int id, uint8_t* tx, size_t size);
};

#endif
