// version: 1.0.0
#pragma once

#ifndef NEC920_HPP
#define NEC920_HPP

#include "Arduino.h"

#define HEADER_0 0x0F
#define HEADER_1 0x5A
#define PACKET_MAX_LENGTH 254

#define SEND_CMD_ID 0x13

class NEC920
{
private:
    /* data */
    uint8_t dstID[4];
    uint8_t srcID[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    HardwareSerial *ser;

    /*送信をロックする用の変数*/
    bool isSendingLocked = 0;
    uint8_t msgNoSendingMSG;
    uint32_t timeSendingMSG;

    /*受信用変数*/
    uint8_t rxbff[256];
    uint8_t rxIndex = 0;
    bool isMsgRecieved = 0;

public:
    void setDstID(uint8_t *id);
    uint8_t makepacket(uint8_t *packet, uint8_t msgID, uint8_t msgNo, uint8_t *dst, uint8_t *src, uint8_t *parameter, uint8_t parameterLength);

    void setSerial(HardwareSerial *serial);

    int send(uint8_t *packet);
    int sendData(uint8_t msgId, uint8_t msgNo, uint8_t *parameter, uint8_t parameterLength);
    int recieve();
    int getRecievedData(uint8_t *arr);

    int getMsgID(uint8_t *arr);
    int getMsgNo(uint8_t *arr);

    void setRfConf(uint8_t Power, uint8_t Channel, uint8_t RF_Band, uint8_t CS_Mode);

    int isWirelessModuleDead();
};

#endif