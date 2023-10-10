// version: 1.0.0
#pragma once

#ifndef NEC920_HPP
#define NEC920_HPP

#include "Arduino.h"

namespace NEC920CONSTS
{
    constexpr uint8_t HEADER_0 = 0x0F;
    constexpr uint8_t HEADER_1 = 0x5A;
    constexpr uint8_t PACKET_MAX_LENGTH = 254;

    constexpr uint8_t MSGID_SEND = 0x11;
    constexpr uint8_t MSGID_SEND_NORESEND = 0x13;
    constexpr uint8_t MSGID_RETURN_OK = 0x00;
    constexpr uint8_t MSGID_RETURN_NG = 0x01;
}

class NEC920
{
private:
    uint8_t dummyID[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    HardwareSerial *ser;

    /*送信をロックする用の変数*/
    bool isSendingLocked = 0;
    uint8_t msgNoLastSendingMSG;
    uint32_t timeSendingLastMSG;

    /*受信用変数*/
    uint8_t rxbff[256];
    uint8_t rxIndex = 0;
    bool isMsgRecieved = 0;

    /*ピン*/
    uint8_t pin920Reset;  // output L...リセット開始
    uint8_t pin920Wakeup; // output L...省電力移行 H...通常動作
    uint8_t pin920Mode;   // input L...受信状態 H...省電力状態

    /*-----------------パケット操作関係の関数-----------------*/
    uint8_t makepacket(uint8_t *packet, uint8_t msgID, uint8_t msgNo, uint8_t *dst, uint8_t *src, uint8_t *parameter, uint8_t parameterLength);
    uint8_t getMsgID(uint8_t *arr);
    uint8_t getMsgNo(uint8_t *arr);
    uint8_t getMsgParam(uint8_t *arr);

public:
    /*-----------------端子インターフェースの関数-----------------*/
    void setPin(uint8_t pin920Reset, uint8_t pin920Wakeup, uint8_t pin920Mode);
    void goSleep();
    void goWakeUp();

    /*-----------------シリアルポート関係の関数-----------------*/
    void beginSerial(HardwareSerial *serial, uint32_t baudrate, uint8_t rx, uint8_t tx);
    uint8_t isSerialValid();

    /*-----------------送受信コア-----------------*/
    uint8_t recieve();

    /*-----------------各種コマンド-----------------*/
    uint8_t setRfConf(uint8_t Power, uint8_t Channel, uint8_t RF_Band, uint8_t CS_Mode);
};

#endif