#ifndef NEC_920_HPP
#define NEC_920_HPP

#define HEADER_0 0x0F
#define HEADER_1 0x5A
#define PACKET_MAX_LENGTH 254

#include <Arduino.h>

class NEC920PRTCL
{
private:
    HardwareSerial Ser;

    /*受信用変数*/
    uint8_t rxbff[256];
    uint8_t rxIndex = 0;
    bool isMsgRecieved = 0;

    /*送信をロックする用の変数*/
    bool isSendingLocked = 0;
    uint8_t msgNoSendingMSG;
    uint32_t timeSendingMSG;

    void makepacket(uint8_t *packet, uint8_t msgID, uint8_t msgNo, uint8_t dstID[4], uint8_t srcID[4], uint8_t *parameter, uint8_t parameterLength);
    int recieveFromWirelessmodule();
    int send(uint8_t *packet);

public:
    void begin(HardwareSerial _Ser, uint8_t power, uint8_t channel, uint8_t rf_band, uint8_t cs_mode);
    bool isSendAvileable();
    void sendTelemetry(uint8_t *parameter, uint8_t parameterLength, uint8_t msgID = 0x13, uint8_t msgNo, uint8_t dstID[4]);
    int recieveData(uint8_t *getParam);
    int checkWirelessmodule();
};

/**
 * @brief パケット作成関数
 *
 * @param packet
 * @param msgID メッセージのID
 * @param msgNo メッセージの識別番号
 * @param dstID 4byte 送信先デバイスID
 * @param srcID 4byte 送信元デバイスID UARTで通信するので基本的に0xFFFFFFFF
 * @param parameter パケットのパラメータ
 * @param parameterLength パラメータの長さ
 * @return int パケットの長さ(パラメータの長さ+13)
 */
int NEC920::makepacket(uint8_t *packet, uint8_t msgID, uint8_t msgNo, uint8_t dstID[4], uint8_t srcID[4], uint8_t *parameter, uint8_t parameterLength)
{
    packet[0] = HEADER_0;
    packet[1] = HEADER_1;
    packet[2] = parameterLength + 13;
    packet[3] = msgID;
    packet[4] = msgNo;
    for (int i = 0; i < 4; i++)
    {
        packet[5 + i] = dstID[i];
    }
    for (int i = 0; i < 4; i++)
    {
        packet[9 + i] = srcID[i];
    }
    for (int i = 0; i < parameterLength; i++)
    {
        packet[13 + i] = parameter[i];
    }
    return packetLength;
}

/**
 * @brief 受信関数
 * 無線通信モジュールからの受信を行う関数，受信したデータはrxbffに格納される．
 * 受信したデータはisMsgRecievedで管理され，isMsgRecievedが1の時は受信済み．
 * @return int 0...正常終了 1...受信済み 2...シリアルポートが設定されていない
 */
int NEC920::recieveFromWirelessmodule()
{
    if (Ser == NULL)
    {
        return 2;
    }

    if (isMsgRecieved == 1)
    {
        return 1;
    }

    while (Ser.available() > 0)
    {
        if (rxIndex == 0)
        {
            if (Ser.read() == HEADER_0)
            {
                rxbff[rxIndex] = HEADER_0;
                rxIndex++;
            }
        }
        else if (rxIndex == 1)
        {
            if (Ser.read() == HEADER_1)
            {
                rxbff[rxIndex] = HEADER_1;
                rxIndex++;
            }
            else
            {
                rxIndex = 0;
            }
        }
        else if (rxIndex == 2)
        {
            rxbff[rxIndex] = Ser.read();
            rxIndex++;
        }
        else if ((rxIndex > 2) && (rxbff[2] - 1 == rxIndex))
        {
            rxbff[rxIndex] = Ser.read();
            rxIndex = 0;
            isMsgRecieved = 1;
            return 0;
        }
        else
        {
            rxbff[rxIndex] = Ser.read();
            rxIndex++;
        }
    }
    return 0;
}

bool NEC920::isSendAvileable()
{
    return !isSendingLocked;
}

int NEC920::send(uint8_t *packet)
{
    if (isSendingLocked == 1)
    {
        return 1;
    }
    isSendingLocked = 1;
    msgNoSendingMSG = packet[4];
    Ser.write(packet, packet[2]);
    timeSendingMSG = micros();

    return 0;
}

void NEC920::begin(HardwareSerial _Ser, uint8_t power, uint8_t channel, uint8_t rf_band, uint8_t cs_mode)
{
    Ser = _Ser;

    uint8_t rfConfParam[4] = {power, channel, rf_band, cs_mode};
    uint8_t rfConfPacket[17];
    makepacket(rfConfPacket, 0x21, 0x00, 0xFFFFFFFF, 0xFFFFFFFF, rfConfParam, 4);
    send(rfConfPacket);
    while (isMsgRecieved == 0)
    {
        recieveFromWirelessmodule();
    }
    isSendingLocked = 0;
    if (rxbff[3] != 0x00)
    {
        ESP_LOGE("NEC920", "RF Configuration Error");
    }
    isMsgRecieved = 0;
}

void NEC920::sendTelemetry(uint8_t *parameter, uint8_t parameterLength, uint8_t msgID = 0x13, uint8_t msgNo, uint8_t dstID[4])
{
    uint8_t telemetryPacket[PACKET_MAX_LENGTH];
    makepacket(telemetryPacket, msgID, msgNo, dstID, 0xFFFFFFFF, parameter, parameterLength);
    send(telemetryPacket);
}

/**
 * @brief 受信関数
 * 同時に送信のロックも解除しているため，高頻度での繰り返し実行を推奨する．
 * @param getParam 受信したパラメータを格納する配列
 * @return int 受信したパラメータの長さ
 */
int NEC920::recieveData(uint8_t *getParam)
{
    recieveFromWirelessmodule();
    if (isMsgRecieved == 1)
    {
        isMsgRecieved = 0;
        if ((rxbff[4] == msgNoSendingMSG) && (rxbff[3] == 0x00 || rxbff[3] == 0x01 || rxbff[3] == 0x12))
        {
            isSendingLocked = 0;
            return 0;
        }
        else if (rxbff[3] == 0x11 || rxbff[3] == 0x13)
        {
            memcpy(getParam, rxbff + 13, rxbff[2] - 13);
            return (rxbff[2] - 13);
        }
    }
    return 0;
}

/**
 * @brief ワイヤレスモジュールの状態を確認する関数
 *
 * @return int 0なら正常，1なら異常，reset端子による再起動を推奨
 */
int NEC920::checkWirelessmodule()
{
    if (isSendingLocked == 0)
    {
        return 0;
    }
    else if (isSendingLocked == 1)
    {
        if (micros() - timeSendingMSG > 1000000)
        {
            isSendingLocked = 0;
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

#endif /* NEC_920_HPP */