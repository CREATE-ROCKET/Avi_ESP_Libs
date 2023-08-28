#include "NEC920.hpp"

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
uint8_t NEC920::makepacket(uint8_t *packet, uint8_t msgID, uint8_t msgNo, uint8_t *dst, uint8_t *src, uint8_t *parameter, uint8_t parameterLength)
{
    packet[0] = HEADER_0;
    packet[1] = HEADER_1;
    packet[2] = parameterLength + 13;
    packet[3] = msgID;
    packet[4] = msgNo;
    for (int i = 0; i < 4; i++)
    {
        packet[5 + i] = dst[i];
    }
    for (int i = 0; i < 4; i++)
    {
        packet[9 + i] = src[i];
    }
    for (int i = 0; i < parameterLength; i++)
    {
        packet[13 + i] = parameter[i];
    }
    return parameterLength + 13;
}

/**
 * @brief 送信先デバイスID設定関数
 *
 * @param id 4byte 送信先デバイスID
 */
void NEC920::setDstID(uint8_t *id)
{
    for (int i = 0; i < 4; i++)
    {
        dstID[i] = id[i];
    }
}

/**
 * @brief シリアルポート設定関数
 *
 * @param serial シリアルポート
 */
void NEC920::setSerial(HardwareSerial *serial)
{
    this->ser = serial;
}

/**
 * @brief
 *
 * @param packet
 * @return int
 */
int NEC920::send(uint8_t *packet)
{
    if (isSendingLocked == 1)
    {
        return 1;
    }
    isSendingLocked = 1;
    msgNoSendingMSG = packet[4];
    ser->write(packet, packet[2]);
    timeSendingMSG = micros();

    return 0;
}

/**
 * @brief データ送信関数
 * 送信先デバイスIDはsetDstIDで設定されたIDになる．
 * @param arr 送信するデータ
 * @param length 送信するデータの長さ
 * @return int 0...正常終了 1...送信ロック中 2...シリアルポートが設定されていない
 */
int NEC920::sendData(uint8_t msgId, uint8_t msgNo, uint8_t *parameter, uint8_t parameterLength)
{
    uint8_t packet[PACKET_MAX_LENGTH];

    makepacket(packet, msgId, msgNo, dstID, srcID, parameter, parameterLength);

    send(packet);

    return 0;
}

/**
 * @brief 受信関数
 * 無線通信モジュールからの受信を行う関数，受信したデータはrxbffに格納される．
 * 受信したデータはisMsgRecievedで管理され，isMsgRecievedが1の時は受信済み．
 * @return int 0...正常終了 1...シリアルポートが設定されていない
 */
int NEC920::recieve()
{
    if (ser == NULL)
    {
        return 1;
    }

    if (isMsgRecieved == 1)
    {
        return 0;
    }

    while (ser->available() > 0)
    {
        if (rxIndex == 0)
        {
            if (ser->read() == HEADER_0)
            {
                rxbff[rxIndex] = HEADER_0;
                rxIndex++;
            }
        }
        else if (rxIndex == 1)
        {
            if (ser->read() == HEADER_1)
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
            rxbff[rxIndex] = ser->read();
            rxIndex++;
        }
        else if ((rxIndex > 2) && (rxbff[2] - 1 == rxIndex))
        {
            // 受信完了
            rxbff[rxIndex] = ser->read();
            rxIndex = 0;
            isMsgRecieved = 1;

            // 送信ロック解除(応答idが0x00または0x12でメッセージ識別番号が送信したメッセージ識別番号と一致する場合)
            if ((getMsgID(rxbff) == 0x00 || getMsgID(rxbff) == 0x12) && getMsgNo(rxbff) == msgNoSendingMSG)
            {
                if (getMsgID(rxbff) == 0x00)
                {
                    ESP_LOGV("NEC920", "ACK(success):%02X", getMsgNo(rxbff));
                }
                else
                {
                    ESP_LOGV("NEC920", "ACK(fail):%02X", getMsgNo(rxbff));
                }
                isMsgRecieved = 0;
                isSendingLocked = 0;
            }

            return 0;
        }
        else
        {
            rxbff[rxIndex] = ser->read();
            rxIndex++;
        }
    }
    return 0;
}

/**
 * @brief 受信データのメッセージIDを取得する関数
 *
 * @return int メッセージID
 */
int NEC920::getMsgID(uint8_t *arr)
{
    return arr[3];
}

/**
 * @brief 受信データのメッセージ識別番号を取得する関数
 *
 * @return int メッセージ識別番号
 */
int NEC920::getMsgNo(uint8_t *arr)
{
    return arr[4];
}

/**
 * @brief 受信データを取得する関数
 *
 * @param arr 受信データを格納する配列
 * @return int 受信データの長さ
 */
int NEC920::getRecievedData(uint8_t *arr)
{
    if (isMsgRecieved == 0)
    {
        return 0;
    }
    for (int i = 0; i < rxbff[2]; i++)
    {
        arr[i] = rxbff[i];
    }
    isMsgRecieved = 0;

    return rxbff[2];
}

/**
 * @brief 無線設定関数
 * 送信出力，チャンネル，RFバンド，CSモードを設定する．
 */
void NEC920::setRfConf(uint8_t Power, uint8_t Channel, uint8_t RF_Band, uint8_t CS_Mode)
{
    uint8_t parameter[4];
    parameter[0] = Power;
    parameter[1] = Channel;
    parameter[2] = RF_Band;
    parameter[3] = CS_Mode;
    uint8_t packet[17];
    makepacket(packet, 0x21, 0x00, srcID, srcID, parameter, 4);
    ser->write(packet, 17);
}

/**
 * @brief 無線モジュールの生存確認関数
 *
 * @return int 0...生存 1...死亡
 */
int NEC920::isWirelessModuleDead()
{
    if (micros() - timeSendingMSG > 1000000)
    {
        return 1;
    }
    return 0;
}