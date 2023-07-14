#ifndef GSECOM_HPP
#define GSECOM_HPP

#include <cstdint>
#include <cstddef>
#include <string.h>
#include "crc8.hpp"

#define GSECOM_MAX_PACKET_SIZE 11

#define GSECOM_HEADER 0x43

#define GSECOM_PACKET_HEADER_ERR 2
#define GSECOM_PACKET_CRC_ERR 1
#define GSECOM_PACKET_OK 0

#define GSECOM_MAP_HEAD 0
#define GSECOM_MAP_CMDID 1
#define GSECOM_MAP_PACKETLENGTH 2
#define GSECOM_MAP_PAYLOAD 3

class GseCom
{
public:
    static void makePacket(uint8_t *packet, uint8_t cmdid, uint8_t *payload, uint8_t payloadLength)
    {
        packet[GSECOM_MAP_HEAD] = GSECOM_HEADER;
        packet[GSECOM_MAP_CMDID] = cmdid;
        packet[GSECOM_MAP_PACKETLENGTH] = payloadLength + 4;
        memcpy(packet + GSECOM_MAP_PAYLOAD, payload, payloadLength);
        packet[payloadLength + 3] = CRC8::calculate(packet, packet[GSECOM_MAP_PACKETLENGTH]-1);
    }

    static void regenPacketCRC(uint8_t *packet)
    {
        packet[packet[2] - 1] = CRC8::calculate(packet, packet[GSECOM_MAP_PACKETLENGTH]-1);
    }

    static uint8_t checkPacket(uint8_t *packet)
    {
        if (packet[GSECOM_MAP_HEAD] != GSECOM_HEADER)
        {
            return GSECOM_PACKET_HEADER_ERR;
        }
        if (CRC8::calculate(packet, packet[GSECOM_MAP_PACKETLENGTH]-1) != packet[packet[GSECOM_MAP_PACKETLENGTH] - 1])
        {
            return GSECOM_PACKET_CRC_ERR;
        }
        return GSECOM_PACKET_OK;
    }

    static void getPayload(uint8_t *packet, uint8_t *payload, uint8_t *payloadLength)
    {
        *payloadLength = packet[GSECOM_MAP_PACKETLENGTH] - 4;
        memcpy(payload, packet + GSECOM_MAP_PAYLOAD, *payloadLength);
    }

    static uint8_t getCmdId(uint8_t *packet)
    {
        return packet[GSECOM_MAP_CMDID];
    }
};

#endif /* GSECOM_HPP */