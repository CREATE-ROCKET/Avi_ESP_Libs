#ifndef CRC8_HPP
#define CRC8_HPP

#include <cstdint>
#include <cstddef>

class CRC8
{
public:
    static const uint8_t polynomial = 0x07;

    static uint8_t calculate(const uint8_t *data, size_t len)
    {
        uint8_t crc = 0x00;

        for (size_t i = 0; i < len; i++)
        {
            crc ^= data[i];
            for (size_t j = 0; j < 8; j++)
            {
                if (crc & 0x80)
                {
                    crc = (crc << 1) ^ polynomial;
                }
                else
                {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }
};

#endif /* CRC8_HPP */