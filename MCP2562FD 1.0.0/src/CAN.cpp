// CAN CREATE Library

#include "CAN.h"

//----------------------------
// return 0:ACK ERROR
//----------------------------
int CAN_CREATE::sendPacket(int id, char data) {
    // set data
    beginPacket(id);
    write(data);
    // send data
    return endPacket();
}
int CAN_CREATE::sendBytes(int id, uint8_t* tx, size_t size) {
    beginPacket(id);
    write(tx, size);
    return endPacket();
}

int CAN_CREATE::available() {
    int packetSize = parsePacket();  // Datafield value (DLC)
    if (packetRtr()) {  // RTR 0:data 1:remote
        return 0;
    }
    return CANControllerClass::available();  //  number of bytes available for
                                             //  reading
}

int CAN_CREATE::read(long* pID) {
    if (!CANControllerClass::available()) {
        return -1;
    }
    *pID = packetId();
    return _rxData[_rxIndex++];
}