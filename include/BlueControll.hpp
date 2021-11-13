#pragma once

#include <Arduino.h>

class BlueControll
{
    private:

    char endPacketChar;

    String reciveBuffer;
    String lastMessage;
    String reciveDada;

    bool ready;

    public:

    BlueControll(char endPacketChar);

    bool send(String message); // send packet (do not use endPacketChar in message)
    bool recive();             // add recived data to packet
    bool isReady();            // packet is ready
    String getReciveData();    // get packet
};