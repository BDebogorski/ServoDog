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

    bool send(String message);
    bool recive();
    bool isReady();
    String getReciveData();
};