#include <BlueControll.hpp>
#include <sstream>
#include <string>

BlueControll::BlueControll(char endPacketChar)
{
    this->endPacketChar = endPacketChar;

    reciveBuffer = "";
    lastMessage = " ";
    reciveDada = "";
    ready = false;
}

bool BlueControll::send(String message)
{
    if(message != lastMessage)
    {
        lastMessage = message;
        Serial1.print(message+endPacketChar);
        return true;
    }

    return false;
}

bool BlueControll::recive()
{
    std::stringstream ssize;
    String output;

    unsigned int size;
    char* buffer;
    int end;
    char c;

    if (!Serial1.available()) return false;

    c = Serial1.read();

    if(c == endPacketChar)
    {
        ssize << reciveBuffer.c_str();
        reciveBuffer = "";

        ssize.seekg(0, ssize.end);
        end = ssize.tellg();
        ssize.seekg(ssize.beg);

        ssize >> size;

        if(!ssize.eof()) ssize.seekg(1, ssize.cur);
        int bufferSize = end-ssize.tellg();
        buffer = new char[bufferSize];
        ssize.get(buffer, bufferSize+1);
        output = String(buffer);
        delete[] buffer;

        if(size != output.length())
        {
            ssize.clear();
            return false;
        }

        reciveDada = output;
        ready = true;
        return true;
    }
    else
    {
        reciveBuffer += c;
    }

    return true;
}

bool BlueControll::isReady()
{
    return ready;
}

String BlueControll::getReciveData()
{
    ready = false;
    return reciveDada;
}