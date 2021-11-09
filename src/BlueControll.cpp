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
    if (!Serial1.available()) return false;

    char c = Serial1.read();

    if(c == endPacketChar)
    {
        std::stringstream ssize;
        unsigned int size;
        int end;
        
        ssize << reciveBuffer.c_str();
        ssize.seekg(0, ssize.end);
        end = ssize.tellg();
        ssize.seekg(ssize.beg);

        ssize >> size;

        ssize.seekg(1, ssize.cur);
        char* buffer = new char[end-ssize.cur];
        ssize.read(buffer, end-ssize.cur);
        String output(buffer);
        delete [] buffer;

        reciveBuffer = "";

        if(size != output.length())
        {
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