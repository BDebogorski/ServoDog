#pragma once
#include <Servo.h>

class ServoMotor
{
    private:

    bool side;
    short offset;
    float rate;

    Servo servo;

    public:

    ServoMotor();
    ServoMotor(int port, short offset, float rate, bool side);
    bool setPosition(short position);
};