#include <Arduino.h>
#include <ServoMotor.hpp>

ServoMotor::ServoMotor() {}

ServoMotor::ServoMotor(int port, short offset, float rate, bool side)
{
    this -> offset = offset;
    this -> rate = rate;
    this -> side = side;

    servo.attach(port);
    this -> setPosition(90);
}

bool ServoMotor::setPosition(short position)
{
    if(position < 0 || position > 180) 
    {
        Serial.println("Error: Illegal position!");
        return false;
    }

    position *= rate;

    if(!side) position = 180 - position;

    position += offset;

    if(position < 10 || position > 170) 
    {
        Serial.println("Error: Servo position out of range!");
        Serial.println(position);
        return false;
    }

    servo.write(position);
    return true;
}