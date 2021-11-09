#include <RemoteControll.hpp>
#include <sstream>
#include <math.h>

RemoteControll::RemoteControll()
{
    xLeft = 0;
    yLeft = 0;

    xRight = 0;
    yRight = 0;

    potLeft = 0;
    potRight = 0;

    swJoyLeft = 0;
    swJoyRight = 0;

    swLeft = 0;
    swRight = 0;

    mixerready = false;
}

void RemoteControll::getValues(String packet)
{
    std::stringstream buffer;

    buffer << packet.c_str();

    buffer >> xLeft >> yLeft >> potLeft >> swJoyLeft >> swLeft;
    buffer >> xRight >> yRight >> potRight >> swJoyRight >> swRight;
}

bool RemoteControll::setMixer(int xMin, int xMax, int yMin, int yMax, unsigned int diff)
{
    if(xMin > xMax || yMin > yMax || diff < 0)
    {
        mixerready = false;
        return false;
    }

    this->xMin = xMin;
    this->xMax = xMax;
    this->yMin = yMin;
    this->yMax = yMax;

    this->diff = diff;

    mixerready = true;
    return true;
}

bool RemoteControll::mixer(float x, float y, float &left, float &right, float outMax)
{
    if(!mixerready) return false;

    int xMid = (xMax-xMin)/2+xMin;
    int yMid = (yMax-yMin)/2+yMin;

    if(x<xMin) x = xMin;
    if(x>xMax) x = xMax;
      
    if(y<yMin) y = yMin;
    if(y>yMax) y = yMax;

    if(fabs(x-xMid) <= diff) x = xMid;
    if(fabs(y-yMid) <= diff) y = yMid;
      
    x = map(x, xMin, xMax, -outMax, outMax);
    y = map(y, yMin, yMax, -outMax, outMax);
         
    right = y*cos(M_PI/4) - x*sin(M_PI/4);
    left = y*sin(M_PI/4) + x*cos(M_PI/4);

    return true;
}

int RemoteControll::getLeftJoyX()
{
    return xLeft;
}

int RemoteControll::getLeftJoyY()
{
    return yLeft;
}

bool RemoteControll::getLeftJoySwitch()
{
    return swJoyLeft;
}

int RemoteControll::getRightJoyX()
{
    return xRight;
}

int RemoteControll::getRightJoyY()
{
    return yRight;
}

bool RemoteControll::getRightJoySwitch()
{
    return swJoyRight;
}

int RemoteControll::getLeftPotentiometer()
{
    return potLeft;
}

int RemoteControll::getRightPotentiometer()
{
    return potRight;
}

bool RemoteControll::getLeftSwitch()
{
    return swLeft;
}

bool RemoteControll::getRightSwitch()
{
    return swRight;
}