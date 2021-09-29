#include <Arduino.h>
#include <RC.hpp>

int RC::radio(int port)
{
    return pulseIn(port, HIGH, 50000);
}

void RC::mixer(bool mixerOn)
{
    float X2;
    float Y2;

    float x = radio(verticalPort);
    float y = radio(horizontalPort);

    if(x<xMin) x = xMin;
    if(x>xMax) x = xMax;
      
    if(y<yMin) y = yMin;
    if(y>yMax) y = yMax;
      
    if(mixerOn)
    {
        x = map(x, xMin, xMax, -maxStepSize, maxStepSize);
        y = map(y, yMin, yMax, -maxStepSize, maxStepSize);
         
        X2 = x*cos(M_PI/4) - y*sin(M_PI/4);
        Y2 = x*sin(M_PI/4) + y*cos(M_PI/4);
    }
    else
    {
        X2 = map(x, xMin, xMax, -maxStepSize, maxStepSize);
        Y2 = map(y, yMin, yMax, -maxStepSize, maxStepSize);
    }

    if(abs(X2) <= diff) X2 = 0;
    if(abs(Y2) <= diff) Y2 = 0; 

    left = X2;
    right = Y2;
}

RC::RC(int horizontalPort, int verticalPort, int heightPort, int stepHeightPort, int speedPort)
{
    left = 0;
    right = 0;

    this->horizontalPort = horizontalPort;
    this->verticalPort = verticalPort;
    this->stepHeightPort = stepHeightPort;
    this->heightPort = heightPort;
    this->speedPort = speedPort;
}

void RC::init(float xMin, float xMid, float xMax, float yMin, float yMid, float yMax, float diff, float maxStepSize)
{
    this->xMin = xMin;
    this->xMid = xMid;
    this->xMax = xMax;

    this->yMin = yMin;
    this->yMid = yMid;
    this->yMax = yMax;

    this->diff = diff;
    this->maxStepSize = maxStepSize;

    Serial.println("Remote control initialization...");

    do
    {
        mixer(true); 
    }
    while(fabs(getLeft()) > diff || fabs(getRight()) > diff);
    Serial.println("Remote control is ready.");
}

float RC::getLeft()
{
    return left;
}

float RC::getRight()
{
    return right;
}

float RC::getHeight(float min, float max)
{
    int channel = radio(heightPort);
    float height = map(channel, xMin, xMax, min*10000, max*10000);
    return height/10000;
}

float RC::getStepHeight(float min, float max)
{
    int channel = radio(stepHeightPort);
    float height = map(channel, xMin, xMax, min*1000, max*1000);
    return height/1000;
}

float RC::getSpeed(float min, float max)
{
    int channel = radio(speedPort);
    float speed = map(channel, xMin, xMax, min*1000, max*1000);
    return max-speed/1000+min;
}