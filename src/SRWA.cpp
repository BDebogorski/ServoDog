#include <SRWA.hpp>
#include <math.h>
/*
void SRWA::calculateDefaultEndPoint(float xLength, int ID)
{
    leg[ID].xDefaultEnd = -xLength/2 + (leg[ID].xStart+xLength)*cos(rotation/2) -
    (leg[ID].yStart+yLength)*sin(rotation/2) - leg[ID].xStart;

    leg[ID].yDefaultEnd = -yLength/2 + (leg[ID].xStart+xLength)*sin(rotation/2) +
    (leg[ID].yStart+yLength)*cos(rotation/2) - leg[ID].yStart;
}

void SRWA::setDefaultEndPoint(int ID)
{
    leg[ID].xEnd = leg[ID].xDefaultEnd;
    leg[ID].xEnd = leg[ID].xDefaultEnd;
}

void SRWA::calculateDownLeg(int pointNumber, float xLength, int ID)
{
    float rotationRate = -pointNumber*rotation/nPoints;
    float xTranslation = leg[ID].xStart - xLength*pointNumber/nPoints;
    float yTranslation = leg[ID].yStart - pointNumber*yLength*pointNumber/nPoints;

    leg[ID].x = (leg[ID].xStart+xTranslation)*cos(rotationRate) -
    (leg[ID].yStart+yTranslation)*sin(rotationRate) - leg[ID].xStart;

    leg[ID].y = (leg[ID].xStart+xTranslation)*sin(rotationRate) +
    (leg[ID].yStart+yTranslation)*cos(rotationRate) - leg[ID].yStart;

    leg[ID].z = zBodyPosition;
}

void SRWA::calculateUpLeg(int pointNumber, float xLength, int ID)
{
    leg[ID].x = leg[ID].xStart + (leg[ID].xEnd-leg[ID].xStart)*pointNumber/nPoints;
    leg[ID].y = leg[ID].yStart + (leg[ID].yEnd-leg[ID].yStart)*pointNumber/nPoints;
    leg[ID].z = zBodyPosition - stepHeight*sin(M_PI*pointNumber/nPoints);
}

SRWA::SRWA()
{
    startLegs = FIRST_GROUP;
    isReady = false;
}

void SRWA::addLeg(bool group)
{
    legs.push_back(Leg);
    legs[legs.size()].group = group;
}

bool SRWA::setStartLegPosition(float x, float y, int ID)
{
    if(ID < 0 || ID > legs.size()) return false;

    legs[ID].xStart = x;
    legs[ID].yStart = y;
    return true;
}

bool SRWA::setEndLegPosition(float x, float y, int ID)
{
    if(ID < 0 || ID > legs.size()) return false;

    legs[ID].xEnd = x;
    legs[ID].yEnd = y;
    return true;
}

void SRWA::setStartLegs(bool group);
{
    startLegs = group;
}

bool SRWA::setWalkParameters
(
    int nPoints,
    float zBodyPosition,
    float stepHeight,
    float xLengthL,
    float xLengthR,
    float yLength,
    float rotation
)
{
    if(stepHeight > zBodyPosition) return false;

    this->nPoints = nPoints;
    this->zBodyPosition = zBodyPosition;
    this->stepHeight = stepHeight;
    this->xLengthL = xLengthL;
    this->xLengthR = xLengthR;
    this->yLength = yLength;
    this->rotation = rotation;

    for(int i = 0; i < legs.size(); i++)
    {
        if(legs[i].xStart < 0)
        {
            calculateDefaultEndPoint(xLengthL, i);
        }
        else 
        {
            calculateDefaultEndPoint(xLengthR, i);
        }

        setDefaultEndPoint(i);
    }

    isReady = true;
    return true;
}

bool SRWA::calculate(int pointNumber)
{
    if(!isReady) return false;
    else isReady = false;

    if(pointNumber > nPoints || pointNumber < 0) return false;

    for(int i = 0; i < legs.size(); i++)
    {
        if(legs[ID].group)
        {
            calculateUpLeg(pointNumber, xLengthL, i);
        }
        else
        {
            calculateDownLeg(pointNumber, xLengthL, i);
        }
    }

    if(pointNumber >= nPoints)
    {
        for(int i = 0; i < legs.size(); i++)
        {
            setDefaultEndPoint(i);
        }

        startLegs = !startLegs;
    }

    isReady = true;
    return true;
}

bool SRWA::getDefaultEndPosition(Cordinates &cordinates, int ID)
{
    if(!isReady) return false;

    cordinates.x = legs[ID].xDefaultEnd;
    cordinates.y = legs[ID].yDefaultEnd;
    cordinates.z = zBodyPosition;

    return true;
}

bool SRWA::getLegPosition(Cordinates &cordinates, int ID)
{
    if(!isReady) return false;

    cordinates.x = legs[ID].x;
    cordinates.y = legs[ID].y;
    cordinates.z = legs[ID].z;

    return true;
}
*/