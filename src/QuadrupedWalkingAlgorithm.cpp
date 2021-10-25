#include <QuadrupedWalkingAlgorithm.hpp>
#include <math.h>

void QuadrupedWalkingAlgorithm::calculateDefaultEndPoint(float xLength, Leg &leg)
{
    leg.xDefaultEnd = -xLength/2 + (leg.xZero+xLength)*cos(rotation/2) - (leg.yZero+yLength)*sin(rotation/2) - leg.xZero;
    leg.yDefaultEnd = -yLength/2 + (leg.xZero+xLength)*sin(rotation/2) + (leg.yZero+yLength)*cos(rotation/2) - leg.yZero;
}

void QuadrupedWalkingAlgorithm::setDefaultEndPoint(Leg &leg)
{
    leg.xEnd = leg.xDefaultEnd;
    leg.xEnd = leg.xDefaultEnd;
}

void QuadrupedWalkingAlgorithm::calculateDownLeg(int pointNumber, float xLength, Leg &leg)
{
    float rotationRate = -pointNumber*rotation/nPoints;
    float xTranslation = leg.xStart - xLength*pointNumber/nPoints;
    float yTranslation = leg.yStart - pointNumber*yLength*pointNumber/nPoints;

    leg.x = (leg.xZero+xTranslation)*cos(rotationRate) - (leg.yZero+yTranslation)*sin(rotationRate) - leg.xZero;
    leg.y = (leg.xZero+xTranslation)*sin(rotationRate) + (leg.yZero+yTranslation)*cos(rotationRate) - leg.yZero;
    leg.z = zBodyPosition;
}

void QuadrupedWalkingAlgorithm::calculateUpLeg(int pointNumber, float xLength, Leg &leg)
{
    leg.x = leg.xStart + (leg.xEnd-leg.xStart)*pointNumber/nPoints;
    leg.y = leg.yStart + (leg.yEnd-leg.yStart)*pointNumber/nPoints;
    leg.z = zBodyPosition - stepHeight*sin(M_PI*pointNumber/nPoints);
}

void QuadrupedWalkingAlgorithm::calculateLeg(int pointNumber, float xLength, bool pair, bool defaultEndPoint, Leg &leg)
{
    if(pair) calculateUpLeg(pointNumber, xLength, leg);
    else calculateDownLeg(pointNumber, xLength, leg);
}

QuadrupedWalkingAlgorithm::QuadrupedWalkingAlgorithm()
{
    startLeg = LeftFront_RightBack;
    isReady = false;

    setLeftFrontStartPosition(0, 0);
    setRightFrontStartPosition(0, 0);
    setLeftBackStartPosition(0, 0);
    setRightBackStartPosition(0, 0);
}

void QuadrupedWalkingAlgorithm::setLeftFrontZeroPosition(float x, float y)
{
    leftFront.xZero = x;
    leftFront.yZero = y;
}

void QuadrupedWalkingAlgorithm::setRightFrontZeroPosition(float x, float y)
{
    rightFront.xZero = x;
    rightFront.yZero = y;
}

void QuadrupedWalkingAlgorithm::setLeftBackZeroPosition(float x, float y)
{
    leftBack.xZero = x;
    leftBack.yZero = y;
}

void QuadrupedWalkingAlgorithm::setRightBackZeroPosition(float x, float y)
{
    rightBack.xZero = x;
    rightBack.yZero = y;
}

void QuadrupedWalkingAlgorithm::setStartLeg(bool startLeg)
{
    this->startLeg = startLeg;
}

bool QuadrupedWalkingAlgorithm::setWalkParameters
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

    calculateDefaultEndPoint(xLengthL, leftFront);
    calculateDefaultEndPoint(xLengthR, rightFront);
    calculateDefaultEndPoint(xLengthL, leftBack);
    calculateDefaultEndPoint(xLengthR, rightBack);

    setDefaultEndPoint(leftFront);
    setDefaultEndPoint(rightFront);
    setDefaultEndPoint(leftBack);
    setDefaultEndPoint(rightBack);

    isReady = true;
    return true;
}

void QuadrupedWalkingAlgorithm::setLeftFrontStartPosition(float x, float y)
{
    leftFront.xStart = x;
    leftFront.yStart = y;
}

void QuadrupedWalkingAlgorithm::setRightFrontStartPosition(float x, float y)
{
    rightFront.xStart = x;
    rightFront.yStart = y;
}

void QuadrupedWalkingAlgorithm::setLeftBackStartPosition(float x, float y)
{
    leftBack.xStart = x;
    leftBack.yStart = y;
}

void QuadrupedWalkingAlgorithm::setRightBackStartPosition(float x, float y)
{
    rightBack.xStart = x;
    rightBack.yStart = y;
}

void QuadrupedWalkingAlgorithm::setLeftFrontEndPosition(float x, float y)
{
    leftFront.xEnd = x;
    leftFront.yEnd = y;
}

void QuadrupedWalkingAlgorithm::setRightFrontEndPosition(float x, float y)
{
    rightFront.xEnd = x;
    rightFront.yEnd = y;
}

void QuadrupedWalkingAlgorithm::setLeftBackEndPosition(float x, float y)
{
    leftBack.xEnd = x;
    leftBack.yEnd = y;
}

void QuadrupedWalkingAlgorithm::setRightBackEndPosition(float x, float y)
{
    rightBack.xEnd = x;
    rightBack.yEnd = y;
}

bool QuadrupedWalkingAlgorithm::calculate(int pointNumber)
{
    if(!isReady) return false;
    else isReady = false;

    if(pointNumber > nPoints || pointNumber < 0) return false;

    calculateLeg(pointNumber, xLengthL, !startLeg, true, leftFront);
    calculateLeg(pointNumber, xLengthR, startLeg, true, rightFront);
    calculateLeg(pointNumber, xLengthL, startLeg, true, leftBack);
    calculateLeg(pointNumber, xLengthR, !startLeg, true, rightBack);

    if(pointNumber >= nPoints)
    {
        setDefaultEndPoint(leftFront);
        setDefaultEndPoint(rightFront);
        setDefaultEndPoint(leftBack);
        setDefaultEndPoint(rightBack);
        startLeg = !startLeg;
    }

    isReady = true;
    return true;
}

bool QuadrupedWalkingAlgorithm::getLeftFrontLegPosition(Cordinates &cordinates)
{
    if(!isReady) return false;

    cordinates.x = leftFront.x;
    cordinates.y = leftFront.y;
    cordinates.z = leftFront.z;

    return true;
}

bool QuadrupedWalkingAlgorithm::getRightFrontLegPosition(Cordinates &cordinates)
{
    if(!isReady) return false;

    cordinates.x = rightFront.x;
    cordinates.y = rightFront.y;
    cordinates.z = rightFront.z;

    return true;
}

bool QuadrupedWalkingAlgorithm::getLeftBackLegPosition(Cordinates &cordinates)
{
    if(!isReady) return false;

    cordinates.x = leftBack.x;
    cordinates.y = leftBack.y;
    cordinates.z = leftBack.z;

    return true;
}

bool QuadrupedWalkingAlgorithm::getRightBackLegPosition(Cordinates &cordinates)
{
    if(!isReady) return false;

    cordinates.x = rightBack.x;
    cordinates.y = rightBack.y;
    cordinates.z = rightBack.z;

    return true;
}

bool QuadrupedWalkingAlgorithm::getLeftFrontDefaultEndPosition(Cordinates &cordinates)
{
    if(!isReady) return false;

    cordinates.x = leftFront.xDefaultEnd;
    cordinates.y = leftFront.yDefaultEnd;
    cordinates.z = zBodyPosition;

    return true;
}

bool QuadrupedWalkingAlgorithm::getRightFrontDefaultEndPosition(Cordinates &cordinates)
{
    if(!isReady) return false;

    cordinates.x = rightFront.xDefaultEnd;
    cordinates.y = rightFront.yDefaultEnd;
    cordinates.z = zBodyPosition;

    return true;
}

bool QuadrupedWalkingAlgorithm::getLeftBackDefaultEndPosition(Cordinates &cordinates)
{
    if(!isReady) return false;

    cordinates.x = leftBack.xDefaultEnd;
    cordinates.y = leftBack.yDefaultEnd;
    cordinates.z = zBodyPosition;

    return true;
}

bool QuadrupedWalkingAlgorithm::getRightBackDefaultEndPosition(Cordinates &cordinates)
{
    if(!isReady) return false;

    cordinates.x = rightBack.xDefaultEnd;
    cordinates.y = rightBack.yDefaultEnd;
    cordinates.z = zBodyPosition;

    return true;
}