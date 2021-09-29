#include <QuadrupedWalkingAlgorithm.hpp>
#include <math.h>

void QuadrupedWalkingAlgorithm::calculateStepParameters
(
    float xLength,
    LegParameters legParameters,
    LegCordinates legCordinates
)
{ 
	float xEnd;
    float yEnd;

    calculateStepPointXY(0, true, xLength, legParameters.xStartStep, legParameters.yStartStep, legParameters, legCordinates);
	calculateStepPointXY(nPoints, true, xLength, xEnd, yEnd, legParameters, legCordinates);
	legParameters.lengthOfMove = sqrt(pow(legParameters.xStartStep - xEnd, 2) + pow(legParameters.yStartStep - yEnd, 2));
}

void QuadrupedWalkingAlgorithm::calculateStepPointXY
(
    int pointNumber,
    int pairLeg,
    float xLength,
    float& xPoint,
    float& yPoint,
    LegParameters legParameters,
    LegCordinates legCordinates
)
{
	if(pointNumber == 0)   // first point of step
	{
		legParameters.xStartStep = legCordinates.xPosition;    // start point of step
		legParameters.yStartStep = legCordinates.yPosition;    // start point of step
	}

	float xEnd = pairLeg * (nPoints/2 - nPoints) * xLength / nPoints;    // end point of step
	float yEnd = pairLeg * (nPoints/2 - nPoints) * yLength / nPoints;    // end point of step

	float rotRate = (nPoints/2 - pointNumber) * pairLeg * rotation / nPoints;    // angle in this point

	xPoint = legParameters.xStartStep - pairLeg * (pairLeg * (legParameters.xStartStep-xEnd) / nPoints * pointNumber) +
    legParameters.xMountingPosition*cos(rotRate) - legParameters.yMountingPosition*sin(rotRate) - legParameters.xMountingPosition;

	yPoint = legParameters.yStartStep - pairLeg * (pairLeg * (legParameters.yStartStep-yEnd) / nPoints * pointNumber) +
    legParameters.xMountingPosition*sin(rotRate) + legParameters.yMountingPosition*cos(rotRate) - legParameters.yMountingPosition;
}

float QuadrupedWalkingAlgorithm::calculateZTrajectory(float xBeg, float yBeg, float x, float y, float lengthOfMove)
{
    float xE = 2*(sqrt(pow(x-xBeg, 2) + pow(y-yBeg, 2)) - lengthOfMove/2);
	float t = acos(xE/lengthOfMove);

	return zBodyPosition - stepHeight*sin(t);    // leg trajectory
}

void QuadrupedWalkingAlgorithm::calculateStepPoint
(
    int pointNumber,
    float xLength,
    bool isOnGround,
    LegParameters legParameters,
    LegCordinates legCordinates
)
{
    float x;
    float y;
    float z;

	if(isOnGround)
	{
		calculateStepPointXY(pointNumber, -1, xLength, x, y, legParameters, legCordinates);
		z = zBodyPosition;
	}
	else
	{
		calculateStepPointXY(pointNumber, 1, xLength, x, y, legParameters, legCordinates);
        z = calculateZTrajectory(legParameters.xStartStep, legParameters.yStartStep, x, y, legParameters.lengthOfMove);
	}

    legCordinates.xPosition = x;
    legCordinates.yPosition = y;
    legCordinates.zPosition = z;
}

QuadrupedWalkingAlgorithm::QuadrupedWalkingAlgorithm()
{
    xSpacing = 0;
    ySpacing = 0;

    startLeg = true;
    isReady = false;

    setLeftFrontLegPosition(0, 0, zBodyPosition);
    setRightFrontLegPosition(0, 0, zBodyPosition);
    setLeftBackLegPosition(0, 0, zBodyPosition);
    setRightBackLegPosition(0, 0, zBodyPosition);
}

void QuadrupedWalkingAlgorithm::setLeftFrontMountingPosition(float x, float y)
{
    leftFrontParameters.xMountingPosition = x;
    leftFrontParameters.yMountingPosition = y;
}

void QuadrupedWalkingAlgorithm::setRightFrontMountingPosition(float x, float y)
{
    rightFrontParameters.xMountingPosition = x;
    rightFrontParameters.yMountingPosition = y;
}

void QuadrupedWalkingAlgorithm::setLeftBackMountingPosition(float x, float y)
{
    leftBackParameters.xMountingPosition = x;
    leftBackParameters.yMountingPosition = y;
}

void QuadrupedWalkingAlgorithm::setRightBackMountingPosition(float x, float y)
{
    rightBackParameters.xMountingPosition = x;
    rightBackParameters.yMountingPosition = y;
}

void QuadrupedWalkingAlgorithm::setSpacing(float xSpacing, float ySpacing)
{
    this->xSpacing = xSpacing;
    this->ySpacing = ySpacing;
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

    calculateStepParameters(xLengthL, leftFrontParameters, leftFrontCordinates);
    calculateStepParameters(xLengthR, rightFrontParameters, rightFrontCordinates);
    calculateStepParameters(xLengthL, leftBackParameters, leftBackCordinates);
    calculateStepParameters(xLengthR, rightBackParameters, rightBackCordinates);

    isReady = true;
    return true;
}

void QuadrupedWalkingAlgorithm::setLeftFrontLegPosition(float x, float y, float z)
{
    leftFrontCordinates.xPosition = x;
    leftFrontCordinates.yPosition = y;
    leftFrontCordinates.zPosition = z;
}

void QuadrupedWalkingAlgorithm::setRightFrontLegPosition(float x, float y, float z)
{
    rightFrontCordinates.xPosition = x;
    rightFrontCordinates.yPosition = y;
    rightFrontCordinates.zPosition = z;
}

void QuadrupedWalkingAlgorithm::setLeftBackLegPosition(float x, float y, float z)
{
    leftBackCordinates.xPosition = x;
    leftBackCordinates.yPosition = y;
    leftBackCordinates.zPosition = z;
}

void QuadrupedWalkingAlgorithm::setRightBackLegPosition(float x, float y, float z)
{
    rightBackCordinates.xPosition = x;
    rightBackCordinates.yPosition = y;
    rightBackCordinates.zPosition = z;
}

bool QuadrupedWalkingAlgorithm::calculate(int pointNumber)
{
    if(!isReady) return false;
    else isReady = false;

    if(pointNumber > nPoints || pointNumber < 0) return false;

    calculateStepPoint(pointNumber, xLengthL, !startLeg, leftFrontParameters, leftFrontCordinates);
    calculateStepPoint(pointNumber, xLengthL, startLeg, rightFrontParameters, rightFrontCordinates);
    calculateStepPoint(pointNumber, xLengthL, startLeg, leftBackParameters, leftBackCordinates);
    calculateStepPoint(pointNumber, xLengthL, !startLeg, rightBackParameters, rightBackCordinates);

    if(pointNumber >= nPoints) startLeg = !startLeg;

    isReady = true;
    return true;
}

bool QuadrupedWalkingAlgorithm::getLeftFrontLegPosition(LegCordinates &legCordinates)
{
    if(!isReady) return false;

    leftFrontCordinates.xPosition = legCordinates.xPosition;
    leftFrontCordinates.yPosition = legCordinates.yPosition;
    leftFrontCordinates.zPosition = legCordinates.zPosition;

    return true;
}

bool QuadrupedWalkingAlgorithm::getRightFrontLegPosition(LegCordinates &legCordinates)
{
    if(!isReady) return false;

    rightFrontCordinates.xPosition = legCordinates.xPosition;
    rightFrontCordinates.yPosition = legCordinates.yPosition;
    rightFrontCordinates.zPosition = legCordinates.zPosition;

    return true;
}

bool QuadrupedWalkingAlgorithm::getLeftBackLegPosition(LegCordinates &legCordinates)
{
    if(!isReady) return false;

    leftBackCordinates.xPosition = legCordinates.xPosition;
    leftBackCordinates.yPosition = legCordinates.yPosition;
    leftBackCordinates.zPosition = legCordinates.zPosition;

    return true;
}

bool QuadrupedWalkingAlgorithm::getRightBackLegPosition(LegCordinates &legCordinates)
{
    if(!isReady) return false;

    rightBackCordinates.xPosition = legCordinates.xPosition;
    rightBackCordinates.yPosition = legCordinates.yPosition;
    rightBackCordinates.zPosition = legCordinates.zPosition;

    return true;
}