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

    calculateStepPointXY(0, true, xLength, legParameters.xStartStep, legParameters.yStartStep, legParameters, legCordinates);    // get start point of step
	calculateStepPointXY(nPoints, true, xLength, xEnd, yEnd, legParameters, legCordinates);  // get end point of step
	legParameters.lengthOfMove = sqrt(pow(legParameters.xStartStep - xEnd, 2) + pow(legParameters.yStartStep - yEnd, 2)); // calculate length on step (XY)
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

void QuadrupedWalkingAlgorithm::calculateStepPoint
(
    int pointNumber,
    float xLength,
    bool isOnGround,
    LegParameters legParameters,
    LegCordinates legCordinates
)
{
    float xStep;
    float yStep;
    float zStep;

	if(isOnGround)
	{
		calculateStepPointXY(pointNumber, -1, xLength, xStep, yStep, legParameters, legCordinates);
		zStep = zBodyPosition;
	}
	else
	{
		calculateStepPointXY(pointNumber, 1, xLength, xStep, yStep, legParameters, legCordinates);

		float xE = 2*(sqrt(pow(xStep-legParameters.xStartStep, 2) + pow(yStep-legParameters.yStartStep, 2)) - legParameters.lengthOfMove/2);
		float t = acos(xE/legParameters.lengthOfMove);
		zStep = zBodyPosition - stepHeight*sin(t);    // leg trajectory
	}

    legCordinates.xPosition = xStep;
    legCordinates.yPosition = yStep;
    legCordinates.zPosition = zStep;
}

QuadrupedWalkingAlgorithm::QuadrupedWalkingAlgorithm()
{
    xSpacing = 0;
    ySpacing = 0;

    startLeg = true;

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

void QuadrupedWalkingAlgorithm::setWalkParameters
(	
    float time,
    float pause,
    int nPoints,
    float zBodyPosition,
    float stepHeight,
    float xLengthL,
    float xLengthR,
    float yLength,
    float rotation
)
{
    this->time = time;
    this->pause = pause;
    this->nPoints = nPoints;
    this->zBodyPosition = zBodyPosition;
    this->stepHeight = stepHeight;
    this->xLengthL = xLengthL;
    this->xLengthR = xLengthR;
    this->yLength = yLength;
    this->rotation = rotation;
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

void QuadrupedWalkingAlgorithm::initStep()
{
    calculateStepParameters(xLengthL, leftFrontParameters, leftFrontCordinates);
    calculateStepParameters(xLengthR, rightFrontParameters, rightFrontCordinates);
    calculateStepParameters(xLengthL, leftBackParameters, leftBackCordinates);
    calculateStepParameters(xLengthR, rightBackParameters, rightBackCordinates);
}

void QuadrupedWalkingAlgorithm::calculate(float stepTime)
{
    float pointNumber = nPoints*stepTime;

    calculateStepPoint(pointNumber, xLengthL, !startLeg, leftFrontParameters, leftFrontCordinates);
    calculateStepPoint(pointNumber, xLengthL, startLeg, rightFrontParameters, rightFrontCordinates);
    calculateStepPoint(pointNumber, xLengthL, startLeg, leftBackParameters, leftBackCordinates);
    calculateStepPoint(pointNumber, xLengthL, !startLeg, rightBackParameters, rightBackCordinates);

    if(stepTime >= time) startLeg = !startLeg;
}

void QuadrupedWalkingAlgorithm::getLeftFrontLegPosition(LegCordinates &legCordinates)
{
    leftFrontCordinates.xPosition = legCordinates.xPosition;
    leftFrontCordinates.yPosition = legCordinates.yPosition;
    leftFrontCordinates.zPosition = legCordinates.zPosition;
}

void QuadrupedWalkingAlgorithm::getRightFrontLegPosition(LegCordinates &legCordinates)
{
    rightFrontCordinates.xPosition = legCordinates.xPosition;
    rightFrontCordinates.yPosition = legCordinates.yPosition;
    rightFrontCordinates.zPosition = legCordinates.zPosition;
}

void QuadrupedWalkingAlgorithm::getLeftBackLegPosition(LegCordinates &legCordinates)
{
    leftBackCordinates.xPosition = legCordinates.xPosition;
    leftBackCordinates.yPosition = legCordinates.yPosition;
    leftBackCordinates.zPosition = legCordinates.zPosition;
}

void QuadrupedWalkingAlgorithm::getRightBackLegPosition(LegCordinates &legCordinates)
{
    rightBackCordinates.xPosition = legCordinates.xPosition;
    rightBackCordinates.yPosition = legCordinates.yPosition;
    rightBackCordinates.zPosition = legCordinates.zPosition;
}