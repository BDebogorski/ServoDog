#include <Arduino.h>
#include <QuadrupedRobot.hpp>

volatile int moveTimer = 0;

void moveClock()
{
	moveTimer++;
}

QuadrupedRobot::QuadrupedRobot(Leg2DoF &leftFront, Leg2DoF &leftBack, Leg2DoF &rightFront, Leg2DoF &rightBack, IMUFilter &imuFilter)
{
	this->leftFront = &leftFront;
	this->leftBack = &leftBack;
	this->rightFront = &rightFront;
	this->rightBack = &rightBack;

	this->imuFilter = &imuFilter;

	xSide = 0;
	ySide = 0;

	xOffset = 0;
	yOffset = 0;
	zOffset = 0;

	xSpacing = 0;
	ySpacing = 0;

	xBodyAngle = 0;
	yBodyAngle = 0;
}

void QuadrupedRobot::setLeftFrontMountingPosition(float x, float y)
{
	walkingAlgorithm.setLeftFrontMountingPosition(x, y);
}

void QuadrupedRobot::setRightFrontMountingPosition(float x, float y)
{
	walkingAlgorithm.setRightFrontMountingPosition(x, y);
}

void QuadrupedRobot::setLeftBackMountingPosition(float x, float y)
{
	walkingAlgorithm.setLeftBackMountingPosition(x, y);
}

void QuadrupedRobot::setRightBacktMountingPosition(float x, float y)
{
	walkingAlgorithm.setRightBackMountingPosition(x, y);
}

bool QuadrupedRobot::setXOffset(float offset)
{
	if(!leftFront->setXOffset(offset+xSpacing)) return false;
	if(!leftBack->setXOffset(offset+xSpacing)) return false;
	if(!rightBack->setXOffset(offset+xSpacing)) return false;
	if(!rightFront->setXOffset(offset+xSpacing)) return false;

	xOffset = offset;
	return true;
}

bool QuadrupedRobot::setYOffset(float offset)
{
	if(!leftFront->setYOffset(offset+ySpacing)) return false;
	if(!leftBack->setYOffset(offset+ySpacing)) return false;
	if(!rightBack->setYOffset(offset+ySpacing)) return false;
	if(!rightFront->setYOffset(offset+ySpacing)) return false;

	yOffset = offset;
	return true;
}

bool QuadrupedRobot::setZOffset(float offset)
{
	if(!leftFront->setZOffset(offset)) return false;
	if(!leftBack->setZOffset(offset)) return false;
	if(!rightBack->setZOffset(offset)) return false;
	if(!rightFront->setZOffset(offset)) return false;

	zOffset = offset;
	return true;
}

bool QuadrupedRobot::setXSpacing(float spacing)
{
	spacing /= 2;

	if(!leftFront->setXOffset(spacing+xOffset)) return false;
	if(!rightFront->setXOffset(spacing+xOffset)) return false;
	if(!leftBack->setXOffset(-spacing+xOffset)) return false;
	if(!rightBack->setXOffset(-spacing+xOffset)) return false;

	ySpacing = spacing;
	return true;
}

bool QuadrupedRobot::setYSpacing(float spacing)
{
	spacing /=2;

	if(!leftFront->setYOffset(spacing+yOffset)) return false;
	if(!rightFront->setYOffset(spacing+yOffset)) return false;
	if(!leftBack->setYOffset(-spacing+yOffset)) return false;
	if(!rightBack->setYOffset(-spacing+yOffset)) return false;

	ySpacing = spacing;
	return true;
}

bool QuadrupedRobot::setAllLegsPosition(float x, float y, float z)
{
	if(!leftFront->setPosition(x, y, z)) return false;
	if(!rightFront->setPosition(x, y, z)) return false;
	if(!leftBack->setPosition(x, y, z)) return false;
	if(!rightBack->setPosition(x, y, z)) return false;

	return true;
}

void QuadrupedRobot::setXBodyAngle(float angle)
{
	xBodyAngle = angle;
}

void QuadrupedRobot::setYBodyAngle(float angle)
{
	yBodyAngle = angle;
}

bool QuadrupedRobot::moveAllLegs()
{
	if(!leftFront->move()) return false;
	if(!rightFront->move()) return false;
	if(!leftBack->move()) return false;
	if(!rightBack->move()) return false;

	return true;
}

bool QuadrupedRobot::levelBody()
{
	bool status = true;

	float xActBodyAngle;
	float yActBodyAngle;

	xActBodyAngle = imuFilter->getXAngle()-M_PI;
	yActBodyAngle = imuFilter->getYAngle()-M_PI;

	xSide= (0.08-xSpacing)*sin(xActBodyAngle+xBodyAngle);
	ySide= (0.12-ySpacing)*sin(yActBodyAngle+yBodyAngle);

	if(!leftFront->setZOffset(xSide-ySide)) return false;
	if(!leftBack->setZOffset(xSide+ySide)) return false;
	if(!rightFront->setZOffset(-xSide-ySide)) return false;
	if(!rightBack->setZOffset(-xSide+ySide)) return false;

	if(!leftFront->move()) status = false;
	if(!leftBack->move()) status = false;
	if(!rightFront->move()) status = false;
	if(!rightBack->move()) status = false;

	return status;
}

void QuadrupedRobot::setStartLegs(bool startLeg)
{
	walkingAlgorithm.setStartLeg(startLeg);
}

void QuadrupedRobot::walk
(
	float time,
	float pause,
	int nPoints,
	float zHeight,
	float stepHeight,
	float xLengthL,
	float xLengthR,
	float yLength,
	float rot,
	bool stabilization
){

	LegCordinates LFCordinates;
	LegCordinates RFCordinates;
	LegCordinates LBCordinates;
	LegCordinates RBCordinates;

	while (moveTimer < pause*100000) // pause
	{
		if(stabilization) levelBody();
	}

	walkingAlgorithm.setLeftFrontLegPosition(leftFront->getXPosition(), leftFront->getYPosition(), leftFront->getZPosition());
	walkingAlgorithm.setRightFrontLegPosition(rightFront->getXPosition(), rightFront->getYPosition(), rightFront->getZPosition());
	walkingAlgorithm.setLeftBackLegPosition(leftBack->getXPosition(), leftBack->getYPosition(), leftBack->getZPosition());
	walkingAlgorithm.setRightBackLegPosition(rightBack->getXPosition(), rightBack->getYPosition(), rightBack->getZPosition());

	walkingAlgorithm.setWalkParameters(nPoints, zHeight, stepHeight, xLengthL, xLengthR, yLength, rot);

	for (int i = 0; i < nPoints+1; i++)
	{
		moveTimer = 0;

		walkingAlgorithm.calculate(i);

		walkingAlgorithm.getLeftFrontLegPosition(LFCordinates);
		walkingAlgorithm.getRightFrontLegPosition(RFCordinates);
		walkingAlgorithm.getLeftBackLegPosition(LBCordinates);
		walkingAlgorithm.getRightBackLegPosition(RBCordinates);

		leftFront->setPosition(LFCordinates.xPosition, LFCordinates.yPosition, LFCordinates.zPosition);
		rightFront->setPosition(RFCordinates.xPosition, RFCordinates.yPosition, RFCordinates.zPosition);
		leftBack->setPosition(LBCordinates.xPosition, LBCordinates.yPosition, LBCordinates.zPosition);
		rightBack->setPosition(RBCordinates.xPosition, RBCordinates.yPosition, RBCordinates.zPosition);

		leftFront->move();
		rightFront->move();
		leftBack->move();
		rightBack->move();

		while (moveTimer < time/nPoints*200000 && i < nPoints) //delay
		{
			if(stabilization) levelBody();
		}
	}

	moveTimer = 0;
}

void QuadrupedRobot::goForAzimuth
(
	float time,
	float pause,
	int nPoints,
	float zHeight,
	float stepHeight,
	float stepLength,
	float actualAngle,
	float azimuth,
	float fullStepAngle,
	bool stabilization
){
	
	float xLengthR = -fabs(stepLength) * map(actualAngle-azimuth, -2*M_PI, 2*M_PI, -M_PI/fullStepAngle, M_PI/fullStepAngle);
	float xLengthL;

	xLengthL = -xLengthR;

	xLengthR += (xLengthR-stepLength);
	xLengthL += (xLengthL-stepLength);

	if(xLengthR > stepLength) xLengthR = stepLength;
	if(xLengthR < -stepLength) xLengthR = -stepLength;

	if(xLengthL > stepLength) xLengthL = stepLength;
	if(xLengthL < -stepLength) xLengthL = -stepLength;

	walk(time, pause, nPoints, zHeight, stepHeight, xLengthL, xLengthR, 0, 0, stabilization);
}