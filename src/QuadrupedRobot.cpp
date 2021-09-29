#include <Arduino.h>
#include <QuadrupedRobot.hpp>

volatile int moveTimer = 0;

void moveClock()
{
	moveTimer++;
}

Robot::Robot(Leg2DoF &leftFront, Leg2DoF &leftBack, Leg2DoF &rightFront, Leg2DoF &rightBack, IMUFilter &imuFilter)
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

	startLeg = true;
}

bool Robot::setXOffset(float offset)
{
	if(!leftFront->setXOffset(offset+xSpacing)) return false;
	if(!leftBack->setXOffset(offset+xSpacing)) return false;
	if(!rightBack->setXOffset(offset+xSpacing)) return false;
	if(!rightFront->setXOffset(offset+xSpacing)) return false;

	xOffset = offset;
	return true;
}

bool Robot::setYOffset(float offset)
{
	if(!leftFront->setYOffset(offset+ySpacing)) return false;
	if(!leftBack->setYOffset(offset+ySpacing)) return false;
	if(!rightBack->setYOffset(offset+ySpacing)) return false;
	if(!rightFront->setYOffset(offset+ySpacing)) return false;

	yOffset = offset;
	return true;
}

bool Robot::setZOffset(float offset)
{
	if(!leftFront->setZOffset(offset)) return false;
	if(!leftBack->setZOffset(offset)) return false;
	if(!rightBack->setZOffset(offset)) return false;
	if(!rightFront->setZOffset(offset)) return false;

	zOffset = offset;
	return true;
}

bool Robot::setXSpacing(float spacing)
{
	spacing /= 2;

	if(!leftFront->setXOffset(spacing+xOffset)) return false;
	if(!rightFront->setXOffset(spacing+xOffset)) return false;
	if(!leftBack->setXOffset(-spacing+xOffset)) return false;
	if(!rightBack->setXOffset(-spacing+xOffset)) return false;

	ySpacing = spacing;
	return true;
}

bool Robot::setYSpacing(float spacing)
{
	spacing /=2;

	if(!leftFront->setYOffset(spacing+yOffset)) return false;
	if(!rightFront->setYOffset(spacing+yOffset)) return false;
	if(!leftBack->setYOffset(-spacing+yOffset)) return false;
	if(!rightBack->setYOffset(-spacing+yOffset)) return false;

	ySpacing = spacing;
	return true;
}

bool Robot::setAllLegsPosition(float x, float y, float z)
{
	if(!leftFront->setPosition(x, y, z)) return false;
	if(!rightFront->setPosition(x, y, z)) return false;
	if(!leftBack->setPosition(x, y, z)) return false;
	if(!rightBack->setPosition(x, y, z)) return false;

	return true;
}

void Robot::setXBodyAngle(float angle)
{
	xBodyAngle = angle;
}

void Robot::setYBodyAngle(float angle)
{
	yBodyAngle = angle;
}

bool Robot::moveAllLegs()
{
	if(!leftFront->move()) return false;
	if(!rightFront->move()) return false;
	if(!leftBack->move()) return false;
	if(!rightBack->move()) return false;

	return true;
}

bool Robot::levelBody()
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

void Robot::setStartLegs(bool startLeg)
{
	this->startLeg = startLeg;
}

void Robot::walk
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

	while (moveTimer < pause*100000) // pause
	{
		if(stabilization) levelBody();
	}

	leftFront->calculateStepParameters(nPoints, xLengthL, yLength, rot);
	leftBack->calculateStepParameters(nPoints, xLengthL, yLength, rot);
	rightFront->calculateStepParameters(nPoints, xLengthR, yLength, rot);
	rightBack->calculateStepParameters(nPoints, xLengthR, yLength, rot);

	for (int i = 0; i < nPoints + 1; i++)
	{
		moveTimer = 0;

		leftFront->setStepPoint(i, nPoints, xLengthL, yLength, rot, stepHeight, zHeight, !startLeg);
		leftBack->setStepPoint(i, nPoints, xLengthL, yLength, rot, stepHeight, zHeight, startLeg);

		rightFront->setStepPoint(i, nPoints, xLengthR, yLength, rot, stepHeight, zHeight, startLeg);
		rightBack->setStepPoint(i, nPoints, xLengthR, yLength, rot, stepHeight, zHeight, !startLeg);

		leftFront->move();
		leftBack->move();
		rightFront->move();
		rightBack->move();

		while (moveTimer < time/nPoints*200000 && i < nPoints) //delay
		{
			if(stabilization) levelBody();
		}
	}

	moveTimer = 0;
	startLeg = !startLeg;
}

void Robot::goForAzimuth
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