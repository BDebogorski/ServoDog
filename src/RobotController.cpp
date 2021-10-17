#include <Arduino.h>
#include <RobotController.hpp>

volatile int moveTimer = 0;

void moveClock()
{
	moveTimer++;
}

RobotController::RobotController
(
    Leg2DoF &leftFront,
    Leg2DoF &leftBack,
    Leg2DoF &rightFront,
    Leg2DoF &rightBack,
    QuadrupedBodyKinematics &bodyKinematics,
    IMUFilter &imuFilter
)
{
	this->leftFront = &leftFront;
	this->leftBack = &leftBack;
	this->rightFront = &rightFront;
	this->rightBack = &rightBack;

    this->bodyKinematics = &bodyKinematics;
	this->imuFilter = &imuFilter;

    float x = this->bodyKinematics->getXMountingSpacing();
    float y = this->bodyKinematics->getYMountingSpacing();

	walkingAlgorithm.setLeftFrontMountingPosition(x/2, y/2);
	walkingAlgorithm.setRightFrontMountingPosition(-x/2, y/2);
	walkingAlgorithm.setLeftBackMountingPosition(x/2, -y/2);
	walkingAlgorithm.setRightBackMountingPosition(-x/2, -y/2);
}

bool RobotController::setBody(float x, float y, float z, float xAngle, float yAngle, float xSpacing, float ySpacing)
{
    bool status = true;

    if(!bodyKinematics->setXOffset(x)) status = false;
    if(!bodyKinematics->setYOffset(y)) status = false;
    if(!bodyKinematics->setZOffset(z)) status = false;

    if(!bodyKinematics->setBodyAngle(xAngle, yAngle)) status = false;

    if(!bodyKinematics->setXSpacing(xSpacing)) status = false;
    if(!bodyKinematics->setYSpacing(ySpacing)) status = false;

    return status;
}

bool RobotController::moveAllLegs()
{

    if(!leftFront->isReadyToMove()) return false;
    if(!rightFront->isReadyToMove()) return false;
    if(!leftBack->isReadyToMove()) return false;
    if(!rightBack->isReadyToMove()) return false;

	if(!leftFront->move()) return false;
	if(!rightFront->move()) return false;
	if(!leftBack->move()) return false;
	if(!rightBack->move()) return false;

	return true;
}

bool RobotController::levelBody()
{
	float xAngle = imuFilter->getXAngle()-M_PI + bodyKinematics->getXAngle();
	float yAngle = imuFilter->getYAngle()-M_PI + bodyKinematics->getYAngle();

    bodyKinematics->setBodyAngle(xAngle, yAngle);

    return moveAllLegs();
}

void RobotController::setStartLegs(bool startLeg)
{
	walkingAlgorithm.setStartLeg(startLeg);
}

bool RobotController::walk
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

		if(!leftFront->setPosition(LFCordinates.xPosition, LFCordinates.yPosition, LFCordinates.zPosition)) return false;
		if(!rightFront->setPosition(RFCordinates.xPosition, RFCordinates.yPosition, RFCordinates.zPosition)) return false;
		if(!leftBack->setPosition(LBCordinates.xPosition, LBCordinates.yPosition, LBCordinates.zPosition)) return false;
		if(!rightBack->setPosition(RBCordinates.xPosition, RBCordinates.yPosition, RBCordinates.zPosition)) return false;

        if(!moveAllLegs()) return false;

		while (moveTimer < time/nPoints*200000 && i < nPoints) //delay
		{
			if(stabilization) levelBody();
		}
	}

	moveTimer = 0;
    return true;
}

bool RobotController::goForAzimuth
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

	return walk(time, pause, nPoints, zHeight, stepHeight, xLengthL, xLengthR, 0, 0, stabilization);
}

bool RobotController::jump(float minH, float maxH, float a)    // prototype
{
    float t;

    float x;
    float y;
    float z;

    float aX = 20;
    float aY = 0;

    float xMax = 0.03;
    float yMax = 0;

    int nPoints = 100;
    float dt =  0.01;

    bool stabilization = false;

    for (int i = 0; i < nPoints; i++)
	{
		moveTimer = 0;

        t = i*dt;

        x =-0.01+aX*t*t/2;
        y = aY*t*t/2;
        z = minH + a*t*t/2;

        if(x > xMax) x = xMax;
        if(y > yMax) y = yMax;
        if(z > maxH) break;

        if(!leftFront->setPosition(x, y, z)) return false;
        if(!leftBack->setPosition(x, y, z)) return false;

        if(!rightFront->setPosition(x, y, z)) return false;
        if(!rightBack->setPosition(x, y, z)) return false;

        if(!moveAllLegs()) return false;

		while (moveTimer < dt*200000 && i < nPoints) //delay
		{
			if(stabilization) levelBody();
		}
	}

	moveTimer = 0;
    if(!bodyKinematics->setAllLegsPosition(-0.01, 0, minH)) return false;
    if(!moveAllLegs()) return false;

    return true;
}