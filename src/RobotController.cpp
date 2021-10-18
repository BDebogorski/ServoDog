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
    PowerSystem &pwrSystem,
    IMUFilter &imuFilter
)
{
	this->leftFront = &leftFront;
	this->leftBack = &leftBack;
	this->rightFront = &rightFront;
	this->rightBack = &rightBack;

    this->bodyKinematics = &bodyKinematics;

    this->pwrSystem = &pwrSystem;
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

bool RobotController::moveBodySmoothly(float time, float nPoints)
{
    float lXLF = leftFront->getLastXOffset();
    float lYLF = leftFront->getLastYOffset();
    float lZLF = leftFront->getLastZOffset();

    float lXRF = rightFront->getLastXOffset();
    float lYRF = rightFront->getLastYOffset();
    float lZRF = rightFront->getLastZOffset();

    float lXLB = leftBack->getLastXOffset();
    float lYLB = leftBack->getLastYOffset();
    float lZLB = leftBack->getLastZOffset();

    float lXRB = rightBack->getLastXOffset();
    float lYRB = rightBack->getLastYOffset();
    float lZRB = rightBack->getLastZOffset();

    float LFXDiff = leftFront->getXOffset() - lXLF;
    float LFYDiff = leftFront->getYOffset() - lYLF;
    float LFZDiff = leftFront->getZOffset() - lZLF;

    float RFXDiff = rightFront->getXOffset() - lXRF;
    float RFYDiff = rightFront->getYOffset() - lYRF;
    float RFZDiff = rightFront->getZOffset() - lZRF;

    float LBXDiff = leftBack->getXOffset() - lXLB;
    float LBYDiff = leftBack->getYOffset() - lYLB;
    float LBZDiff = leftBack->getZOffset() - lZLB;

    float RBXDiff = rightBack->getXOffset() - lXRB;
    float RBYDiff = rightBack->getYOffset() - lYRB;
    float RBZDiff = rightBack->getZOffset() - lZRB;

    for (int i = 0; i < nPoints+1; i++)
	{
        moveTimer = 0;

        if(!leftFront->setOffset(lXLF+LFXDiff/nPoints*i, lYLF+LFYDiff/nPoints*i, lZLF+LFZDiff/nPoints*i)) return false;
        if(!rightFront->setOffset(lXRF+RFXDiff/nPoints*i, lYRF+RFYDiff/nPoints*i, lZRF+RFZDiff/nPoints*i)) return false;
        if(!leftBack->setOffset(lXLB+LBXDiff/nPoints*i, lYLB+LBYDiff/nPoints*i, lZLB+LBZDiff/nPoints*i)) return false;
        if(!rightBack->setOffset(lXRB+RBXDiff/nPoints*i, lYRB+RBYDiff/nPoints*i, lZRB+RBZDiff/nPoints*i)) return false;

        if(!moveAllLegs()) return false;

        while (moveTimer < time/nPoints*100000 && i < nPoints) //delay
		{
			//if(stabilization) levelBody();
		}
    }

    moveTimer = 0;
    return true;
}

bool RobotController::moveAllLegsSmoothly(float time, int nPoints)
{
    float lXLF = leftFront->getLastXPosition();
    float lYLF = leftFront->getLastYPosition();
    float lZLF = leftFront->getLastZPosition();

    float lXRF = rightFront->getLastXPosition();
    float lYRF = rightFront->getLastYPosition();
    float lZRF = rightFront->getLastZPosition();

    float lXLB = leftBack->getLastXPosition();
    float lYLB = leftBack->getLastYPosition();
    float lZLB = leftBack->getLastZPosition();

    float lXRB = rightBack->getLastXPosition();
    float lYRB = rightBack->getLastYPosition();
    float lZRB = rightBack->getLastZPosition();

    float LFXDiff = leftFront->getXPosition() - lXLF;
    float LFYDiff = leftFront->getYPosition() - lYLF;
    float LFZDiff = leftFront->getZPosition() - lZLF;

    float RFXDiff = rightFront->getXPosition() - lXRF;
    float RFYDiff = rightFront->getYPosition() - lYRF;
    float RFZDiff = rightFront->getZPosition() - lZRF;

    float LBXDiff = leftBack->getXPosition() - lXLB;
    float LBYDiff = leftBack->getYPosition() - lYLB;
    float LBZDiff = leftBack->getZPosition() - lZLB;

    float RBXDiff = rightBack->getXPosition() - lXRB;
    float RBYDiff = rightBack->getYPosition() - lYRB;
    float RBZDiff = rightBack->getZPosition() - lZRB;

    for (int i = 0; i < nPoints+1; i++)
	{
        moveTimer = 0;

        if(!leftFront->setPosition(lXLF+LFXDiff/nPoints*i, lYLF+LFYDiff/nPoints*i, lZLF+LFZDiff/nPoints*i)) return false;
        if(!rightFront->setPosition(lXRF+RFXDiff/nPoints*i, lYRF+RFYDiff/nPoints*i, lZRF+RFZDiff/nPoints*i)) return false;
        if(!leftBack->setPosition(lXLB+LBXDiff/nPoints*i, lYLB+LBYDiff/nPoints*i, lZLB+LBZDiff/nPoints*i)) return false;
        if(!rightBack->setPosition(lXRB+RBXDiff/nPoints*i, lYRB+RBYDiff/nPoints*i, lZRB+RBZDiff/nPoints*i)) return false;

        if(!moveAllLegs()) return false;

        while (moveTimer < time/nPoints*100000 && i < nPoints) //delay
		{
			//if(stabilization) levelBody();
		}
    }

    moveTimer = 0;
    return true;
}

bool RobotController::levelBody()
{
	float xAngle = imuFilter->getXAngle()-M_PI + bodyKinematics->getXAngle();
	float yAngle = imuFilter->getYAngle()-M_PI + bodyKinematics->getYAngle();

    if(!bodyKinematics->setBodyAngle(xAngle, yAngle)) return false;

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

bool RobotController::jump   // jumping algorithm prototype
(
    float xAcceleration,
    float yAcceleration,
    float zAcceleration,
    float xZero,
    float yZero,
    float xAmplitude,
    float yAmplitude,
    float zMin,
    float zMax,
    float dt,
    bool stabilization
)
{
    float t = 0;

    float x;
    float y;
    float z;

    while(z < zMax)
	{
		moveTimer = 0;

        x = xZero + xAcceleration*t*t/2;    // x = a*t^2/2
        y = yZero + yAcceleration*t*t/2;
        z = zMin + zAcceleration*t*t/2;

        if(x > xAmplitude) x = xAmplitude;
        if(x < -xAmplitude) x = -xAmplitude;
        if(y > yAmplitude) y = yAmplitude;
        if(y < -yAmplitude) y = -yAmplitude;
    
        if(!bodyKinematics->setAllLegsPosition(x, y, z)) return false;
        if(!moveAllLegs()) return false;

		while (moveTimer < dt*200000) //delay
		{
			if(stabilization) levelBody();
		}

        t += dt;    // time
	}

    if(!bodyKinematics->setAllLegsPosition(xZero, yZero, zMin)) return false;
    if(!moveAllLegs()) return false;

	moveTimer = 0;
    return true;
}

void RobotController::sitAndTurnOff(float time, int nPoints)
{
    leftFront->setStartPosition();
    rightFront->setStartPosition();
    leftBack->setStartPosition();
    rightBack->setStartPosition();

    moveAllLegsSmoothly(time, nPoints);
    pwrSystem->off();
}