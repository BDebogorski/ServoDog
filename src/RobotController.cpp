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
}

void RobotController::setLegMountingSpacing()
{
    float x = bodyKinematics->getXMountingSpacing();
    float y = bodyKinematics->getYMountingSpacing();

	walkingAlgorithm.setLeftFrontMountingPosition(x/2, y/2);
	walkingAlgorithm.setRightFrontMountingPosition(-x/2, y/2);
	walkingAlgorithm.setLeftBackMountingPosition(x/2, -y/2);
	walkingAlgorithm.setRightBackMountingPosition(-x/2, -y/2);
}

bool RobotController::levelBody()
{
	bool status = true;

	float xAngle = imuFilter->getXAngle()-M_PI + bodyKinematics->getXAngle();
	float yAngle = imuFilter->getYAngle()-M_PI + bodyKinematics->getYAngle();

    bodyKinematics->setBodyAngle(xAngle, yAngle);

	if(!leftFront->move()) status = false;
	if(!leftBack->move()) status = false;
	if(!rightFront->move()) status = false;
	if(!rightBack->move()) status = false;

	return status;
}

void RobotController::setStartLegs(bool startLeg)
{
	walkingAlgorithm.setStartLeg(startLeg);
}

void RobotController::walk
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

void RobotController::goForAzimuth
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