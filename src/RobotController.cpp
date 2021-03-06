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
    IMUFilter &imuFilter,
    RemoteControll &remoteControll
)
{
	this->leftFront = &leftFront;
	this->leftBack = &leftBack;
	this->rightFront = &rightFront;
	this->rightBack = &rightBack;

    this->bodyKinematics = &bodyKinematics;

    this->pwrSystem = &pwrSystem;
	this->imuFilter = &imuFilter;

    this->remoteControll = &remoteControll;

    xAngleOffset = 0;
    yAngleOffset = 0;

    float x = this->bodyKinematics->getXMountingSpacing();
    float y = this->bodyKinematics->getYMountingSpacing();

	walkingAlgorithm.setLeftFrontZeroPosition(x/2, y/2);
	walkingAlgorithm.setRightFrontZeroPosition(-x/2, y/2);
	walkingAlgorithm.setLeftBackZeroPosition(x/2, -y/2);
	walkingAlgorithm.setRightBackZeroPosition(-x/2, -y/2);

    readyToControll = false;
    zeroLeg = true;
    isOn = false;
}

void RobotController::setXAngleOffset(float offset)
{
    xAngleOffset = offset;
}

void RobotController::setYAngleOffset(float offset)
{
    yAngleOffset = offset;
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

bool RobotController::moveBodySmoothly(float time, float nPoints, bool stabilization)
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
			if(stabilization) levelBody();
		}
    }

    moveTimer = 0;
    return true;
}

bool RobotController::moveAllLegsSmoothly(float time, int nPoints, bool stabilization)
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
			if(stabilization) levelBody();
		}
    }

    moveTimer = 0;
    return true;
}

bool RobotController::levelBody()
{
	float xAngle = imuFilter->getXAngle() - M_PI + xAngleOffset;
	float yAngle = imuFilter->getYAngle() - M_PI + yAngleOffset;

    //float xDiff = fabs(xAngle-bodyKinematics->getXAngle());
    //float yDiff = fabs(yAngle-bodyKinematics->getYAngle());

    if(!bodyKinematics->setBodyAngle(xAngle, yAngle)) return false;

    //float diff = 0.05;

    //if(xDiff > diff || yDiff > diff) return moveBodySmoothly(0.5, 100, false);
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
    Cordinates LFCordinates;
	Cordinates RFCordinates;
	Cordinates LBCordinates;
	Cordinates RBCordinates;

	while (moveTimer < pause*100000) // pause
	{
		if(stabilization) levelBody();
	}

	walkingAlgorithm.setLeftFrontStartPosition(leftFront->getXPosition(), leftFront->getYPosition());
	walkingAlgorithm.setRightFrontStartPosition(rightFront->getXPosition(), rightFront->getYPosition());
	walkingAlgorithm.setLeftBackStartPosition(leftBack->getXPosition(), leftBack->getYPosition());
	walkingAlgorithm.setRightBackStartPosition(rightBack->getXPosition(), rightBack->getYPosition());

	walkingAlgorithm.setWalkParameters(nPoints, zHeight, stepHeight, xLengthL, xLengthR, yLength, rot);

	for (int i = 0; i < nPoints+1; i++)
	{
		moveTimer = 0;

        walkingAlgorithm.calculate(i);

		walkingAlgorithm.getLeftFrontLegPosition(LFCordinates);
		walkingAlgorithm.getRightFrontLegPosition(RFCordinates);
		walkingAlgorithm.getLeftBackLegPosition(LBCordinates);
		walkingAlgorithm.getRightBackLegPosition(RBCordinates);

		if(!leftFront->setPosition(LFCordinates.x, LFCordinates.y, LFCordinates.z)) return false;
		if(!rightFront->setPosition(RFCordinates.x, RFCordinates.y, RFCordinates.z)) return false;
		if(!leftBack->setPosition(LBCordinates.x, LBCordinates.y, LBCordinates.z)) return false;
		if(!rightBack->setPosition(RBCordinates.x, RBCordinates.y, RBCordinates.z)) return false;

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
    float xLeftAcceleration,
    float xRightAcceleration,
    float yAcceleration,
    float zAcceleration,
    float xAmplitude,
    float yAmplitude,
    float zMin,
    float zMax,
    float dt,
    bool stabilization
)
{
    float t = 0;

    float xLeft;
    float xRight;
    float y;
    float z;

    while
    (
        leftFront->getZPosition() + leftFront->getXOffset() < zMax &&
        rightFront->getZPosition() + rightFront->getXOffset() < zMax &&
        leftBack->getZPosition() + leftBack->getXOffset() < zMax &&
        rightBack->getZPosition() + rightBack->getXOffset() < zMax
    )
	{
		moveTimer = 0;

        xLeft = -xLeftAcceleration*t*t/2;    // x = a*t^2/2
        xRight = -xRightAcceleration*t*t/2;    // x = a*t^2/2
        y = yAcceleration*t*t/2;
        z = zMin + zAcceleration*t*t/2;

        if(xLeft > xAmplitude) xLeft = xAmplitude;
        if(xLeft < -xAmplitude) xLeft = -xAmplitude;

        if(xRight > xAmplitude) xRight = xAmplitude;
        if(xRight < -xAmplitude) xRight = -xAmplitude;

        if(y > yAmplitude) y = yAmplitude;
        if(y < -yAmplitude) y = -yAmplitude;
    
        if(!leftFront->setPosition(xLeft, y, z)) return false;
        if(!leftBack->setPosition(xLeft, y, z)) return false;

        if(!rightFront->setPosition(xRight, y, z)) return false;
        if(!rightBack->setPosition(xRight, y, z)) return false;

        if(!moveAllLegs()) return false;

		while (moveTimer < dt*200000) //delay
		{
			if(stabilization) levelBody();
		}

        t += dt;    // time
	}

    if(!bodyKinematics->setAllLegsPosition(0, 0, zMin)) return false;
    if(!moveAllLegs()) return false;

	moveTimer = 0;
    return true;
}

bool RobotController::jumpForAzimuth
(
    float xAcceleration,
    float zAcceleration,
    float amplitude,
    float zMin,
    float zMax,
    float dt,
    float actualAngle,
	float azimuth,
	float fullStepAngle,
    bool stabilization
)
{
    float xRightAcceleration = -fabs(xAcceleration) * map(actualAngle-azimuth, -2*M_PI, 2*M_PI, -M_PI/fullStepAngle, M_PI/fullStepAngle);
	float xLeftAcceleration;

	xLeftAcceleration = -xRightAcceleration;

	xRightAcceleration += (xRightAcceleration-xAcceleration);
	xLeftAcceleration += (xLeftAcceleration-xAcceleration);

	if(xLeftAcceleration > xAcceleration) xLeftAcceleration = xAcceleration;
	if(xLeftAcceleration < -xAcceleration) xLeftAcceleration = -xAcceleration;

    Serial.println(xRightAcceleration);

    if(xRightAcceleration > xAcceleration) xRightAcceleration = xAcceleration;
	if(xRightAcceleration < -xAcceleration) xRightAcceleration = -xAcceleration;

    Serial.println(xRightAcceleration);

    return jump(xLeftAcceleration, xRightAcceleration, 0, zAcceleration, amplitude, 0, zMin, zMax, dt, stabilization);
}

bool RobotController::zeroByWalking
(
    float time,
    float pause,
    int nPoints,
    float zHeight,
    float stepHeight,
    bool stabilization
)
{
    if(!walk(time, pause, nPoints, zHeight, stepHeight, 0, 0, 0, 0, stabilization)) return false;
    if(!walk(time, 0, nPoints, zHeight, stepHeight, 0, 0, 0, 0, stabilization)) return false;

    return true;
}

bool RobotController::remoteBodyPosition(float time, int nPoints, float x, float y, float z, float xAngle, float yAngle, bool stabilization)
{
    bool move = false;

    //if(!readyToControll) return false;

    if(bodyKinematics->getXOffset() != x) move = true;
    if(bodyKinematics->getYOffset() != y) move = true;
    if(bodyKinematics->getZOffset() != z) move = true;

    if(!stabilization)
    {
        if(bodyKinematics->getXAngle() != xAngle) move = true;
        if(bodyKinematics->getYAngle() != yAngle) move = true;
    }

    if(move)
    {
        if(!bodyKinematics->setXOffset(x)) return false;
        if(!bodyKinematics->setYOffset(y)) return false;
        if(!bodyKinematics->setZOffset(z)) return false;

        if(stabilization)
        {
            //setXAngleOffset(xAngle);
            //setYAngleOffset(yAngle);
        }
        else
        {
            if(!bodyKinematics->setBodyAngle(xAngle, yAngle)) return false;
        }

        if(!moveBodySmoothly(time, nPoints, false)) return false;
    }

    return true;
}

bool RobotController::remoteControllTankWalk
(
    float time,
    float pause,
    int nPoints,
    int xJoy,
    int yJoy,
    float heightBody,
    float stepHeight,
    float maxStepLength,
    bool stop,
    bool stabilization
)
{
    float left;
    float right;

    remoteControll->mixer(xJoy, yJoy, left, right, maxStepLength);

    if(!stop || left != 0 || right != 0)
    {
        if(readyToControll)
        {
            if(!walk(time, pause, nPoints, heightBody, stepHeight, left, right, 0, 0, stabilization)) return false;
        }

        zeroLeg = false;
    }
    else
    {
        readyToControll = true;

        if(!zeroLeg)
        {
            if(!zeroByWalking(time, pause, nPoints, heightBody, stepHeight, stabilization)) return false;
            zeroLeg = true;
        }
      
        if(stabilization) levelBody();
    }

    return true;
}

void RobotController::remoteOff(bool on, float time, int nPoints)
{
    if(isOn && !on) sitAndTurnOff(time, nPoints);
    else if(on) isOn = true;
}

void RobotController::sitAndTurnOff(float time, int nPoints)
{
    setBody(0, 0, 0, 0, 0, bodyKinematics->getXMountingSpacing(), bodyKinematics->getYMountingSpacing());
    moveBodySmoothly(time, nPoints, false);

    leftFront->setStartPosition();
    rightFront->setStartPosition();
    leftBack->setStartPosition();
    rightBack->setStartPosition();

    moveAllLegsSmoothly(time, nPoints, false);
    pwrSystem->off();
}

bool RobotController::safeController(int iMUTemperature, int CPUTemperature, int maxIMUTemperature, int maxCPUTemperature)
{
  if(pwrSystem->isEmpty())
  {
    Serial.print("Batterry low! (");
    Serial.print(pwrSystem->getBatteryVoltage());
    Serial.println(")");
    return false;
  }

  if(iMUTemperature > maxIMUTemperature)
  {
    Serial.print("IMU Overheating! (");
    Serial.print(iMUTemperature);
    Serial.println("*C)");
    return false;
  }

  if(CPUTemperature > maxCPUTemperature)
  {
    Serial.print("CPU Overheating! (");
    Serial.print(CPUTemperature);
    Serial.println("*C)");
    return false;
  }

  return true;
}