#include <Leg2DoF.hpp>
#include <Arduino.h>

bool Leg2DoF::invKinematics(float x, float y, float z)
{
    if(xSide == false) x = -x;
	if(ySide == false) y = -y;
    if(zSide == false) z = -z;

    if(sqrt(x*x + z*z) >= r1+r2 || sqrt(x*x + z*z) < fabs(r1-r2)+rFeet) return false;

    z -= rFeet;

    float xSqr = x*x;
    float zSqr = z*z;
	
	if(x==0 && z==0) // kinematic singularity
	{
        t1 = 0;
		t2 = M_PI/2;
	}
	else
	{
        t1 = 2*atan((21250*z - sqrt(-(62500*xSqr + 62500*zSqr - 529)*(1000000*xSqr + 1000000*zSqr - 49)))/(1250*x*(200*x + 17) + 250000*zSqr - 161));
		t2 = 2*atan((4*sqrt(-(62500*xSqr + 62500*zSqr - 529)*(1000000*xSqr + 1000000*zSqr - 49)))/(1000000*xSqr + 1000000*zSqr - 49));
	}

	return true;
}

void Leg2DoF::getStepPointXY
(
    int nPoints,
    int pointNumber,
    int pairLeg,
    float xLength,
    float yLength,
    float rotationAngle,
    float& xPoint,
    float& yPoint
)
{
	if(pointNumber == 0)   // first point of step
	{
		xStartStep = xPosition;    // start point of step
		yStartStep = yPosition;    // start point of step
	}

	float xEnd = pairLeg * (nPoints/2 - nPoints) * xLength / nPoints;    // end point of step
	float yEnd = pairLeg * (nPoints/2 - nPoints) * yLength / nPoints;    // end point of step

	float rotRate = (nPoints/2 - pointNumber) * pairLeg * rotationAngle / nPoints;    // angle in this point

	xPoint = xStartStep - pairLeg * (pairLeg * (xStartStep-xEnd) / nPoints * pointNumber) +    // translation
    xMountingPosition*cos(rotRate) - yMountingPosition*sin(rotRate) - xMountingPosition;       // rotation

	yPoint = yStartStep - pairLeg * (pairLeg * (yStartStep-yEnd) / nPoints * pointNumber) +   // translation
    xMountingPosition*sin(rotRate) + yMountingPosition*cos(rotRate) - yMountingPosition;      // rotation
}

Leg2DoF::Leg2DoF() {}

Leg2DoF::Leg2DoF
(
	float r1,
	float r2,
	float rFeet,
    float xMountingPosition,
    float yMountingPosition,
    bool xSide,
	bool ySide,
    bool zSide,
	ServoMotor &hip,
	ServoMotor &knee
)
{

    this->r1 = r1;
    this->r2 = r2;
    this->rFeet = rFeet;

    this->xSide = xSide;
    this->ySide = ySide;
    this->zSide = zSide;

    this->xMountingPosition = xMountingPosition;
    this->yMountingPosition = yMountingPosition;

    this->hip = &hip;
    this->knee = &knee;

    xPosition = 0;
    yPosition = 0;
    zPosition = 0;

    lastXPosition = 0;
    lastYPosition = 0;
    lastYPosition = 0;

    xOffset = 0;
    yOffset = 0;
    zOffset = 0;

    t1 = 0;
    t2 = 0;

    xStartStep = 0;
    yStartStep = 0;

    lengthOfMove = 0;

    readyToMove = false;
}

bool Leg2DoF::setXOffset(float offset)
{
    if(invKinematics(xPosition-xOffset+offset, yPosition, zPosition))
    {
        xOffset = offset;
        setPosition(xPosition, yPosition, zPosition);

        return true;
    }

    return false;
}

bool Leg2DoF::setYOffset(float offset)
{
    if(invKinematics(xPosition, yPosition-yOffset+offset, zPosition))
    {
        yOffset = offset;
        setPosition(xPosition, yPosition, zPosition);

        return true;
    }

    return false;
}

bool Leg2DoF::setZOffset(float offset)
{
    if(invKinematics(xPosition, yPosition, zPosition-zOffset+offset))
    {
        zOffset = offset;
        setPosition(xPosition, yPosition, zPosition);

        return true;
    }

    return false;
}

bool Leg2DoF::setOffset(float xOffset, float yOffset, float zOffset)
{
    if(!setXOffset(xOffset)) return false;
    if(!setYOffset(yOffset)) return false;
    if(!setZOffset(zOffset)) return false;

    return true;
}

bool Leg2DoF::setPosition(float x, float y, float z)
{
    if(invKinematics(x+xOffset, y+yOffset, z+zOffset))
    {
        xPosition = x; // save actual position
        yPosition = y;
        zPosition = z;

        readyToMove = true;
        return true;
    }

    Serial.println("Kinematics error!");
    readyToMove = false;
    return false;
}

bool Leg2DoF::move()
{
    if(!readyToMove) return false;

    if(!hip->setPosition(t1*180/M_PI)) return false;
    if(!knee->setPosition(t2*180/M_PI)) return false;

    lastXPosition = xPosition;
    lastYPosition = yPosition;
    lastZPosition = zPosition;

    return true;
}

void Leg2DoF::calculateStepParameters(int nPoints, float xLength, float yLength, float rotation)
{
	float xEnd;
    float yEnd;

    getStepPointXY(nPoints, 0, true, xLength, yLength, rotation, xStartStep, yStartStep);    // get start point of step
	getStepPointXY(nPoints, nPoints, true, xLength, yLength, rotation, xEnd, yEnd);          // get end point of step
	lengthOfMove = sqrt(pow(xStartStep - xEnd, 2) + pow(yStartStep - yEnd, 2));              // calculate length on step (XY)
}

void Leg2DoF::setStepPoint
(
    int pointNumber,
    int nPoints,
    float xLength,
    float yLength,
    float rotation,
    float stepHeight,
    float zBodyPosition,
    bool isOnGround
)
{
    float xStep;
    float yStep;
    float zStep;

	if(isOnGround)
	{
		getStepPointXY(nPoints, pointNumber, -1, xLength, yLength, rotation, xStep, yStep);
		zStep = zBodyPosition;
	}
	else
	{
		getStepPointXY(nPoints, pointNumber, 1, xLength, yLength, rotation, xStep, yStep);

		float xE = 2 * (sqrt(pow(xStep-xStartStep, 2) + pow(yStep-yStartStep, 2)) - lengthOfMove/2);
		float t = acos(xE/lengthOfMove);
		zStep = zBodyPosition - stepHeight*sin(t);    // leg trajectory
	}

    setPosition(xStep, yStep, zStep);    // set point of step
}

float Leg2DoF::getXOffset()
{
    return xOffset;
}

float Leg2DoF::getYOffset()
{
    return yOffset;
}

float Leg2DoF::getZOffset()
{
    return zOffset;
}

float Leg2DoF::getXPosition()
{
    return xPosition;
}

float Leg2DoF::getYPosition()
{
    return yPosition;
}

float Leg2DoF::getZPosition()
{
    return zPosition;
}

float Leg2DoF::getLastXPosition()
{
    return lastXPosition;
}

float Leg2DoF::getLastYPosition()
{
    return lastYPosition;
}

float Leg2DoF::getLastZPosition()
{
    return lastZPosition;
}

float Leg2DoF::getKneeAngle()
{
    return t1;
}

float Leg2DoF::getHipAngle()
{
    return t2;
}