#include <Arduino.h>
#include <QuadrupedBodyKinematics.hpp>

bool QuadrupedBodyKinematics::LFSetPosToBodyAngle(float xAngle, float yAngle)
{
	float xSide = (leftFront->getXPosition()-leftFront->getXOffset()+xSpacing/2)*sin(yAngle);
	float ySide = (leftFront->getYPosition()-leftFront->getYOffset()+ySpacing/2)*sin(xAngle);

	return leftFront->setZOffset(xSide-ySide+zOffset);
}

bool QuadrupedBodyKinematics::RFSetPosToBodyAngle(float xAngle, float yAngle)
{
	float xSide = (rightFront->getXPosition()-rightFront->getXOffset()+xSpacing/2)*sin(yAngle);
	float ySide = (rightFront->getYPosition()-rightFront->getYOffset()+ySpacing/2)*sin(xAngle);

	return rightFront->setZOffset(xSide+ySide+zOffset);
}

bool QuadrupedBodyKinematics::LBSetPosToBodyAngle(float xAngle, float yAngle)
{
	float xSide = (leftBack->getXPosition()-leftBack->getXOffset()+xSpacing/2)*sin(yAngle);
	float ySide = (leftBack->getYPosition()-leftBack->getYOffset()+ySpacing/2)*sin(xAngle);

	return leftBack->setZOffset(-xSide-ySide+zOffset);
}

bool QuadrupedBodyKinematics::RBSetPosToBodyAngle(float xAngle, float yAngle)
{
	float xSide = (rightBack->getXPosition()-rightBack->getXOffset()+xSpacing/2)*sin(yAngle);
	float ySide = (rightBack->getYPosition()-rightBack->getYOffset()+ySpacing/2)*sin(xAngle);

	return rightBack->setZOffset(-xSide+ySide+zOffset);
}

QuadrupedBodyKinematics::QuadrupedBodyKinematics() {}

QuadrupedBodyKinematics::QuadrupedBodyKinematics
(
	float xLegsMountingSpacing,
	float yLegsMountingSpacing,
	Leg2DoF &leftFront,
	Leg2DoF &leftBack,
	Leg2DoF &rightFront,
	Leg2DoF &rightBack
)
{
	this->leftFront = &leftFront;
	this->leftBack = &leftBack;
	this->rightFront = &rightFront;
	this->rightBack = &rightBack;

	xOffset = 0;
	yOffset = 0;
	zOffset = 0;

	xBodyAngle = 0;
	yBodyAngle = 0;

	this->xLegsMountingSpacing = xLegsMountingSpacing;
	this->yLegsMountingSpacing = yLegsMountingSpacing;

	setXSpacing(xLegsMountingSpacing);
	setYSpacing(yLegsMountingSpacing);
}

bool QuadrupedBodyKinematics::setXOffset(float offset)
{
	if(!leftFront->setXOffset(offset+(xSpacing-xLegsMountingSpacing)/2)) return false;
	if(!leftBack->setXOffset(offset+(xLegsMountingSpacing-xSpacing)/2)) return false;
	if(!rightBack->setXOffset(offset+(xLegsMountingSpacing-xSpacing)/2)) return false;
	if(!rightFront->setXOffset(offset+(xSpacing-xLegsMountingSpacing)/2)) return false;

	xOffset = offset;
	return true;
}

bool QuadrupedBodyKinematics::setYOffset(float offset)
{
	if(!leftFront->setYOffset(offset+(ySpacing-yLegsMountingSpacing)/2)) return false;
	if(!leftBack->setYOffset(offset+(yLegsMountingSpacing-ySpacing)/2)) return false;
	if(!rightBack->setYOffset(offset+(ySpacing-yLegsMountingSpacing)/2)) return false;
	if(!rightFront->setYOffset(offset+(yLegsMountingSpacing-ySpacing)/2)) return false;

	yOffset = offset;
	return true;
}

bool QuadrupedBodyKinematics::setZOffset(float offset)
{
	if(!leftFront->setZOffset(offset)) return false;
	if(!leftBack->setZOffset(offset)) return false;
	if(!rightBack->setZOffset(offset)) return false;
	if(!rightFront->setZOffset(offset)) return false;

	zOffset = offset;
	return true;
}

bool QuadrupedBodyKinematics::setXSpacing(float spacing)
{
	if(!leftFront->setXOffset((spacing-xLegsMountingSpacing)/2+xOffset)) return false;
	if(!rightFront->setXOffset((spacing-xLegsMountingSpacing)/2+xOffset)) return false;
	if(!leftBack->setXOffset((xLegsMountingSpacing-spacing)/2+xOffset)) return false;
	if(!rightBack->setXOffset((xLegsMountingSpacing-spacing)/2+xOffset)) return false;

	xSpacing = spacing;
	return true;
}

bool QuadrupedBodyKinematics::setYSpacing(float spacing)
{
	if(!leftFront->setYOffset((spacing-yLegsMountingSpacing)/2+yOffset)) return false;
	if(!rightFront->setYOffset((yLegsMountingSpacing-spacing)/2+yOffset)) return false;
	if(!leftBack->setYOffset((spacing-yLegsMountingSpacing)/2+yOffset)) return false;
	if(!rightBack->setYOffset((yLegsMountingSpacing-spacing)/2+yOffset)) return false;

	ySpacing = spacing;
	return true;
}

bool QuadrupedBodyKinematics::setAllLegsPosition(float x, float y, float z)
{
	if(!leftFront->setPosition(x, y, z)) return false;
	if(!rightFront->setPosition(x, y, z)) return false;
	if(!leftBack->setPosition(x, y, z)) return false;
	if(!rightBack->setPosition(x, y, z)) return false;

	return true;
}

bool QuadrupedBodyKinematics::setBodyAngle(float xAngle, float yAngle)
{
	if(!LFSetPosToBodyAngle(xAngle, yAngle)) return false;
	if(!RFSetPosToBodyAngle(xAngle, yAngle)) return false;
	if(!LBSetPosToBodyAngle(xAngle, yAngle)) return false;
	if(!RBSetPosToBodyAngle(xAngle, yAngle)) return false;

	xBodyAngle = xAngle;
	yBodyAngle = yAngle;

	return true;
}

float QuadrupedBodyKinematics::getXMountingSpacing()
{
	return xLegsMountingSpacing;
}

float QuadrupedBodyKinematics::getYMountingSpacing()
{
	return yLegsMountingSpacing;
}

float QuadrupedBodyKinematics::getXSpacing()
{
	return xSpacing;
}

float QuadrupedBodyKinematics::getYSpacing()
{
	return ySpacing;
}

float QuadrupedBodyKinematics::getXOffset()
{
	return xOffset;
}

float QuadrupedBodyKinematics::getYOffset()
{
	return yOffset;
}

float QuadrupedBodyKinematics::getZOffset()
{
	return zOffset;
}

float QuadrupedBodyKinematics::getXAngle()
{
	return xBodyAngle;
}

float QuadrupedBodyKinematics::getYAngle()
{
	return yBodyAngle;
}