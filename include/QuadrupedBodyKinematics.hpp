#pragma once
#include <Leg2DoF.hpp>
#include <IMUFilter.hpp>

class QuadrupedBodyKinematics
{
private:

	Leg2DoF* leftFront;
	Leg2DoF* rightFront;
	Leg2DoF* leftBack;
	Leg2DoF* rightBack;

	IMUFilter* imuFilter;

	float xSide;
	float ySide;

	float xOffset;
	float yOffset;
	float zOffset;

	float xLegsMountingSpacing;
	float yLegsMountingSpacing;

	float xSpacing;
	float ySpacing;

	float xBodyAngle;
	float yBodyAngle;

	bool LFSetPosToBodyAngle(float xAngle, float yAngle);
	bool RFSetPosToBodyAngle(float xAngle, float yAngle);
	bool LBSetPosToBodyAngle(float xAngle, float yAngle);
	bool RBSetPosToBodyAngle(float xAngle, float yAngle);

public:

	QuadrupedBodyKinematics();
	QuadrupedBodyKinematics
	(
		float xLegsMountingSpacing,	// x axis legs mounting spacing
		float yLegsMountingSpacing, // y axis legs mounting spacing
		Leg2DoF &leftFront,
		Leg2DoF &leftBack,
		Leg2DoF &rightFront,
		Leg2DoF &rightBack
	);

	bool setXOffset(float offset);    // set body offset
	bool setYOffset(float offset);
	bool setZOffset(float offset);
	
	bool setXSpacing(float spacing);    // set spacing between legs
	bool setYSpacing(float spacing);

	bool setAllLegsPosition(float x, float y, float z);    // set same positions of all the legs
	bool setBodyAngle(float xAngle, float yAngle);         // set angle of body

	float getXMountingSpacing();    // get x cordinate legs mounting spacing
	float getYMountingSpacing();    // get x cordinate legs mounting spacing
	float getXSpacing();            // get x cordinate legs spacing
	float getYSpacing();            // get x cordinate legs spacing

	float getXOffset();    // get x body position
	float getYOffset();    // get y body position
	float getZOffset();    // get z body position

	float getXAngle();    // get x body orientation
	float getYAngle();    // get y body orientation
};