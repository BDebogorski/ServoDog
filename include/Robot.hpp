#pragma once
#include <Leg2DoF.hpp>
#include <IMUFilter.hpp>

extern volatile int moveTimer;
void moveClock();

class Robot
{
private:

	Leg2DoF* leftFront;
	Leg2DoF* rightFront;
	Leg2DoF* leftBack;
	Leg2DoF* rightBack;

	IMUFilter* imuFilter;

	bool startLeg;

	float xSide;
	float ySide;

	float xOffset;
	float yOffset;
	float zOffset;

	float xSpacing;
	float ySpacing;

	float xBodyAngle;
	float yBodyAngle;

public:

	Robot(Leg2DoF &leftFront, Leg2DoF &leftBack, Leg2DoF &rightFront, Leg2DoF &rightBack, IMUFilter &imuFilter);

	bool setXOffset(float offset);
	bool setYOffset(float offset);
	bool setZOffset(float offset);
	bool setXSpacing(float spacing);
	bool setYSpacing(float spacing);
	void setXBodyAngle(float angle);
	void setYBodyAngle(float angle);

	bool setAllLegsPosition(float x, float y, float z);

	bool moveAllLegs();
	bool moveAllLegsSmoothly(int nPoints, float time);
	bool levelBody();

	void setStartLegs(bool startLeg);

	void walk
	(
		float time,           // time of one step [s]
		float pause,          // time between steps [s]
		int nPoints,          // number of points in leg trajectory
		float zHeight,        // z axis body position
		float stepHeight,     // height of step
		float xLenghtL,       // left legs one step lenght on x axis
		float xLengthR,       // right legs one step length on x axis
		float yLength,        // one step length on y axis
		float rot,            // body rotation for one step
		bool stabilization    // stabilization on/off
	);

	void goForAzimuth
	(
		float time,           // time of one step [s]
		float pause,          // time between steps [s]
		int nPoints,          // number of points in leg trajectory
		float zHeight,        // z axis body position
		float stepHeight,     // height of step
		float stepLength,     // maximum length of one step
		float actualAngle,    // z axis orientation in space
		float azimuth,        // goal angle
		float fullStepAngle,  // angle at which take full step
		bool stabilization    // stabilization on/off
	);
};