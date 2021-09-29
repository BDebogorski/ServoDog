#pragma once
#include <Leg2DoF.hpp>
#include <IMUFilter.hpp>
#include <QuadrupedWalkingAlgorithm.hpp>

extern volatile int moveTimer;
void moveClock();

class QuadrupedRobot
{
private:

	Leg2DoF* leftFront;
	Leg2DoF* rightFront;
	Leg2DoF* leftBack;
	Leg2DoF* rightBack;

	IMUFilter* imuFilter;
	QuadrupedWalkingAlgorithm walkingAlgorithm;

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

	QuadrupedRobot(Leg2DoF &leftFront, Leg2DoF &leftBack, Leg2DoF &rightFront, Leg2DoF &rightBack, IMUFilter &imuFilter);

	void setLeftFrontMountingPosition(float x, float y);    // legs mounting position relative to the center of body
	void setRightFrontMountingPosition(float x, float y);
	void setLeftBackMountingPosition(float x, float y);
	void setRightBacktMountingPosition(float x, float y);

	bool setXOffset(float offset);    // set body offset
	bool setYOffset(float offset);
	bool setZOffset(float offset);
	bool setXSpacing(float spacing);    // set spacing between legs
	bool setYSpacing(float spacing);
    bool setAllLegsPosition(float x, float y, float z);    // set same positions of all the legs

    void setXBodyAngle(float angle);    // set body orientation offset
	void setYBodyAngle(float angle);

	bool moveAllLegs();    // move legs to set positions
	bool levelBody();      // level body to angle

	void setStartLegs(bool startLeg);    // set first moving leg pair (true - leftFront, rightBack)

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