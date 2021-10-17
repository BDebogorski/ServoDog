#pragma once
#include <Leg2DoF.hpp>
#include <IMUFilter.hpp>
#include <QuadrupedBodyKinematics.hpp>
#include <QuadrupedWalkingAlgorithm.hpp>

extern volatile int moveTimer;
void moveClock();

class RobotController
{
private:

	Leg2DoF* leftFront;
	Leg2DoF* rightFront;
	Leg2DoF* leftBack;
	Leg2DoF* rightBack;

    QuadrupedBodyKinematics* bodyKinematics;
	IMUFilter* imuFilter;

	QuadrupedWalkingAlgorithm walkingAlgorithm;

public:

	RobotController
    (
        Leg2DoF &leftFront,
        Leg2DoF &leftBack,
        Leg2DoF &rightFront,
        Leg2DoF &rightBack,
        QuadrupedBodyKinematics &bodyKinematics,
        IMUFilter &imuFilter
    );

	void setLegMountingSpacing();        // legs mounting spacing
	bool levelBody();                    // level body to angle
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