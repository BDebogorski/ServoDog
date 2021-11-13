#pragma once
#include <Leg2DoF.hpp>
#include <IMUFilter.hpp>
#include <PowerSystem.hpp>
#include <RemoteControll.hpp>
#include <QuadrupedBodyKinematics.hpp>
#include <QuadrupedWalkingAlgorithm.hpp>

extern volatile int moveTimer;    // timer variable
void moveClock();                 // interrupt function

class RobotController
{
private:

	Leg2DoF* leftFront;
	Leg2DoF* rightFront;
	Leg2DoF* leftBack;
	Leg2DoF* rightBack;

    QuadrupedBodyKinematics* bodyKinematics;
	QuadrupedWalkingAlgorithm walkingAlgorithm;

    PowerSystem* pwrSystem;
    IMUFilter* imuFilter;
	RemoteControll* remoteControll;

	float xAngleOffset;
	float yAngleOffset;

	bool readyToControll;
	bool zeroLeg;
	bool isOn;

public:

	RobotController
    (
        Leg2DoF &leftFront,
        Leg2DoF &leftBack,
        Leg2DoF &rightFront,
        Leg2DoF &rightBack,
        QuadrupedBodyKinematics &bodyKinematics,    // inv kinematics of body
        PowerSystem &pwrSystem,                     // power system functions
        IMUFilter &imuFilter,                       // AHRS algorithm filter
		RemoteControll &remoteControll              // remote controller class
    );

	void setXAngleOffset(float offset);
	void setYAngleOffset(float offset);

    bool setBody(float x, float y, float z, float xAngle, float yAngle, float xSpacing, float ySpacing);    // set all body kinematics

    bool moveAllLegs();                                                       // moving all legs to goal positions
    bool moveBodySmoothly(float time, float nPoints, bool stabilization);     // moving body offset smoothly
    bool moveAllLegsSmoothly(float time, int nPoints, bool stabilization);    // moving smoothly all legs to goal positions

	bool levelBody();                    // level body to angle
	void setStartLegs(bool startLeg);    // set first moving leg pair (true - leftFront, rightBack)

	bool walk    // dynamic walking algorithm
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

	bool goForAzimuth
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

    bool jump   // jumping algorithm
    (
        float xLeftAcceleration,    // acceleration on x axis
        float xRightAcceleration,   // left legs acceleration on y axis
		float yAcceleration,        // right legs acceleration on y axis
        float zAcceleration,        // acceleration on z axis
        float xAmplitude,           // maximum amplitude of x axis
        float yAmplitude,           // maximum amplitude of y axis
        float zMin,                 // minimum z body position
        float zMax,                 // maximum z body position
        float dt,                   // delta of time
        bool stabilization          // stabilization on/off
    );

	bool jumpForAzimuth
    (
        float xAcceleration,    // acceleration on x axis
        float zAcceleration,    // acceleration on z axis
        float amplitude,        // maximum amplitude of x axis
        float zMin,             // minimum z body position
        float zMax,             // maximum z body position
        float dt,               // delta of time
		float actualAngle,      // z axis orientation in space
		float azimuth,          // goal angle
		float fullStepAngle,    // angle at which take full step
        bool stabilization      // stabilization on/off
    );

	bool zeroByWalking
	(
		float time,           // time of one step [s]
		float pause,          // time between steps [s]
		int nPoints,          // number of points in leg trajectory
		float zHeight,        // z axis body position
		float stepHeight,     // height of step
		bool stabilization    // stabilization on/off
	);

	bool remoteBodyPosition(float time, int nPoints, float x, float y, float z, float xAngle, float yAngle, bool stabilization);

	bool remoteControllTankWalk
	(
		float time,           // time of one step [s]
		float pause,          // time between steps [s]
		int nPoints,          // number of points in leg trajectory
		int xJoy,
		int yJoy,
		float heightBody,     // z axis body position
		float stepHeight,     // height of step
		float maxStepLength,  // macimum step length
		bool stop,            // stop or stamping if xJoy = 0 and yJoy = 0
		bool stabilization    // stabilization on/off
	);

	void remoteOff(bool on, float time, int nPoints);    //sit and turn off if on = 0

    void sitAndTurnOff(float time, int nPoints);    // sit smoothly and turn off

	/* false if low batterry or overheating */
	bool safeController(int iMUTemperature, int CPUTemperature, int maxIMUTemperature, int maxCPUTemperature);
};