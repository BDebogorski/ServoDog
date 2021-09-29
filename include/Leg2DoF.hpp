#pragma once
#include <ServoMotor.hpp>

class Leg2DoF
{
    private:

    ServoMotor* hip;
    ServoMotor* knee;

    float r1;
	float r2;
    float rFeet;

    bool xSide;
	bool ySide;
    bool zSide;

    float xMountingPosition;
    float yMountingPosition;

    float xPosition;    // x position
    float yPosition;    // y position
    float zPosition;    // z position

    float lastXPosition;    // last x position set on motors
    float lastYPosition;    // last x position set on motors
    float lastZPosition;    // last x position set on motors

    float t1;    // angle of first motor
    float t2;    // angle of second motor

    float xOffset;
    float yOffset;
    float zOffset;

    float xStartStep;    // start point of step
    float yStartStep;    // start point of step

    float lengthOfMove;    // length of step move (XY)

    bool readyToMove;

    bool invKinematics(float x, float y, float z);    // in this model y = 0

    void getStepPointXY
    (
        int nPoints,        // number of points in trajectory
        int pointNumber,    // number of point in trajectory
        int pairLeg,        // pair of legs {-1, 1}
        float xLength,      // one step lenght on x axis
        float yLength,      // one step lenght on y axis
        float rotation,     // robot rotation of one step
        float& xPoint,      // x cordinate of point in trajectory
        float& yPoint       // y cordinate of point in trajectory
    );

    public:

    Leg2DoF();

	Leg2DoF
	(
		float r1,                   // length of first part
		float r2,                   // length of second part
		float rFeet,                // radius of feet
        float xMountingPosition,    // y leg mounting position relative to the center of body
        float yMountingPosition,    // y leg mounting position relative to the center of body
        bool xSide,                 // direction of x axis
		bool ySide,                 // direction of y axis
        bool zSide,                 // direction of z axis
		ServoMotor &hip,            // hip motor
		ServoMotor &knee            // knee motor
	);

    bool setXOffset(float offset);
    bool setYOffset(float offset);
    bool setZOffset(float offset);

    bool setOffset(float xOffset, float yOffset, float zOffset);

    bool setPosition(float x, float y, float z);    // set leg position
    bool move();                                    // movement of the leg to the set position

    void calculateStepParameters(int nPoints, float xLength, float yLength, float rotation);  // use before step

    void setStepPoint
    (
        int pointNumber,     // number of point in trajectory
        int nPoints,         // number of points in trajectory
        float xLength,       // one step lenght on x axis
        float yLength,       // one step lenght on y axis
        float rotation,      // rotation for one step
        float stepHeight,    // height of step
        float zPos,          // z position of body
        bool isOnGround      // leg is on ground
    );

    float getXOffset();
    float getYOffset();
    float getZOffset();

    float getXPosition();
    float getYPosition();
    float getZPosition();

    float getLastXPosition();
    float getLastYPosition();
    float getLastZPosition();

    float getKneeAngle();    // radians
    float getHipAngle();     // radians
};