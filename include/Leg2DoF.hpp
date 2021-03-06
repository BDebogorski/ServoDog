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

    float xStart;    // x start (zero) position
    float yStart;    // y start (zero) position
    float zStart;    // z start (zero) position

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

    float lastXOffset = 0;
    float lastYOffset = 0;
    float lastZOffset = 0;

    bool readyToMove;

    bool invKinematics(float x, float y, float z);    // in this model y = 0

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
        float xStart,               // x axis starting position
        float yStart,               // y axis starting position
        float zStart,               // z axis starting position
        ServoMotor &hip,            // hip motor
        ServoMotor &knee            // knee motor
	);

    bool setXOffset(float offset);
    bool setYOffset(float offset);
    bool setZOffset(float offset);

    bool setOffset(float xOffset, float yOffset, float zOffset);

    bool setPosition(float x, float y, float z);    // set leg position
    bool setStartPosition();                        // set start leg position
    bool move();                                    // movement of the leg to the set position

    float getXOffset();
    float getYOffset();
    float getZOffset();

    float getLastXOffset();
    float getLastYOffset();
    float getLastZOffset();

    float getXPosition();
    float getYPosition();
    float getZPosition();

    float getLastXPosition();
    float getLastYPosition();
    float getLastZPosition();

    float getKneeAngle();    // radians
    float getHipAngle();     // radians

    float getXStartPosition();
    float getYStartPosition();
    float getZStartPosition();

    bool isReadyToMove();
};