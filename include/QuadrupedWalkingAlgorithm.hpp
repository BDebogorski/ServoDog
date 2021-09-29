#pragma once

struct LegCordinates
{
    float xPosition;
    float yPosition;
    float zPosition;
};

struct LegParameters
{
    float xStartStep;
    float yStartStep;
    float lengthOfMove;

    float xMountingPosition;
    float yMountingPosition;
};

class QuadrupedWalkingAlgorithm
{
    private:

    LegCordinates leftFrontCordinates;
    LegCordinates rightFrontCordinates;
    LegCordinates leftBackCordinates;
    LegCordinates rightBackCordinates;

    LegParameters leftFrontParameters;
    LegParameters rightFrontParameters;
    LegParameters leftBackParameters;
    LegParameters rightBackParameters;

    float xSpacing;
    float ySpacing;

    bool startLeg;

    int nPoints;
    float zBodyPosition;
    float stepHeight;
    float xLengthL;
    float xLengthR;
    float yLength;
    float rotation;

    bool isReady;

    void calculateStepParameters    // calculate initial step parameters
    (
        float xLength,
        LegParameters &legParameters,
        LegCordinates &legCordinates
    );

    void calculateStepPointXY    // calculate x and y cordinates of one point in step trajectory
    (
        int pointNumber,                // number of point in trajectory
        int pairLeg,                    // pair of legs {-1, 1}
        float xLength,                  // one step lenght on x axis
        float& xPoint,                  // x cordinate of point in trajectory
        float& yPoint,                  // y cordinate of point in trajectory
        LegParameters &legParameters,    // struct
        LegCordinates &legCordinates     // struct
    );

    float calculateZTrajectory(float xBeg, float yBeg, float x, float y, float lengthOfMove);    // calculate z of eliptic trajectory

    void calculateStepPoint    // calculate all cordinates of one point in step trajectory
    (
        int pointNumber,                // number of point in trajectory
        float xLength,                  // one step lenght on x axis
        bool isOnGround,                // leg is on ground
        LegParameters &legParameters,    // struct
        LegCordinates &legCordinates     // struct
    );

    public:

    QuadrupedWalkingAlgorithm();

    void setLeftFrontMountingPosition(float x, float y);    // mounting position
    void setRightFrontMountingPosition(float x, float y);
    void setLeftBackMountingPosition(float x, float y);
    void setRightBackMountingPosition(float x, float y);

    void setSpacing(float xSpacing, float ySpacing);
    void setStartLeg(bool startLeg);

    bool setWalkParameters    // set parameters of step
    (	
        int nPoints,            // number of points in leg trajectory
        float zBodyPosition,    // z axis body position
        float stepHeight,       // height of step
        float xLengthL,         // left legs one step lenght on x axis
        float xLengthR,         // right legs one step length on x axis
        float yLength,          // one step length on y axis
        float rotation          // body rotation for one step
    );

    void setLeftFrontLegPosition(float x, float y, float z);    // set current position of legs (default [0,0,zBodyPosition])
    void setRightFrontLegPosition(float x, float y, float z);
    void setLeftBackLegPosition(float x, float y, float z);
    void setRightBackLegPosition(float x, float y, float z);

    bool calculate(int poinNumber);    // calculate position for number of point in step

    bool getLeftFrontLegPosition(LegCordinates &legCordinates);    // get goal cordinates of legs
    bool getRightFrontLegPosition(LegCordinates &legCordinates);
    bool getLeftBackLegPosition(LegCordinates &legCordinates);
    bool getRightBackLegPosition(LegCordinates &legCordinates);
};