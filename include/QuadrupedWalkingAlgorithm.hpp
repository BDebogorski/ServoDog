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

    float time;
    float pause;
    int nPoints;
    float zBodyPosition;
    float stepHeight;
    float xLengthL;
    float xLengthR;
    float yLength;
    float rotation;

    void calculateStepParameters    // calculate initial step parameters
    (
        float xLength,
        LegParameters legParameters,
        LegCordinates legCordinates
    );

    void calculateStepPointXY    // calculate x and y cordinates of one point in step trajectory
    (
        int pointNumber,    // number of point in trajectory
        int pairLeg,        // pair of legs {-1, 1}
        float xLength,      // one step lenght on x axis
        float& xPoint,      // x cordinate of point in trajectory
        float& yPoint,      // y cordinate of point in trajectory
        LegParameters legParameters,
        LegCordinates legCordinates
    );

    void calculateStepPoint    // calculate all cordinates of one point in step trajectory
    (
        int pointNumber,     // number of point in trajectory
        float xLength,       // one step lenght on x axis
        bool isOnGround,     // leg is on ground
        LegParameters legParameters,
        LegCordinates legCordinates
    );

    public:

    QuadrupedWalkingAlgorithm();

    void setLeftFrontMountingPosition(float x, float y);
    void setRightFrontMountingPosition(float x, float y);
    void setLeftBackMountingPosition(float x, float y);
    void setRightBackMountingPosition(float x, float y);

    void setSpacing(float xSpacing, float ySpacing);
    void setStartLeg(bool startLeg);

    void setWalkParameters
    (	
        float time,
        float pause,
        int nPoints,
        float zBodyPosition,
        float stepHeight,
        float xLengthL,
        float xLengthR,
        float yLength,
        float rotation
    );

    void initStep();  // to przenieść do setWalkParameters

    void setLeftFrontLegPosition(float x, float y, float z);
    void setRightFrontLegPosition(float x, float y, float z);
    void setLeftBackLegPosition(float x, float y, float z);
    void setRightBackLegPosition(float x, float y, float z);

    void calculate(float stepTime);

    void getLeftFrontLegPosition(LegCordinates &legCordinates);
    void getRightFrontLegPosition(LegCordinates &legCordinates);
    void getLeftBackLegPosition(LegCordinates &legCordinates);
    void getRightBackLegPosition(LegCordinates &legCordinates);
};