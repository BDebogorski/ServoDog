#pragma once

#include <vector>

#define FIRST_GROUP  0
#define SECOND_GROUP 1

struct Cordinates
{
    float x;
    float y;
    float z;
};

struct Leg
{
    float x;
    float y;
    float z;

    float xStart;
    float yStart;

    float xDefaultEnd;
    float yDefaultEnd;

    float xEnd;
    float yEnd;

    bool group;
};

class SRWA
{
    private:

    std::vector <Leg> legs;

    int nPoints;
    float zBodyPosition;
    float stepHeight;
    float xLengthL;
    float xLengthR;
    float yLength;
    float rotation;

    bool startLegs;
    bool isReady;

    void calculateDefaultEndPoint(float xLength, int ID);
    void setDefaultEndPoint(int ID);
    void calculateUpLeg(int pointNumber, float xLength, int ID);
    void calculateDownLeg(int pointNumber, float xLength, int ID);

    void calculateLeg(int pointNumber, float xLength, bool pair, int ID);

    public:

    SRWA();    // constructor

    void addLeg(bool group);

    bool setStartLegPosition(float x, float y, int ID);
    bool setEndLegPosition(float x, float y, int ID);

    void setStartLegs(bool group);    // set first moving forward leg pair

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

    bool calculate(int poinNumber);    // calculate position for number of point in step (walking)

    bool getDefaultEndPosition(Cordinates &cordinates, int ID);
    bool getLegPosition(Cordinates &cordinates, int ID);
};