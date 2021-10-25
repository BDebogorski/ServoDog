#pragma once

#define LeftFront_RightBack 0
#define LeftBack_RightFront 1

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

    float xZero;
    float yZero;

    float xStart;
    float yStart;

    float xEnd;
    float yEnd;
};

class QWA
{
    private:

    Leg leftFront;
    Leg rightFront;
    Leg leftBack;
    Leg rightBack;

    int nPoints;
    float zBodyPosition;
    float stepHeight;
    float xLengthL;
    float xLengthR;
    float yLength;
    float rotation;

    bool startLeg;
    bool isReady;

    void setDefaultEndPoint(float xLength, Leg &leg);
    void calculateUpLeg(int pointNumber, float xLength, Leg &leg);
    void calculateDownLeg(int pointNumber, float xLength, Leg &leg);

    void calculateLeg(int pointNumber, float xLength, bool pair, bool defaultEndPoint, Leg &leg);

    public:

    QWA();    // constructor

    void setLeftFrontZeroPosition(float x, float y);    // zero position relative to center of body
    void setRightFrontZeroPosition(float x, float y);
    void setLeftBackZeroPosition(float x, float y);
    void setRightBackZeroPosition(float x, float y);

    void setStartLeg(bool startLeg);    // set first moving forward leg pair

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

    void setLeftFrontStartPosition(float x, float y);    // set current position of leg, optional (default [0,0])
    void setRightFrontStartPosition(float x, float y);
    void setLeftBackStartPosition(float x, float y);
    void setRightBackStartPosition(float x, float y);

    void setLeftFrontEndPosition(float x, float y);    // set goal position of leg, optional (default is optimal)
    void setRightFrontEndPosition(float x, float y);
    void setLeftBackEndPosition(float x, float y);
    void setRightBackEndPosition(float x, float y);

    bool calculate(int poinNumber);    // calculate position for number of point in step (walking)

    bool getLeftFrontLegPosition(Cordinates &cordinates);    // get goal cordinates of leg (x, y, z)
    bool getRightFrontLegPosition(Cordinates &cordinates);
    bool getLeftBackLegPosition(Cordinates &cordinates);
    bool getRightBackLegPosition(Cordinates &cordinates);
};