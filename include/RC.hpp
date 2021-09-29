#pragma once

class RC
{
    private:

    float xMin;
    float xMid;
    float xMax;
    float yMin;
    float yMid;
    float yMax;
    float diff;

    float maxStepSize;

    float left;
    float right;

    int horizontalPort;
    int verticalPort;
    int stepHeightPort;
    int heightPort;
    int speedPort;

    int radio(int port);

    public:

    RC(int horizontalPort, int verticalPort, int heightPort, int stepHightPort, int speedPort);
    void init(float xMin, float xMid, float xMax, float yMin, float yMid, float yMax, float diff, float maxStepSize);
    void mixer(bool mixerOn);
    float getLeft();
    float getRight();
    float getHeight(float min, float max);
    float getStepHeight(float min, float max);
    float getSpeed(float min, float max);
};