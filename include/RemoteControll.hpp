#pragma once

#include <Arduino.h>

class RemoteControll
{
    private:

        int xLeft;
        int yLeft;

        int xRight;
        int yRight;

        int potLeft;
        int potRight;

        bool swJoyLeft;
        bool swJoyRight;

        bool swLeft;
        bool swRight;

        int xMin;
        int xMax;
        int yMin;
        int yMax;
        unsigned int diff;

        bool mixerready;

    public:

        RemoteControll();

        void getValues(String packet);

        bool setMixer(int xMin, int xMax, int yMin, int yMax, unsigned int diff);
        bool mixer(float x, float y, float &left, float &right, float outMax);

        int getLeftJoyX();
        int getLeftJoyY();
        bool getLeftJoySwitch();

        int getRightJoyX();
        int getRightJoyY();
        bool getRightJoySwitch();

        int getLeftPotentiometer();
        int getRightPotentiometer();

        bool getLeftSwitch();
        bool getRightSwitch();
};