#pragma once
#include <AlphaBeta.hpp>

class IMUFilter
{
    private:

    alpha_beta xAccFilter;
    alpha_beta yAccFilter;
    alpha_beta xSmoothingFilter;
    alpha_beta ySmoothingFilter;

    bool isReady;

    float dt;

    float xAngle;
    float yAngle;
    float zAngle;

    float accSensitivity;
    float gyroSensitivity;

    float normalize(float in, float minimum, float maximum);
    float roll(float angle);

    public:

    IMUFilter();
    
    void init    // set filters parameters
    (
        float alphaAccFilter,
        float betaAccfilter,
        float alphaSmoothingFilter,
        float betaSmoothingFilter,
        float accSensitivity,
        float gyroSensitivity,
        float dt
    );

    void zeroZAngle();    // set angle on z axis on 0

    bool update(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ);    // use in timer interrupt (dt)
    float getXAngle();
    float getYAngle();
    float getZAngle();            // <0, 2pi> 
    float getAbsoluteZAngle();    // <-inf, inf>
};