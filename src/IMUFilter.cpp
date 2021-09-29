#include <Arduino.h>
#include <IMUFilter.hpp>

const float G = 9.81;

float IMUFilter::normalize(float in, float minimum, float maximum)
{
    float out;

    if(in>maximum) out = maximum;
    else if(in<minimum) out = minimum;
    else out = in;

    return out;
}

float IMUFilter::roll(float angle)
{
    if(angle > 2*M_PI) return angle - 2*M_PI;// - 2*M_PI*uint(angle/(2*M_PI));
    if(angle < 0) return angle + 2*M_PI;// + 2*M_PI*uint(angle/(2*M_PI));
    else return angle;
}

IMUFilter::IMUFilter()
{
    isReady = false;
}

void IMUFilter::init
(
    float alphaAccFilter,
    float betaAccfilter,
    float alphaSmoothingFilter,
    float betaSmoothingFilter,
    float accSensitivity,
    float gyroSensitivity,
    float dt
){
    
    xAngle = M_PI;
    yAngle = M_PI;
    zAngle = 0;

    this->dt = dt;
    this->accSensitivity = accSensitivity;
    this->gyroSensitivity = gyroSensitivity;

    xAccFilter.setParam(alphaAccFilter, betaAccfilter);
    yAccFilter.setParam(alphaAccFilter, betaAccfilter);

    xSmoothingFilter.setParam(alphaSmoothingFilter, betaSmoothingFilter);
    ySmoothingFilter.setParam(alphaSmoothingFilter, betaSmoothingFilter);

    isReady = true;
}

void IMUFilter::zeroZAngle()
{
    zAngle = 0;
}

bool IMUFilter::update(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ)
{
    if(!isReady) return false;

    accX = normalize(xAccFilter.addSample(accX, dt), -G, G);
    accY = normalize(yAccFilter.addSample(accY, dt), -G, G);
    accZ = normalize(accZ, -G, G);

    if(fabs(sqrt(accX*accX+accY*accY+accZ*accZ)-G) < accSensitivity && fabs(gyroX) < gyroSensitivity && fabs(gyroY) < gyroSensitivity )
    {
        if(accZ < 0)
        {
            xAngle = asin(accY/G);
            yAngle = -asin(accX/G);
        }
        else
        {
            xAngle = M_PI-asin(accY/G);
            yAngle = -M_PI+asin(accX/G);
        }
    }
    else
    {
        xAngle -= gyroX*dt;
        yAngle -= gyroY*dt;
        zAngle -= gyroZ*dt;
    }

    xSmoothingFilter.addSample(roll(xAngle), dt); //przeniesc do getAngle() i dac % 
    ySmoothingFilter.addSample(roll(yAngle), dt);

    return true;
}

float IMUFilter::getXAngle()
{
    return xSmoothingFilter.getValue();
}

float IMUFilter::getYAngle()
{
    return ySmoothingFilter.getValue();
}

float IMUFilter::getAbsoluteZAngle()
{
    return zAngle;
}

float IMUFilter::getZAngle()
{
    float angle;

    angle = zAngle - 2*M_PI*int(zAngle/(2*M_PI));

    if(angle > M_PI) angle = angle - 2*M_PI;
    if(angle < -M_PI) angle = angle + 2*M_PI;

    return angle;
}