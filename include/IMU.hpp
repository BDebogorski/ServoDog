#pragma once
#include <MPU9250.h>

class IMU
{
    private:

    MPU9250 * MPU;

    float xGyroOffset;
    float yGyroOffset;
    float zGyroOffset;

    public:

    IMU();
    ~IMU();

    int init(int port);
    void calibrate(int time, float gyroSensitivity);
    void readSensor();

    float getAccX();
    float getAccY();
    float getAccZ();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    float getTemperature();
};