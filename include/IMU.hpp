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

    int init(int port);                                 // CS port
    void calibrate(int time, float gyroSensitivity);    // time [s], sensitivity [radians]
    void readSensor();                                  // read all axis of acc, gyro, mag

    float getAccX();     // [m/s^2]
    float getAccY();
    float getAccZ();
    float getGyroX();    // [radians]
    float getGyroY();
    float getGyroZ();
    float getTemperature();    // *C
};