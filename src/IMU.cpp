#include <IMU.hpp>

const float dtMin = 0.005; //minimalny okres probkowania dla imu

IMU::IMU()
{
    xGyroOffset = 0;
    yGyroOffset = 0;
    zGyroOffset = 0;
}

IMU::~IMU()
{
    delete MPU;
}

int IMU::init(int port)
{
    MPU = new MPU9250(SPI, port);
    int status = MPU->begin();

    if (status < 0) 
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("IMU Status: ");
        Serial.println(status);
       //return status;
    }
       
    MPU->setAccelRange(MPU9250::ACCEL_RANGE_8G);
    MPU->setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    MPU->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);

    return status;
}

void IMU::calibrate(int time, float gyroSensitivity)
{
    float xGyro;
    float yGyro;
    float zGyro;

    float avGyroX = 0;
    float avGyroY = 0;
    float avGyroZ = 0;

    float xGyroStart;
    float yGyroStart;
    float zGyroStart;

    Serial.println("IMU calibration...");

    MPU->readSensor();
    xGyroStart = MPU->getGyroX_rads();
    yGyroStart = MPU->getGyroY_rads();
    zGyroStart = MPU->getGyroZ_rads();

    for(int i = 0; i<time/dtMin; i++)
    {
        MPU->readSensor();
        xGyro = MPU->getGyroX_rads();
        yGyro = MPU->getGyroY_rads();
        zGyro = MPU->getGyroZ_rads();

        if(
            fabs(xGyro-xGyroStart) > gyroSensitivity ||
            fabs(yGyro-yGyroStart) > gyroSensitivity ||
            fabs(zGyro-zGyroStart) > gyroSensitivity
        ){
            i = 0;
            avGyroX = 0;
            avGyroY = 0;
            avGyroZ = 0;

            xGyroStart = xGyro;
            yGyroStart = yGyro;
            zGyroStart = zGyro;
            
            Serial.println("Restart IMU calibration!");
            delay(500);
        }
        else
        {
            MPU->readSensor();

            avGyroX += xGyro;
            avGyroY += yGyro;
            avGyroZ += zGyro;
        }

        delay(dtMin*1000);
    }

    xGyroOffset = avGyroX/time*dtMin;
    yGyroOffset = avGyroY/time*dtMin;
    zGyroOffset = avGyroZ/time*dtMin;

    Serial.println("IMU is ready.");
}

void IMU::readSensor()
{
    MPU->readSensor();
}

float IMU::getAccX()
{
    return MPU->getAccelX_mss();
}

float IMU::getAccY()
{
    return MPU->getAccelY_mss();
}

float IMU::getAccZ()
{
    return MPU->getAccelZ_mss();
}

float IMU::getGyroX()
{
    return MPU->getGyroX_rads() - xGyroOffset;
}

float IMU::getGyroY()
{
    return MPU->getGyroY_rads() - yGyroOffset;
}

float IMU::getGyroZ()
{
    return MPU->getGyroZ_rads() - zGyroOffset;
}

float IMU::getTemperature()
{
    return MPU->getTemperature_C();
}