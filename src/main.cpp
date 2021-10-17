#include <Arduino.h>
#include <TimerOne.h>
#include <MsTimer2.h>
#include <InternalTemperature.h>

#include <RC.hpp>
#include <IMU.hpp>
#include <IMUFilter.hpp>
#include <PowerSystem.hpp>
#include <QuadrupedBodyKinematics.hpp>
#include <RobotController.hpp>

const float dt = 0.005;

ServoMotor 
frontLeftHip(2, 4, 1.12, true),
frontLeftKnee(3, 5, 1.1, false),

frontRightHip(23, -13, 1.16, false),
frontRightKnee(22, -22, 1.15, true),

backLeftHip(29, -11, 1.14, false),
backLeftKnee(31, 3, 1.1, true),

backRightHip(14, 4, 1.1, true),
backRightKnee(15, 12, 1.13, false);

Leg2DoF
frontLeft(0.0425, 0.055, 0.0058, 0.06, 0.06, false, true, true, 0.003, 0, 0.025, frontLeftHip, frontLeftKnee),
backLeft(0.0425, 0.055, 0.0058, -0.06, -0.06, true, true, true, 0.003, 0, 0.025, backLeftHip, backLeftKnee),
frontRight(0.0425, 0.055, 0.0058, 0.06, 0.06, false, false, true, 0.003, 0, 0.025, frontRightHip, frontRightKnee),
backRight(0.0425, 0.055, 0.0058, -0.06, -0.06, true, false, true, 0.003, 0, 0.025, backRightHip, backRightKnee);

IMU imuSensor;
IMUFilter filter;
RC remote(30, 33, 9, 16, 17);
PowerSystem pwrSystem(28, 27, 7, 8.4, 9.9);
QuadrupedBodyKinematics robot(0.12, 0.08, frontLeft, backLeft, frontRight, backRight);
RobotController controller(frontLeft, backLeft, frontRight, backRight, robot, filter);

void getAngle()
{
  imuSensor.readSensor();

  filter.update
  (
    imuSensor.getAccX(),
    imuSensor.getAccY(),
    imuSensor.getAccZ(),
    imuSensor.getGyroX(),
    imuSensor.getGyroY(),
    imuSensor.getGyroZ()
  );
}

void printImuSensor()
{
  Serial.print(imuSensor.getGyroX());
  Serial.print(" ");
  Serial.print(imuSensor.getGyroY());
  Serial.print(" ");
  Serial.print(imuSensor.getGyroZ());

  Serial.print(" ... ");
  Serial.print(imuSensor.getAccX());
  Serial.print(" ");
  Serial.print(imuSensor.getAccY());
  Serial.print(" ");
  Serial.print(imuSensor.getAccZ());

  Serial.print(" ... ");
  Serial.print(imuSensor.getTemperature());
  Serial.println("*C");
}

void printOrientation()
{
  Serial.print(filter.getXAngle()*180/M_PI);
  Serial.print("   ");
  Serial.print(filter.getYAngle()*180/M_PI);
  Serial.print("   ");
  Serial.println(filter.getZAngle()*180/M_PI);
}

void printBatteryStatus()
{
  Serial.print("Battery voltage: ");
  Serial.print(pwrSystem.getBatteryVoltage());
  Serial.print("V (");
  Serial.print(pwrSystem.getBatteryLevel());
  Serial.println("%)");
}

void setup() 
{
  pwrSystem.on();

  Serial.begin(9600);
  delay(100);

  Timer1.initialize(10);
  Timer1.attachInterrupt(moveClock);

  robot.setAllLegsPosition(0, 0, 0.07);
  controller.setBody(0, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveAllLegs();

  controller.setStartLegs(true); //leftFront, rightBack

  imuSensor.init(10);
  filter.init(0.03, 0.00001, 0.03, 0.00001, 0.1, 0.01, dt);

  MsTimer2::set(dt*1000, getAngle);
  MsTimer2::start();
  delay(1000);

  imuSensor.calibrate(1, 0.1);
  delay(1000);

  filter.zeroZAngle();
}

void loop()
{
  //controller.walk(0.06, 0.1, 20, 0.08, 0.005, 0.02, 0.02, 0, 0, false);
  //controller.goForAzimuth(0.06, 0.1, 20, 0.08, 0.015, 0.02, filter.getZAngle(), 0, M_PI/12, false);
  //controller.walk(0.06, 0.1, 20, 0.07, 0.008, 0.01, 0.01, 0, 0, false);
  //controller.levelBody();
  controller.jump(0.05, 0.08, 4);
  delay(150);

  //printImuSensor();
  //printOrientation();

  if(pwrSystem.getBatteryLevel() == 0)
  {
    while(true) 
    {
      Serial.println("Battery is empty! Charge now!");
      delay(1000);
    }
  }

  if(imuSensor.getTemperature() > 60)
  {
    while(imuSensor.getTemperature() > 50) 
    {
      Serial.print("Board Overheating! (");
      Serial.print(imuSensor.getTemperature());
      Serial.println("*C)");
      delay(1000);
    }
  }

  if(InternalTemperature.readTemperatureC() > 80)
  {
    while(InternalTemperature.readTemperatureC() > 70) 
    {
      Serial.print("CPU Overheating! (");
      Serial.print(InternalTemperature.readTemperatureC());
      Serial.println("*C)");
      delay(1000);
    }
  }
}
