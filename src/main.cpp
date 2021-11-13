#include <Arduino.h>
#include <TimerOne.h>
#include <MsTimer2.h>
#include <InternalTemperature.h>

#include <RC.hpp>
#include <IMU.hpp>
#include <ADC.hpp>
#include <BlueControll.hpp>
#include <RemoteControll.hpp>
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
frontLeft(0.0425, 0.055, 0.0058, 0.06, 0.06, false, true, true, 0.00488, 0, 0.021, frontLeftHip, frontLeftKnee),
backLeft(0.0425, 0.055, 0.0058, -0.06, -0.06, true, true, true, 0.00488, 0, 0.0211, backLeftHip, backLeftKnee),
frontRight(0.0425, 0.055, 0.0058, 0.06, 0.06, false, false, true, 0.00488, 0, 0.021, frontRightHip, frontRightKnee),
backRight(0.0425, 0.055, 0.0058, -0.06, -0.06, true, false, true, 0.00488, 0, 0.021, backRightHip, backRightKnee);

IMU imuSensor;
IMUFilter filter;
ADC adc(0x16, 0x17);
RemoteControll remoteControll;
BlueControll blueRC('%');
PowerSystem pwrSystem(28, 27, 7, 8.4, 9.9);
QuadrupedBodyKinematics bodyKinematics(0.12, 0.08, frontLeft, backLeft, frontRight, backRight);
RobotController controller(frontLeft, backLeft, frontRight, backRight, bodyKinematics, pwrSystem, filter, remoteControll);

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

void update()
{
  imuSensor.readSensor();

  filter.update
  (
    -imuSensor.getAccX(),
    -imuSensor.getAccY(),
    imuSensor.getAccZ(),
    -imuSensor.getGyroX(),
    -imuSensor.getGyroY(),
    -imuSensor.getGyroZ()
  );

  blueRC.recive();
  //printOrientation();
}

void setup() 
{
  pwrSystem.on();

  Serial.begin(9600);
  Serial1.begin(9600);
  delay(100);

  Timer1.initialize(10);
  Timer1.attachInterrupt(moveClock);

  imuSensor.init(10);
  imuSensor.calibrate(1, 0.1);
  filter.init(0.03, 0.00001, 0.3, 0.00001, 0.01, 0.01, dt);
  MsTimer2::set(dt*1000, update);
  MsTimer2::start();

  bodyKinematics.setAllLegsPosition(0, 0, 0.08);
  controller.moveAllLegsSmoothly(0.8, 100, false);
  controller.setStartLegs(LeftFront_RightBack); //up

  bodyKinematics.setXSpacing(0.11);
  controller.moveBodySmoothly(0.5, 100, false);

  remoteControll.setMixer(1000, 2000, 1000, 2000, 20);

  filter.zeroZAngle();
}

void loop()
{
  if(!controller.safeController(imuSensor.getTemperature(), InternalTemperature.readTemperatureC(), 45, 70))
  {
    controller.sitAndTurnOff(0.3, 100);
  }

  if(blueRC.isReady())
  {
    remoteControll.getValues(blueRC.getReciveData());
  }

  if(remoteControll.getRightSwitch()) controller.levelBody();

  if(remoteControll.getLeftJoySwitch()) 
  {
    controller.jump(1.2, 1.2, 0, 0.8, 0.04, 0, 0.065, 0.085, 0.0001, false);
    delay(110);
  }

  if(remoteControll.getRightPotentiometer() > 1800) filter.zeroZAngle();

  if(remoteControll.getRightJoySwitch()) 
  {
    controller.goForAzimuth(0.06, 0.1, 20, 0.08, 0.015, -0.02, filter.getZAngle(), 0, M_PI/12, false);
  }

  //float x = float(map(remoteControll.getRightJoyY(), 1000, 2000, -15, 15))/1000;
  float z = float(map(remoteControll.getLeftPotentiometer(), 1000, 2000, -40, 0))/1000;
  float xAngle = -float(map(remoteControll.getRightJoyX(), 1000, 2000, -8, 8))/180*M_PI;
  float yAngle = -float(map(remoteControll.getRightJoyY(), 1000, 2000, -10, 10))/180*M_PI;
  controller.remoteBodyPosition(0.6, 50, 0, 0, z, xAngle, yAngle, remoteControll.getRightSwitch());

  controller.remoteControllTankWalk(0.06, 0.1, 20, remoteControll.getLeftJoyX(), remoteControll.getLeftJoyY(), 0.08, 0.01, 0.03, true, false);
  controller.remoteOff(remoteControll.getLeftSwitch(), 0.8, 100);

/*
  controller.setBody(0, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);

  controller.setBody(0, 0, 0, M_PI/18, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);
  controller.setBody(0, 0, 0, -M_PI/18, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);
  controller.setBody(0, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);

  controller.setBody(0, 0, 0, 0, -M_PI/18, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);
  controller.setBody(0, 0, 0, 0, M_PI/18, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);
  controller.setBody(0, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);

  controller.setBody(-0.02, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);
  controller.setBody(0.02, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);

  controller.setBody(0, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100, false);

  for(int i = 0; i<20; i++)
  {
    controller.walk(0.06, 0.1, 20, 0.08, 0.01, float(i)/800, float(i)/800, 0, 0, false);
  }

  for(int i = 0; i<20; i++)
  {
    controller.walk(0.06, 0.1, 20, 0.08, 0.01, -float(i)/800, -float(i)/800, 0, 0, false);
  }

  controller.zeroByWalking(0.06, 20, 0.08, 0.01, false);

  bodyKinematics.setAllLegsPosition(0, 0, 0.065);
  controller.moveAllLegsSmoothly(0.8, 100, false);

  for(int i = 0; i<6; i++)
  {
    controller.jump(+1.2, +1.2, 0, 0.8, 0.04, 0, 0.065, 0.085, 0.0001, false);
    delay(110);
  }

  controller.sitAndTurnOff(0.8, 100);
*/
  //controller.walk(0.06, 0.1, 20, 0.08, 0.005, 0.02, 0.02, 0, 0, false);
  //controller.goForAzimuth(0.06, 0.1, 20, 0.08, 0.015, 0.02, filter.getZAngle(), 0, M_PI/12, false);
  //controller.walk(0.06, 0.1, 20, 0.07, 0.008, 0.01, 0.01, 0, 0, false);
  //controller.levelBody();

  //controller.jump(-1.2, 0, 0.8, 0, 0, 0.04, 0, 0.065, 0.085, 0.0001, false);
  //controller.jumpForAzimuth(1.2, 0.8, 0.04, 0.065, 0.085, 0.0001, filter.getZAngle(), 0, M_PI/20, false);
  //delay(80);

  //bodyKinematics.setAllLegsPosition(0.02, 0, 0.08);
  //controller.moveAllLegsSmoothly(1, 100);
  printImuSensor();
  //printOrientation();
  //delay(10);
  //printBatteryStatus();

  //controller.sitAndTurnOff(0.8, 100);
}