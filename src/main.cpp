#include <Arduino.h>
#include <TimerOne.h>
#include <MsTimer2.h>
#include <RN487x_BLE.h>
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
frontLeft(0.0425, 0.055, 0.0058, 0.06, 0.06, false, true, true, 0.00488, 0, 0.021, frontLeftHip, frontLeftKnee),
backLeft(0.0425, 0.055, 0.0058, -0.06, -0.06, true, true, true, 0.00488, 0, 0.0211, backLeftHip, backLeftKnee),
frontRight(0.0425, 0.055, 0.0058, 0.06, 0.06, false, false, true, 0.00488, 0, 0.021, frontRightHip, frontRightKnee),
backRight(0.0425, 0.055, 0.0058, -0.06, -0.06, true, false, true, 0.00488, 0, 0.021, backRightHip, backRightKnee);

IMU imuSensor;
IMUFilter filter;
RC remote(30, 33, 9, 16, 17);
PowerSystem pwrSystem(28, 27, 7, 8.4, 9.9);
QuadrupedBodyKinematics bodyKinematics(0.12, 0.08, frontLeft, backLeft, frontRight, backRight);
RobotController controller(frontLeft, backLeft, frontRight, backRight, bodyKinematics, pwrSystem, filter);

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

  controller.setStartLegs(LeftFront_RightBack); //up

  imuSensor.init(10);
  filter.init(0.03, 0.00001, 0.03, 0.00001, 0.1, 0.0001, dt);

  MsTimer2::set(dt*1000, getAngle);
  MsTimer2::start();

  //imuSensor.calibrate(1, 0.1);

  bodyKinematics.setAllLegsPosition(0, 0, 0.08);
  controller.moveAllLegsSmoothly(0.8, 100);
  
  //controller.setBody(0, 0, 0, 0, 0, 0.12, 0.07);
  //controller.setBody(0.01, 0, 0, 0, -M_PI/18, 0.12, 0.08);
  //controller.moveBodySmoothly(0.8, 100);

  controller.setBody(0, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);

  controller.setBody(0, 0, 0, -M_PI/18, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);
  controller.setBody(0, 0, 0, M_PI/18, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);
  controller.setBody(0, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);

  controller.setBody(0, 0, 0, 0, M_PI/12, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);
  controller.setBody(0, 0, 0, 0, -M_PI/12, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);
  controller.setBody(0, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);

  controller.setBody(-0.02, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);
  controller.setBody(0.02, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);

  controller.setBody(0, 0, 0, 0, 0, 0.12, 0.08);
  controller.moveBodySmoothly(0.8, 100);

  for(int i = 0; i<20; i++)
  {
    controller.walk(0.06, 0.1, 20, 0.08, 0.01, float(i)/800, float(i)/800, 0, 0, false);
  }

  for(int i = 0; i<20; i++)
  {
    controller.walk(0.06, 0.1, 20, 0.08, 0.01, -float(i)/800, -float(i)/800, 0, 0, false);
  }

  controller.walk(0.06, 0.1, 20, 0.08, 0.01, 0, 0, 0, 0, false);
  controller.walk(0.06, 0.1, 20, 0.08, 0.01, 0, 0, 0, 0, false);

  for(int i = 0; i<6; i++)
  {
    controller.jump(+1.2, +1.2, 0, 0.8, 0.04, 0, 0.065, 0.085, 0.0001, false);
    delay(110);
  }

  controller.sitAndTurnOff(0.8, 100);
}

void loop()
{
  //controller.walk(0.06, 0.1, 20, 0.08, 0.005, 0.02, 0.02, 0, 0, false);
  //controller.goForAzimuth(0.06, 0.1, 20, 0.08, 0.015, 0.02, filter.getZAngle(), 0, M_PI/12, false);
  //controller.walk(0.06, 0.1, 20, 0.07, 0.008, 0.01, 0.01, 0, 0, false);
  //controller.levelBody();

  //controller.jump(-1.2, 0, 0.8, 0, 0, 0.04, 0, 0.065, 0.085, 0.0001, false);
  //controller.jumpForAzimuth(1.2, 0.8, 0.04, 0.065, 0.085, 0.0001, filter.getZAngle(), 0, M_PI/20, false);
  //delay(80);

  //bodyKinematics.setAllLegsPosition(0.02, 0, 0.08);
  //controller.moveAllLegsSmoothly(1, 100);
  //printImuSensor();
  //printOrientation();

  //controller.sitAndTurnOff(0.8, 100);

  if(pwrSystem.getBatteryLevel() == 0)
  {
    controller.sitAndTurnOff(0.8, 100);
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

/*
//include <Arduino.h>
#include <RN487x_BLE.h>

#define debugSerial Serial
#define bleSerial Serial1

#define SERIAL_TIMEOUT  10000

// MAC address added to the white list
const char* peerAddressToScan = "F0A1B40302D3" ;

void setup()
{

  pwrSystem.on();

  Serial.begin(9600);
  Serial.println("dzialam");

  while ((!debugSerial) && (millis() < SERIAL_TIMEOUT)) ;
  
  debugSerial.begin(115200) ;

  debugSerial.println("ok");

  // Set the optional debug stream
  rn487xBle.setDiag(debugSerial) ;
  // Initialize the BLE hardware
  rn487xBle.hwInit() ;
  // Open the communication pipe with the BLE module
  bleSerial.begin(rn487xBle.getDefaultBaudRate()) ;
  // Assign the BLE serial port to the BLE library
  rn487xBle.initBleStream(&bleSerial) ;
  // Finalize the init. process
  if (rn487xBle.swInit())
  {
    debugSerial.println("Init. procedure done!") ;
  }
  else
  {
    debugSerial.print("adress: ");
    debugSerial.println("Init. procedure failed!") ;
    debugSerial.println(rn487xBle.getBtAddress());
    rn487xBle.enterCommandMode() ;
    rn487xBle.setDefaultServices(DEVICE_INFO_SERVICE);
    rn487xBle.reboot() ;
    //while(1) ;
  }


  // >> Configuring the BLE
  // First enter in command/configuration mode
  rn487xBle.enterCommandMode() ;
  // Remove GATT services
  rn487xBle.setDefaultServices(NO_SERVICE) ;
  // Set passive scan and does not filter out duplicate scan results
  rn487xBle.setSupportedFeatures(PASSIVE_SCAN_BMP | NO_DUPLICATE_SCAN_BMP) ;
  // Take into account the settings by issuing a reboot
  rn487xBle.reboot() ;
  rn487xBle.enterCommandMode() ;
  // Clear the white list
  rn487xBle.clearWhiteList() ;
  // Add the MAC address to scan in the white list
  rn487xBle.addMacAddrWhiteList(true, peerAddressToScan) ;
  // Halt advertisement
  rn487xBle.stopAdvertising() ;

  // Start scanning
  rn487xBle.startScanning() ;

  debugSerial.println("Starter Kit as a Central with filtering a device added to the white list when performing a scan") ;
  debugSerial.println("===============================================================================================") ;
  
}

void loop()
{
  // Display the result of the scanning
  if (bleSerial.available())
  {
    debugSerial.print((char)bleSerial.read()) ;
  }
}
*/
