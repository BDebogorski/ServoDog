#include <PowerSystem.hpp>
#include <Arduino.h>

PowerSystem::PowerSystem(int enablePin, int voltagePin, float minBatteryVoltage, float maxBatteryVoltage, float divisorRate)
{
    this->enablePin = enablePin;
    this->voltagePin = voltagePin;

    this->minBatteryVoltage = minBatteryVoltage;
    this->maxBatteryVoltage = maxBatteryVoltage;
    this->divisorRate = divisorRate;

    batteryLevel = 100;
}

void PowerSystem::on()
{
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);
    delay(2000);
    pinMode(enablePin,OUTPUT_OPENDRAIN);
}

void PowerSystem::off()
{
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW);
}

float PowerSystem::getBatteryVoltage()
{
    float voltage = analogRead(voltagePin);
    batteryVoltage =  voltage/1024*divisorRate;
    return batteryVoltage;
}

int PowerSystem::getBatteryLevel()
{
    getBatteryVoltage();
    if(batteryVoltage < minBatteryVoltage) return 0;
    if(batteryVoltage > maxBatteryVoltage) return 100;

    batteryLevel = (batteryVoltage-minBatteryVoltage)/(maxBatteryVoltage-minBatteryVoltage)*100;
    return batteryLevel;
}